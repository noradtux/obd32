#include <Arduino.h>
#include <CAN.h>
#include <inttypes.h>
#include <stdlib.h>
#include <assert.h>
#include "obd.h"
#include "hexdump.h"
#include "car_struct.h"

#define DEBUG

const uint8_t CanFlowMsg[] = {0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

struct can_frame {
	uint32_t can_id;
	size_t length;
	uint8_t data[CAN_MAX_FRAME_LEN];
};

union bit_field {
	uint8_t value;
	uint8_t bit1 : 1;
	uint8_t bit2 : 1;
	uint8_t bit3 : 1;
	uint8_t bit4 : 1;
	uint8_t bit5 : 1;
	uint8_t bit6 : 1;
	uint8_t bit7 : 1;
	uint8_t bit8 : 1;
};

bool IsoTpInitialized = false;

int read_frame(uint32_t can_rx, uint16_t timeout, struct can_frame* buffer) {
	// Busy wait for response

	CAN.filter(can_rx);

	timeout = timeout + millis();
	size_t packet_size = 0;
	while (millis() < timeout && !packet_size) {
		packet_size = CAN.parsePacket();
	}

	assert (packet_size <= CAN_MAX_FRAME_LEN);

	buffer->can_id = CAN.packetId();
	buffer->length = packet_size;
	CAN.readBytes(buffer->data, buffer->length);
}
int read_frame(uint32_t can_rx, struct can_frame* buffer) {
	return read_frame(can_rx, CAN_DEFAULT_READ_TIMEOUT, buffer);
}

struct {
	size_t data_buf_len;
	size_t msg_len;
	uint32_t can_tx;
	uint16_t data_offset;
	uint8_t last_frame_idx;
	uint8_t data_buf[ISOTP_MAX_MSG_LEN];
	void (*isotp_callback)(uint8_t *, size_t);
	void (*frame_callback)(struct can_frame *);
	unsigned int can_receive_registered : 1;
	unsigned int transaction_running : 1;
} IsoTpState;

void isotp_receive(int packet_size) {
	struct can_frame frame;

	assert(packet_size <= CAN_MAX_FRAME_LEN);

	frame.can_id = CAN.packetId();
	frame.length = packet_size;
	size_t bytes_read = CAN.readBytes((char*)&frame.data, frame.length);
	assert(bytes_read == frame.length);
#if DEBUG
	Serial.print("Received frame: ");
	hexdump(frame.data, frame.length, 8);
#endif

	// XXX: how to handle timouts / missed frames, leading to transaction not ending ??

	if (IsoTpState.transaction_running) {
		switch (frame.data[0] & 0xf0) {  // frame_type
			case 0x00:                    // single frame
				IsoTpState.msg_len = frame.data[0] & 0x0f;
				assert(IsoTpState.msg_len <= ISOTP_MAX_MSG_LEN);
				memcpy(IsoTpState.data_buf, frame.data + 1,
						 min(IsoTpState.msg_len, ISOTP_MAX_MSG_LEN));
				IsoTpState.data_offset = IsoTpState.msg_len;
				break;
			case 0x10:                    // first frame
				assert(IsoTpState.data_offset == 0);  // if data_offset != 0 we have already seen the first frame
				IsoTpState.msg_len = (frame.data[0] & 0x0f) + frame.data[1];
				assert(IsoTpState.msg_len <= ISOTP_MAX_MSG_LEN);
				memcpy(IsoTpState.data_buf, frame.data + 2,
						 min(frame.length - 2, ISOTP_MAX_MSG_LEN));
				IsoTpState.data_offset = frame.length - 2;
				IsoTpState.last_frame_idx = 0;
#if DEBUG
				Serial.print("Send frame:     ");
				hexdump((uint8_t*)&CanFlowMsg, sizeof(CanFlowMsg), 8);
#endif
				CAN.beginPacket(IsoTpState.can_tx);
				CAN.write(CanFlowMsg, sizeof(CanFlowMsg));
				CAN.endPacket();
				break;
			case 0x20:                    // continuation frame
				uint8_t frame_idx = frame.data[0] & 0x0f;
				assert((IsoTpState.last_frame_idx + 1) % 0x10 == frame_idx); // misordered frame received ?!?
				size_t payload_len = min(7, (uint16_t)IsoTpState.msg_len - IsoTpState.data_offset);
				if (IsoTpState.data_offset < ISOTP_MAX_MSG_LEN)	// Avoid copying past the target buffer
					memcpy(IsoTpState.data_buf + IsoTpState.data_offset, frame.data + 1,
						 	min(payload_len, ISOTP_MAX_MSG_LEN - IsoTpState.data_offset));
				IsoTpState.data_offset += payload_len;
				IsoTpState.last_frame_idx = frame_idx;
				break;
			case 0x30:                    // flow control frame
				// Flow control frame, cannot handle, since we are asking we should never see this
				assert(false);
				break;
		}

		if (IsoTpState.data_offset == IsoTpState.msg_len) {
			if (IsoTpState.isotp_callback) {	// Thou shalt not follow the NULL pointer!
				IsoTpState.isotp_callback(IsoTpState.data_buf, ISOTP_MAX_MSG_LEN);
				IsoTpState.transaction_running = false;
				CAN.filter(0, 0);	// open receiver for broadcast frames like on Zoe
			}
		}
	}
	else {	// received "free" can frame
		if (IsoTpState.frame_callback)	// Thou shalt not follow the NULL pointer!
			IsoTpState.frame_callback(&frame);
	}
}

int isotp_cmd(uint32_t can_rx, uint32_t can_tx, uint8_t* cmd, size_t cmd_len,
              void (*callback)(uint8_t *, size_t)) {

	if (!IsoTpInitialized) {
		memset(&IsoTpState, 0, sizeof(IsoTpState));
		CAN.onReceive(isotp_receive);
		IsoTpInitialized = true;
	}

	assert(cmd_len < CAN_MAX_FRAME_LEN);

	struct can_frame frame;

	memcpy(frame.data+1, cmd, cmd_len);
	frame.data[0] = cmd_len;
	memset(frame.data + cmd_len + 1, 0, 7 - cmd_len);
	frame.length = 8;

#ifdef DEBUG
	Serial.print("Send frame:     ");
	hexdump(frame.data, frame.length, 8);
#endif

	while(IsoTpState.transaction_running);	// block if another transaction is already running

	CAN.filter(can_rx, 0x7ff);

	IsoTpState.can_tx = can_tx;
	IsoTpState.isotp_callback = callback;
	IsoTpState.transaction_running = true;

	CAN.beginPacket(can_tx);
	CAN.write(frame.data, frame.length);
	CAN.endPacket();
}

int isotp_cmd(uint32_t can_rx, uint32_t can_tx, uint8_t* cmd, size_t cmd_len,
              uint8_t* data_buf, size_t data_buf_len) {
   isotp_cmd(can_rx, can_tx, cmd, cmd_len, NULL);

	while(IsoTpState.transaction_running);	// wait for end of transaction

	memcpy(data_buf, IsoTpState.data_buf, min(data_buf_len, IsoTpState.msg_len));

   IsoTpState.transaction_running = false;
	CAN.filter(0, 0);	// open receiver for broadcast frames like on Zoe
}

void car_decoder(float *evn, struct vehicle_d *vehicle) {
	for (uint8_t cu_idx=0; cu_idx < vehicle->num_cu; cu_idx++) {
		struct control_unit_d *cu = &vehicle->cu[cu_idx];
		for (uint8_t cmd_idx=0; cmd_idx < cu->num_cmd; cmd_idx++) {
			struct cmd_d *cmd = &cu->cmd[cmd_idx];
			uint8_t buffer[cmd->return_bytes];
			isotp_cmd(cu->rx, cu->tx, cmd->cmd, cmd->cmd_len, (uint8_t*)&buffer, cmd->return_bytes);
			for (uint8_t field_idx=0; field_idx < cmd->num_field; field_idx++) {
				struct field_d *field = &cmd->field[field_idx];
				union value_d value;
				value.uint32 = 0;
				for (uint8_t i=0; i < field->length; i++) {
					value.uint8[4-i] = buffer[field->position + i];
				}
				evn[field->field_id] = ((value.uint32 & field->mask) >> field->shift) * field->scale + field->offset;
			}
		}
	}
}

// vim: ts=3 sw=3
