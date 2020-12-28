//#include <Arduino.h>
#include <inttypes.h>
#include <stdlib.h>

const unsigned short CAN_PIN_RX = 4;
const unsigned short CAN_PIN_TX = 5;
const unsigned long CAN_BUS_SPEED = 500E3;

const size_t CAN_MAX_FRAME_LEN = 8;
const int CAN_DEFAULT_READ_TIMEOUT = 500;

const size_t ISOTP_MAX_MSG_LEN = 128;	// twice as much as the biggest structure currently known

int isotp_cmd(uint32_t can_rx, uint32_t can_tx, uint8_t* cmd, size_t cmd_len,
              uint8_t* data_buf, size_t data_len);

