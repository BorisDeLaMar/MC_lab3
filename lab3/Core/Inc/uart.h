#ifndef INC_UART_H
#define INC_UART_H

#include "main.h"

#define MAX_PACKET_LENGTH 512
#define START_BIT 0x7A
#define STOP_BIT 0x79
#define NUM_ASCII_OFFSET 48
#define PACKET_LENGTH 7

enum request {
	Ping = NUM_ASCII_OFFSET + 1,
	Freq = NUM_ASCII_OFFSET + 2,
	Amp = NUM_ASCII_OFFSET + 3
};

struct header {
	uint8_t request_id;
	uint8_t data_size;
	uint8_t data[512];
};

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

#endif /* INC_UART_H */
