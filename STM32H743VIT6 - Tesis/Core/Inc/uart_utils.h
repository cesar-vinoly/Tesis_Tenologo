#ifndef UART_UTILS_H
#define UART_UTILS_H

#include "main.h"

#define UARTUTILS_LINE_MAX  96

void UARTUTILS_Init(UART_HandleTypeDef *huart);
void UARTUTILS_Task(void);

uint8_t UARTUTILS_LineAvailable(void);
void UARTUTILS_GetLine(char *dest, uint16_t max_len);

#endif
