#ifndef SURFACE_UART_UTILS_H_
#define SURFACE_UART_UTILS_H_

#include "main.h"

#define UARTUTILS_LINE_MAX  96U

void UARTUTILS_Init(UART_HandleTypeDef *huart);
void UARTUTILS_Task(void);

uint8_t UARTUTILS_LineAvailable(void);
void UARTUTILS_GetLine(char *dest, uint16_t max_len);

#endif /* SURFACE_UART_UTILS_H_ */
