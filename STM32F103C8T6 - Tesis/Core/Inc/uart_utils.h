#ifndef UART_UTILS_H
#define UART_UTILS_H

#include "main.h"

HAL_StatusTypeDef UART_SendBuffer(UART_HandleTypeDef *huart, const uint8_t *data, uint16_t len);
HAL_StatusTypeDef UART_SendString(UART_HandleTypeDef *huart, const char *str);
HAL_StatusTypeDef UART_SendLine(UART_HandleTypeDef *huart, const char *str);

#endif /* UART_UTILS_H */
