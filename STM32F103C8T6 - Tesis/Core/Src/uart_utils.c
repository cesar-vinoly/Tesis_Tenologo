#include "uart_utils.h"
#include <string.h>

HAL_StatusTypeDef UART_SendBuffer(UART_HandleTypeDef *huart, const uint8_t *data, uint16_t len)
{
    if (huart == NULL || data == NULL || len == 0)
    {
        return HAL_ERROR;
    }

    return HAL_UART_Transmit(huart, (uint8_t *)data, len, HAL_MAX_DELAY);
}

HAL_StatusTypeDef UART_SendString(UART_HandleTypeDef *huart, const char *str)
{
    uint16_t len;

    if (huart == NULL || str == NULL)
    {
        return HAL_ERROR;
    }

    len = (uint16_t)strlen(str);
    return HAL_UART_Transmit(huart, (uint8_t *)str, len, HAL_MAX_DELAY);
}

HAL_StatusTypeDef UART_SendLine(UART_HandleTypeDef *huart, const char *str)
{
    HAL_StatusTypeDef status;
    uint16_t len;

    if (huart == NULL || str == NULL)
    {
        return HAL_ERROR;
    }

    len = (uint16_t)strlen(str);

    status = HAL_UART_Transmit(huart, (uint8_t *)str, len, HAL_MAX_DELAY);
    if (status != HAL_OK)
    {
        return status;
    }

    return HAL_UART_Transmit(huart, (uint8_t *)"\r\n", 2, HAL_MAX_DELAY);
}
