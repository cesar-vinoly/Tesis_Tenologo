#include "uart_utils.h"
#include <string.h>

static UART_HandleTypeDef *s_huart = NULL;

static uint8_t s_rx_byte = 0;

static volatile uint16_t s_rx_index = 0;
static volatile uint8_t  s_line_ready = 0;

static char s_rx_buffer[UARTUTILS_LINE_MAX];
static char s_ready_line[UARTUTILS_LINE_MAX];

void UARTUTILS_Init(UART_HandleTypeDef *huart)
{
    s_huart = huart;

    s_rx_index = 0;
    s_line_ready = 0;
    s_rx_byte = 0;

    memset(s_rx_buffer, 0, sizeof(s_rx_buffer));
    memset(s_ready_line, 0, sizeof(s_ready_line));

    HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (s_huart == NULL)
    {
        return;
    }

    if (huart->Instance != s_huart->Instance)
    {
        return;
    }

    /*
      Acepta como fin de línea:
      - '\n'
      - '\r'
      - '\r\n'

      Si llega '\r\n', el '\r' cierra la línea y el '\n' siguiente
      se ignora porque s_rx_index ya está en 0.
    */
    if ((s_rx_byte == '\n') || (s_rx_byte == '\r'))
    {
        if ((s_rx_index > 0U) && (s_line_ready == 0U))
        {
            s_rx_buffer[s_rx_index] = '\0';

            strncpy(s_ready_line, s_rx_buffer, UARTUTILS_LINE_MAX - 1U);
            s_ready_line[UARTUTILS_LINE_MAX - 1U] = '\0';

            s_line_ready = 1U;
        }

        s_rx_index = 0U;
        memset(s_rx_buffer, 0, sizeof(s_rx_buffer));
    }
    else
    {
        if (s_line_ready == 0U)
        {
            if (s_rx_index < (UARTUTILS_LINE_MAX - 1U))
            {
                s_rx_buffer[s_rx_index] = (char)s_rx_byte;
                s_rx_index++;
            }
            else
            {
                /*
                  Si el mensaje es demasiado largo, se corta y se reinicia
                  para evitar desbordes.
                */
                s_rx_index = 0U;
                memset(s_rx_buffer, 0, sizeof(s_rx_buffer));
            }
        }
    }

    HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (s_huart == NULL)
    {
        return;
    }

    if (huart->Instance != s_huart->Instance)
    {
        return;
    }

    /*
      Si hubo error de UART, por ejemplo overrun, ruido o framing error,
      se reinicia la recepción por interrupción.
    */
    HAL_UART_AbortReceive_IT(huart);

    s_rx_index = 0U;
    memset(s_rx_buffer, 0, sizeof(s_rx_buffer));

    HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1);
}

void UARTUTILS_Task(void)
{
    /*
      No se usa porque la recepción se hace por interrupción.
    */
}

uint8_t UARTUTILS_LineAvailable(void)
{
    return s_line_ready;
}

void UARTUTILS_GetLine(char *dest, uint16_t max_len)
{
    if ((dest == NULL) || (max_len == 0U))
    {
        return;
    }

    __disable_irq();

    strncpy(dest, s_ready_line, max_len - 1U);
    dest[max_len - 1U] = '\0';

    s_ready_line[0] = '\0';
    s_line_ready = 0U;

    __enable_irq();
}
