#include "uart_utils.h"
#include <string.h>

/* Permite conservar varias respuestas aunque lleguen mientras la pantalla
 * esta siendo actualizada. */
#define UARTUTILS_QUEUE_DEPTH  8U

static UART_HandleTypeDef *s_huart = NULL;
static uint8_t  s_rx_byte          = 0U;
static uint16_t s_rx_index         = 0U;
static uint8_t  s_discard_line     = 0U;

static char s_rx_buffer[UARTUTILS_LINE_MAX];
static char s_line_queue[UARTUTILS_QUEUE_DEPTH][UARTUTILS_LINE_MAX];

/* La interrupcion agrega por head y el programa principal retira por tail. */
static volatile uint8_t s_queue_head  = 0U;
static volatile uint8_t s_queue_tail  = 0U;
static volatile uint8_t s_queue_count = 0U;

void UARTUTILS_Init(UART_HandleTypeDef *huart)
{
    s_huart        = huart;
    s_rx_byte      = 0U;
    s_rx_index     = 0U;
    s_discard_line = 0U;
    s_queue_head   = 0U;
    s_queue_tail   = 0U;
    s_queue_count  = 0U;

    memset(s_rx_buffer, 0, sizeof(s_rx_buffer));
    memset(s_line_queue, 0, sizeof(s_line_queue));

    if (s_huart != NULL)
    {
        (void)HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1U);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((huart == NULL) || (s_huart == NULL) ||
        (huart->Instance != s_huart->Instance))
    {
        return;
    }

    /* Se aceptan '\n', '\r' y '\r\n'. En el ultimo caso, el segundo
     * caracter se ignora porque el indice ya fue reiniciado. */
    if ((s_rx_byte == '\n') || (s_rx_byte == '\r'))
    {
        if ((s_rx_index > 0U) && (s_discard_line == 0U))
        {
            s_rx_buffer[s_rx_index] = '\0';

            /* Si la cola se llena, se elimina la linea mas antigua. Los
             * comandos se reintentan y para los sensores interesa conservar
             * siempre la medicion mas reciente. */
            if (s_queue_count >= UARTUTILS_QUEUE_DEPTH)
            {
                s_queue_tail = (uint8_t)((s_queue_tail + 1U) %
                                         UARTUTILS_QUEUE_DEPTH);
                s_queue_count--;
            }

            strncpy(s_line_queue[s_queue_head],
                    s_rx_buffer,
                    UARTUTILS_LINE_MAX - 1U);
            s_line_queue[s_queue_head][UARTUTILS_LINE_MAX - 1U] = '\0';

            s_queue_head = (uint8_t)((s_queue_head + 1U) %
                                     UARTUTILS_QUEUE_DEPTH);
            s_queue_count++;
        }

        s_rx_index     = 0U;
        s_discard_line = 0U;
        s_rx_buffer[0] = '\0';
    }
    else if (s_discard_line == 0U)
    {
        if (s_rx_index < (UARTUTILS_LINE_MAX - 1U))
        {
            s_rx_buffer[s_rx_index++] = (char)s_rx_byte;
        }
        else
        {
            /* Descartar la linea completa si supera el buffer; no conservar
             * solamente su parte final porque podria parecer un comando. */
            s_rx_index     = 0U;
            s_discard_line = 1U;
            s_rx_buffer[0] = '\0';
        }
    }

    (void)HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1U);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if ((huart == NULL) || (s_huart == NULL) ||
        (huart->Instance != s_huart->Instance))
    {
        return;
    }

    /* Conservar las lineas ya completas y descartar solo la que estaba
     * llegando cuando ocurrio el error. */
    s_rx_index     = 0U;
    s_discard_line = 0U;
    s_rx_buffer[0] = '\0';

    /* No usar HAL_UART_AbortReceive_IT seguido inmediatamente de Receive_IT:
     * el aborto es asincrono y puede dejar la nueva recepcion en HAL_BUSY. */
    (void)HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1U);
}

void UARTUTILS_Task(void)
{
    /* La recepcion se realiza por interrupcion. */
}

uint8_t UARTUTILS_LineAvailable(void)
{
    return (s_queue_count > 0U) ? 1U : 0U;
}

void UARTUTILS_GetLine(char *dest, uint16_t max_len)
{
    if ((dest == NULL) || (max_len == 0U))
    {
        return;
    }

    dest[0] = '\0';

    /* El productor es la ISR. La seccion critica evita que la ISR modifique
     * los indices mientras se retira una linea. */
    __disable_irq();

    if (s_queue_count > 0U)
    {
        strncpy(dest, s_line_queue[s_queue_tail], max_len - 1U);
        dest[max_len - 1U] = '\0';

        s_line_queue[s_queue_tail][0] = '\0';
        s_queue_tail = (uint8_t)((s_queue_tail + 1U) %
                                 UARTUTILS_QUEUE_DEPTH);
        s_queue_count--;
    }

    __enable_irq();
}
