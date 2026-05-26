/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body — Módulo Superficie STM32H743VIT6
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "uart_utils.h"
#include "ili9488.h"
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum
{
    STATE_INICIO = 0,
    STATE_ESTABLECER_CONEXION,
    STATE_ERROR_CONEXION,
    STATE_COMPROBACION_SISTEMA,
    STATE_SISTEMA_OPERANDO,
    STATE_INICIAR_MUESTREO,
    STATE_MUESTREO_COMPLETO,
    STATE_CONSULTA,
    STATE_DESCARGA,
    STATE_ARMADO
} SystemState_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ---- Protocolo UART (strings terminados en \r\n) ----
 *
 *  Superficie ENVÍA        Sumergido RESPONDE
 *  ------------------      --------------------------
 *  PING\r\n           →    ACK\r\n
 *  STATUS\r\n         →    VASTAGO:INIT\r\n  | VASTAGO:FINAL\r\n
 *  CMD:MUESTREO\r\n   →    MUESTREO_OK\r\n  (confirmación inicio)
 *                     →    MUESTREO_DONE\r\n (muestra realizada, asíncrono)
 *  CMD:DESCARGA\r\n   →    DESCARGA_OK\r\n
 *  CMD:ARMAR\r\n      →    ARMADO_OK\r\n
 */
#define CMD_PING              "PING\r\n"
#define CMD_STATUS            "STATUS\r\n"
#define CMD_INICIAR_MUESTREO  "CMD:MUESTREO\r\n"
#define CMD_DESCARGA          "CMD:DESCARGA\r\n"
#define CMD_ARMAR             "CMD:ARMAR\r\n"

#define RSP_ACK               "ACK"
#define RSP_VASTAGO_INIT      "VASTAGO:INIT"
#define RSP_VASTAGO_FINAL     "VASTAGO:FINAL"
#define RSP_MUESTREO_OK       "MUESTREO_OK"
#define RSP_MUESTREO_DONE     "MUESTREO_DONE"
#define RSP_DESCARGA_OK       "DESCARGA_OK"
#define RSP_ARMADO_OK         "ARMADO_OK"

/* ---- Tiempos (ms) ---- */
#define TIMEOUT_CONEXION_MS    20000U  /* Tiempo máx. para establecer conexión */
#define TIMEOUT_RESPUESTA_MS    3000U  /* Tiempo máx. esperando respuesta a cmd */
#define INTERVALO_PING_MS       1000U  /* Período entre reintentos de PING      */
#define TIMEOUT_MUESTREO_MS    60000U  /* Tiempo máx. para completar muestreo   */
#define DEBOUNCE_MS               50U  /* Antirrebote de botones                */

/* ---- GPIO: Botones (entradas — GPIOD) ----
 *   PD8  → Acción principal (muestreo en SISTEMA_OPERANDO / descarga en CONSULTA)
 *   PD9  → Confirmar armado (en CONSULTA)
 *   PD10 → Cancelar (en DESCARGA)
 *   PB15 → Reservado para uso futuro
 */
#define BTN_ACCION_PORT       GPIOD
#define BTN_ACCION_PIN        GPIO_PIN_8
#define BTN_CONFIRMAR_PORT    GPIOD
#define BTN_CONFIRMAR_PIN     GPIO_PIN_9
#define BTN_CANCELAR_PORT     GPIOD
#define BTN_CANCELAR_PIN      GPIO_PIN_10

/* ---- GPIO: Salidas (GPIOE) ----
 *   PE11, PE12, PE13 están reservados por la librería ILI9488 (RST, DC, CS).
 *   NO usar estos pines para otras salidas.
 *   Si se necesitan LEDs indicadores, usar pines libres de otro puerto.
 */

/* ---- ADC3: Medición de batería ----
 *   Canal 10, resolución 16 bits, modo continuo.
 *   Ajustar BATT_FULL_RAW y BATT_EMPTY_RAW según el divisor resistivo real.
 *   Por defecto se asume escala completa del ADC = batería llena.
 */
#define BATT_FULL_RAW         65535U
#define BATT_EMPTY_RAW        20000U   /* ~3.3 V con divisor a ajustar */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc3;

I2C_HandleTypeDef hi2c1;

SD_HandleTypeDef hsd1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */

/* ---- Variables originales ---- */
static char    g_uart_line[UARTUTILS_LINE_MAX];
static uint8_t g_sd_available = 0U;

/* ---- Estado de la máquina ---- */
static SystemState_t g_estado        = STATE_INICIO;   /* Estado activo */
static uint32_t      g_ts_conexion   = 0U;  /* Tick al entrar a ESTABLECER_CONEXION */
static uint32_t      g_ts_ping       = 0U;  /* Tick del último PING enviado         */
static uint32_t      g_ts_respuesta  = 0U;  /* Tick al enviar un comando            */
static uint32_t      g_ts_muestreo   = 0U;  /* Tick al iniciar el muestreo          */
static uint8_t       g_esperando_rsp = 0U;  /* 1 = comando enviado, esperando reply */
static char          g_uart_rx[UARTUTILS_LINE_MAX]; /* Última línea UART recibida   */

/* ---- Datos del sensor (actualizados en tiempo real) ---- */
static char g_sensor_temp[16]  = "--.-";   /* Temperatura parseada, ej: "23.5" */
static char g_sensor_depth[16] = "--.-";   /* Profundidad parseada, ej: "12.3" */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART4_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC3_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

static uint8_t LineHasVisibleChars(const char *s);

/* Máquina de estados */
static void    SM_Run(void);
static void    SM_SetState(SystemState_t nuevo);
static void    SM_DrawScreen(void);

/* Helpers */
static void    SM_SendCmd(const char *cmd);
static uint8_t SM_CheckResponse(const char *expected);
static uint8_t SM_ButtonPressed(GPIO_TypeDef *port, uint16_t pin);
static uint8_t SM_BatteryPercent(void);
static uint8_t SM_ParseSensor(const char *line);
static void    SM_UpdateSensorDisplay(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* --------------------------------------------------------------------------
 * LineHasVisibleChars — función original del proyecto
 * -------------------------------------------------------------------------- */
static uint8_t LineHasVisibleChars(const char *s)
{
    if (s == NULL) return 0U;
    while (*s != '\0')
    {
        if ((*s >= 32) && (*s <= 126)) return 1U;
        s++;
    }
    return 0U;
}

/* --------------------------------------------------------------------------
 * SM_SendCmd
 * Transmite un string de comando por UART4 hacia el módulo sumergido.
 * -------------------------------------------------------------------------- */
static void SM_SendCmd(const char *cmd)
{
    HAL_UART_Transmit(&huart4, (const uint8_t *)cmd, (uint16_t)strlen(cmd), 500U);
}

/* --------------------------------------------------------------------------
 * SM_CheckResponse
 * Devuelve 1 si g_uart_rx contiene la subcadena 'expected'.
 * -------------------------------------------------------------------------- */
static uint8_t SM_CheckResponse(const char *expected)
{
    return (strstr(g_uart_rx, expected) != NULL) ? 1U : 0U;
}

/* --------------------------------------------------------------------------
 * SM_ButtonPressed
 * Lectura de botón con antirrebote simple de DEBOUNCE_MS.
 * Asume lógica directa (activo alto). Adaptar si los botones son activo bajo.
 * -------------------------------------------------------------------------- */
static uint8_t SM_ButtonPressed(GPIO_TypeDef *port, uint16_t pin)
{
    if (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET)
    {
        HAL_Delay(DEBOUNCE_MS);
        return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET) ? 1U : 0U;
    }
    return 0U;
}

/* --------------------------------------------------------------------------
 * SM_BatteryPercent
 * Lee el último valor del ADC3 (modo continuo, ya arrancado) y lo convierte
 * a porcentaje (0–100). Ajustar BATT_FULL_RAW / BATT_EMPTY_RAW según
 * el divisor resistivo del hardware real.
 * -------------------------------------------------------------------------- */
static uint8_t SM_BatteryPercent(void)
{
    uint32_t raw = HAL_ADC_GetValue(&hadc3);

    if (raw >= BATT_FULL_RAW)  return 100U;
    if (raw <= BATT_EMPTY_RAW) return 0U;

    return (uint8_t)(((raw - BATT_EMPTY_RAW) * 100U) /
                     (BATT_FULL_RAW - BATT_EMPTY_RAW));
}

/* --------------------------------------------------------------------------
 * SM_ParseSensor
 * Parsea una línea con formato: T=XX.XXD=XX.XX\r\n
 *   - Temperatura: desde "T=" hasta "D="
 *   - Profundidad: desde "D=" hasta \r, \n o fin de cadena
 * Guarda los valores en g_sensor_temp y g_sensor_depth.
 * Devuelve 1 si la línea era un dato de sensor válido, 0 si no.
 * -------------------------------------------------------------------------- */
static uint8_t SM_ParseSensor(const char *line)
{
    const char *p;
    const char *end;
    uint8_t     i;

    if (line == NULL) return 0U;
    if (strstr(line, "T=") == NULL) return 0U;

    /* ---- Temperatura: desde T= hasta D= ---- */
    p = strstr(line, "T=");
    if (p != NULL)
    {
        p += 2U;                    /* Saltar "T="                  */
        end = strstr(p, "D=");      /* El valor termina donde empieza D= */
        if (end != NULL)
        {
            i = (uint8_t)(end - p);
            if (i >= sizeof(g_sensor_temp)) i = (uint8_t)(sizeof(g_sensor_temp) - 1U);
            strncpy(g_sensor_temp, p, i);
            g_sensor_temp[i] = '\0';
        }
    }

    /* ---- Profundidad: desde D= hasta \r, \n o fin ---- */
    p = strstr(line, "D=");
    if (p != NULL)
    {
        p += 2U;                    /* Saltar "D="                  */
        end = p;
        while (*end != '\0' && *end != '\r' && *end != '\n') end++;
        i = (uint8_t)(end - p);
        if (i >= sizeof(g_sensor_depth)) i = (uint8_t)(sizeof(g_sensor_depth) - 1U);
        strncpy(g_sensor_depth, p, i);
        g_sensor_depth[i] = '\0';
    }

    return 1U;
}

/* --------------------------------------------------------------------------
 * SM_UpdateSensorDisplay
 * Actualiza ÚNICAMENTE el área de datos del sensor en pantalla, sin
 * redibujar el resto. Se llama cada vez que llega un dato nuevo.
 *
 * Layout fijo (solo válido en STATE_SISTEMA_OPERANDO):
 *   Y=140  →  Fila temperatura   "Temp:  XX.X"
 *   Y=185  →  Fila profundidad   "Prof:  XX.X"
 * -------------------------------------------------------------------------- */
static void SM_UpdateSensorDisplay(void)
{
    char buf[32];

    /*
     * Sin FillRect previo — el texto nuevo sobreescribe píxel a píxel
     * el texto anterior gracias al fondo negro del DrawString.
     * Para que el texto nuevo cubra siempre al anterior aunque sea más
     * corto, se rellena con espacios hasta un ancho fijo de 20 chars.
     */
    snprintf(buf, sizeof(buf), "Temp: %-8s C  ", g_sensor_temp);
    ILI9488_DrawString(18, 140, buf, ILI9488_COLOR_YELLOW, ILI9488_COLOR_BLACK, 3);

    snprintf(buf, sizeof(buf), "Prof: %-8s m  ", g_sensor_depth);
    ILI9488_DrawString(18, 185, buf, ILI9488_COLOR_CYAN,   ILI9488_COLOR_BLACK, 3);
}

/* --------------------------------------------------------------------------
 * SM_DrawScreen
 * Redibuja la pantalla ILI9488 completa según el estado activo.
 * Se llama ÚNICAMENTE desde SM_SetState, es decir, solo cuando hay un
 * cambio de estado. Así se evita redibujar en cada iteración del loop.
 * -------------------------------------------------------------------------- */
static void SM_DrawScreen(void)
{
    char buf[32];
    uint8_t batt = SM_BatteryPercent();

    ILI9488_FillScreen(ILI9488_COLOR_BLACK);

    /* ---- Barra de estado superior: nivel de batería ---- */
    snprintf(buf, sizeof(buf), "Bat: %d%%", (int)batt);
    ILI9488_DrawString(ILI9488_WIDTH - 130, 6,
                       buf, ILI9488_COLOR_CYAN, ILI9488_COLOR_BLACK, 2);

    /* ---- Contenido según estado ---- */
    switch (g_estado)
    {
    /* ------------------------------------------------------------------ */
    case STATE_INICIO:
        ILI9488_DrawString(18, 60,
            "Iniciando...",
            ILI9488_COLOR_WHITE, ILI9488_COLOR_BLACK, 3);
        break;

    /* ------------------------------------------------------------------ */
    case STATE_ESTABLECER_CONEXION:
        ILI9488_DrawString(18, 40,
            "Conectando",
            ILI9488_COLOR_YELLOW, ILI9488_COLOR_BLACK, 3);
        ILI9488_DrawTextWrapped(18, 100, ILI9488_WIDTH - 36,
            "Buscando modulo sumergido...",
            ILI9488_COLOR_WHITE, ILI9488_COLOR_BLACK, 2);
        ILI9488_DrawTextWrapped(18, 200, ILI9488_WIDTH - 36,
            "Reintentando cada 1 s (max 20 s)",
            ILI9488_COLOR_CYAN, ILI9488_COLOR_BLACK, 2);
        break;

    /* ------------------------------------------------------------------ */
    case STATE_ERROR_CONEXION:
        ILI9488_FillScreen(ILI9488_COLOR_RED);
        ILI9488_DrawString(18, 40,
            "ERROR",
            ILI9488_COLOR_WHITE, ILI9488_COLOR_RED, 4);
        ILI9488_DrawTextWrapped(18, 120, ILI9488_WIDTH - 36,
            "Sin conexion con modulo sumergido.",
            ILI9488_COLOR_WHITE, ILI9488_COLOR_RED, 2);
        ILI9488_DrawTextWrapped(18, 200, ILI9488_WIDTH - 36,
            "Resetee el equipo para reintentar.",
            ILI9488_COLOR_YELLOW, ILI9488_COLOR_RED, 2);
        break;

    /* ------------------------------------------------------------------ */
    case STATE_COMPROBACION_SISTEMA:
        ILI9488_DrawString(18, 40,
            "Verificando",
            ILI9488_COLOR_YELLOW, ILI9488_COLOR_BLACK, 3);
        ILI9488_DrawTextWrapped(18, 100, ILI9488_WIDTH - 36,
            "Consultando posicion del vastago...",
            ILI9488_COLOR_WHITE, ILI9488_COLOR_BLACK, 2);
        break;

    /* ------------------------------------------------------------------ */
    case STATE_SISTEMA_OPERANDO:
        ILI9488_DrawString(18, 40,
            "Sistema OK",
            ILI9488_COLOR_GREEN, ILI9488_COLOR_BLACK, 3);
        ILI9488_DrawTextWrapped(18, 100, ILI9488_WIDTH - 36,
            "Vastago en posicion inicial.",
            ILI9488_COLOR_WHITE, ILI9488_COLOR_BLACK, 2);
        /* Área Y=130..230 reservada para datos del sensor (SM_UpdateSensorDisplay) */
        SM_UpdateSensorDisplay();
        ILI9488_DrawTextWrapped(18, 245, ILI9488_WIDTH - 36,
            "[PD8] Iniciar muestreo",
            ILI9488_COLOR_CYAN, ILI9488_COLOR_BLACK, 2);
        break;

    /* ------------------------------------------------------------------ */
    case STATE_INICIAR_MUESTREO:
        ILI9488_DrawString(18, 40,
            "Muestreo",
            ILI9488_COLOR_YELLOW, ILI9488_COLOR_BLACK, 3);
        ILI9488_DrawTextWrapped(18, 100, ILI9488_WIDTH - 36,
            "Muestra en proceso...",
            ILI9488_COLOR_WHITE, ILI9488_COLOR_BLACK, 2);
        ILI9488_DrawTextWrapped(18, 160, ILI9488_WIDTH - 36,
            "Esperando confirmacion del sumergido.",
            ILI9488_COLOR_CYAN, ILI9488_COLOR_BLACK, 2);
        break;

    /* ------------------------------------------------------------------ */
    case STATE_MUESTREO_COMPLETO:
        ILI9488_DrawString(18, 40,
            "Completado",
            ILI9488_COLOR_GREEN, ILI9488_COLOR_BLACK, 3);
        ILI9488_DrawTextWrapped(18, 100, ILI9488_WIDTH - 36,
            "Muestra realizada correctamente.",
            ILI9488_COLOR_WHITE, ILI9488_COLOR_BLACK, 2);
        break;

    /* ------------------------------------------------------------------ */
    case STATE_CONSULTA:
        ILI9488_DrawString(18, 40,
            "Consulta",
            ILI9488_COLOR_YELLOW, ILI9488_COLOR_BLACK, 3);
        ILI9488_DrawTextWrapped(18, 100, ILI9488_WIDTH - 36,
            "Vastago en posicion final.",
            ILI9488_COLOR_WHITE, ILI9488_COLOR_BLACK, 2);
        ILI9488_DrawTextWrapped(18, 160, ILI9488_WIDTH - 36,
            "[PD8]  Confirmar descarga",
            ILI9488_COLOR_CYAN, ILI9488_COLOR_BLACK, 2);
        ILI9488_DrawTextWrapped(18, 210, ILI9488_WIDTH - 36,
            "[PD9]  Confirmar armado",
            ILI9488_COLOR_CYAN, ILI9488_COLOR_BLACK, 2);
        break;

    /* ------------------------------------------------------------------ */
    case STATE_DESCARGA:
        ILI9488_DrawString(18, 40,
            "Descarga",
            ILI9488_COLOR_YELLOW, ILI9488_COLOR_BLACK, 3);
        ILI9488_DrawTextWrapped(18, 100, ILI9488_WIDTH - 36,
            "Descarga y limpieza en curso...",
            ILI9488_COLOR_WHITE, ILI9488_COLOR_BLACK, 2);
        ILI9488_DrawTextWrapped(18, 210, ILI9488_WIDTH - 36,
            "[PD10] Cancelar descarga",
            ILI9488_COLOR_YELLOW, ILI9488_COLOR_BLACK, 2);
        break;

    /* ------------------------------------------------------------------ */
    case STATE_ARMADO:
        ILI9488_DrawString(18, 40,
            "Armando",
            ILI9488_COLOR_YELLOW, ILI9488_COLOR_BLACK, 3);
        ILI9488_DrawTextWrapped(18, 100, ILI9488_WIDTH - 36,
            "Preparando sistema para siguiente toma...",
            ILI9488_COLOR_WHITE, ILI9488_COLOR_BLACK, 2);
        break;

    default:
        break;
    }
}

/* --------------------------------------------------------------------------
 * SM_SetState
 * Cambia el estado activo, resetea flags compartidos y redibuja la pantalla.
 * Es el único punto de transición entre estados.
 * -------------------------------------------------------------------------- */
static void SM_SetState(SystemState_t nuevo)
{
    g_estado        = nuevo;
    g_esperando_rsp = 0U;
    g_uart_rx[0]    = '\0';

    /* Pantalla */
    SM_DrawScreen();
}

/* --------------------------------------------------------------------------
 * SM_Run
 * Ejecuta un tick de la máquina de estados. Debe llamarse en cada
 * iteración del while(1), DESPUÉS de UARTUTILS_Task().
 * -------------------------------------------------------------------------- */
static void SM_Run(void)
{
    uint32_t ahora    = HAL_GetTick();
    uint8_t  hay_linea = 0U;

    /* ---- Capturar línea UART si está disponible ---- */
    if (UARTUTILS_LineAvailable())
    {
        UARTUTILS_GetLine(g_uart_line, sizeof(g_uart_line));
        if (LineHasVisibleChars(g_uart_line))
        {
            /* ¿Es un dato del sensor? Parsearlo y actualizar pantalla */
            if (SM_ParseSensor(g_uart_line))
            {
                if (g_estado == STATE_SISTEMA_OPERANDO)
                {
                    SM_UpdateSensorDisplay();
                }
                /* No es un comando de protocolo: no activar hay_linea */
            }
            else
            {
                strncpy(g_uart_rx, g_uart_line, sizeof(g_uart_rx) - 1U);
                g_uart_rx[sizeof(g_uart_rx) - 1U] = '\0';
                hay_linea = 1U;
            }
        }
    }

    /* ---- Lógica de cada estado ---- */
    switch (g_estado)
    {

    /* ================================================================== */
    case STATE_INICIO:
    /*
     * Inicializa timestamps y transiciona inmediatamente.
     * La pantalla "Iniciando..." se muestra sólo un instante.
     */
        g_ts_conexion   = ahora;
        g_ts_ping       = 0U;
        g_esperando_rsp = 0U;
        g_uart_rx[0]    = '\0';
        SM_SetState(STATE_ESTABLECER_CONEXION);
        break;

    /* ================================================================== */
    case STATE_ESTABLECER_CONEXION:
    /*
     * Envía PING cada INTERVALO_PING_MS.
     * Si recibe ACK → COMPROBACION_SISTEMA.
     * Si pasan TIMEOUT_CONEXION_MS sin ACK → ERROR_CONEXION.
     */
        /* ¿Tiempo agotado? */
        if ((ahora - g_ts_conexion) >= TIMEOUT_CONEXION_MS)
        {
            SM_SetState(STATE_ERROR_CONEXION);
            break;
        }

        /* ¿Llegó ACK? */
        if (hay_linea && SM_CheckResponse(RSP_ACK))
        {
            SM_SetState(STATE_COMPROBACION_SISTEMA);
            break;
        }

        /* ¿Hay que enviar (o reenviar) PING? */
        if (!g_esperando_rsp || (ahora - g_ts_ping) >= INTERVALO_PING_MS)
        {
            SM_SendCmd(CMD_PING);
            g_ts_ping       = ahora;
            g_esperando_rsp = 1U;
        }
        break;

    /* ================================================================== */
    case STATE_ERROR_CONEXION:
    /*
     * Estado terminal. No hace nada; la pantalla ya muestra el error
     * y el LED_ERROR está encendido. El operador debe resetear el equipo.
     */
        break;

    /* ================================================================== */
    case STATE_COMPROBACION_SISTEMA:
    /*
     * Solicita STATUS al sumergido.
     * VASTAGO:INIT  → SISTEMA_OPERANDO
     * VASTAGO:FINAL → CONSULTA
     * Sin respuesta en TIMEOUT_RESPUESTA_MS → reenvía STATUS.
     */
        if (!g_esperando_rsp)
        {
            SM_SendCmd(CMD_STATUS);
            g_ts_respuesta  = ahora;
            g_esperando_rsp = 1U;
            break;
        }

        if ((ahora - g_ts_respuesta) >= TIMEOUT_RESPUESTA_MS)
        {
            g_esperando_rsp = 0U;   /* Forzar reenvío en próxima iteración */
            break;
        }

        if (hay_linea)
        {
            if (SM_CheckResponse(RSP_VASTAGO_INIT))
            {
                SM_SetState(STATE_SISTEMA_OPERANDO);
            }
            else if (SM_CheckResponse(RSP_VASTAGO_FINAL))
            {
                SM_SetState(STATE_CONSULTA);
            }
            else
            {
                /* Respuesta inesperada: descartar y reenviar */
                g_uart_rx[0]    = '\0';
                g_esperando_rsp = 0U;
            }
        }
        break;

    /* ================================================================== */
    case STATE_SISTEMA_OPERANDO:
    /*
     * El vástago está en posición inicial.
     * El operador pulsa PD8 para iniciar el muestreo.
     */
        if (SM_ButtonPressed(BTN_ACCION_PORT, BTN_ACCION_PIN))
        {
            SM_SetState(STATE_INICIAR_MUESTREO);
        }
        break;

    /* ================================================================== */
    case STATE_INICIAR_MUESTREO:
    /*
     * Envía CMD:MUESTREO al sumergido.
     * Espera MUESTREO_DONE (puede llegar después de MUESTREO_OK).
     * Si pasan TIMEOUT_MUESTREO_MS sin DONE → vuelve a COMPROBACION.
     */
        if (!g_esperando_rsp)
        {
            SM_SendCmd(CMD_INICIAR_MUESTREO);
            g_ts_respuesta  = ahora;
            g_ts_muestreo   = ahora;
            g_esperando_rsp = 1U;
            break;
        }

        /* Timeout global del proceso de muestreo */
        if ((ahora - g_ts_muestreo) >= TIMEOUT_MUESTREO_MS)
        {
            SM_SetState(STATE_COMPROBACION_SISTEMA);
            break;
        }

        if (hay_linea)
        {
            if (SM_CheckResponse(RSP_MUESTREO_DONE))
            {
                SM_SetState(STATE_MUESTREO_COMPLETO);
            }
            else
            {
                /* MUESTREO_OK u otro: seguir esperando DONE */
                g_uart_rx[0] = '\0';
            }
        }
        break;

    /* ================================================================== */
    case STATE_MUESTREO_COMPLETO:
    /*
     * Muestra el resultado brevemente y regresa a COMPROBACION.
     */
        HAL_Delay(1500U);
        SM_SetState(STATE_COMPROBACION_SISTEMA);
        break;

    /* ================================================================== */
    case STATE_CONSULTA:
    /*
     * El vástago está en posición final. El operador elige:
     *   PD8  → descarga
     *   PD9  → armado directo
     */
        if (SM_ButtonPressed(BTN_ACCION_PORT, BTN_ACCION_PIN))
        {
            SM_SetState(STATE_DESCARGA);
        }
        else if (SM_ButtonPressed(BTN_CONFIRMAR_PORT, BTN_CONFIRMAR_PIN))
        {
            SM_SetState(STATE_ARMADO);
        }
        break;

    /* ================================================================== */
    case STATE_DESCARGA:
    /*
     * Envía CMD:DESCARGA. Cuando recibe DESCARGA_OK → ARMADO.
     * PD10 cancela la descarga y vuelve a CONSULTA.
     */
        if (!g_esperando_rsp)
        {
            SM_SendCmd(CMD_DESCARGA);
            g_ts_respuesta  = ahora;
            g_esperando_rsp = 1U;
            break;
        }

        if ((ahora - g_ts_respuesta) >= TIMEOUT_RESPUESTA_MS)
        {
            g_esperando_rsp = 0U;   /* Reenviar */
            break;
        }

        if (hay_linea && SM_CheckResponse(RSP_DESCARGA_OK))
        {
            SM_SetState(STATE_ARMADO);
            break;
        }

        /* Cancelación manual */
        if (SM_ButtonPressed(BTN_CANCELAR_PORT, BTN_CANCELAR_PIN))
        {
            SM_SetState(STATE_CONSULTA);
        }
        break;

    /* ================================================================== */
    case STATE_ARMADO:
    /*
     * Envía CMD:ARMAR. Cuando recibe ARMADO_OK → COMPROBACION_SISTEMA.
     */
        if (!g_esperando_rsp)
        {
            SM_SendCmd(CMD_ARMAR);
            g_ts_respuesta  = ahora;
            g_esperando_rsp = 1U;
            break;
        }

        if ((ahora - g_ts_respuesta) >= TIMEOUT_RESPUESTA_MS)
        {
            g_esperando_rsp = 0U;   /* Reenviar */
            break;
        }

        if (hay_linea && SM_CheckResponse(RSP_ARMADO_OK))
        {
            SM_SetState(STATE_COMPROBACION_SISTEMA);
        }
        break;

    /* ================================================================== */
    default:
        SM_SetState(STATE_INICIO);
        break;
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_UART4_Init();
  MX_I2C1_Init();
  MX_ADC3_Init();
  MX_SDMMC1_SD_Init();
  g_sd_available = (HAL_SD_GetState(&hsd1) == HAL_SD_STATE_READY) ? 1U : 0U;
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  /* Pantalla */
  ILI9488_Init(&hspi1);

  /* UART hacia módulo sumergido */
  UARTUTILS_Init(&huart4);

  /* ADC batería: arrancar en modo continuo (configurado en MX_ADC3_Init) */
  HAL_ADC_Start(&hadc3);

  /* Pantalla inicial antes de entrar al loop */
  SM_DrawScreen();   /* Muestra STATE_INICIO ("Iniciando...") */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* Procesar bytes UART entrantes (debe llamarse en cada iteración) */
    UARTUTILS_Task();

    /* Ejecutar un tick de la máquina de estados */
    SM_Run();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.Resolution = ADC_RESOLUTION_16B;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc3.Init.OversamplingMode = DISABLE;
  hadc3.Init.Oversampling.Ratio = 1;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x307075B1;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{
    hsd1.Instance                 = SDMMC1;
    hsd1.Init.ClockEdge           = SDMMC_CLOCK_EDGE_RISING;
    hsd1.Init.ClockPowerSave      = SDMMC_CLOCK_POWER_SAVE_DISABLE;
    hsd1.Init.BusWide             = SDMMC_BUS_WIDE_4B;
    hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
    hsd1.Init.ClockDiv            = 0;

    /* No llamar Error_Handler: si no hay SD simplemente continúa */
    if (HAL_SD_Init(&hsd1) != HAL_OK)
    {
        return;  /* SD no disponible, continuar sin ella */
    }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE11 PE12 PE13 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 PD10 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
