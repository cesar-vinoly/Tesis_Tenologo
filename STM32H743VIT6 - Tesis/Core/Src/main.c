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
#include "fatfs.h"
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

typedef enum
{
    MENU_PRINCIPAL = 0,
    MENU_VACIAR,
    MENU_LLENAR,
    MENU_VALVULA,
    MENU_TOTAL
} MenuOption_t;

typedef enum
{
    SD_LOG_PENDIENTE = 0,
    SD_LOG_GUARDADO,
    SD_LOG_NO_DISPONIBLE,
    SD_LOG_ERROR
} SdLogStatus_t;

/* Texto ya dibujado en pantalla. Permite actualizar únicamente los
 * caracteres que cambian, sin borrar previamente todo el renglón. */
typedef struct
{
    char     text[24];
    uint16_t color;
    uint8_t  valid;
} DisplayTextCache_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ---- Protocolo UART (strings terminados en \r\n) ----
 *
 *  Superficie ENVÍA          Sumergido RESPONDE
 *  ------------------        --------------------------
 *  PING\r\n             →    ACK\r\n
 *  STATUS\r\n           →    VASTAGO:INIT\r\n  | VASTAGO:FINAL\r\n
 *  SENSOR\r\n           →    T=XX.XXD=XX.XX\r\n
 *  CMD:MUESTREO\r\n     →    MUESTREO_OK\r\n  (confirmación inicio)
 *                       →    MUESTREO_DONE\r\n (muestra realizada, asíncrono)
 *  CMD:ARMAR\r\n        →    ARMADO_OK\r\n
 *
 *  ---- Modo DESCARGA (jog manual del vástago) ----
 *  CMD:DESCARGA\r\n     →    MODO:DESCARGA\r\n      (entra a modo jog)
 *  CMD:AVANZAR\r\n      →    JOG:AVANZANDO\r\n      | LIMITE:INICIAL\r\n
 *  CMD:RETROCEDER\r\n   →    JOG:RETROCEDIENDO\r\n  | LIMITE:FINAL\r\n
 *  CMD:DETENER\r\n      →    JOG:DETENIDO\r\n
 *  CMD:VALVULA_ABRIR\r\n →    VALVULA:ABIERTA\r\n
 *  CMD:VALVULA_CERRAR\r\n→    JOG:DETENIDO\r\n + VALVULA:CERRADA\r\n
 *  (LIMITE:INICIAL / LIMITE:FINAL también llegan de forma asíncrona
 *   si el fin de carrera se activa mientras el motor está en marcha)
 *  CMD:ARMAR\r\n        →    ARMADO_OK\r\n          (salto directo, ya confirmado en pantalla)
 */
#define CMD_PING              "PING\r\n"
#define CMD_STATUS            "STATUS\r\n"
#define CMD_SENSOR            "SENSOR\r\n"
#define CMD_INICIAR_MUESTREO  "CMD:MUESTREO\r\n"
#define CMD_DESCARGA          "CMD:DESCARGA\r\n"
#define CMD_AVANZAR           "CMD:AVANZAR\r\n"
#define CMD_RETROCEDER        "CMD:RETROCEDER\r\n"
#define CMD_DETENER           "CMD:DETENER\r\n"
#define CMD_VALVULA_ABRIR     "CMD:VALVULA_ABRIR\r\n"
#define CMD_VALVULA_CERRAR    "CMD:VALVULA_CERRAR\r\n"
#define CMD_ARMAR             "CMD:ARMAR\r\n"

#define RSP_ACK               "ACK"
#define RSP_VASTAGO_INIT      "VASTAGO:INIT"
#define RSP_VASTAGO_FINAL     "VASTAGO:FINAL"
#define RSP_VASTAGO_INTERMEDIO "VASTAGO:INTERMEDIO"
#define RSP_MUESTREO_OK       "MUESTREO_OK"
#define RSP_MUESTREO_DONE     "MUESTREO_DONE"
#define RSP_MODO_DESCARGA     "MODO:DESCARGA"
#define RSP_JOG_AVANZANDO     "JOG:AVANZANDO"
#define RSP_JOG_RETROCEDIENDO "JOG:RETROCEDIENDO"
#define RSP_JOG_DETENIDO      "JOG:DETENIDO"
#define RSP_VALVULA_ABIERTA   "VALVULA:ABIERTA"
#define RSP_VALVULA_CERRADA   "VALVULA:CERRADA"
#define RSP_LIMITE_INICIAL    "LIMITE:INICIAL"
#define RSP_LIMITE_FINAL      "LIMITE:FINAL"
#define RSP_JOG_TIMEOUT       "JOG_ERROR:TIMEOUT"
#define RSP_ARMADO_OK         "ARMADO_OK"
#define RSP_ARMADO_TIMEOUT    "ARMADO_ERROR:TIMEOUT"

/* ---- Tiempos (ms) ---- */
#define TIMEOUT_CONEXION_MS    20000U  /* Tiempo máx. para establecer conexión */
#define TIMEOUT_RESPUESTA_MS    3000U  /* Tiempo máx. esperando respuesta a cmd */
#define INTERVALO_PING_MS       1000U  /* Período entre reintentos de PING      */
#define INTERVALO_SENSOR_MS      500U  /* Consulta periodica de T=...D=...      */
#define DASH_CLOCK_UPDATE_MS   10000U  /* El encabezado muestra solo HH:MM      */
#define TIMEOUT_MUESTREO_MS    60000U  /* Tiempo máx. para completar muestreo   */
#define TIMEOUT_ARMADO_MS      45000U  /* Espera el timeout de 35 s del sumergido */
#define DEBOUNCE_MS               50U  /* Antirrebote de botones                */

/* ---- GPIO: Botones del menu ----
 *   Btn 1 (PD8)  -> ejecutar la opcion seleccionada.
 *                   En Vaciar/Llenar: mantener presionado para mover.
 *   Btn 2 (PD9)  -> desplazar la seleccion hacia la izquierda.
 *   Btn 3 (PB15) -> desplazar la seleccion hacia la derecha.
 *   Btn 4 (PD10) -> reservado para una funcion futura.
 */
#define BTN_SELECT_PORT       GPIOD
#define BTN_SELECT_PIN        GPIO_PIN_8
#define BTN_LEFT_PORT         GPIOD
#define BTN_LEFT_PIN          GPIO_PIN_9
#define BTN_RIGHT_PORT        GPIOB
#define BTN_RIGHT_PIN         GPIO_PIN_15
#define BTN_4_PORT            GPIOD
#define BTN_4_PIN             GPIO_PIN_10

/* ---- GPIO: Salidas (GPIOE) ----
 *   PE11, PE12, PE13 están reservados por la librería ILI9488 (RST, DC, CS).
 *   NO usar estos pines para otras salidas.
 *   Si se necesitan LEDs indicadores, usar pines libres de otro puerto.
 */

/* ---- ADC3: Medición de batería ----
 *   Rank 1: canal 10 / PC0, tensión entregada por el circuito de batería.
 *   Rank 2: VREFINT, usado para calcular VREF+ real en cada medición.
 *
 *   El ADC entrega una relación respecto de VREF+. Medir VREFINT junto con
 *   PC0 evita que una variación de la alimentación analógica se interprete
 *   erróneamente como una variación de carga de la batería.
 */
#define ADC_MAX_RAW            65535UL  /* ADC de 16 bits */
#define BATT_FULL_MV            2990UL
#define BATT_EMPTY_MV           2770UL

/* Punto de calibración medido previamente sobre este equipo. Se conserva la
 * corrección de ganancia y VREFINT se usa además para compensar variaciones
 * de VREF+ respecto de los 3300 mV presentes en ese punto de calibración. */
#define BATT_CAL_RAW           53672UL
#define BATT_CAL_VREF_MV        3300UL

/* La batería cambia lentamente, por lo que se prioriza estabilidad sobre
 * velocidad: se promedian varias conversiones válidas. */
#define BATT_ADC_SAMPLES        64U
#define BATT_ADC_TIMEOUT_MS     10U
#define BATT_PERCENT_INVALID   255U
#define BATT_UPDATE_INTERVAL_MS 30000U
#define BATT_FILTER_DIVISOR       8UL
#define BATT_PERCENT_HYSTERESIS   3U
#define BATT_FULL_ZONE_PERCENT   98U
#define BATT_FULL_RELEASE_PERCENT 92U

/* Diagnóstico temporal: muestra las cuentas y los milivoltios calculados
 * por el ADC en la esquina superior izquierda. Cambiar a 0U al finalizar. */
#define BATT_ADC_DEBUG_SCREEN     0U

/* ---- Posiciones horizontales de los valores del tablero ---- */
#define DASH_VALUE_X             250U
#define SAMPLE_LABEL_X           115U
#define SAMPLE_VALUE_X           280U

/* ---- RTC DS3231 por I2C1 (registros en BCD) ----
 *   0x00 seg  0x01 min  0x02 hora  0x03 dia-semana
 *   0x04 dia  0x05 mes  0x06 anio (00-99, se asume 20xx)
 */
#define DS3231_I2C_ADDRESS     (0x68U << 1)  /* HAL usa direccion desplazada */
#define DS3231_REG_SECONDS     0x00U
#define DS3231_I2C_TIMEOUT_MS  100U

/* ---- Log de muestreos en microSD (requiere middleware FatFs sobre
 *      SDMMC1 habilitado en CubeMX: Middleware > FATFS > SD, con
 *      "fatfs.h" y las funciones f_mount/f_open/f_printf/f_close) ---- */
#define MUESTREO_LOG_FILENAME  "MUESTREOS.CSV"

/* Poner en 1U cuando la microSD esté instalada.
 * En 0U no se inicializa SDMMC ni se ejecutan operaciones FatFs, por lo que
 * el equipo continúa funcionando normalmente aunque no haya tarjeta. */
#define MICROSD_HABILITADA      0U

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
static SdLogStatus_t g_sd_log_status = SD_LOG_PENDIENTE;
static uint8_t g_adc_bateria_ready = 0U;
static uint8_t g_batt_last_percent = BATT_PERCENT_INVALID;
static volatile uint32_t g_batt_last_raw = 0U;
static volatile uint32_t g_batt_last_mv  = 0U;
static volatile uint32_t g_adc_vref_mv   = 0U;
static uint32_t g_batt_filtered_mv = 0U;
static uint32_t g_batt_last_measure_tick = 0U;
static uint8_t g_batt_measurement_valid = 0U;

/* ---- Estado de la máquina ---- */
static SystemState_t g_estado        = STATE_INICIO;   /* Estado activo */
static uint32_t      g_ts_conexion   = 0U;  /* Tick al entrar a ESTABLECER_CONEXION */
static uint32_t      g_ts_ping       = 0U;  /* Tick del último PING enviado         */
static uint32_t      g_ts_respuesta  = 0U;  /* Tick al enviar un comando            */
static uint32_t      g_ts_muestreo   = 0U;  /* Tick al iniciar el muestreo          */
static uint8_t       g_esperando_rsp = 0U;  /* 1 = comando enviado, esperando reply */
static char          g_uart_rx[UARTUTILS_LINE_MAX]; /* Última línea UART recibida   */
static uint8_t       g_screen_initialized = 0U;
static SystemState_t g_screen_last_state  = STATE_INICIO;
static uint8_t       g_screen_last_sample_layout = 0U;

/* ---- Datos del sensor (actualizados en tiempo real) ---- */
static char g_sensor_temp[16]  = "--.-";   /* Temperatura parseada, ej: "23.5" */
static char g_sensor_depth[16] = "--.-";   /* Profundidad parseada, ej: "12.3" */
static char g_sample_temp[16]  = "--.-";   /* Valor congelado al terminar muestra */
static char g_sample_depth[16] = "--.-";
static uint32_t g_ts_periodic_request    = 0U;
static uint8_t  g_periodic_status_next   = 1U;

/* ---- Menu inferior y modo manual ---- */
#define JOG_NONE            0U
#define JOG_AVANZANDO       1U
#define JOG_RETROCEDIENDO   2U
#define MENU_PENDING_NONE   0xFFU
static MenuOption_t g_menu_selected       = MENU_PRINCIPAL;
static uint8_t      g_menu_post_sample    = 0U;
static uint8_t      g_sample_available    = 0U;
static uint8_t      g_pending_manual_menu = MENU_PENDING_NONE;
static uint8_t      g_descarga_listo      = 0U;
static uint8_t      g_jog_actual          = JOG_NONE;
static uint8_t      g_jog_bloqueado       = 0U;
static uint8_t      g_valvula_abierta     = 0U;
static uint8_t      g_btn_select_prev     = 0U;
static uint8_t      g_btn_left_prev       = 0U;
static uint8_t      g_btn_right_prev      = 0U;
static uint32_t     g_ts_dashboard_clock  = 0U;

/* ---- Fecha/hora del último muestreo (RTC DS3231), para log y pantalla ---- */
static char g_ultimo_muestreo_ts[24] = "";
static char g_sample_date[11] = "--/--/----";
static char g_sample_time[6]  = "--:--";

/* ---- Caché de renderizado para evitar parpadeos ---- */
static DisplayTextCache_t g_draw_depth;
static DisplayTextCache_t g_draw_temp;
static DisplayTextCache_t g_draw_motor;
static DisplayTextCache_t g_draw_valve;
static DisplayTextCache_t g_draw_comm;
static DisplayTextCache_t g_draw_clock;
static DisplayTextCache_t g_draw_battery;
static DisplayTextCache_t g_draw_menu[MENU_TOTAL];
static uint8_t g_draw_sensor_sample_layout = 0xFFU;

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
static uint8_t SM_ProcessPositionResponse(void);
static void    SM_ServiceDashboardQueries(void);
static uint8_t SM_ButtonPressed(GPIO_TypeDef *port, uint16_t pin);
static uint8_t SM_ButtonEdge(GPIO_TypeDef *port, uint16_t pin, uint8_t *prev);
static uint8_t SM_BatteryPercent(void);
static uint8_t SM_ParseSensor(const char *line);
static void    SM_DrawCachedText(uint16_t x, uint16_t y, const char *text,
                                 uint16_t color, uint8_t scale,
                                 DisplayTextCache_t *cache);
static void    SM_InvalidateDashboardBodyCache(void);
static void    SM_InvalidateDashboardCache(void);
static void    SM_UpdateSensorDisplay(void);
static uint8_t SM_IsDashboardState(SystemState_t state);
static void    SM_DrawDashboard(uint8_t batt);
static void    SM_DrawDashboardBody(void);
static void    SM_ClearDashboardLayout(uint8_t sample_layout);
static void    SM_UpdateDashboardClock(void);
static void    SM_UpdateDashboardBattery(uint8_t batt);
static void    SM_UpdateDashboardStatus(void);
static void    SM_UpdateMenuDisplay(void);
static void    SM_HandleMenu(void);
static void    SM_RequestManualMode(MenuOption_t option);
static void    SM_ToggleValve(void);
static void    SM_ClearPreviousScreenContent(void);

/* RTC DS3231 (I2C1) */
static HAL_StatusTypeDef DS3231_ReadDateTime(uint8_t *year, uint8_t *month, uint8_t *day,
                                              uint8_t *hour, uint8_t *min,  uint8_t *sec);

/* Registro de muestreos en microSD */
static void    SM_LogMuestreo(void);

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

    /* Cualquier orden de operacion pospone la siguiente consulta periodica.
     * De este modo nunca se concatena STATUS/SENSOR inmediatamente despues
     * de una orden de motor, valvula o detencion. */
    g_ts_periodic_request = HAL_GetTick();
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
 * SM_ProcessPositionResponse
 * Mantiene la primera opcion del menu sincronizada con los finales de carrera:
 *   VASTAGO:INIT                 -> [Muestrear]
 *   VASTAGO:FINAL/INTERMEDIO     -> [Armar]
 * La pantalla de una muestra terminada permanece bloqueada en [Armar] hasta
 * que el usuario ejecute esa opcion, independientemente de la posicion.
 * -------------------------------------------------------------------------- */
static uint8_t SM_ProcessPositionResponse(void)
{
    uint8_t menu_post_sample;

    if (SM_CheckResponse(RSP_VASTAGO_INIT))
    {
        menu_post_sample = 0U;
    }
    else if (SM_CheckResponse(RSP_VASTAGO_FINAL) ||
             SM_CheckResponse(RSP_VASTAGO_INTERMEDIO))
    {
        menu_post_sample = 1U;
    }
    else
    {
        return 0U;
    }

    g_uart_rx[0] = '\0';

    if (g_menu_post_sample && g_sample_available)
    {
        return 1U;
    }

    if (g_menu_post_sample != menu_post_sample)
    {
        g_menu_post_sample = menu_post_sample;
        g_menu_selected    = MENU_PRINCIPAL;
        SM_UpdateMenuDisplay();
    }

    return 1U;
}

/* --------------------------------------------------------------------------
 * SM_ServiceDashboardQueries
 * Envia una sola consulta cada 500 ms, alternando posicion y sensores. Así
 * cada dato se renueva una vez por segundo sin superponer comandos UART.
 * -------------------------------------------------------------------------- */
static void SM_ServiceDashboardQueries(void)
{
    uint32_t ahora = HAL_GetTick();

    if ((g_ts_periodic_request != 0U) &&
        ((ahora - g_ts_periodic_request) < INTERVALO_SENSOR_MS))
    {
        return;
    }

    if (g_periodic_status_next)
    {
        SM_SendCmd(CMD_STATUS);
    }
    else
    {
        SM_SendCmd(CMD_SENSOR);
    }

    g_periodic_status_next ^= 1U;
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
 * SM_ButtonEdge
 * Igual que SM_ButtonPressed pero devuelve 1 sólo en la transición de
 * "no presionado" a "presionado" (flanco de subida), usando *prev como
 * memoria entre llamadas. Útil para acciones de "un solo disparo" como
 * abrir/cerrar el prompt de confirmación (evita repetirlo en cada tick
 * mientras el botón sigue presionado).
 * -------------------------------------------------------------------------- */
static uint8_t SM_ButtonEdge(GPIO_TypeDef *port, uint16_t pin, uint8_t *prev)
{
    uint8_t actual = SM_ButtonPressed(port, pin);
    uint8_t flanco = (actual && !(*prev)) ? 1U : 0U;
    *prev = actual;
    return flanco;
}

/* --------------------------------------------------------------------------
 * DS3231_ReadDateTime
 * Lee fecha/hora del RTC DS3231 por I2C1 y convierte de BCD a decimal.
 * 'year' se devuelve como los dos últimos dígitos (00-99); se asume 20xx.
 * -------------------------------------------------------------------------- */
static HAL_StatusTypeDef DS3231_ReadDateTime(uint8_t *year, uint8_t *month, uint8_t *day,
                                              uint8_t *hour, uint8_t *min,  uint8_t *sec)
{
    uint8_t reg = DS3231_REG_SECONDS;
    uint8_t data[7];
    HAL_StatusTypeDef result;

    result = HAL_I2C_Master_Transmit(&hi2c1, DS3231_I2C_ADDRESS, &reg, 1U,
                                      DS3231_I2C_TIMEOUT_MS);
    if (result != HAL_OK)
    {
        return result;
    }

    result = HAL_I2C_Master_Receive(&hi2c1, DS3231_I2C_ADDRESS, data, sizeof(data),
                                     DS3231_I2C_TIMEOUT_MS);
    if (result != HAL_OK)
    {
        return result;
    }

    /* data[0]=seg data[1]=min data[2]=hora(24h, bit6=0) data[3]=dia-semana
     * data[4]=dia data[5]=mes(bit7=siglo, ignorado) data[6]=anio */
    *sec   = (uint8_t)(((data[0] >> 4) & 0x07U) * 10U + (data[0] & 0x0FU));
    *min   = (uint8_t)(((data[1] >> 4) & 0x07U) * 10U + (data[1] & 0x0FU));
    *hour  = (uint8_t)(((data[2] >> 4) & 0x03U) * 10U + (data[2] & 0x0FU));
    *day   = (uint8_t)(((data[4] >> 4) & 0x03U) * 10U + (data[4] & 0x0FU));
    *month = (uint8_t)(((data[5] >> 4) & 0x01U) * 10U + (data[5] & 0x0FU));
    *year  = (uint8_t)(((data[6] >> 4) & 0x0FU) * 10U + (data[6] & 0x0FU));

    return HAL_OK;
}

/* --------------------------------------------------------------------------
 * SM_LogMuestreo
 * Obtiene fecha/hora del DS3231, la guarda en g_ultimo_muestreo_ts (para
 * mostrarla en pantalla) y agrega una línea a MUESTREOS.CSV en la microSD.
 * Es tolerante a fallos: si no hay RTC o no hay SD, no bloquea la máquina
 * de estados, sólo deja constancia en pantalla de que no se pudo guardar.
 * -------------------------------------------------------------------------- */
static void SM_LogMuestreo(void)
{
    uint8_t year, month, day, hour, min, sec;
    FATFS   fs;
    FIL     file;
    UINT    bw;

    if (DS3231_ReadDateTime(&year, &month, &day, &hour, &min, &sec) != HAL_OK)
    {
        strncpy(g_ultimo_muestreo_ts, "RTC no disponible",
                sizeof(g_ultimo_muestreo_ts) - 1U);
        g_ultimo_muestreo_ts[sizeof(g_ultimo_muestreo_ts) - 1U] = '\0';
        strncpy(g_sample_date, "--/--/----", sizeof(g_sample_date));
        strncpy(g_sample_time, "--:--", sizeof(g_sample_time));
        g_sd_log_status = g_sd_available ? SD_LOG_ERROR
                                         : SD_LOG_NO_DISPONIBLE;
        return;
    }

    snprintf(g_ultimo_muestreo_ts, sizeof(g_ultimo_muestreo_ts),
              "20%02u-%02u-%02u %02u:%02u:%02u",
              (unsigned)year, (unsigned)month, (unsigned)day,
              (unsigned)hour, (unsigned)min, (unsigned)sec);

    snprintf(g_sample_date, sizeof(g_sample_date),
             "%02u/%02u/20%02u",
             (unsigned)day, (unsigned)month, (unsigned)year);
    snprintf(g_sample_time, sizeof(g_sample_time),
             "%02u:%02u", (unsigned)hour, (unsigned)min);

    if (!g_sd_available)
    {
        g_sd_log_status = SD_LOG_NO_DISPONIBLE;
        return;
    }

    if (f_mount(&fs, "0:/", 1) != FR_OK)
    {
        /* f_mount puede haber registrado &fs aun cuando falla. Como fs es
         * local, desmontar antes de salir evita conservar un puntero inválido. */
        (void)f_mount(NULL, "0:/", 0);
        g_sd_log_status = SD_LOG_ERROR;
        return;
    }

    /* FA_OPEN_APPEND crea el archivo si no existe y posiciona al final */
    if (f_open(&file, MUESTREO_LOG_FILENAME, FA_WRITE | FA_OPEN_APPEND) == FR_OK)
    {
        char linea[32];
        int  len = snprintf(linea, sizeof(linea), "%s\r\n", g_ultimo_muestreo_ts);
        FRESULT write_result = FR_INVALID_PARAMETER;
        FRESULT close_result;

        if (len > 0)
        {
            write_result = f_write(&file, linea, (UINT)len, &bw);
        }

        close_result = f_close(&file);

        if ((write_result == FR_OK) &&
            (bw == (UINT)len) &&
            (close_result == FR_OK))
        {
            g_sd_log_status = SD_LOG_GUARDADO;
        }
        else
        {
            g_sd_log_status = SD_LOG_ERROR;
        }
    }
    else
    {
        g_sd_log_status = SD_LOG_ERROR;
    }

    (void)f_mount(NULL, "0:/", 0);
}

/* --------------------------------------------------------------------------
 * SM_BatteryPercent
 * Promedia BATT_ADC_SAMPLES, filtra la tensión y aplica histéresis antes de
 * convertirla a porcentaje (0–100) entre 2.77 V y 2.99 V.
 * -------------------------------------------------------------------------- */
static uint8_t SM_BatteryPercent(void)
{
    uint32_t suma_bateria = 0U;
    uint32_t suma_vrefint = 0U;
    uint32_t raw;
    uint32_t raw_vrefint;
    uint32_t vrefint_cal;
    uint32_t bateria_mv;
    uint32_t vref_mv;
    uint32_t ahora = HAL_GetTick();
    uint8_t  muestras_validas = 0U;
    uint8_t  porcentaje;
    uint8_t  i;

    if (!g_adc_bateria_ready)
    {
        return g_batt_last_percent;
    }

    /* El nivel real de una batería no cambia de forma brusca. Reutilizar la
     * última lectura durante 30 s evita medir nuevamente en cada transición
     * de pantalla y mostrar variaciones instantáneas producidas por ruido. */
    if (g_batt_measurement_valid &&
        ((ahora - g_batt_last_measure_tick) < BATT_UPDATE_INTERVAL_MS))
    {
        return g_batt_last_percent;
    }

    for (i = 0U; i < BATT_ADC_SAMPLES; i++)
    {
        /* Una secuencia por muestra garantiza siempre el orden:
         * primero PC0 y después VREFINT. */
        if (HAL_ADC_Start(&hadc3) != HAL_OK)
        {
            break;
        }

        if (HAL_ADC_PollForConversion(&hadc3, BATT_ADC_TIMEOUT_MS) != HAL_OK)
        {
            (void)HAL_ADC_Stop(&hadc3);
            break;
        }
        suma_bateria += HAL_ADC_GetValue(&hadc3);

        if (HAL_ADC_PollForConversion(&hadc3, BATT_ADC_TIMEOUT_MS) != HAL_OK)
        {
            (void)HAL_ADC_Stop(&hadc3);
            break;
        }
        suma_vrefint += HAL_ADC_GetValue(&hadc3);

        (void)HAL_ADC_Stop(&hadc3);
        muestras_validas++;
    }

    if (muestras_validas == 0U)
    {
        return g_batt_last_percent;
    }

    raw         = suma_bateria / muestras_validas;
    raw_vrefint = suma_vrefint / muestras_validas;
    vrefint_cal = (uint32_t)(*VREFINT_CAL_ADDR);

    if ((raw_vrefint == 0U) || (vrefint_cal == 0U) ||
        (vrefint_cal == 0xFFFFU))
    {
        return g_batt_last_percent;
    }

    /* VREFINT_CAL fue medido en fábrica con VREF+ = 3300 mV.
     * El cociente permite calcular VREF+ real sin asumir 3.3 V constantes. */
    vref_mv = ((VREFINT_CAL_VREF * vrefint_cal) + (raw_vrefint / 2UL)) /
              raw_vrefint;

    /* Corrección combinada:
     *   1) BATT_CAL_RAW = 53672 correspondió a 2990 mV con VREF+=3300 mV.
     *   2) vref_mv corrige los cambios posteriores de la referencia analógica.
     */
    {
        uint64_t numerador = (uint64_t)raw * (uint64_t)vref_mv *
                             (uint64_t)BATT_FULL_MV;
        uint64_t denominador = (uint64_t)BATT_CAL_RAW *
                               (uint64_t)BATT_CAL_VREF_MV;
        bateria_mv = (uint32_t)((numerador + (denominador / 2ULL)) /
                                denominador);
    }

    /* Filtro asimétrico: una recuperación de tensión se acepta de inmediato,
     * mientras que una caída sólo aporta 1/8 del cambio. Así se rechazan los
     * descensos breves provocados al accionar motor o electroválvula. */
    if (!g_batt_measurement_valid)
    {
        g_batt_filtered_mv = bateria_mv;
    }
    else if (bateria_mv >= g_batt_filtered_mv)
    {
        g_batt_filtered_mv = bateria_mv;
    }
    else
    {
        g_batt_filtered_mv =
            (((BATT_FILTER_DIVISOR - 1UL) * g_batt_filtered_mv) +
             bateria_mv + (BATT_FILTER_DIVISOR / 2UL)) /
            BATT_FILTER_DIVISOR;
    }

    g_batt_last_raw          = raw;
    g_batt_last_mv           = g_batt_filtered_mv;
    g_adc_vref_mv            = vref_mv;
    g_batt_last_measure_tick = ahora;
    g_batt_measurement_valid = 1U;

    if (g_batt_filtered_mv >= BATT_FULL_MV)
    {
        porcentaje = 100U;
    }
    else if (g_batt_filtered_mv <= BATT_EMPTY_MV)
    {
        porcentaje = 0U;
    }
    else
    {
        porcentaje = (uint8_t)((((g_batt_filtered_mv - BATT_EMPTY_MV) * 100UL) +
                                ((BATT_FULL_MV - BATT_EMPTY_MV) / 2UL)) /
                               (BATT_FULL_MV - BATT_EMPTY_MV));
    }

    /* Zona superior e histéresis visual: 98–100 % se considera carga
     * completa y cambios menores de tres puntos no alteran la pantalla. */
    if ((g_batt_last_percent == 100U) &&
        (porcentaje >= BATT_FULL_RELEASE_PERCENT))
    {
        /* Una batería ya reconocida como llena conserva 100 % frente a
         * pequeñas caídas de tensión; se libera al bajar de 92 %. */
        porcentaje = 100U;
    }
    else if (porcentaje >= BATT_FULL_ZONE_PERCENT)
    {
        porcentaje = 100U;
    }

    if (g_batt_last_percent == BATT_PERCENT_INVALID)
    {
        g_batt_last_percent = porcentaje;
    }
    else if (((porcentaje > g_batt_last_percent) &&
              (((uint32_t)porcentaje - (uint32_t)g_batt_last_percent) >=
               (uint32_t)BATT_PERCENT_HYSTERESIS)) ||
             ((g_batt_last_percent > porcentaje) &&
              (((uint32_t)g_batt_last_percent - (uint32_t)porcentaje) >=
               (uint32_t)BATT_PERCENT_HYSTERESIS)))
    {
        g_batt_last_percent = porcentaje;
    }

    return g_batt_last_percent;
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
    const char *temp_start;
    const char *depth_tag;
    const char *depth_start;
    const char *end;
    size_t      temp_len;
    size_t      depth_len;

    if (line == NULL) return 0U;

    temp_start = strstr(line, "T=");
    if (temp_start == NULL) return 0U;
    temp_start += 2U;

    depth_tag = strstr(temp_start, "D=");
    if (depth_tag == NULL) return 0U;

    depth_start = depth_tag + 2U;
    end = depth_start;
    while (*end != '\0' && *end != '\r' && *end != '\n') end++;

    temp_len  = (size_t)(depth_tag - temp_start);
    depth_len = (size_t)(end - depth_start);
    if ((temp_len == 0U) || (depth_len == 0U)) return 0U;

    if (temp_len >= sizeof(g_sensor_temp))
        temp_len = sizeof(g_sensor_temp) - 1U;
    if (depth_len >= sizeof(g_sensor_depth))
        depth_len = sizeof(g_sensor_depth) - 1U;

    memcpy(g_sensor_temp, temp_start, temp_len);
    g_sensor_temp[temp_len] = '\0';

    memcpy(g_sensor_depth, depth_start, depth_len);
    g_sensor_depth[depth_len] = '\0';

    return 1U;
}

/* --------------------------------------------------------------------------
 * Pantalla principal y menu inferior
 * -------------------------------------------------------------------------- */
static uint8_t SM_IsDashboardState(SystemState_t state)
{
    return ((state == STATE_COMPROBACION_SISTEMA) ||
            (state == STATE_SISTEMA_OPERANDO) ||
            (state == STATE_INICIAR_MUESTREO) ||
            (state == STATE_MUESTREO_COMPLETO) ||
            (state == STATE_CONSULTA) ||
            (state == STATE_DESCARGA) ||
            (state == STATE_ARMADO)) ? 1U : 0U;
}

/* Dibuja el texto completo la primera vez. En actualizaciones posteriores
 * transmite solamente las celdas de caracteres cuyo contenido cambió. Como
 * cada celda incluye su fondo negro, no hace falta borrar antes y desaparece
 * el destello producido por la secuencia borrar-dibujar. */
static void SM_DrawCachedText(uint16_t x, uint16_t y, const char *text,
                              uint16_t color, uint8_t scale,
                              DisplayTextCache_t *cache)
{
    size_t new_len;
    size_t old_len;
    size_t i;
    char cell[2] = {'\0', '\0'};

    if ((text == NULL) || (cache == NULL) || (scale == 0U))
    {
        return;
    }

    new_len = strlen(text);
    old_len = cache->valid ? strlen(cache->text) : 0U;

    if (!cache->valid || (cache->color != color) || (old_len != new_len))
    {
        if (cache->valid && (old_len != new_len))
        {
            size_t clear_len = (old_len > new_len) ? old_len : new_len;
            ILI9488_FillRect(x, y,
                             (uint16_t)(clear_len * 6U * scale),
                             (uint16_t)(7U * scale),
                             ILI9488_COLOR_BLACK);
        }

        ILI9488_DrawString(x, y, text, color, ILI9488_COLOR_BLACK, scale);
    }
    else
    {
        for (i = 0U; i < new_len; i++)
        {
            if (cache->text[i] != text[i])
            {
                cell[0] = text[i];
                ILI9488_DrawString((uint16_t)(x + (i * 6U * scale)), y,
                                   cell, color, ILI9488_COLOR_BLACK, scale);
            }
        }
    }

    strncpy(cache->text, text, sizeof(cache->text) - 1U);
    cache->text[sizeof(cache->text) - 1U] = '\0';
    cache->color = color;
    cache->valid = 1U;
}

static void SM_InvalidateDashboardBodyCache(void)
{
    g_draw_depth.valid = 0U;
    g_draw_temp.valid  = 0U;
    g_draw_motor.valid = 0U;
    g_draw_valve.valid = 0U;
    g_draw_comm.valid  = 0U;
    g_draw_sensor_sample_layout = 0xFFU;
}

static void SM_InvalidateDashboardCache(void)
{
    uint8_t i;

    SM_InvalidateDashboardBodyCache();
    g_draw_clock.valid   = 0U;
    g_draw_battery.valid = 0U;

    for (i = 0U; i < (uint8_t)MENU_TOTAL; i++)
    {
        g_draw_menu[i].valid = 0U;
    }
}

/* Borra exclusivamente los textos pertenecientes a la distribución anterior.
 * Es mucho más rápido que transferir nuevamente los 153600 píxeles del panel
 * completo y mantiene intactos el encabezado y el menú inferior. */
static void SM_ClearDashboardLayout(uint8_t sample_layout)
{
    uint8_t i;

    if (sample_layout)
    {
        ILI9488_FillRect(130U,  58U, 220U, 20U, ILI9488_COLOR_BLACK);
        ILI9488_FillRect(170U,  94U, 140U, 20U, ILI9488_COLOR_BLACK);
        ILI9488_FillRect(200U, 124U,  80U, 20U, ILI9488_COLOR_BLACK);
        ILI9488_FillRect( 80U, 168U, 330U, 20U, ILI9488_COLOR_BLACK);
        ILI9488_FillRect(105U, 214U, 310U, 20U, ILI9488_COLOR_BLACK);
        ILI9488_FillRect(105U, 244U, 310U, 20U, ILI9488_COLOR_BLACK);
    }
    else
    {
        static const uint16_t row_y[5] = {88U, 118U, 148U, 178U, 208U};

        for (i = 0U; i < 5U; i++)
        {
            ILI9488_FillRect(12U, row_y[i], 150U, 18U,
                             ILI9488_COLOR_BLACK);
            ILI9488_FillRect(244U, row_y[i], 145U, 18U,
                             ILI9488_COLOR_BLACK);
        }
    }
}

static void SM_UpdateSensorDisplay(void)
{
    const char *temperature;
    const char *depth;
    uint16_t value_x;
    uint16_t depth_y;
    uint16_t temp_y;
    char depth_text[16];
    char temp_text[16];
    uint8_t sample_layout;

    if (!SM_IsDashboardState(g_estado))
    {
        return;
    }

    sample_layout = (g_menu_post_sample && g_sample_available) ? 1U : 0U;

    if (sample_layout)
    {
        temperature = g_sample_temp;
        depth       = g_sample_depth;
        value_x = SAMPLE_VALUE_X;
        depth_y = 218U;
        temp_y  = 248U;
    }
    else
    {
        temperature = g_sensor_temp;
        depth       = g_sensor_depth;
        value_x = DASH_VALUE_X;
        depth_y = 88U;
        temp_y  = 118U;
    }

    if (g_draw_sensor_sample_layout != sample_layout)
    {
        g_draw_depth.valid = 0U;
        g_draw_temp.valid  = 0U;
        g_draw_sensor_sample_layout = sample_layout;
    }

    /* Cadenas de ancho fijo: los espacios finales reemplazan cualquier cifra
     * anterior sin limpiar el renglón completo. */
    snprintf(depth_text, sizeof(depth_text), "%-8.8s m", depth);
    snprintf(temp_text, sizeof(temp_text), "%-8.8s C", temperature);

    SM_DrawCachedText(value_x, depth_y, depth_text,
                      ILI9488_COLOR_WHITE, 2U, &g_draw_depth);
    SM_DrawCachedText(value_x, temp_y, temp_text,
                      ILI9488_COLOR_WHITE, 2U, &g_draw_temp);
}

static void SM_UpdateDashboardStatus(void)
{
    const char *motor_text;
    uint8_t motor_activo;
    char motor_fixed[12];
    char valve_fixed[12];

    if (!SM_IsDashboardState(g_estado) ||
        (g_menu_post_sample && g_sample_available))
    {
        return;
    }

    if ((g_estado == STATE_INICIAR_MUESTREO) ||
        (g_estado == STATE_ARMADO))
    {
        motor_text   = "Activo";
        motor_activo = 1U;
    }
    else if (g_jog_actual == JOG_AVANZANDO)
    {
        motor_text   = "Vaciando";
        motor_activo = 1U;
    }
    else if (g_jog_actual == JOG_RETROCEDIENDO)
    {
        motor_text   = "Llenando";
        motor_activo = 1U;
    }
    else
    {
        motor_text   = "Detenido";
        motor_activo = 0U;
    }

    snprintf(motor_fixed, sizeof(motor_fixed), "%-9.9s", motor_text);
    snprintf(valve_fixed, sizeof(valve_fixed), "%-9.9s",
             g_valvula_abierta ? "Abierta" : "Cerrada");

    SM_DrawCachedText(DASH_VALUE_X, 148U, motor_fixed,
                      motor_activo ? ILI9488_COLOR_GREEN
                                   : ILI9488_COLOR_WHITE,
                      2U, &g_draw_motor);
    SM_DrawCachedText(DASH_VALUE_X, 178U, valve_fixed,
                      g_valvula_abierta ? ILI9488_COLOR_GREEN
                                        : ILI9488_COLOR_WHITE,
                      2U, &g_draw_valve);
    SM_DrawCachedText(DASH_VALUE_X, 208U, "OK       ",
                      ILI9488_COLOR_GREEN, 2U, &g_draw_comm);
}

static void SM_UpdateMenuDisplay(void)
{
    const char *labels[MENU_TOTAL];
    static const uint16_t slot_x[MENU_TOTAL] = {12U, 156U, 264U, 372U};
    static const uint16_t slot_w[MENU_TOTAL] = {132U,  96U,  96U,  96U};
    uint16_t text_width;
    uint16_t text_x;
    uint16_t color;
    uint8_t label_changed;
    uint8_t i;

    if (!SM_IsDashboardState(g_estado))
    {
        return;
    }

    labels[MENU_PRINCIPAL] = g_menu_post_sample ? "[Armar]" : "[Muestrear]";
    labels[MENU_VACIAR]    = "[Vaciar]";
    labels[MENU_LLENAR]    = "[Llenar]";
    labels[MENU_VALVULA]   = g_valvula_abierta ? "[Cerrar]" : "[Abrir]";

    /* Cada opción tiene un espacio fijo. Al desplazar la selección sólo se
     * vuelven a transmitir las dos palabras cuyo color cambia; el renglón
     * completo ya no se borra ni se dibuja nuevamente. */
    for (i = 0U; i < (uint8_t)MENU_TOTAL; i++)
    {
        color = ((MenuOption_t)i == g_menu_selected)
                    ? ILI9488_COLOR_YELLOW : ILI9488_COLOR_WHITE;

        if ((((MenuOption_t)i == MENU_VACIAR) &&
             (g_jog_actual == JOG_AVANZANDO)) ||
            (((MenuOption_t)i == MENU_LLENAR) &&
             (g_jog_actual == JOG_RETROCEDIENDO)) ||
            (((MenuOption_t)i == MENU_VALVULA) && g_valvula_abierta))
        {
            color = ILI9488_COLOR_GREEN;
        }

        text_width = (uint16_t)strlen(labels[i]) * 12U;
        text_x = (uint16_t)(slot_x[i] +
                 ((slot_w[i] - text_width) / 2U));

        label_changed = (!g_draw_menu[i].valid ||
                         (strcmp(g_draw_menu[i].text, labels[i]) != 0))
                            ? 1U : 0U;

        if (g_draw_menu[i].valid && label_changed)
        {
            ILI9488_FillRect(slot_x[i], 282U, slot_w[i], 28U,
                             ILI9488_COLOR_BLACK);
        }

        if (label_changed || (g_draw_menu[i].color != color))
        {
            ILI9488_DrawString(text_x, 288U, labels[i], color,
                               ILI9488_COLOR_BLACK, 2U);

            strncpy(g_draw_menu[i].text, labels[i],
                    sizeof(g_draw_menu[i].text) - 1U);
            g_draw_menu[i].text[sizeof(g_draw_menu[i].text) - 1U] = '\0';
            g_draw_menu[i].color = color;
            g_draw_menu[i].valid = 1U;
        }
    }
}

static void SM_UpdateDashboardClock(void)
{
    uint8_t year, month, day, hour, min, sec;
    char time_text[8] = "--:--";

    if (DS3231_ReadDateTime(&year, &month, &day, &hour, &min, &sec) == HAL_OK)
    {
        (void)year;
        (void)month;
        (void)day;
        (void)sec;
        snprintf(time_text, sizeof(time_text), "%02u:%02u",
                 (unsigned)hour, (unsigned)min);
    }

    /* HH:MM ocupa 60 px a escala 2; sólo cambian las cifras necesarias. */
    SM_DrawCachedText(210U, 18U, time_text,
                      ILI9488_COLOR_YELLOW, 2U, &g_draw_clock);
    g_ts_dashboard_clock = HAL_GetTick();
}

static void SM_UpdateDashboardBattery(uint8_t batt)
{
    char batt_text[16];
    uint16_t batt_color;
    size_t len;

    if (batt <= 100U)
    {
        snprintf(batt_text, sizeof(batt_text), "BAT %u%%", (unsigned)batt);
        if (batt <= 20U)
            batt_color = ILI9488_COLOR_RED;
        else if (batt <= 50U)
            batt_color = ILI9488_COLOR_YELLOW;
        else
            batt_color = ILI9488_COLOR_GREEN;
    }
    else
    {
        snprintf(batt_text, sizeof(batt_text), "BAT --%%");
        batt_color = ILI9488_COLOR_WHITE;
    }

    /* Mantener ocho caracteres para que una cifra anterior se reemplace con
     * espacios sin borrar la barra superior. */
    len = strlen(batt_text);
    while ((len < 8U) && (len < (sizeof(batt_text) - 1U)))
    {
        batt_text[len++] = ' ';
    }
    batt_text[len] = '\0';

    SM_DrawCachedText(372U, 18U, batt_text,
                      batt_color, 2U, &g_draw_battery);
}

static void SM_DrawDashboardBody(void)
{

    if (g_menu_post_sample && g_sample_available)
    {
        ILI9488_DrawString(138U, 62U, "Muestra realizada",
            ILI9488_COLOR_GREEN, ILI9488_COLOR_BLACK, 2);

        ILI9488_DrawString(180U, 98U, g_sample_date,
                           ILI9488_COLOR_CYAN, ILI9488_COLOR_BLACK, 2);
        ILI9488_DrawString(210U, 128U, g_sample_time,
                           ILI9488_COLOR_CYAN, ILI9488_COLOR_BLACK, 2);
        ILI9488_DrawString(90U, 172U, "Condiciones de la muestra",
                           ILI9488_COLOR_YELLOW, ILI9488_COLOR_BLACK, 2);

        ILI9488_DrawString(SAMPLE_LABEL_X, 218U, "Profundidad",
                           ILI9488_COLOR_CYAN, ILI9488_COLOR_BLACK, 2);
        ILI9488_DrawString(SAMPLE_LABEL_X, 248U, "Temperatura",
                           ILI9488_COLOR_CYAN, ILI9488_COLOR_BLACK, 2);
    }
    else
    {
        ILI9488_DrawString(18U, 88U, "Profundidad",
                           ILI9488_COLOR_CYAN, ILI9488_COLOR_BLACK, 2);
        ILI9488_DrawString(18U, 118U, "Temperatura",
                           ILI9488_COLOR_CYAN, ILI9488_COLOR_BLACK, 2);
        ILI9488_DrawString(18U, 148U, "Motor",
                           ILI9488_COLOR_CYAN, ILI9488_COLOR_BLACK, 2);
        ILI9488_DrawString(18U, 178U, "Valvula",
                           ILI9488_COLOR_CYAN, ILI9488_COLOR_BLACK, 2);
        ILI9488_DrawString(18U, 208U, "Comunicacion",
                           ILI9488_COLOR_CYAN, ILI9488_COLOR_BLACK, 2);
    }

    SM_UpdateSensorDisplay();
    SM_UpdateDashboardStatus();
    SM_UpdateMenuDisplay();
}

static void SM_DrawDashboard(uint8_t batt)
{
    /* Esta función se usa sólo al entrar por primera vez al tablero o después
     * de una limpieza completa. Las transiciones internas usan actualizaciones
     * parciales y no vuelven a transmitir el encabezado. */
    ILI9488_DrawString(4U, 18U, "SISTEMA OPERANDO",
                       ILI9488_COLOR_CYAN, ILI9488_COLOR_BLACK, 2U);
    SM_UpdateDashboardClock();
    SM_UpdateDashboardBattery(batt);
    SM_DrawDashboardBody();
}

static void SM_ToggleValve(void)
{
    if (g_valvula_abierta)
    {
        SM_SendCmd(CMD_VALVULA_CERRAR);
        g_valvula_abierta = 0U;
        g_jog_actual      = JOG_NONE;
    }
    else
    {
        SM_SendCmd(CMD_VALVULA_ABRIR);
        g_valvula_abierta = 1U;
    }

    SM_UpdateDashboardStatus();
    SM_UpdateMenuDisplay();
}

static void SM_RequestManualMode(MenuOption_t option)
{
    g_pending_manual_menu = (uint8_t)option;
    if (g_estado != STATE_DESCARGA)
    {
        SM_SetState(STATE_DESCARGA);
    }
}

static void SM_HandleMenu(void)
{
    uint8_t select_now;
    uint8_t select_edge;
    uint8_t left_edge;
    uint8_t right_edge;

    select_now  = SM_ButtonPressed(BTN_SELECT_PORT, BTN_SELECT_PIN);
    select_edge = (select_now && !g_btn_select_prev) ? 1U : 0U;
    g_btn_select_prev = select_now;

    left_edge  = SM_ButtonEdge(BTN_LEFT_PORT, BTN_LEFT_PIN,
                               &g_btn_left_prev);
    right_edge = SM_ButtonEdge(BTN_RIGHT_PORT, BTN_RIGHT_PIN,
                               &g_btn_right_prev);

    /* Un fin de carrera bloquea nuevos comandos mientras Btn 1 continúe
     * presionado. La siguiente orden solo se habilita después de soltarlo. */
    if (!select_now)
    {
        g_jog_bloqueado = 0U;
    }

    /* Al soltar Btn 1, detener inmediatamente cualquier movimiento manual.
     * El nodo sumergido tambien cierra la valvula al recibir CMD:DETENER. */
    if (!select_now && (g_jog_actual != JOG_NONE))
    {
        SM_SendCmd(CMD_DETENER);
        g_jog_actual      = JOG_NONE;
        g_valvula_abierta = 0U;
        SM_UpdateDashboardStatus();
        SM_UpdateMenuDisplay();
    }

    if (left_edge)
    {
        if (g_menu_selected == MENU_PRINCIPAL)
            g_menu_selected = (MenuOption_t)(MENU_TOTAL - 1U);
        else
            g_menu_selected = (MenuOption_t)((uint8_t)g_menu_selected - 1U);
        SM_UpdateMenuDisplay();
        return;
    }

    if (right_edge)
    {
        g_menu_selected = (MenuOption_t)(((uint8_t)g_menu_selected + 1U) %
                                         (uint8_t)MENU_TOTAL);
        SM_UpdateMenuDisplay();
        return;
    }

    switch (g_menu_selected)
    {
    case MENU_PRINCIPAL:
        if (select_edge)
        {
            if (g_menu_post_sample)
                SM_SetState(STATE_ARMADO);
            else
                SM_SetState(STATE_INICIAR_MUESTREO);
        }
        break;

    case MENU_VACIAR:
    case MENU_LLENAR:
        if (g_estado != STATE_DESCARGA)
        {
            if (select_edge)
                SM_RequestManualMode(g_menu_selected);
        }
        else if (select_now && !g_jog_bloqueado)
        {
            uint8_t target_jog = (g_menu_selected == MENU_VACIAR)
                                     ? JOG_AVANZANDO : JOG_RETROCEDIENDO;
            if (g_jog_actual != target_jog)
            {
                SM_SendCmd((g_menu_selected == MENU_VACIAR)
                               ? CMD_AVANZAR : CMD_RETROCEDER);
                g_jog_actual      = target_jog;
                g_valvula_abierta = 1U;
                SM_UpdateDashboardStatus();
                SM_UpdateMenuDisplay();
            }
        }
        break;

    case MENU_VALVULA:
        if (select_edge)
        {
            if (g_estado != STATE_DESCARGA)
                SM_RequestManualMode(MENU_VALVULA);
            else
                SM_ToggleValve();
        }
        break;

    case MENU_TOTAL:
    default:
        g_menu_selected = MENU_PRINCIPAL;
        SM_UpdateMenuDisplay();
        break;
    }
}

/* --------------------------------------------------------------------------
 * SM_ClearPreviousScreenContent
 * Borra sólo los renglones que realmente utilizaba el estado anterior. Esto
 * evita el barrido de pantalla completa y también evita DISPOFF/DISPON, que en
 * algunos módulos ILI9488 produce un destello blanco visible.
 * -------------------------------------------------------------------------- */
static void SM_ClearPreviousScreenContent(void)
{
    const uint16_t text_width = ILI9488_WIDTH - 36U;

    if (g_screen_last_state == STATE_INICIO)
    {
        /* STATE_INICIO es el único cuyo título está en Y=60. */
        ILI9488_FillRect(18U, 60U, 216U, 21U, ILI9488_COLOR_BLACK);
        return;
    }

    /* Todos los demás estados normales usan un título de escala 3 en Y=40 y
     * un mensaje superior (hasta dos renglones) en Y=100. */
    ILI9488_FillRect(18U,  40U, 216U, 21U, ILI9488_COLOR_BLACK);
    ILI9488_FillRect(18U, 100U, text_width, 30U, ILI9488_COLOR_BLACK);

    switch (g_screen_last_state)
    {
    case STATE_ESTABLECER_CONEXION:
        ILI9488_FillRect(18U, 200U, text_width, 14U, ILI9488_COLOR_BLACK);
        break;

    case STATE_SISTEMA_OPERANDO:
        /* Dos valores de ancho fijo y la opción inferior. */
        ILI9488_FillRect(18U, 140U, 324U, 21U, ILI9488_COLOR_BLACK);
        ILI9488_FillRect(18U, 185U, 324U, 21U, ILI9488_COLOR_BLACK);
        ILI9488_FillRect(18U, 245U, 300U, 14U, ILI9488_COLOR_BLACK);
        break;

    case STATE_INICIAR_MUESTREO:
        ILI9488_FillRect(18U, 160U, text_width, 30U, ILI9488_COLOR_BLACK);
        break;

    case STATE_MUESTREO_COMPLETO:
        ILI9488_FillRect(18U, 160U, text_width, 30U, ILI9488_COLOR_BLACK);
        ILI9488_FillRect(18U, 210U, text_width, 30U, ILI9488_COLOR_BLACK);
        break;

    case STATE_CONSULTA:
        ILI9488_FillRect(18U, 160U, text_width, 14U, ILI9488_COLOR_BLACK);
        ILI9488_FillRect(18U, 210U, text_width, 14U, ILI9488_COLOR_BLACK);
        break;

    case STATE_DESCARGA:
        /* Se limpian todas las variantes posibles del modo descarga: jog,
         * válvula y cuadro de confirmación. */
        ILI9488_FillRect(18U, 150U, text_width, 14U, ILI9488_COLOR_BLACK);
        ILI9488_FillRect(18U, 160U, text_width, 14U, ILI9488_COLOR_BLACK);
        ILI9488_FillRect(18U, 195U, text_width, 14U, ILI9488_COLOR_BLACK);
        ILI9488_FillRect(18U, 210U, text_width, 14U, ILI9488_COLOR_BLACK);
        ILI9488_FillRect(18U, 240U, text_width, 14U, ILI9488_COLOR_BLACK);
        break;

    case STATE_COMPROBACION_SISTEMA:
    case STATE_ARMADO:
    case STATE_ERROR_CONEXION: /* La pantalla roja se trata con FillScreen. */
    case STATE_INICIO:
    default:
        break;
    }
}

/* --------------------------------------------------------------------------
 * SM_DrawScreen
 * Actualiza la pantalla ILI9488 según el estado activo.
 * Se llama ÚNICAMENTE desde SM_SetState, es decir, solo cuando hay un
 * cambio de estado. Así se evita redibujar en cada iteración del loop.
 * -------------------------------------------------------------------------- */
static void SM_DrawScreen(void)
{
    char buf[32];
    uint8_t batt = SM_BatteryPercent();
    uint8_t current_dashboard  = SM_IsDashboardState(g_estado);
    uint8_t previous_dashboard = SM_IsDashboardState(g_screen_last_state);
    uint8_t current_sample_layout =
        (g_menu_post_sample && g_sample_available) ? 1U : 0U;
    uint8_t layout_changed =
        (current_dashboard && previous_dashboard &&
         (current_sample_layout != g_screen_last_sample_layout)) ? 1U : 0U;
    uint8_t full_redraw =
        (!g_screen_initialized ||
         (g_estado == STATE_ERROR_CONEXION) ||
         (g_screen_last_state == STATE_ERROR_CONEXION) ||
         (current_dashboard != previous_dashboard)) ? 1U : 0U;

    /* La pantalla completa sólo se limpia al entrar o salir del tablero. Un
     * cambio entre la vista normal y "Muestra realizada" borra únicamente
     * los textos de la distribución anterior. */
    if (full_redraw)
    {
        ILI9488_FillScreen((g_estado == STATE_ERROR_CONEXION)
                          ? ILI9488_COLOR_RED : ILI9488_COLOR_BLACK);

        if (current_dashboard)
        {
            SM_InvalidateDashboardCache();
        }
    }
    else if (layout_changed)
    {
        SM_ClearDashboardLayout(g_screen_last_sample_layout);
        SM_InvalidateDashboardBodyCache();
    }
    else if (!current_dashboard)
    {
        SM_ClearPreviousScreenContent();
    }

#if BATT_ADC_DEBUG_SCREEN
    /* Diagnóstico visible para comprobar qué está entregando realmente ADC3. */
    if (g_batt_measurement_valid)
    {
        snprintf(buf, sizeof(buf), "A:%lu V:%lu %lumV",
                 (unsigned long)g_batt_last_raw,
                 (unsigned long)g_adc_vref_mv,
                 (unsigned long)g_batt_last_mv);
    }
    else
    {
        snprintf(buf, sizeof(buf), "ADC:ERROR");
    }
    ILI9488_DrawString(6, 6, buf,
                       ILI9488_COLOR_YELLOW, ILI9488_COLOR_BLACK, 2);
#endif

    /* Los estados de operación comparten el mismo tablero. En transiciones
     * internas se actualizan sólo los campos dinámicos; no se retransmiten
     * título, etiquetas ni fondo. */
    if (current_dashboard)
    {
        if (full_redraw)
        {
            SM_DrawDashboard(batt);
        }
        else if (layout_changed)
        {
            SM_UpdateDashboardBattery(batt);
            SM_DrawDashboardBody();
        }
        else
        {
            SM_UpdateDashboardBattery(batt);
            SM_UpdateSensorDisplay();
            SM_UpdateDashboardStatus();
            SM_UpdateMenuDisplay();
        }

        g_screen_last_state         = g_estado;
        g_screen_last_sample_layout = current_sample_layout;
        g_screen_initialized        = 1U;
        return;
    }

    /* ---- Barra de estado superior: nivel de batería ---- */
    if ((g_estado != STATE_ERROR_CONEXION) && !current_dashboard)
    {
        if (batt <= 100U)
        {
            snprintf(buf, sizeof(buf), "Bat: %3d%%", (int)batt);
        }
        else
        {
            snprintf(buf, sizeof(buf), "Bat:  --%%");
        }
        ILI9488_DrawString(ILI9488_WIDTH - 130, 6,
                           buf, ILI9488_COLOR_CYAN, ILI9488_COLOR_BLACK, 2);
    }

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

    default:
        break;
    }

    g_screen_last_state  = g_estado;
    g_screen_last_sample_layout = current_sample_layout;
    g_screen_initialized = 1U;
}

/* --------------------------------------------------------------------------
 * SM_SetState
 * Cambia el estado activo, resetea flags compartidos y redibuja la pantalla.
 * Es el único punto de transición entre estados.
 * -------------------------------------------------------------------------- */
static void SM_SetState(SystemState_t nuevo)
{
    if (nuevo == STATE_MUESTREO_COMPLETO)
    {
        /* Registrar antes de dibujar para que fecha y hora aparezcan en la
         * primera presentacion de la pantalla "Muestra realizada". */
        g_ultimo_muestreo_ts[0] = '\0';
        g_sd_log_status          = SD_LOG_PENDIENTE;
        strncpy(g_sample_date, "--/--/----", sizeof(g_sample_date));
        strncpy(g_sample_time, "--:--", sizeof(g_sample_time));
        SM_LogMuestreo();
    }

    if (nuevo == STATE_COMPROBACION_SISTEMA)
    {
        /* STATUS decide el siguiente estado. La telemetria se solicita luego
         * sin bloquear la salida de la pantalla de verificacion. */
        g_ts_periodic_request    = 0U;
    }
    else if (nuevo == STATE_SISTEMA_OPERANDO)
    {
        /* Entrar inmediatamente. Las lecturas se solicitan periodicamente
         * mientras permanezca visible el tablero principal. */
        g_ts_periodic_request    = 0U;
        g_periodic_status_next   = 1U;
        g_menu_post_sample       = 0U;
        g_menu_selected          = MENU_PRINCIPAL;
        g_sample_available       = 0U;
    }
    else if (nuevo == STATE_MUESTREO_COMPLETO)
    {
        g_menu_post_sample = 1U;
        g_menu_selected    = MENU_PRINCIPAL;
    }
    else if (nuevo == STATE_CONSULTA)
    {
        g_ts_periodic_request  = 0U;
        g_periodic_status_next = 1U;
        g_menu_post_sample     = 1U;
        g_menu_selected        = MENU_PRINCIPAL;
    }
    else if (nuevo == STATE_ARMADO)
    {
        /* Abandonar definitivamente la pantalla de la muestra anterior. */
        g_menu_post_sample = 0U;
        g_menu_selected    = MENU_PRINCIPAL;
        g_sample_available = 0U;
    }

    g_estado        = nuevo;
    g_esperando_rsp = 0U;
    g_uart_rx[0]    = '\0';

    /* Cada entrada al modo manual exige un nuevo MODO:DESCARGA. La opcion
     * pendiente se conserva para ejecutarla cuando llegue la confirmacion. */
    if (nuevo == STATE_DESCARGA)
    {
        g_periodic_status_next = 1U;
        g_descarga_listo  = 0U;
        g_jog_actual      = JOG_NONE;
        g_jog_bloqueado   = 0U;
        g_valvula_abierta = 0U;
    }
    else if (nuevo == STATE_INICIAR_MUESTREO)
    {
        /* Reflejar el accionamiento en el tablero sin abrir una pantalla
         * intermedia. El nodo sumergido abre la valvula antes del motor. */
        g_descarga_listo      = 0U;
        g_pending_manual_menu = MENU_PENDING_NONE;
        g_jog_actual          = JOG_NONE;
        g_jog_bloqueado       = 0U;
        g_valvula_abierta     = 1U;
    }
    else
    {
        g_descarga_listo      = 0U;
        g_pending_manual_menu = MENU_PENDING_NONE;
        g_jog_actual          = JOG_NONE;
        g_jog_bloqueado       = 0U;
        g_valvula_abierta     = 0U;
    }

    g_btn_select_prev = 0U;
    g_btn_left_prev   = 0U;
    g_btn_right_prev  = 0U;

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
                if (SM_IsDashboardState(g_estado) &&
                    (!g_menu_post_sample || !g_sample_available))
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

    /* En las pantallas de operacion, una respuesta STATUS actualiza la opcion
     * principal sin provocar una transicion ni redibujar toda la pantalla.
     * COMPROBACION_SISTEMA conserva su tratamiento propio más abajo. */
    if (hay_linea &&
        ((g_estado == STATE_SISTEMA_OPERANDO) ||
         (g_estado == STATE_CONSULTA) ||
         (g_estado == STATE_DESCARGA)) &&
        SM_ProcessPositionResponse())
    {
        hay_linea = 0U;
    }

    if (SM_IsDashboardState(g_estado) &&
        ((ahora - g_ts_dashboard_clock) >= DASH_CLOCK_UPDATE_MS))
    {
        SM_UpdateDashboardClock();
        SM_UpdateDashboardBattery(SM_BatteryPercent());
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
     * STATUS decide la transicion; una lectura de sensores nunca puede dejar
     * el sistema bloqueado en esta pantalla.
     * VASTAGO:INIT → SISTEMA_OPERANDO
     * VASTAGO:FINAL o VASTAGO:INTERMEDIO → CONSULTA (menu con [Armar])
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
            else if (SM_CheckResponse(RSP_VASTAGO_FINAL) ||
                     SM_CheckResponse(RSP_VASTAGO_INTERMEDIO))
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
     * Pantalla principal: Btn 2/3 desplazan el menu y Btn 1 ejecuta.
     */
        SM_HandleMenu();

        /* Alternar sensores y posicion. La comprobacion de estado evita
         * consultar si Btn 1 acaba de iniciar otra operacion. */
        if (g_estado == STATE_SISTEMA_OPERANDO)
        {
            SM_ServiceDashboardQueries();
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
                /* Congelar las condiciones asociadas a esta muestra. Las
                 * telemetrias posteriores no modificaran estos valores. */
                strncpy(g_sample_temp, g_sensor_temp,
                        sizeof(g_sample_temp) - 1U);
                g_sample_temp[sizeof(g_sample_temp) - 1U] = '\0';
                strncpy(g_sample_depth, g_sensor_depth,
                        sizeof(g_sample_depth) - 1U);
                g_sample_depth[sizeof(g_sample_depth) - 1U] = '\0';
                g_sample_available = 1U;
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
     * Pantalla persistente de resultado. El menu cambia su primera opcion
     * de [Muestrear] a [Armar].
     */
        SM_HandleMenu();
        break;

    /* ================================================================== */
    case STATE_CONSULTA:
    /*
     * El equipo arranco fuera de la posicion inicial. STATUS mantiene la
     * opcion principal sincronizada si luego cambia la posicion del vastago.
     */
        SM_HandleMenu();

        if ((g_estado == STATE_CONSULTA) && !g_sample_available)
        {
            SM_ServiceDashboardQueries();
        }
        break;

    /* ================================================================== */
    case STATE_DESCARGA:
    /*
     * Estado interno del menu manual. La pantalla no cambia de formato:
     * Btn 2/3 siguen desplazando la seleccion y Btn 1 ejecuta o mantiene
     * Vaciar/Llenar. Al entrar se negocia MODO:DESCARGA con el sumergido.
     */
        if (!g_descarga_listo)
        {
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

            if (hay_linea && SM_CheckResponse(RSP_MODO_DESCARGA))
            {
                g_descarga_listo  = 1U;
                g_valvula_abierta = 0U;

                /* Abrir/Cerrar es una accion por pulsacion, por eso debe
                 * ejecutarse aunque Btn 1 se haya soltado durante el enlace.
                 * Para Vaciar/Llenar se comprueba el nivel del boton en el
                 * siguiente tick y solo se mueve mientras siga presionado. */
                if (g_pending_manual_menu == (uint8_t)MENU_VALVULA)
                {
                    SM_ToggleValve();
                }
                /* La entrada a este estado se origino con un flanco de Btn 1.
                 * Marcarlo consumido evita alternar la valvula por segunda vez
                 * si el usuario aun mantiene el boton presionado. */
                g_btn_select_prev = 1U;
                g_pending_manual_menu = MENU_PENDING_NONE;
                SM_UpdateDashboardStatus();
                SM_UpdateMenuDisplay();
            }
            break;
        }

        /* ---- Notificaciones asíncronas del sumergido ---- */
        if (hay_linea)
        {
            if (SM_CheckResponse(RSP_LIMITE_FINAL))
            {
                /* Posicion final: detener una sola vez y ofrecer [Armar].
                 * El bloqueo evita reabrir la valvula mientras Btn 1 siga
                 * presionado sobre [Llenar]. */
                g_jog_actual      = JOG_NONE;
                g_jog_bloqueado   = 1U;
                g_valvula_abierta = 0U;
                g_menu_post_sample = 1U;
                g_uart_rx[0]      = '\0';
                SM_UpdateDashboardStatus();
                SM_UpdateMenuDisplay();
            }
            else if (SM_CheckResponse(RSP_LIMITE_INICIAL))
            {
                /* El fin inicial detiene el mecanismo, pero una pantalla de
                 * resultado solo puede abandonarse pulsando [Armar]. */
                g_jog_actual      = JOG_NONE;
                g_jog_bloqueado   = 1U;
                g_valvula_abierta = 0U;
                g_uart_rx[0]      = '\0';

                if (!(g_menu_post_sample && g_sample_available))
                {
                    /* Fuera de "Muestra realizada", la posicion inicial sí
                     * habilita un nuevo ciclo automático. */
                    g_menu_post_sample = 0U;
                }

                SM_UpdateDashboardStatus();
                SM_UpdateMenuDisplay();
            }
            else if (SM_CheckResponse(RSP_JOG_TIMEOUT))
            {
                g_jog_actual      = JOG_NONE;
                g_jog_bloqueado   = 1U;
                g_valvula_abierta = 0U;
                g_uart_rx[0]      = '\0';
                SM_UpdateDashboardStatus();
                SM_UpdateMenuDisplay();
            }
            else if (SM_CheckResponse(RSP_JOG_DETENIDO))
            {
                g_jog_actual      = JOG_NONE;
                g_valvula_abierta = 0U;
                g_uart_rx[0]      = '\0';
                SM_UpdateDashboardStatus();
                SM_UpdateMenuDisplay();
            }
            else if (SM_CheckResponse(RSP_JOG_AVANZANDO))
            {
                g_jog_actual      = JOG_AVANZANDO;
                g_valvula_abierta = 1U;
                if (!(g_menu_post_sample && g_sample_available))
                {
                    g_menu_post_sample = 1U;
                }
                g_uart_rx[0]      = '\0';
                SM_UpdateDashboardStatus();
                SM_UpdateMenuDisplay();
            }
            else if (SM_CheckResponse(RSP_JOG_RETROCEDIENDO))
            {
                g_jog_actual      = JOG_RETROCEDIENDO;
                g_valvula_abierta = 1U;
                if (!(g_menu_post_sample && g_sample_available))
                {
                    g_menu_post_sample = 1U;
                }
                g_uart_rx[0]      = '\0';
                SM_UpdateDashboardStatus();
                SM_UpdateMenuDisplay();
            }
            else if (SM_CheckResponse(RSP_VALVULA_ABIERTA))
            {
                g_valvula_abierta = 1U;
                g_uart_rx[0]      = '\0';
                SM_UpdateDashboardStatus();
                SM_UpdateMenuDisplay();
            }
            else if (SM_CheckResponse(RSP_VALVULA_CERRADA))
            {
                g_jog_actual      = JOG_NONE;
                g_valvula_abierta = 0U;
                g_uart_rx[0]      = '\0';
                SM_UpdateDashboardStatus();
                SM_UpdateMenuDisplay();
            }
        }

        SM_HandleMenu();

        /* Si el usuario suelta Vaciar/Llenar antes de alcanzar un extremo,
         * STATUS detecta VASTAGO:INTERMEDIO y reemplaza [Muestrear] por
         * [Armar]. Sólo se consulta con el motor detenido. */
        if ((g_estado == STATE_DESCARGA) && g_descarga_listo &&
            (g_jog_actual == JOG_NONE))
        {
            SM_ServiceDashboardQueries();
        }
        break;

    /* ================================================================== */
    case STATE_ARMADO:
    /*
     * Envia CMD:ARMAR.
     * ARMADO_OK o ARMADO_ERROR:TIMEOUT -> COMPROBACION_SISTEMA.
     */
        if (!g_esperando_rsp)
        {
            SM_SendCmd(CMD_ARMAR);
            g_ts_respuesta  = ahora;
            g_esperando_rsp = 1U;
            break;
        }

        /* Procesar primero la respuesta para no perderla justo cuando
         * tambien vence TIMEOUT_RESPUESTA_MS. */
        if (hay_linea)
        {
            if (SM_CheckResponse(RSP_ARMADO_OK) ||
                SM_CheckResponse(RSP_ARMADO_TIMEOUT))
            {
                SM_SetState(STATE_COMPROBACION_SISTEMA);
                break;
            }

            g_uart_rx[0] = '\0';
        }

        /* No reenviar CMD:ARMAR cada 3 s: el recorrido sumergido admite hasta
         * 35 s. Si tampoco llega su respuesta, comprobar igualmente estado. */
        if ((ahora - g_ts_respuesta) >= TIMEOUT_ARMADO_MS)
        {
            SM_SetState(STATE_COMPROBACION_SISTEMA);
            break;
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
  if (g_sd_available)
  {
    /* Vincula el driver FatFs sólo después de inicializar correctamente
     * la tarjeta. No se llama si la microSD está ausente o deshabilitada. */
    MX_FATFS_Init();
  }
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  /* Pantalla */
  ILI9488_Init(&hspi1);

  /* UART hacia módulo sumergido */
  UARTUTILS_Init(&huart4);

  /* Calibrar el ADC antes de comenzar. SM_BatteryPercent inicia una secuencia
   * PC0 + VREFINT por muestra, lee ambos valores y vuelve a detener el ADC. */
  if (HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET_LINEARITY,
                                  ADC_SINGLE_ENDED) == HAL_OK)
  {
    g_adc_bateria_ready = 1U;
    HAL_Delay(100U); /* Estabilización de VREFINT y de la etapa analógica */
  }

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
  /* PLL2P entrega 80 MHz. Dividir entre 8 deja el reloj ADC en 10 MHz,
   * apropiado para una medición estable de 16 bits en LQFP100. */
  hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV8;
  hadc3.Init.Resolution = ADC_RESOLUTION_16B;
  hadc3.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.NbrOfConversion = 2;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc3.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
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
  /* La medición de batería no requiere velocidad. El tiempo largo permite que
   * el capacitor interno de muestreo se cargue aun con una fuente de impedancia
   * relativamente alta y evita lecturas inferiores al voltaje real del pin. */
  sConfig.SamplingTime = ADC_SAMPLETIME_810CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure VREFINT as the second conversion in the regular sequence.
   */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  sConfig.SamplingTime = ADC_SAMPLETIME_810CYCLES_5;
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

    g_sd_available = 0U;

#if MICROSD_HABILITADA
    /* La disponibilidad se determina con el resultado real de HAL_SD_Init.
     * HAL_SD_GetState() no sirve para detectar una tarjeta: algunas versiones
     * de HAL vuelven a READY incluso después de un error de inicialización. */
    if (HAL_SD_Init(&hsd1) == HAL_OK)
    {
        g_sd_available = 1U;
    }
#endif
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
  /* Aumentar el enlace de la TFT de aproximadamente 12 MHz a 24 MHz. El
   * prescaler 8 reduce a la mitad el tiempo de transferencia sin recurrir a
   * la frecuencia más exigente del prescaler 4. */
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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
