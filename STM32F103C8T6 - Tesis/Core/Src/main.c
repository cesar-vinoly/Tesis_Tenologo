/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body — Módulo SUMERGIDO STM32F103C8T6
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "uart_utils.h"
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/*
 * El módulo sumergido es ESCLAVO: espera comandos de la superficie,
 * los ejecuta físicamente y responde cuando termina.
 *
 * Estado principal:
 *   ESPERA    → Idle, responde PING y STATUS en cualquier momento.
 *   MUESTREO  → Abre la electrovalvula y desplaza el vastago hasta PB1.
 *   MUESTREO_COMPLETO → Detiene el motor, cierra la electrovalvula y
 *                       envia MUESTREO_DONE o MUESTREO_ERROR:TIMEOUT.
 *   DESCARGA  → Modo "jog": el usuario controla avance/retroceso del
 *               vástago en vivo desde la superficie (CMD:AVANZAR /
 *               CMD:RETROCEDER / CMD:DETENER), con corte automático
 *               por fin de carrera. También acepta CMD:ARMAR para
 *               saltar directamente a ARMANDO.
 *   ARMANDO   → Mueve el vástago a posición inicial.
 *   EMERGENCIA → Detiene motor, cierra válvula y mantiene el enclavamiento.
 *   RECUPERACION_ESPERA → Espera las órdenes expresamente confirmadas.
 *   RECUPERANDO_FINAL → Mueve únicamente hacia PB1, sin generar una muestra.
 *
 * Sub-pasos del muestreo:
 *   CONFIRMAR      → Enviar MUESTREO_OK a superficie.
 *   INICIAR        → Abrir la electrovalvula y mover hacia posición final.
 *   ESPERAR_PB1    → Aguardar fin de carrera posición final (PB1).
 *
 * Jog de DESCARGA (sin sub-pasos, es reactivo a comandos):
 *   CMD:AVANZAR     → Abre la valvula y ejecuta MOTOR_SUBIR(); se corta
 *                     si se activa PB0 (posicion INICIAL).
 *   CMD:RETROCEDER  → Abre la valvula y ejecuta MOTOR_BAJAR(); se corta
 *                     si se activa PB1 (posicion FINAL).
 *   CMD:DETENER     → MOTOR_STOP().
 *   CMD:VALVULA_ABRIR / CMD:VALVULA_CERRAR → control manual de PA6.
 *   CMD:ARMAR       → aborta el jog y pasa a ARMANDO.
 *
 * Parada y recuperacion:
 *   CMD:EMERGENCIA       → detiene motor, cierra valvula y enclava el estado.
 *   CMD:RESET_EMERGENCIA → habilita la recuperacion, todavia sin movimiento.
 *   CMD:RECUPERAR_FINAL  → abre valvula y mueve solo hasta PB1.
 *   CMD:ARMAR            → se acepta despues de haber verificado PB1.
 *
 *   NOTA: si en el hardware real "avanzar" físicamente corresponde a
 *   subir en vez de bajar, alcanza con intercambiar MOTOR_BAJAR()/
 *   MOTOR_SUBIR() dentro del case STATE_DESCARGA.
 *
 * Sub-pasos del armado:
 *   SUBIR          → Activar motor hacia posición inicial.
 *   ESPERAR_PB0    → Aguardar fin de carrera posición inicial (PB0).
 *   CONFIRMAR_ARM  → Enviar ARMADO_OK a superficie.
 */
typedef enum
{
    STATE_ESPERA = 0,
    STATE_MUESTREO,
    STATE_MUESTREO_COMPLETO,
    STATE_DESCARGA,
    STATE_ARMANDO,
    STATE_EMERGENCIA,
    STATE_RECUPERACION_ESPERA,
    STATE_RECUPERANDO_FINAL
} SubState_t;
typedef enum
{
    STEP_MUE_CONFIRMAR = 0,
    STEP_MUE_INICIAR,
    STEP_MUE_ESPERAR_PB1
} MuestreoStep_t;
typedef enum
{
    STEP_ARM_SUBIR = 0,
    STEP_ARM_ESPERAR_PB0,
    STEP_ARM_CONFIRMAR
} ArmadoStep_t;
typedef enum
{
    JOG_NONE = 0,
    JOG_AVANZANDO,
    JOG_RETROCEDIENDO
} JogDir_t;

typedef struct
{
    float pressure_bar;
    float temperature_c;
    float depth_m;
} KellerMeasurement_t;
/* USER CODE END PTD */
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ---- Protocolo UART — igual que en el módulo superficie ---- */
#define CMD_PING              "PING"
#define CMD_STATUS            "STATUS"
#define CMD_SENSOR            "SENSOR"
#define CMD_INICIAR_MUESTREO  "CMD:MUESTREO"
#define CMD_DESCARGA          "CMD:DESCARGA"     /* Entrar en modo jog de descarga */
#define CMD_AVANZAR           "CMD:AVANZAR"      /* Jog: mover vastago (ver nota arriba) */
#define CMD_RETROCEDER        "CMD:RETROCEDER"   /* Jog: mover vastago en sentido opuesto */
#define CMD_DETENER           "CMD:DETENER"      /* Jog: detener motor                    */
#define CMD_VALVULA_ABRIR     "CMD:VALVULA_ABRIR"
#define CMD_VALVULA_CERRAR    "CMD:VALVULA_CERRAR"
#define CMD_ARMAR             "CMD:ARMAR"
#define CMD_EMERGENCIA        "CMD:EMERGENCIA"
#define CMD_RESET_EMERGENCIA  "CMD:RESET_EMERGENCIA"
#define CMD_RECUPERAR_FINAL   "CMD:RECUPERAR_FINAL"
#define RSP_ACK               "ACK\r\n"
#define RSP_VASTAGO_INIT      "VASTAGO:INIT\r\n"
#define RSP_VASTAGO_FINAL     "VASTAGO:FINAL\r\n"
#define RSP_VASTAGO_INTERMEDIO "VASTAGO:INTERMEDIO\r\n"
#define RSP_MUESTREO_OK       "MUESTREO_OK\r\n"
#define RSP_MUESTREO_DONE     "MUESTREO_DONE\r\n"
#define RSP_MODO_DESCARGA     "MODO:DESCARGA\r\n"     /* Ack de entrada a modo jog       */
#define RSP_JOG_AVANZANDO     "JOG:AVANZANDO\r\n"
#define RSP_JOG_RETROCEDIENDO "JOG:RETROCEDIENDO\r\n"
#define RSP_JOG_DETENIDO      "JOG:DETENIDO\r\n"
#define RSP_VALVULA_ABIERTA   "VALVULA:ABIERTA\r\n"
#define RSP_VALVULA_CERRADA   "VALVULA:CERRADA\r\n"
#define RSP_LIMITE_INICIAL    "LIMITE:INICIAL\r\n"    /* Corte automatico por fin de carrera */
#define RSP_LIMITE_FINAL      "LIMITE:FINAL\r\n"
#define RSP_MUESTREO_TIMEOUT  "MUESTREO_ERROR:TIMEOUT\r\n"
#define RSP_JOG_TIMEOUT       "JOG_ERROR:TIMEOUT\r\n"
#define RSP_ARMADO_OK         "ARMADO_OK\r\n"
#define RSP_ARMADO_TIMEOUT    "ARMADO_ERROR:TIMEOUT\r\n"
#define RSP_ARMADO_FINALES    "ARMADO_ERROR:FINALES_INCOMPATIBLES\r\n"
#define RSP_EMERGENCIA_ACTIVA "EMERGENCIA:ACTIVA\r\n"
#define RSP_EMERGENCIA_LIBERADA "EMERGENCIA:LIBERADA\r\n"
#define RSP_RECUPERACION_INICIADA "RECUPERACION:INICIADA\r\n"
#define RSP_RECUPERACION_FINAL_OK "RECUPERACION:FINAL_OK\r\n"
#define RSP_RECUPERACION_TIMEOUT "RECUPERACION_ERROR:TIMEOUT\r\n"
#define RSP_RECUPERACION_FINALES "RECUPERACION_ERROR:FINALES_INCOMPATIBLES\r\n"
#define RSP_ARMADO_FINAL_REQUERIDO "ARMADO_ERROR:FINAL_REQUERIDO\r\n"
/* ---- Tiempos (ms) ---- */
#define TIMEOUT_MOTOR_MS      28400U   /* 26,4 s teoricos de recorrido + margen */

/* ---- Keller 4LD / Bar30XT por I2C ---- */
#define KELLER_I2C_ADDRESS          (0x40U << 1) /* HAL usa direccion desplazada */
#define KELLER_CMD_MEASURE          0xACU
#define KELLER_REG_IDENTIFICATION   0x00U
#define KELLER_REG_MODE             0x12U
#define KELLER_REG_PMIN_MSW         0x13U
#define KELLER_REG_PMIN_LSW         0x14U
#define KELLER_REG_PMAX_MSW         0x15U
#define KELLER_REG_PMAX_LSW         0x16U
#define KELLER_STATUS_READY         0x40U
#define KELLER_STATUS_BUSY          0x20U
#define KELLER_STATUS_VALID_MASK    0xDCU
#define KELLER_I2C_TIMEOUT_MS       100U

/* El protocolo requiere al menos 8 ms para medir. Se dejan 15 ms para
 * tolerar tambien el tiempo de respuesta del emulador ESP32. */
#define KELLER_MEMORY_WAIT_MS       10U
#define KELLER_MEASURE_WAIT_MS      15U

#define SENSOR_POLL_PERIOD_MS       500U
#define SENSOR_RETRY_PERIOD_MS      500U
#define WATER_DENSITY_KG_M3         997.0f
#define GRAVITY_M_S2                9.80665f
#define ATM_PRESSURE_PA             101325.0f
/* ---- GPIO: Finales de carrera (entradas GPIOB) ----
 *   PB0  -> Fin de carrera posicion INICIAL del vastago
 *   PB1  -> Fin de carrera posicion FINAL  del vastago
 *   PB10 -> Futuro sensor efecto Hall (reservado)
 */
#define FC_INIT_PORT          GPIOB
#define FC_INIT_PIN           GPIO_PIN_0
#define FC_FINAL_PORT         GPIOB
#define FC_FINAL_PIN          GPIO_PIN_1
#define HALL_PORT             GPIOB
#define HALL_PIN              GPIO_PIN_10
/* ---- GPIO del motor y PWM de la electrovalvula ----
 *
 *   Motor vastago  -> IN1=PA4, IN2=PA5
 *     PA4=1, PA5=0 -> BAJA (hacia posicion final)
 *     PA4=0, PA5=1 -> SUBE (hacia posicion inicial)
 *     PA4=0, PA5=0 -> STOP
 *
 *   Electrovalvula -> PA6 = TIM3_CH1, salida PWM hacia el D514
 *     duty 100 % -> ABIERTA
 *     duty   0 % -> CERRADA
 *
 *   NOTA: ajustar polaridades segun el cableado real del L298N.
 */
#define MOTOR_IN1_PORT        GPIOA
#define MOTOR_IN1_PIN         GPIO_PIN_4
#define MOTOR_IN2_PORT        GPIOA
#define MOTOR_IN2_PIN         GPIO_PIN_5
#define VALVULA_PWM_PSC       71U
#define VALVULA_PWM_ARR       999U
#define VALVULA_DUTY_ABIERTA  100U
/* Macros de accionamiento */
#define MOTOR_BAJAR()  do { HAL_GPIO_WritePin(MOTOR_IN1_PORT, MOTOR_IN1_PIN, GPIO_PIN_SET);   \
                            HAL_GPIO_WritePin(MOTOR_IN2_PORT, MOTOR_IN2_PIN, GPIO_PIN_RESET); } while(0)
#define MOTOR_SUBIR()  do { HAL_GPIO_WritePin(MOTOR_IN1_PORT, MOTOR_IN1_PIN, GPIO_PIN_RESET); \
                            HAL_GPIO_WritePin(MOTOR_IN2_PORT, MOTOR_IN2_PIN, GPIO_PIN_SET);   } while(0)
#define MOTOR_STOP()   do { HAL_GPIO_WritePin(MOTOR_IN1_PORT, MOTOR_IN1_PIN, GPIO_PIN_RESET); \
                            HAL_GPIO_WritePin(MOTOR_IN2_PORT, MOTOR_IN2_PIN, GPIO_PIN_RESET); } while(0)
#define VALVULA_ABRIR()  Electrovalvula_Abrir()
#define VALVULA_CERRAR() Electrovalvula_Cerrar()
/* Lectura de finales de carrera — activo alto (ajustar si son activo bajo) */
#define FC_INIT_ACTIVO()   (HAL_GPIO_ReadPin(FC_INIT_PORT,  FC_INIT_PIN)  == GPIO_PIN_SET)
#define FC_FINAL_ACTIVO()  (HAL_GPIO_ReadPin(FC_FINAL_PORT, FC_FINAL_PIN) == GPIO_PIN_SET)
/* ---- Buffer UART1: comandos desde superficie ---- */
#define RX1_BUF_SIZE  64
/* USER CODE END PD */
/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */
/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart1;
/* USER CODE BEGIN PV */
/* ---- Recepcion USART1 (comandos de la superficie) byte a byte ---- */
static uint8_t rx1_byte;
static char    rx1_buf[RX1_BUF_SIZE];
static uint8_t rx1_idx        = 0U;
static volatile uint8_t rx1_line_ready = 0U;
static char    rx1_line[RX1_BUF_SIZE];

/* ---- Estado del Bar30XT / Keller 4LD ---- */
static float    g_keller_p_min = 0.0f;
static float    g_keller_p_max = 30.0f;
static float    g_keller_p_mode = 1.0f;
static uint8_t  g_keller_ready = 0U;
static uint32_t g_sensor_last_poll = 0U;
static uint32_t g_sensor_last_retry = 0U;
/* ---- Estado de la maquina ---- */
static SubState_t     g_estado    = STATE_ESPERA;
static MuestreoStep_t g_step_mue  = STEP_MUE_CONFIRMAR;
static ArmadoStep_t   g_step_arm  = STEP_ARM_SUBIR;
static JogDir_t        g_jog_dir  = JOG_NONE;   /* Direccion activa del jog en DESCARGA */
static uint32_t       g_ts_paso   = 0U;
static uint8_t        g_emergencia_latched = 0U;
static uint8_t        g_armado_emergencia  = 0U;
static uint8_t        g_recuperacion_final_ok = 0U;
static uint8_t        g_muestreo_por_timeout = 0U;
/* USER CODE END PV */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
static void    SM_Run(void);
static void    SM_SetState(SubState_t nuevo);
static void    UART1_Send(const char *msg);
static uint8_t UART1_CheckCmd(const char *cmd);
static void    UART1_ClearLine(void);
static void    Electrovalvula_PWM_Init(void);
static void    Electrovalvula_SetDuty(uint8_t duty_percent);
static void    Electrovalvula_Abrir(void);
static void    Electrovalvula_Cerrar(void);
static void    Actuadores_Stop(void);
static HAL_StatusTypeDef Keller_Init(void);
static HAL_StatusTypeDef Keller_ReadMemoryWord(uint8_t address, uint16_t *word);
static HAL_StatusTypeDef Keller_ReadMeasurement(KellerMeasurement_t *measurement);
static HAL_StatusTypeDef Sensor_ReadAndSend(void);
static void    Sensor_Service(uint32_t now);
static void    FormatFixed2(char *buffer, size_t buffer_size, float value);
/* USER CODE END PFP */
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void UART1_Send(const char *msg)
{
    UART_SendString(&huart1, msg);
}
static uint8_t UART1_CheckCmd(const char *cmd)
{
    return (strstr(rx1_line, cmd) != NULL) ? 1U : 0U;
}
static void UART1_ClearLine(void)
{
    memset(rx1_line, 0, sizeof(rx1_line));
    rx1_line_ready = 0U;
}

/* --------------------------------------------------------------------------
 * Electrovalvula D514 por PWM en PA6 / TIM3_CH1.
 *
 * Con el reloj actual:
 *   TIM3CLK = 72 MHz
 *   PSC      = 71   -> contador a 1 MHz
 *   ARR      = 999  -> PWM de 1 kHz
 * -------------------------------------------------------------------------- */
static void Electrovalvula_PWM_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();

    /* PA6 como salida alternativa push-pull de TIM3_CH1. */
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Detener y configurar TIM3 antes de habilitar la salida. */
    TIM3->CR1 = 0U;
    TIM3->PSC = VALVULA_PWM_PSC;
    TIM3->ARR = VALVULA_PWM_ARR;
    TIM3->CCR1 = 0U;

    /* Canal 1 en PWM mode 1, con preload habilitado. */
    TIM3->CCMR1 &= ~(TIM_CCMR1_CC1S | TIM_CCMR1_OC1M);
    TIM3->CCMR1 |= TIM_CCMR1_OC1PE |
                   TIM_CCMR1_OC1M_1 |
                   TIM_CCMR1_OC1M_2;

    /* Salida activa en alto por PA6. */
    TIM3->CCER &= ~TIM_CCER_CC1P;
    TIM3->CCER |= TIM_CCER_CC1E;
    TIM3->CR1 |= TIM_CR1_ARPE;

    /* Cargar PSC/ARR/CCR y arrancar el temporizador con duty 0 %. */
    TIM3->EGR = TIM_EGR_UG;
    TIM3->CR1 |= TIM_CR1_CEN;
}

static void Electrovalvula_SetDuty(uint8_t duty_percent)
{
    uint32_t period_counts;
    uint32_t compare;

    if (duty_percent > 100U)
    {
        duty_percent = 100U;
    }

    period_counts = VALVULA_PWM_ARR + 1U;
    compare = (period_counts * duty_percent) / 100U;

    /* CCR1=ARR+1 mantiene PA6 continuamente alto para duty=100 %. */
    TIM3->CCR1 = compare;
}

static void Electrovalvula_Abrir(void)
{
    Electrovalvula_SetDuty(VALVULA_DUTY_ABIERTA);
}

static void Electrovalvula_Cerrar(void)
{
    Electrovalvula_SetDuty(0U);
}

static uint8_t Keller_StatusIsValid(uint8_t status)
{
    return ((status & KELLER_STATUS_VALID_MASK) == KELLER_STATUS_READY) ? 1U : 0U;
}

static HAL_StatusTypeDef Keller_ReadMemoryWord(uint8_t address, uint16_t *word)
{
    uint8_t response[3];
    HAL_StatusTypeDef result;

    if (word == NULL)
    {
        return HAL_ERROR;
    }

    result = HAL_I2C_Master_Transmit(&hi2c1,
                                     KELLER_I2C_ADDRESS,
                                     &address,
                                     1U,
                                     KELLER_I2C_TIMEOUT_MS);
    if (result != HAL_OK)
    {
        return result;
    }

    HAL_Delay(KELLER_MEMORY_WAIT_MS);

    result = HAL_I2C_Master_Receive(&hi2c1,
                                    KELLER_I2C_ADDRESS,
                                    response,
                                    sizeof(response),
                                    KELLER_I2C_TIMEOUT_MS);
    if (result != HAL_OK)
    {
        return result;
    }

    if (!Keller_StatusIsValid(response[0]))
    {
        return HAL_ERROR;
    }

    *word = ((uint16_t)response[1] << 8) | response[2];
    return HAL_OK;
}

static HAL_StatusTypeDef Keller_Init(void)
{
    uint16_t identification;
    uint16_t mode_word;
    uint16_t p_min_msw;
    uint16_t p_min_lsw;
    uint16_t p_max_msw;
    uint16_t p_max_lsw;
    uint32_t raw_float;
    float p_min;
    float p_max;
    uint8_t mode;

    if (HAL_I2C_IsDeviceReady(&hi2c1,
                              KELLER_I2C_ADDRESS,
                              3U,
                              KELLER_I2C_TIMEOUT_MS) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (Keller_ReadMemoryWord(KELLER_REG_IDENTIFICATION, &identification) != HAL_OK ||
        Keller_ReadMemoryWord(KELLER_REG_MODE, &mode_word) != HAL_OK ||
        Keller_ReadMemoryWord(KELLER_REG_PMIN_MSW, &p_min_msw) != HAL_OK ||
        Keller_ReadMemoryWord(KELLER_REG_PMIN_LSW, &p_min_lsw) != HAL_OK ||
        Keller_ReadMemoryWord(KELLER_REG_PMAX_MSW, &p_max_msw) != HAL_OK ||
        Keller_ReadMemoryWord(KELLER_REG_PMAX_LSW, &p_max_lsw) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* El codigo de equipo 63 indica que no se identifico un Keller valido. */
    if ((identification >> 10) == 63U)
    {
        return HAL_ERROR;
    }

    raw_float = ((uint32_t)p_min_msw << 16) | p_min_lsw;
    memcpy(&p_min, &raw_float, sizeof(p_min));

    raw_float = ((uint32_t)p_max_msw << 16) | p_max_lsw;
    memcpy(&p_max, &raw_float, sizeof(p_max));

    if (!(p_max > p_min) || p_min < -1000.0f || p_max > 1000.0f)
    {
        return HAL_ERROR;
    }

    mode = (uint8_t)(mode_word & 0x03U);
    if (mode == 0U)
    {
        g_keller_p_mode = 1.01325f; /* PR: referencia atmosferica */
    }
    else if (mode == 1U)
    {
        g_keller_p_mode = 1.0f;     /* PA: referencia sellada de 1 bar */
    }
    else
    {
        g_keller_p_mode = 0.0f;     /* PAA: presion absoluta */
    }

    g_keller_p_min = p_min;
    g_keller_p_max = p_max;
    return HAL_OK;
}

static HAL_StatusTypeDef Keller_ReadMeasurement(KellerMeasurement_t *measurement)
{
    uint8_t command = KELLER_CMD_MEASURE;
    uint8_t response[5];
    uint16_t pressure_raw;
    uint16_t temperature_raw;
    int32_t temperature_code;
    HAL_StatusTypeDef result;

    if (measurement == NULL)
    {
        return HAL_ERROR;
    }

    result = HAL_I2C_Master_Transmit(&hi2c1,
                                     KELLER_I2C_ADDRESS,
                                     &command,
                                     1U,
                                     KELLER_I2C_TIMEOUT_MS);
    if (result != HAL_OK)
    {
        return result;
    }

    HAL_Delay(KELLER_MEASURE_WAIT_MS);

    result = HAL_I2C_Master_Receive(&hi2c1,
                                    KELLER_I2C_ADDRESS,
                                    response,
                                    sizeof(response),
                                    KELLER_I2C_TIMEOUT_MS);
    if (result != HAL_OK)
    {
        return result;
    }

    if (!Keller_StatusIsValid(response[0]))
    {
        return HAL_ERROR;
    }
    if ((response[0] & KELLER_STATUS_BUSY) != 0U)
    {
        return HAL_BUSY;
    }

    pressure_raw = ((uint16_t)response[1] << 8) | response[2];
    temperature_raw = ((uint16_t)response[3] << 8) | response[4];

    measurement->pressure_bar =
        ((float)((int32_t)pressure_raw - 16384) *
         (g_keller_p_max - g_keller_p_min) / 32768.0f) +
        g_keller_p_min + g_keller_p_mode;

    temperature_code = (int32_t)(temperature_raw >> 4) - 24;
    measurement->temperature_c =
        ((float)temperature_code * 0.05f) - 50.0f;

    measurement->depth_m =
        ((measurement->pressure_bar * 100000.0f) - ATM_PRESSURE_PA) /
        (WATER_DENSITY_KG_M3 * GRAVITY_M_S2);

    /* Evita transmitir -0.00 m por la cuantizacion del Bar30XT. */
    if (measurement->depth_m < 0.0f)
    {
        measurement->depth_m = 0.0f;
    }

    return HAL_OK;
}

static void FormatFixed2(char *buffer, size_t buffer_size, float value)
{
    int32_t scaled;
    uint32_t magnitude;

    scaled = (int32_t)((value >= 0.0f) ?
                       (value * 100.0f + 0.5f) :
                       (value * 100.0f - 0.5f));

    if (scaled < 0)
    {
        magnitude = (uint32_t)(-scaled);
        (void)snprintf(buffer,
                       buffer_size,
                       "-%lu.%02lu",
                       (unsigned long)(magnitude / 100U),
                       (unsigned long)(magnitude % 100U));
    }
    else
    {
        magnitude = (uint32_t)scaled;
        (void)snprintf(buffer,
                       buffer_size,
                       "%lu.%02lu",
                       (unsigned long)(magnitude / 100U),
                       (unsigned long)(magnitude % 100U));
    }
}

static HAL_StatusTypeDef Sensor_ReadAndSend(void)
{
    KellerMeasurement_t measurement;
    char temperature_text[16];
    char depth_text[16];
    char telemetry[40];
    HAL_StatusTypeDef result;

    if (!g_keller_ready)
    {
        return HAL_ERROR;
    }

    result = Keller_ReadMeasurement(&measurement);
    if (result != HAL_OK)
    {
        g_keller_ready = 0U;
        return result;
    }

    FormatFixed2(temperature_text, sizeof(temperature_text), measurement.temperature_c);
    FormatFixed2(depth_text, sizeof(depth_text), measurement.depth_m);

    /* Formato exacto esperado por el nodo de superficie. */
    (void)snprintf(telemetry,
                   sizeof(telemetry),
                   "T=%sD=%s\r\n",
                   temperature_text,
                   depth_text);
    UART1_Send(telemetry);
    return HAL_OK;
}

static void Sensor_Service(uint32_t now)
{
    if (!g_keller_ready)
    {
        if ((now - g_sensor_last_retry) >= SENSOR_RETRY_PERIOD_MS)
        {
            g_sensor_last_retry = now;
            g_keller_ready = (Keller_Init() == HAL_OK) ? 1U : 0U;
        }
        return;
    }

    if ((now - g_sensor_last_poll) >= SENSOR_POLL_PERIOD_MS)
    {
        g_sensor_last_poll = now;
        if (Sensor_ReadAndSend() != HAL_OK)
        {
            g_sensor_last_retry = now;
        }
    }
}
static void Actuadores_Stop(void)
{
    MOTOR_STOP();
    VALVULA_CERRAR();
}
static void SM_SetState(SubState_t nuevo)
{
    g_estado   = nuevo;
    g_step_mue = STEP_MUE_CONFIRMAR;
    g_step_arm = STEP_ARM_SUBIR;
    g_jog_dir  = JOG_NONE;
    g_ts_paso  = HAL_GetTick();
    UART1_ClearLine();
}
/* --------------------------------------------------------------------------
 * SM_Run — tick de la maquina de estados.
 * -------------------------------------------------------------------------- */
static void SM_Run(void)
{
    uint32_t ahora = HAL_GetTick();

    /* La parada tiene prioridad absoluta sobre cualquier otro comando y sobre
     * todos los pasos de motor. Queda enclavada hasta completar la secuencia
     * final -> confirmacion -> armado. */
    if (rx1_line_ready && UART1_CheckCmd(CMD_EMERGENCIA))
    {
        Actuadores_Stop();
        g_emergencia_latched = 1U;
        g_armado_emergencia  = 0U;
        g_recuperacion_final_ok = 0U;
        SM_SetState(STATE_EMERGENCIA);
        UART1_Send(RSP_EMERGENCIA_ACTIVA);
        return;
    }

    /* ================================================================
     * PING, STATUS y SENSOR se atienden sin cambiar el estado activo
     * la operacion en curso.
     * ================================================================ */
    if (rx1_line_ready)
    {
        if (UART1_CheckCmd(CMD_PING))
        {
            /* Un PING durante una recuperacion indica que la superficie pudo
             * reiniciarse. Detener antes de aceptar una nueva sesion. */
            if (g_emergencia_latched &&
                ((g_estado == STATE_RECUPERANDO_FINAL) ||
                 ((g_estado == STATE_ARMANDO) && g_armado_emergencia)))
            {
                Actuadores_Stop();
                g_armado_emergencia = 0U;
                g_recuperacion_final_ok = 0U;
                SM_SetState(STATE_EMERGENCIA);
            }
            UART1_Send(RSP_ACK);
            UART1_ClearLine();
            return;
        }
        if (UART1_CheckCmd(CMD_STATUS))
        {
            if (g_emergencia_latched)
            {
                /* STATUS no puede sacar al equipo de la parada. Si proviene de
                 * una superficie reiniciada, cancelar antes cualquier marcha. */
                if ((g_estado == STATE_RECUPERANDO_FINAL) ||
                    ((g_estado == STATE_ARMANDO) && g_armado_emergencia))
                {
                    Actuadores_Stop();
                    g_armado_emergencia = 0U;
                    g_recuperacion_final_ok = 0U;
                    SM_SetState(STATE_EMERGENCIA);
                }
                UART1_Send(RSP_EMERGENCIA_ACTIVA);
            }
            else if (FC_INIT_ACTIVO() && FC_FINAL_ACTIVO())
                UART1_Send("ERROR:FINALES_INCOMPATIBLES\r\n");
            else if (FC_INIT_ACTIVO())
                UART1_Send(RSP_VASTAGO_INIT);
            else if (FC_FINAL_ACTIVO())
                UART1_Send(RSP_VASTAGO_FINAL);
            else
                UART1_Send(RSP_VASTAGO_INTERMEDIO);
            UART1_ClearLine();
            return;
        }
        if (UART1_CheckCmd(CMD_SENSOR))
        {
            /* La superficie solicita esta lectura al entrar en Sistema
             * operando. Si el sensor no estaba listo durante el arranque,
             * intentar inicializarlo aquí; la superficie volverá a solicitar
             * cada 500 ms hasta recibir T=...D=.... */
            UART1_ClearLine();

            if (!g_keller_ready)
            {
                g_keller_ready = (Keller_Init() == HAL_OK) ? 1U : 0U;
                g_sensor_last_retry = ahora;
            }

            if (g_keller_ready)
            {
                if (Sensor_ReadAndSend() == HAL_OK)
                {
                    g_sensor_last_poll = ahora;
                }
                else
                {
                    g_sensor_last_retry = ahora;
                }
            }
            return;
        }
    }
    /* ================================================================
     * MAQUINA DE ESTADOS
     * ================================================================ */
    switch (g_estado)
    {
    /* ----------------------------------------------------------------
     * ESPERA — Idle, atiende comandos de operacion.
     * ---------------------------------------------------------------- */
    case STATE_ESPERA:
        if (rx1_line_ready)
        {
            if (UART1_CheckCmd(CMD_INICIAR_MUESTREO))
            {
                UART1_ClearLine();
                g_muestreo_por_timeout = 0U;
                SM_SetState(STATE_MUESTREO);
            }
            else if (UART1_CheckCmd(CMD_DESCARGA))
            {
                UART1_ClearLine();
                Actuadores_Stop();
                SM_SetState(STATE_DESCARGA);
                UART1_Send(RSP_MODO_DESCARGA);
            }
            else if (UART1_CheckCmd(CMD_ARMAR))
            {
                UART1_ClearLine();
                g_armado_emergencia = 0U;
                SM_SetState(STATE_ARMANDO);
            }
            else
            {
                UART1_ClearLine();
            }
        }
        /* Consultar periodicamente el Bar30XT por I2C y enviar la trama
         * T=XX.XXD=XX.XX por USART1. */
        Sensor_Service(ahora);
        break;
    /* ----------------------------------------------------------------
     * MUESTREO
     *   [0] CONFIRMAR   -> Enviar MUESTREO_OK.
     *   [1] INICIAR     -> Abrir valvula y mover hacia posicion final.
     *   [2] ESPERAR_PB1 -> Detener al activar el final PB1.
     * ---------------------------------------------------------------- */
    case STATE_MUESTREO:
        switch (g_step_mue)
        {
        case STEP_MUE_CONFIRMAR:
            UART1_Send(RSP_MUESTREO_OK);
            g_step_mue = STEP_MUE_INICIAR;
            g_ts_paso  = ahora;
            break;

        case STEP_MUE_INICIAR:
            /* La valvula debe estar abierta antes de mover el vastago. */
            VALVULA_ABRIR();

            if (FC_FINAL_ACTIVO())
            {
                MOTOR_STOP();
                g_muestreo_por_timeout = 0U;
                SM_SetState(STATE_MUESTREO_COMPLETO);
            }
            else
            {
                MOTOR_BAJAR();
                g_step_mue = STEP_MUE_ESPERAR_PB1;
                g_ts_paso  = ahora;
            }
            break;

        case STEP_MUE_ESPERAR_PB1:
            if (FC_FINAL_ACTIVO())
            {
                MOTOR_STOP();
                g_muestreo_por_timeout = 0U;
                SM_SetState(STATE_MUESTREO_COMPLETO);
            }
            else if ((ahora - g_ts_paso) >= TIMEOUT_MOTOR_MS)
            {
                /*
                 * Si no se detecta el final de carrera dentro del tiempo
                 * previsto, detener el mecanismo y cerrar igualmente el
                 * ciclo de muestreo.
                 */
                Actuadores_Stop();
                g_muestreo_por_timeout = 1U;
                SM_SetState(STATE_MUESTREO_COMPLETO);
            }
            break;

        default:
            Actuadores_Stop();
            SM_SetState(STATE_ESPERA);
            break;
        }
        break;

    /* ----------------------------------------------------------------
     * MUESTREO COMPLETO
     * Primero detiene el motor y cierra la electrovalvula. Solo despues
     * informa si termino mediante PB1 o por tiempo, para que la superficie
     * muestre y registre el resultado sin ocultar la falta del final.
     * ---------------------------------------------------------------- */
    case STATE_MUESTREO_COMPLETO:
        MOTOR_STOP();
        VALVULA_CERRAR();

        /* Conservar el envio de una medicion final sin impedir el cierre. */
        if (g_keller_ready)
        {
            (void)Sensor_ReadAndSend();
        }

        UART1_Send(g_muestreo_por_timeout ? RSP_MUESTREO_TIMEOUT
                                           : RSP_MUESTREO_DONE);
        g_muestreo_por_timeout = 0U;
        SM_SetState(STATE_ESPERA);
        break;
    /* ----------------------------------------------------------------
     * DESCARGA — modo jog manual.
     * El usuario controla avance/retroceso del vastago en vivo desde
     * la superficie. El corte por fin de carrera es siempre automatico
     * y tiene prioridad sobre cualquier comando en curso.
     * ---------------------------------------------------------------- */
    case STATE_DESCARGA:
        /* Vigilancia continua del fin de carrera mientras hay jog activo */
        if (g_jog_dir == JOG_AVANZANDO && FC_INIT_ACTIVO())
        {
            /* Al finalizar el movimiento también se cierra la válvula. */
            Actuadores_Stop();
            g_jog_dir = JOG_NONE;
            UART1_Send(RSP_LIMITE_INICIAL);
        }
        else if (g_jog_dir == JOG_RETROCEDIENDO && FC_FINAL_ACTIVO())
        {
            Actuadores_Stop();
            g_jog_dir = JOG_NONE;
            UART1_Send(RSP_LIMITE_FINAL);
        }
        else if (g_jog_dir != JOG_NONE &&
                 (ahora - g_ts_paso) >= TIMEOUT_MOTOR_MS)
        {
            Actuadores_Stop();
            g_jog_dir = JOG_NONE;
            UART1_Send(RSP_JOG_TIMEOUT);
        }

        if (rx1_line_ready)
        {
            if (UART1_CheckCmd(CMD_INICIAR_MUESTREO))
            {
                /* El nuevo menu permite volver a [Muestrear] despues de usar
                 * Vaciar/Llenar/Abrir. Salir del jog y comenzar el ciclo
                 * automatico sin exigir un armado intermedio. */
                UART1_ClearLine();
                Actuadores_Stop();
                SM_SetState(STATE_MUESTREO);
            }
            else if (UART1_CheckCmd(CMD_AVANZAR))
            {
                UART1_ClearLine();

                if (FC_INIT_ACTIVO())
                {
                    /* No abrir la válvula si el movimiento está bloqueado. */
                    Actuadores_Stop();
                    g_jog_dir = JOG_NONE;
                    UART1_Send(RSP_LIMITE_INICIAL);
                }
                else
                {
                    /* La válvula permanece abierta solamente mientras
                     * el motor está ejecutando el movimiento solicitado. */
                    VALVULA_ABRIR();
                    MOTOR_SUBIR();
                    g_jog_dir = JOG_AVANZANDO;
                    g_ts_paso = ahora;
                    UART1_Send(RSP_JOG_AVANZANDO);
                }
            }
            else if (UART1_CheckCmd(CMD_RETROCEDER))
            {
                UART1_ClearLine();

                if (FC_FINAL_ACTIVO())
                {
                    Actuadores_Stop();
                    g_jog_dir = JOG_NONE;
                    UART1_Send(RSP_LIMITE_FINAL);
                }
                else
                {
                    VALVULA_ABRIR();
                    MOTOR_BAJAR();
                    g_jog_dir = JOG_RETROCEDIENDO;
                    g_ts_paso = ahora;
                    UART1_Send(RSP_JOG_RETROCEDIENDO);
                }
            }
            else if (UART1_CheckCmd(CMD_DETENER))
            {
                UART1_ClearLine();
                /* La superficie envía este comando al soltar el botón:
                 * detener el motor y cerrar inmediatamente la válvula. */
                Actuadores_Stop();
                g_jog_dir = JOG_NONE;
                UART1_Send(RSP_JOG_DETENIDO);
            }
            else if (UART1_CheckCmd(CMD_VALVULA_ABRIR))
            {
                UART1_ClearLine();
                VALVULA_ABRIR();
                UART1_Send(RSP_VALVULA_ABIERTA);
            }
            else if (UART1_CheckCmd(CMD_VALVULA_CERRAR))
            {
                UART1_ClearLine();

                /* No permitir movimiento con la valvula cerrada. */
                MOTOR_STOP();
                g_jog_dir = JOG_NONE;
                VALVULA_CERRAR();

                UART1_Send(RSP_JOG_DETENIDO);
                UART1_Send(RSP_VALVULA_CERRADA);
            }
            else if (UART1_CheckCmd(CMD_ARMAR))
            {
                /* Salto directo a armado, confirmado ya en la superficie */
                UART1_ClearLine();
                Actuadores_Stop();
                g_armado_emergencia = 0U;
                SM_SetState(STATE_ARMANDO);
            }
            else
            {
                UART1_ClearLine();
            }
        }

        /* Mantener profundidad y temperatura actualizadas tambien mientras
         * el usuario utiliza las opciones manuales del menu. */
        if (g_estado == STATE_DESCARGA)
        {
            Sensor_Service(ahora);
        }
        break;
    /* ----------------------------------------------------------------
     * EMERGENCIA — actuadores detenidos y orden enclavada.
     * RESET_EMERGENCIA solo habilita el estado de recuperacion; el latch se
     * conserva hasta que el armado posterior termina correctamente.
     * ---------------------------------------------------------------- */
    case STATE_EMERGENCIA:
        Actuadores_Stop();

        if (rx1_line_ready)
        {
            if (UART1_CheckCmd(CMD_RESET_EMERGENCIA))
            {
                UART1_ClearLine();
                g_recuperacion_final_ok = 0U;
                SM_SetState(STATE_RECUPERACION_ESPERA);
                UART1_Send(RSP_EMERGENCIA_LIBERADA);
            }
            else
            {
                UART1_ClearLine();
            }
        }
        break;

    /* ----------------------------------------------------------------
     * RECUPERACION_ESPERA — parada aun enclavada, sin movimiento.
     * Solo admite ir a posicion final o armar una vez que la superficie lo
     * confirma. RESET es idempotente para tolerar reintentos de UART.
     * ---------------------------------------------------------------- */
    case STATE_RECUPERACION_ESPERA:
        Actuadores_Stop();

        if (rx1_line_ready)
        {
            if (UART1_CheckCmd(CMD_RESET_EMERGENCIA))
            {
                UART1_ClearLine();
                UART1_Send(RSP_EMERGENCIA_LIBERADA);
            }
            else if (UART1_CheckCmd(CMD_RECUPERAR_FINAL))
            {
                UART1_ClearLine();

                if (FC_INIT_ACTIVO() && FC_FINAL_ACTIVO())
                {
                    g_emergencia_latched = 1U;
                    g_recuperacion_final_ok = 0U;
                    SM_SetState(STATE_EMERGENCIA);
                    UART1_Send(RSP_RECUPERACION_FINALES);
                }
                else if (FC_FINAL_ACTIVO())
                {
                    /* Ya esta en el extremo requerido: no energizar motor. */
                    g_recuperacion_final_ok = 1U;
                    UART1_Send(RSP_RECUPERACION_FINAL_OK);
                }
                else
                {
                    g_recuperacion_final_ok = 0U;
                    SM_SetState(STATE_RECUPERANDO_FINAL);
                    VALVULA_ABRIR();
                    MOTOR_BAJAR();
                    UART1_Send(RSP_RECUPERACION_INICIADA);
                }
            }
            else if (UART1_CheckCmd(CMD_ARMAR))
            {
                UART1_ClearLine();
                if (!g_recuperacion_final_ok || !FC_FINAL_ACTIVO() ||
                    FC_INIT_ACTIVO())
                {
                    /* Defensa local: ni un comando atrasado puede omitir el
                     * paso obligatorio por el final de carrera. */
                    g_recuperacion_final_ok = 0U;
                    UART1_Send(RSP_ARMADO_FINAL_REQUERIDO);
                }
                else
                {
                    /* Solo se recibe despues de la segunda confirmacion visible. */
                    g_armado_emergencia = 1U;
                    SM_SetState(STATE_ARMANDO);
                }
            }
            else
            {
                UART1_ClearLine();
            }
        }
        break;

    /* ----------------------------------------------------------------
     * RECUPERANDO_FINAL — desplazamiento exclusivo de recuperacion.
     * No usa STATE_MUESTREO_COMPLETO y, por lo tanto, nunca emite DONE.
     * ---------------------------------------------------------------- */
    case STATE_RECUPERANDO_FINAL:
        if (FC_INIT_ACTIVO() && FC_FINAL_ACTIVO())
        {
            Actuadores_Stop();
            g_emergencia_latched = 1U;
            g_recuperacion_final_ok = 0U;
            SM_SetState(STATE_EMERGENCIA);
            UART1_Send(RSP_RECUPERACION_FINALES);
            break;
        }

        if (FC_FINAL_ACTIVO())
        {
            Actuadores_Stop();
            g_recuperacion_final_ok = 1U;
            SM_SetState(STATE_RECUPERACION_ESPERA);
            UART1_Send(RSP_RECUPERACION_FINAL_OK);
            break;
        }

        if ((ahora - g_ts_paso) >= TIMEOUT_MOTOR_MS)
        {
            Actuadores_Stop();
            g_emergencia_latched = 1U;
            g_recuperacion_final_ok = 0U;
            SM_SetState(STATE_EMERGENCIA);
            UART1_Send(RSP_RECUPERACION_TIMEOUT);
            break;
        }

        if (rx1_line_ready)
        {
            if (UART1_CheckCmd(CMD_RECUPERAR_FINAL))
            {
                /* Repeticion por perdida del ACK: no reiniciar el timeout. */
                UART1_ClearLine();
                UART1_Send(RSP_RECUPERACION_INICIADA);
            }
            else
            {
                UART1_ClearLine();
            }
        }
        break;

    /* ----------------------------------------------------------------
     * ARMANDO
     *   [0] SUBIR         -> Activar motor (subir vastago)
     *   [1] ESPERAR_PB0   -> Aguardar fin de carrera inicial
     *   [2] CONFIRMAR_ARM -> Enviar ARMADO_OK, volver a ESPERA
     * ---------------------------------------------------------------- */
    case STATE_ARMANDO:
        switch (g_step_arm)
        {
        case STEP_ARM_SUBIR:
            if (FC_INIT_ACTIVO() && FC_FINAL_ACTIVO())
            {
                Actuadores_Stop();
                UART1_Send(RSP_ARMADO_FINALES);
                if (g_armado_emergencia)
                {
                    g_armado_emergencia  = 0U;
                    g_emergencia_latched = 1U;
                    g_recuperacion_final_ok = 0U;
                    SM_SetState(STATE_EMERGENCIA);
                }
                else
                {
                    SM_SetState(STATE_ESPERA);
                }
            }
            else if (FC_INIT_ACTIVO())
            {
                Actuadores_Stop();
                g_step_arm = STEP_ARM_CONFIRMAR;
                g_ts_paso  = ahora;
            }
            else
            {
                /* En recuperacion se abre la valvula antes de invertir el
                 * recorrido, evitando mover contra presion atrapada. */
                if (g_armado_emergencia)
                {
                    VALVULA_ABRIR();
                }
                MOTOR_SUBIR();
                g_step_arm = STEP_ARM_ESPERAR_PB0;
                g_ts_paso  = ahora;
            }
            break;
        case STEP_ARM_ESPERAR_PB0:
            if (FC_INIT_ACTIVO() && FC_FINAL_ACTIVO())
            {
                Actuadores_Stop();
                UART1_Send(RSP_ARMADO_FINALES);
                if (g_armado_emergencia)
                {
                    g_armado_emergencia  = 0U;
                    g_emergencia_latched = 1U;
                    g_recuperacion_final_ok = 0U;
                    SM_SetState(STATE_EMERGENCIA);
                }
                else
                {
                    SM_SetState(STATE_ESPERA);
                }
            }
            else if (FC_INIT_ACTIVO())
            {
                Actuadores_Stop();
                g_step_arm = STEP_ARM_CONFIRMAR;
                g_ts_paso  = ahora;
            }
            else if ((ahora - g_ts_paso) >= TIMEOUT_MOTOR_MS)
            {
                /* Timeout: vastago no llego a posicion inicial */
                Actuadores_Stop();
                UART1_Send(RSP_ARMADO_TIMEOUT);
                if (g_armado_emergencia)
                {
                    g_armado_emergencia  = 0U;
                    g_emergencia_latched = 1U;
                    g_recuperacion_final_ok = 0U;
                    SM_SetState(STATE_EMERGENCIA);
                }
                else
                {
                    SM_SetState(STATE_ESPERA);
                }
            }
            break;
        case STEP_ARM_CONFIRMAR:
            Actuadores_Stop();
            UART1_Send(RSP_ARMADO_OK);
            if (g_armado_emergencia)
            {
                g_emergencia_latched = 0U;
            }
            g_armado_emergencia = 0U;
            g_recuperacion_final_ok = 0U;
            SM_SetState(STATE_ESPERA);
            break;
        default:
            Actuadores_Stop();
            if (g_armado_emergencia)
            {
                g_armado_emergencia  = 0U;
                g_emergencia_latched = 1U;
                g_recuperacion_final_ok = 0U;
                SM_SetState(STATE_EMERGENCIA);
            }
            else
            {
                SM_SetState(STATE_ESPERA);
            }
            break;
        }
        break;
    default:
        Actuadores_Stop();
        if (g_emergencia_latched)
            SM_SetState(STATE_EMERGENCIA);
        else
            SM_SetState(STATE_ESPERA);
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
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  /* Inicializar PWM de 1 kHz para la electrovalvula en PA6/TIM3_CH1. */
  Electrovalvula_PWM_Init();

  /* USART1 recibe los comandos del nodo de superficie. */
  HAL_UART_Receive_IT(&huart1, &rx1_byte, 1U);
  /* Garantizar actuadores apagados al iniciar */
  Actuadores_Stop();

  /* La inicializacion se vuelve a intentar desde Sensor_Service si el
   * emulador ESP32 todavia no esta listo durante el arranque. */
  g_keller_ready = (Keller_Init() == HAL_OK) ? 1U : 0U;
  g_sensor_last_poll = HAL_GetTick();
  g_sensor_last_retry = HAL_GetTick();
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    SM_Run();
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */
  /* USER CODE END I2C1_Init 2 */
}
/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
  /* USER CODE BEGIN USART1_Init 0 */
  /* USER CODE END USART1_Init 0 */
  /* USER CODE BEGIN USART1_Init 1 */
  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  /* Habilitar interrupción USART1 en el NVIC (necesario para Receive_IT) */
  HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE END USART1_Init 2 */
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA,
                    GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_7,
                    GPIO_PIN_RESET);
  /*Configure GPIO pins : PA4 PA5 PA7
   * PA6 se configura luego como TIM3_CH1 en Electrovalvula_PWM_Init(). */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}
/* USER CODE BEGIN 4 */
/**
  * @brief  Callback de recepción UART — llamado por la HAL tras recibir cada byte.
  *
  *         Acumula bytes en rx1_buf hasta recibir '\n'.
  *         Cuando la línea está completa la copia a rx1_line y activa
  *         el flag para que SM_Run la procese.
  *         Relanza inmediatamente la recepción del siguiente byte.
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    /* ---- USART1: comandos desde el módulo superficie ---- */
    if (huart->Instance == USART1)
    {
        if (rx1_byte == '\n')
        {
            rx1_buf[rx1_idx] = '\0';
            strncpy(rx1_line, rx1_buf, RX1_BUF_SIZE - 1U);
            rx1_line[RX1_BUF_SIZE - 1U] = '\0';
            rx1_line_ready = 1U;
            rx1_idx = 0U;
            memset(rx1_buf, 0, sizeof(rx1_buf));
        }
        else if (rx1_byte != '\r')
        {
            if (rx1_idx < RX1_BUF_SIZE - 1U)
                rx1_buf[rx1_idx++] = (char)rx1_byte;
            else
            {
                rx1_idx = 0U;
                memset(rx1_buf, 0, sizeof(rx1_buf));
            }
        }
        HAL_UART_Receive_IT(&huart1, &rx1_byte, 1U);
    }
}
/* USER CODE END 4 */
/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1) {}
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
