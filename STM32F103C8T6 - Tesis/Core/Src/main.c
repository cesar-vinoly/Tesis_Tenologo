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
 *   MUESTREO  → Secuencia de toma de muestra (varios pasos internos).
 *   DESCARGA  → Secuencia de descarga y limpieza.
 *   ARMANDO   → Mueve el vástago a posición inicial.
 *
 * Sub-pasos del muestreo:
 *   CONFIRMAR      → Enviar MUESTREO_OK a superficie.
 *   BAJAR          → Activar motor hacia posición final.
 *   ESPERAR_PB1    → Aguardar fin de carrera posición final (PB1).
 *   TOMAR_MUESTRA  → Abrir válvula y capturar dato del sensor.
 *   DONE           → Enviar MUESTREO_DONE a superficie.
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
    STATE_DESCARGA,
    STATE_ARMANDO
} SubState_t;

typedef enum
{
    STEP_MUE_CONFIRMAR = 0,
    STEP_MUE_BAJAR,
    STEP_MUE_ESPERAR_PB1,
    STEP_MUE_TOMAR_MUESTRA,
    STEP_MUE_DONE
} MuestreoStep_t;

typedef enum
{
    STEP_ARM_SUBIR = 0,
    STEP_ARM_ESPERAR_PB0,
    STEP_ARM_CONFIRMAR
} ArmadoStep_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ---- Protocolo UART — igual que en el módulo superficie ---- */
#define CMD_PING              "PING"
#define CMD_STATUS            "STATUS"
#define CMD_INICIAR_MUESTREO  "CMD:MUESTREO"
#define CMD_DESCARGA          "CMD:DESCARGA"
#define CMD_ARMAR             "CMD:ARMAR"

#define RSP_ACK               "ACK\r\n"
#define RSP_VASTAGO_INIT      "VASTAGO:INIT\r\n"
#define RSP_VASTAGO_FINAL     "VASTAGO:FINAL\r\n"
#define RSP_MUESTREO_OK       "MUESTREO_OK\r\n"
#define RSP_MUESTREO_DONE     "MUESTREO_DONE\r\n"
#define RSP_DESCARGA_OK       "DESCARGA_OK\r\n"
#define RSP_ARMADO_OK         "ARMADO_OK\r\n"

/* ---- Tiempos (ms) ---- */
#define TIMEOUT_MOTOR_MS      15000U   /* Tiempo max. para que el vastago llegue */
#define TIMEOUT_VALVULA_MS     5000U   /* Tiempo que permanece abierta la valvula */
#define TIMEOUT_DESCARGA_MS   10000U   /* Tiempo de descarga y limpieza           */
#define TIMEOUT_SENSOR_MS      3000U   /* Tiempo max. esperando dato del sensor   */

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

/* ---- GPIO: Salidas L298N (GPIOA) ----
 *
 *   Motor vastago  -> IN1=PA4, IN2=PA5
 *     PA4=1, PA5=0 -> BAJA (hacia posicion final)
 *     PA4=0, PA5=1 -> SUBE (hacia posicion inicial)
 *     PA4=0, PA5=0 -> STOP
 *
 *   Electrovalvula -> IN3=PA6, IN4=PA7
 *     PA6=1, PA7=0 -> ABIERTA
 *     PA6=0, PA7=0 -> CERRADA
 *
 *   NOTA: ajustar polaridades segun el cableado real del L298N.
 */
#define MOTOR_IN1_PORT        GPIOA
#define MOTOR_IN1_PIN         GPIO_PIN_4
#define MOTOR_IN2_PORT        GPIOA
#define MOTOR_IN2_PIN         GPIO_PIN_5

#define VALVULA_IN3_PORT      GPIOA
#define VALVULA_IN3_PIN       GPIO_PIN_6
#define VALVULA_IN4_PORT      GPIOA
#define VALVULA_IN4_PIN       GPIO_PIN_7

/* Macros de accionamiento */
#define MOTOR_BAJAR()  do { HAL_GPIO_WritePin(MOTOR_IN1_PORT, MOTOR_IN1_PIN, GPIO_PIN_SET);   \
                            HAL_GPIO_WritePin(MOTOR_IN2_PORT, MOTOR_IN2_PIN, GPIO_PIN_RESET); } while(0)

#define MOTOR_SUBIR()  do { HAL_GPIO_WritePin(MOTOR_IN1_PORT, MOTOR_IN1_PIN, GPIO_PIN_RESET); \
                            HAL_GPIO_WritePin(MOTOR_IN2_PORT, MOTOR_IN2_PIN, GPIO_PIN_SET);   } while(0)

#define MOTOR_STOP()   do { HAL_GPIO_WritePin(MOTOR_IN1_PORT, MOTOR_IN1_PIN, GPIO_PIN_RESET); \
                            HAL_GPIO_WritePin(MOTOR_IN2_PORT, MOTOR_IN2_PIN, GPIO_PIN_RESET); } while(0)

#define VALVULA_ABRIR()  do { HAL_GPIO_WritePin(VALVULA_IN3_PORT, VALVULA_IN3_PIN, GPIO_PIN_SET);   \
                              HAL_GPIO_WritePin(VALVULA_IN4_PORT, VALVULA_IN4_PIN, GPIO_PIN_RESET); } while(0)

#define VALVULA_CERRAR() do { HAL_GPIO_WritePin(VALVULA_IN3_PORT, VALVULA_IN3_PIN, GPIO_PIN_RESET); \
                              HAL_GPIO_WritePin(VALVULA_IN4_PORT, VALVULA_IN4_PIN, GPIO_PIN_RESET); } while(0)

/* Lectura de finales de carrera — activo alto (ajustar si son activo bajo) */
#define FC_INIT_ACTIVO()   (HAL_GPIO_ReadPin(FC_INIT_PORT,  FC_INIT_PIN)  == GPIO_PIN_SET)
#define FC_FINAL_ACTIVO()  (HAL_GPIO_ReadPin(FC_FINAL_PORT, FC_FINAL_PIN) == GPIO_PIN_SET)

/* ---- Buffers UART ---- */
#define RX1_BUF_SIZE  64    /* USART1: comandos desde superficie */
#define RX2_BUF_SIZE  128   /* USART2: datos del sensor          */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* ---- Recepcion USART1 (comandos de la superficie) byte a byte ---- */
static uint8_t rx1_byte;
static char    rx1_buf[RX1_BUF_SIZE];
static uint8_t rx1_idx        = 0U;
static uint8_t rx1_line_ready = 0U;
static char    rx1_line[RX1_BUF_SIZE];

/* ---- Recepcion USART2 (sensor / Arduino) byte a byte ---- */
static uint8_t rx2_byte;
static char    rx2_buf[RX2_BUF_SIZE];
static uint8_t rx2_idx        = 0U;
static uint8_t rx2_line_ready = 0U;
static char    rx2_line[RX2_BUF_SIZE];

/* ---- Estado de la maquina ---- */
static SubState_t     g_estado    = STATE_ESPERA;
static MuestreoStep_t g_step_mue  = STEP_MUE_CONFIRMAR;
static ArmadoStep_t   g_step_arm  = STEP_ARM_SUBIR;
static uint32_t       g_ts_paso   = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

static void    SM_Run(void);
static void    SM_SetState(SubState_t nuevo);
static void    UART1_Send(const char *msg);
static uint8_t UART1_CheckCmd(const char *cmd);
static void    UART1_ClearLine(void);
static void    UART2_ClearLine(void);
static void    Actuadores_Stop(void);

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

static void UART2_ClearLine(void)
{
    memset(rx2_line, 0, sizeof(rx2_line));
    rx2_line_ready = 0U;
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
    g_ts_paso  = HAL_GetTick();
    UART1_ClearLine();
}

/* --------------------------------------------------------------------------
 * SM_Run — tick de la maquina de estados.
 * -------------------------------------------------------------------------- */
static void SM_Run(void)
{
    uint32_t ahora = HAL_GetTick();

    /* ================================================================
     * PING y STATUS se atienden en CUALQUIER estado sin interrumpir
     * la operacion en curso.
     * ================================================================ */
    if (rx1_line_ready)
    {
        if (UART1_CheckCmd(CMD_PING))
        {
            UART1_Send(RSP_ACK);
            UART1_ClearLine();
            return;
        }

        if (UART1_CheckCmd(CMD_STATUS))
        {
            if (FC_INIT_ACTIVO())
                UART1_Send(RSP_VASTAGO_INIT);
            else if (FC_FINAL_ACTIVO())
                UART1_Send(RSP_VASTAGO_FINAL);
            else
                UART1_Send(RSP_VASTAGO_FINAL);   /* Default conservador: posicion final */
            UART1_ClearLine();
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
                SM_SetState(STATE_MUESTREO);
            }
            else if (UART1_CheckCmd(CMD_DESCARGA))
            {
                UART1_ClearLine();
                SM_SetState(STATE_DESCARGA);
            }
            else if (UART1_CheckCmd(CMD_ARMAR))
            {
                UART1_ClearLine();
                SM_SetState(STATE_ARMANDO);
            }
            else
            {
                UART1_ClearLine();
            }
        }

        /* Reenviar dato del sensor a la superficie en tiempo real.
         * Si llega un dato del Arduino (USART2) mientras el sistema
         * esta idle, se reenvía a la superficie para mostrarlo. */
        if (rx2_line_ready)
        {
            UART1_Send(rx2_line);
            UART1_Send("\r\n");
            UART2_ClearLine();
        }
        break;

    /* ----------------------------------------------------------------
     * MUESTREO
     *   [0] CONFIRMAR     -> Enviar MUESTREO_OK
     *   [1] BAJAR         -> Activar motor (bajar vastago)
     *   [2] ESPERAR_PB1   -> Aguardar fin de carrera final
     *   [3] TOMAR_MUESTRA -> Abrir valvula, capturar sensor
     *   [4] DONE          -> Enviar MUESTREO_DONE, volver a ESPERA
     * ---------------------------------------------------------------- */
    case STATE_MUESTREO:
        switch (g_step_mue)
        {
        case STEP_MUE_CONFIRMAR:
            UART1_Send(RSP_MUESTREO_OK);
            g_step_mue = STEP_MUE_BAJAR;
            g_ts_paso  = ahora;
            break;

        case STEP_MUE_BAJAR:
            MOTOR_BAJAR();
            g_step_mue = STEP_MUE_ESPERAR_PB1;
            g_ts_paso  = ahora;
            break;

        case STEP_MUE_ESPERAR_PB1:
            if (FC_FINAL_ACTIVO())
            {
                MOTOR_STOP();
                VALVULA_ABRIR();
                UART2_ClearLine();          /* Descartar datos viejos del sensor */
                g_step_mue = STEP_MUE_TOMAR_MUESTRA;
                g_ts_paso  = ahora;
            }
            else if ((ahora - g_ts_paso) >= TIMEOUT_MOTOR_MS)
            {
                /* Timeout: vastago no llego a posicion final */
                Actuadores_Stop();
                SM_SetState(STATE_ESPERA);
            }
            break;

        case STEP_MUE_TOMAR_MUESTRA:
            /*
             * Espera un dato del sensor (USART2).
             * Cuando llega lo reenvía a superficie via USART1.
             * Si vence el timeout cierra la valvula y avanza igual.
             */
            if (rx2_line_ready)
            {
                UART1_Send(rx2_line);
                UART1_Send("\r\n");
                UART2_ClearLine();
                VALVULA_CERRAR();
                g_step_mue = STEP_MUE_DONE;
                g_ts_paso  = ahora;
            }
            else if ((ahora - g_ts_paso) >= TIMEOUT_VALVULA_MS)
            {
                VALVULA_CERRAR();
                g_step_mue = STEP_MUE_DONE;
                g_ts_paso  = ahora;
            }
            break;

        case STEP_MUE_DONE:
            UART1_Send(RSP_MUESTREO_DONE);
            SM_SetState(STATE_ESPERA);
            break;

        default:
            Actuadores_Stop();
            SM_SetState(STATE_ESPERA);
            break;
        }
        break;

    /* ----------------------------------------------------------------
     * DESCARGA
     * Abre la valvula TIMEOUT_DESCARGA_MS para vaciar y limpiar,
     * luego responde DESCARGA_OK.
     * ---------------------------------------------------------------- */
    case STATE_DESCARGA:
        if ((ahora - g_ts_paso) < 10U)
        {
            /* Primera iteracion: abrir valvula */
            VALVULA_ABRIR();
        }
        else if ((ahora - g_ts_paso) >= TIMEOUT_DESCARGA_MS)
        {
            VALVULA_CERRAR();
            UART1_Send(RSP_DESCARGA_OK);
            SM_SetState(STATE_ESPERA);
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
            MOTOR_SUBIR();
            g_step_arm = STEP_ARM_ESPERAR_PB0;
            g_ts_paso  = ahora;
            break;

        case STEP_ARM_ESPERAR_PB0:
            if (FC_INIT_ACTIVO())
            {
                MOTOR_STOP();
                g_step_arm = STEP_ARM_CONFIRMAR;
                g_ts_paso  = ahora;
            }
            else if ((ahora - g_ts_paso) >= TIMEOUT_MOTOR_MS)
            {
                /* Timeout: vastago no llego a posicion inicial */
                Actuadores_Stop();
                SM_SetState(STATE_ESPERA);
            }
            break;

        case STEP_ARM_CONFIRMAR:
            UART1_Send(RSP_ARMADO_OK);
            SM_SetState(STATE_ESPERA);
            break;

        default:
            Actuadores_Stop();
            SM_SetState(STATE_ESPERA);
            break;
        }
        break;

    default:
        Actuadores_Stop();
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* Arrancar recepcion por interrupcion en ambas UARTs */
  HAL_UART_Receive_IT(&huart1, &rx1_byte, 1U);
  HAL_UART_Receive_IT(&huart2, &rx2_byte, 1U);

  /* Garantizar actuadores apagados al iniciar */
  Actuadores_Stop();

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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */
  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */
  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  /* Habilitar interrupción USART2 en el NVIC (necesario para Receive_IT) */
  HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE END USART2_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA4 PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10;
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
  *         Acumula bytes en rx1_buf / rx2_buf hasta recibir '\n'.
  *         Cuando la línea está completa la copia a rx1_line / rx2_line
  *         y activa el flag correspondiente para que SM_Run la procese.
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

    /* ---- USART2: datos del sensor / Arduino ---- */
    if (huart->Instance == USART2)
    {
        if (rx2_byte == '\n')
        {
            rx2_buf[rx2_idx] = '\0';
            strncpy(rx2_line, rx2_buf, RX2_BUF_SIZE - 1U);
            rx2_line[RX2_BUF_SIZE - 1U] = '\0';
            rx2_line_ready = 1U;
            rx2_idx = 0U;
            memset(rx2_buf, 0, sizeof(rx2_buf));
        }
        else if (rx2_byte != '\r')
        {
            if (rx2_idx < RX2_BUF_SIZE - 1U)
                rx2_buf[rx2_idx++] = (char)rx2_byte;
            else
            {
                rx2_idx = 0U;
                memset(rx2_buf, 0, sizeof(rx2_buf));
            }
        }
        HAL_UART_Receive_IT(&huart2, &rx2_byte, 1U);
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

