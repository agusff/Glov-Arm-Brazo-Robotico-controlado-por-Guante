/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* PIN MAP */
//
//Servos:
// Muñeca: PA0
// Pinza: PA1
// Codo: PA2
// Hombro: PA3

//Boton (con pull-up): PB12

//USART:
// Rx: PA10
// Tx: PA9

//Pantalla LCD (I2C)
// SDA: PB7
// SCL: PB6

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include "i2c_lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {				//Estructura para los servomotores
    TIM_HandleTypeDef *htim; // Puntero al Timer
    uint32_t Channel;        // Canal del PWM
    uint16_t Min_Pulse;      // Pulso mínimo real en microsegundos
    uint16_t Max_Pulse;      // Pulso máximo real en microsegundos
    // VARIABLES DE CONTROL DE TRAYECTORIA
    volatile uint16_t Current_Pulse;  // Posición actual del Duty Cycle
    volatile uint16_t Target_Pulse;   // Posición objetivo (Set-point)
    uint16_t Step_Size;      // Magnitud del paso (en microsegundos por actualización)
} ServoConfig_t;

// Máquina de estados para 4 servos y estado de conexion/desconexion
typedef enum {
    WAIT_START, WAIT_S1, WAIT_S2, WAIT_S3, WAIT_S4, WAIT_STATUS, WAIT_CHK, WAIT_END
} ProtocolState_t;

// LCD
typedef enum { // Estados de actulizacion para la LCD
    LCD_UNKNOWN,
    LCD_DESCONECTADO,
    LCD_CONECTADO,
    LCD_PRUEBA
} LcdState_t;

/* USER CODE END PV */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RING_BUFFER_SIZE  128

#define DEBOUNCE_TIME 50
#define LONG_PRESS_TIME 1500
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

extern I2C_HandleTypeDef hi2c1;
I2C_LCD_HandleTypeDef lcd1;
LcdState_t current_lcd_state = LCD_UNKNOWN; // estado actual de la pantalla

/* USER CODE BEGIN PV */
ServoConfig_t servo_mun = {
    .htim = &htim2,
    .Channel = TIM_CHANNEL_1,
    .Min_Pulse = 900,	//530
    .Max_Pulse = 2430,	//2430

    .Current_Pulse = 530,
    .Target_Pulse = 530,
    .Step_Size = 10
};

ServoConfig_t servo_pnza = {
    .htim = &htim2,
    .Channel = TIM_CHANNEL_2,
    .Min_Pulse = 500, //595
    .Max_Pulse = 1200,	//2445

    .Current_Pulse = 620,
    .Target_Pulse = 620,
    .Step_Size = 10
};

ServoConfig_t servo_cdo = {
    .htim = &htim2,
    .Channel = TIM_CHANNEL_3,
    .Min_Pulse = 800,	//530
    .Max_Pulse = 2480,	//2480

    .Current_Pulse = 530,
    .Target_Pulse = 530,
    .Step_Size = 10
};

ServoConfig_t servo_hmbro = {
    .htim = &htim2,
    .Channel = TIM_CHANNEL_4,
    .Min_Pulse = 700,  //540
    .Max_Pulse = 2200,	//2430

    .Current_Pulse = 540,
    .Target_Pulse = 540,
    .Step_Size = 10
};

// BANDERAS DE ESTADO DEL SISTEMA
uint8_t is_linked = 0;     // 0 = Desvinculado, 1 = Vinculado
uint8_t home_active = 0;   // 1 = Ejecutando rutina HOME, 0 = Inactivo

// VARIABLES DEL Btn
volatile uint32_t button_press_start_time = 0;
volatile uint8_t flag_short_press = 0;
volatile uint8_t flag_long_press = 0;

volatile ProtocolState_t fsm_state = WAIT_START;

// Variables de direcciones
volatile uint8_t target_pos[4] = {90, 90, 90, 90}; // Inicializa en posicion media

// Buffer y control DMA
uint8_t dma_rx_buffer[RING_BUFFER_SIZE];
uint16_t ring_buffer_head = 0;

volatile uint32_t tramas_exitosas =0;
volatile uint32_t errores_checksum = 0;

// CONTROL DE RETRANSMISION
uint8_t pending_link_request = 0; // 1 = Esperando respuesta del ESP32
uint8_t target_link_state = 0;    // El estado buscado
uint32_t last_tx_time = 0;        // delay para el reintento
volatile uint8_t rx_estado_espnow = 0;

static uint8_t tx_buffer[3];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void Run_Home_Routine(void);
void Servo_SetTargetAngle(ServoConfig_t *, uint8_t);
void Servo_ISR_Update(ServoConfig_t *);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  // INICIAMOS EL TIMER DE INTERRUPCIÓN
  HAL_TIM_Base_Start_IT(&htim3);


  // Limpiamos banderas por seguridad
    __HAL_UART_CLEAR_OREFLAG(&huart1);
    __HAL_UART_CLEAR_NEFLAG(&huart1);
    __HAL_UART_CLEAR_FEFLAG(&huart1);
    __HAL_UART_CLEAR_PEFLAG(&huart1);

  // Encendemos el DMA en modo infinito sobre el dma_rx_buffer
  HAL_UART_Receive_DMA(&huart1, dma_rx_buffer, RING_BUFFER_SIZE);

  // INICIALIZACIÓN DEL LCD
    lcd1.hi2c = &hi2c1;
    lcd1.address = 0x4E;  // Dirección I2C desplazada (0x27 << 1)

    lcd_init(&lcd1);      // Inicializa el LCD
    lcd_clear(&lcd1);     // Limpia basura en pantalla
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  	  	// ==========================================================
	        // GESTIÓN DE BOTÓN Y BANDERAS
	        // ==========================================================
	        if (flag_long_press)
	        {
	            flag_long_press = 0;

	            // Iniciamos la petición de cambio de estado
	            pending_link_request = 1;

	            if (!is_linked) {
	            	target_link_state = 1; //  VINCULAR
	            } else {
	            	target_link_state = 0; //  DESVINCULAR
	            }

	            last_tx_time = 0; // Forzamos a que el primer envio sea inmediato

	        }

	        if (flag_short_press)
	        {
	            flag_short_press = 0;
	            if (!is_linked && !home_active) {

	                home_active = 1;

	            }
	            else if (is_linked) {

	            }
	        }

	        // ==========================================================
	        // MOTOR DE HANDSHAKING
	        // ==========================================================
	        if (pending_link_request)
	        {
	        // Reintento de transmisión cada 500 ms
	        	if ((HAL_GetTick() - last_tx_time) >= 500)
	            {
	                last_tx_time = HAL_GetTick(); // Actualiza el reloj

	                tx_buffer[0] = 0xBB;
	                tx_buffer[1] = target_link_state; // 0x01 o 0x00
	                tx_buffer[2] = 0x55;

	                // Transmisión con un timeout estricto de 10ms
	                HAL_UART_Transmit(&huart1, tx_buffer, 3, 10);
	            }
	        }

	        // ==========================================================
	        // EJECUCIÓN DE RUTINA HOME
	        // ==========================================================
	        if (home_active)
	        {
	            Run_Home_Routine(); // Ejecuta pasos
	        }

	        // ==========================================================
	        // ACTUALIZACION DEL LCD
	        // ==========================================================
	        LcdState_t target_lcd_state;

	        if (home_active) {
	            target_lcd_state = LCD_PRUEBA;
	        } else if (is_linked) {
	            target_lcd_state = LCD_CONECTADO;
	        } else {
	            target_lcd_state = LCD_DESCONECTADO;
	        }

	        // Solo transmite por I2C si el estado cambio
	        if (target_lcd_state != current_lcd_state) {

	            current_lcd_state = target_lcd_state;

	            lcd_clear(&lcd1);
	            lcd_gotoxy(&lcd1, 0, 0);
	            lcd_puts(&lcd1, "GLOV-ARM V1.0"); // Título fijo

	            lcd_gotoxy(&lcd1, 0, 1);

	            switch (current_lcd_state) {
	                case LCD_DESCONECTADO:
	                    lcd_puts(&lcd1, "Desconectado");
	                    break;
	                case LCD_CONECTADO:
	                    lcd_puts(&lcd1, "Enlace Activo ");
	                    break;
	                case LCD_PRUEBA:
	                    lcd_puts(&lcd1, "Rutina Home ");
	                    break;
	                default:
	                	break;
	            }
	        }

	        //===============================================================
	        //	MAQUINA DE ESTADOS PRINCIPAL
	        //================================================================
	        uint16_t dma_write_ptr = RING_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);

	        while (ring_buffer_head != dma_write_ptr)
	        {
	            uint8_t byte = dma_rx_buffer[ring_buffer_head];
	            ring_buffer_head = (ring_buffer_head + 1) % RING_BUFFER_SIZE;

	            switch (fsm_state)
	            {
	                case WAIT_START:
	                    if (byte == 0xAA) fsm_state = WAIT_S1;
	                    break;
	                case WAIT_S1: target_pos[0] = byte; fsm_state = WAIT_S2; break;
	                case WAIT_S2: target_pos[1] = byte; fsm_state = WAIT_S3; break;
	                case WAIT_S3: target_pos[2] = byte; fsm_state = WAIT_S4; break;
	                case WAIT_S4: target_pos[3] = byte; fsm_state = WAIT_STATUS; break;

	                case WAIT_STATUS:
	                        rx_estado_espnow = byte; // Se captura el byte de [Estado_ESPNOW]
	                        fsm_state = WAIT_CHK;
	                        break;

	                case WAIT_CHK:
	                    // el byte debe ser el XOR de los 4 datos previos
	                    if (byte == (target_pos[0] ^ target_pos[1] ^ target_pos[2] ^ target_pos[3] ^ rx_estado_espnow^ 0xFF)) {
	                        fsm_state = WAIT_END;
	                    } else {
	                    	errores_checksum++; //Debug
	                        fsm_state = WAIT_START; // Error, descartamos
	                    }
	                    break;

	                case WAIT_END:
	                    if (byte == 0x55) {
	                        // PAQUETE VÁLIDO.
	                        // Las variables target_pos[0..3] ya están actualizadas.
	                        // El sistema de servos las leerá en la próxima pasada del Timer.

	                    	tramas_exitosas++; //Debug

	                    	// Se oficializa el estado en el STM32 solo si el ESP32 confirmó el requerimiento.
	                    	if (pending_link_request && (rx_estado_espnow == target_link_state)) {
	                    	    pending_link_request = 0;          // Detenemos los reintentos
	                    	    is_linked = target_link_state;     // Oficializamos el nuevo estado
	                    	}

	                    	if (rx_estado_espnow == 0x00 && is_linked == 1) {
	                    	    is_linked = 0;
	                    	    // Opcional: Detener peticiones pendientes por seguridad
	                    	    pending_link_request = 0;
	                    	}

	                    	if (is_linked && !home_active) {
	                    	Servo_SetTargetAngle(&servo_mun, target_pos[2]);
	                    	Servo_SetTargetAngle(&servo_pnza,   target_pos[3]);
	                    	Servo_SetTargetAngle(&servo_cdo,   target_pos[0]);
	                    	Servo_SetTargetAngle(&servo_hmbro,  target_pos[1]);
	                    	}
	                    }
	                    fsm_state = WAIT_START;
	                    break;
	            }
	        }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 99;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 99;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 7999; //Define la cantidad de microsegundos que se va ejecutar la interrupcion
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : Pin_Btn_Pin */
  GPIO_InitStruct.Pin = Pin_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Pin_Btn_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief Asigna un nuevo ángulo objetivo protegiendo los datos de la ISR.
  */
void Servo_SetTargetAngle(ServoConfig_t *servo, uint8_t angle)
{
    if(angle > 180) angle = 180;

    uint32_t pulse_range = servo->Max_Pulse - servo->Min_Pulse;
    uint16_t calculated_pulse = servo->Min_Pulse + ((angle * pulse_range) / 180);

    // Deshabilitamos interrupciones momentáneamente para evitar que el TIM3
    // lea el Target_Pulse justo mientras lo estamos escribiendo.
    __disable_irq();

    servo->Target_Pulse = calculated_pulse;

    __enable_irq();
}

/**
  * @brief Lógica de interpolación impulsada por ISR.
  */
void Servo_ISR_Update(ServoConfig_t *servo)
{
    if (servo->Current_Pulse == servo->Target_Pulse) return;  // Si el servo  esta en el angulo esperado no hace nada

    int32_t diff = servo->Target_Pulse - servo->Current_Pulse;	// Calcula lo que le falta para llegar al angulo deseado

    if (abs(diff) <= servo->Step_Size)
    {
        servo->Current_Pulse = servo->Target_Pulse;	// Si la diferencia es menor al cambio por angulo que hace entonces hace el salto
    }
    else if (diff > 0)
    {
        servo->Current_Pulse += servo->Step_Size;
    }
    else
    {
        servo->Current_Pulse -= servo->Step_Size;
    }

    __HAL_TIM_SET_COMPARE(servo->htim, servo->Channel, servo->Current_Pulse);
}
void Run_Home_Routine(void)
{
    // Variables estáticas para mantener el estado entre las iteraciones del while(1)
    static uint8_t home_step = 0;
    static uint32_t step_start_time = 0;

    switch (home_step)
    {
        case 0: // INICIO: Todos a 90°
            Servo_SetTargetAngle(&servo_mun, 90);
            Servo_SetTargetAngle(&servo_pnza, 90);
            Servo_SetTargetAngle(&servo_cdo, 90);
            Servo_SetTargetAngle(&servo_hmbro, 90);

            step_start_time = HAL_GetTick(); // Sellamos el tiempo actual
            home_step++;                     // Avanzamos al siguiente paso
            break;

        case 1: // ESPERA 1 seg y luego: Todos a 180°
            if ((HAL_GetTick() - step_start_time) >= 1000)
            {
                Servo_SetTargetAngle(&servo_mun, 180);
                Servo_SetTargetAngle(&servo_pnza, 180);
                Servo_SetTargetAngle(&servo_cdo, 180);
                Servo_SetTargetAngle(&servo_hmbro, 180);

                step_start_time = HAL_GetTick();
                home_step++;
            }
            break;

        case 2:
            if ((HAL_GetTick() - step_start_time) >= 1000)
            {
                Servo_SetTargetAngle(&servo_mun, 180);
                Servo_SetTargetAngle(&servo_pnza, 180);
                Servo_SetTargetAngle(&servo_cdo, 180);
                Servo_SetTargetAngle(&servo_hmbro, 0);

                step_start_time = HAL_GetTick();
                home_step++;
            }
            break;

        case 3: // ESPERA 1 seg y luego: Todos a 0°
            if ((HAL_GetTick() - step_start_time) >= 1000)
            {
                Servo_SetTargetAngle(&servo_mun, 0);
                Servo_SetTargetAngle(&servo_pnza, 0);
                Servo_SetTargetAngle(&servo_cdo, 0);
                Servo_SetTargetAngle(&servo_hmbro, 0);

                step_start_time = HAL_GetTick();
                home_step++;
            }
            break;

        case 4: // ESPERA 1 seg y luego: Hombro a 180° (el resto queda en 0°)
            if ((HAL_GetTick() - step_start_time) >= 1000)
            {
                Servo_SetTargetAngle(&servo_hmbro, 180);
                // Los demás ya están en 0° por el paso anterior

                step_start_time = HAL_GetTick();
                home_step++;
            }
            break;

        case 5: // ESPERA 0.5 seg y luego: Todos a 180°
            if ((HAL_GetTick() - step_start_time) >= 1000)
            {
                Servo_SetTargetAngle(&servo_mun, 180);
                Servo_SetTargetAngle(&servo_pnza, 180);
                Servo_SetTargetAngle(&servo_cdo, 180);
                // El hombro ya estaba en 180° por el paso anterior

                step_start_time = HAL_GetTick();
                home_step++;
            }
            break;

        case 6:
            if ((HAL_GetTick() - step_start_time) >= 1000)
            {
                Servo_SetTargetAngle(&servo_mun, 180);
                Servo_SetTargetAngle(&servo_pnza, 180);
                Servo_SetTargetAngle(&servo_cdo, 180);
                Servo_SetTargetAngle(&servo_hmbro, 90);

                step_start_time = HAL_GetTick();
                home_step++;
            }
            break;

        case 7: // ESPERA 0.5 seg y luego: Todos a 90° (Posición de descanso final)
            if ((HAL_GetTick() - step_start_time) >= 500)
            {
                Servo_SetTargetAngle(&servo_mun, 110);
                Servo_SetTargetAngle(&servo_pnza, 90);
                Servo_SetTargetAngle(&servo_cdo, 110);
                Servo_SetTargetAngle(&servo_hmbro, 90);

                step_start_time = HAL_GetTick();
                home_step++;
            }
            break;

        case 8: // ESPERA FINAL: Dar tiempo a que los motores lleguen a 90° antes de salir
            if ((HAL_GetTick() - step_start_time) >= 1000)
            {
                home_step = 0;   // 1. Reseteamos la FSM de la rutina para el futuro
                home_active = 0; // 2. Apagamos la bandera global para devolver el control
            }
            break;
    }
}
/**
  * @brief  Rutina de atención (Callback) para la interrupción de periodo transcurrido.
  * Esta función es llamada automáticamente por la HAL cuando el TIM3 llega a su tope (ARR).
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // Evaluamos si la interrupción fue disparada específicamente por el Timer 3
    if (htim->Instance == TIM3)
    {
        // Ejecutamos la interpolación crítica de los 4 motores de forma segura
        Servo_ISR_Update(&servo_mun);
        Servo_ISR_Update(&servo_pnza);
        Servo_ISR_Update(&servo_cdo);
        Servo_ISR_Update(&servo_hmbro);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        //Limpiamos los flags de error
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);

        // Forzamos a la maquina de estados a resincronizarse
        fsm_state = WAIT_START;

        // Sincroniza el puntero de software con el hardware DMA
       ring_buffer_head = 0;

        // Detenemos formalmente el proceso y rearmamos el DMA Circular
        HAL_UART_DMAStop(huart);
        HAL_UART_Receive_DMA(&huart1, dma_rx_buffer, RING_BUFFER_SIZE);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == Pin_Btn_Pin)
    {
        if (HAL_GPIO_ReadPin(Pin_Btn_GPIO_Port, Pin_Btn_Pin) == GPIO_PIN_RESET)
        {
            // Botón presionado (flanco bajada)
            button_press_start_time = HAL_GetTick();
        }
        else
        {
            // Botón soltado (flanco subida)
            uint32_t press_duration = HAL_GetTick() - button_press_start_time;

            if (press_duration >= LONG_PRESS_TIME) {
                flag_long_press = 1;
            } else if (press_duration >= DEBOUNCE_TIME) {
                flag_short_press = 1;
            }
        }
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
