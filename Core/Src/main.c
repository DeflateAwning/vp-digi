/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>
#include <stdint.h>

#include "drivers/modem.h"
#include "ax25.h"
#include "drivers/uart.h"
#include "drivers/systick.h"
#include "stm32l4xx.h"
#include "digipeater.h"
#include "common.h"
#include "drivers/watchdog.h"
#include "beacon.h"
#include "terminal.h"
#include "config.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac1;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * \brief Handle received frame from RF
 */
void handleFrame(void)
{
	uint8_t modemReceived = ax25.frameReceived; //store states
	ax25.frameReceived = 0; //clear flag

	uint8_t bufto[FRAMELEN + 30], buf[FRAMELEN]; //buffer for raw frames to TNC2 frames conversion
	uint16_t bufidx = 0;
	uint16_t i = ax25.frameBufRd;

	while(i != ax25.frameBufWr)
	{
		if(ax25.frameBuf[i] != 0xFF)
		{
			buf[bufidx++] = ax25.frameBuf[i++]; //store frame in temporary buffer
		}
		else
		{
			break;
		}
		i %= (FRAMEBUFLEN);
	}

	ax25.frameBufRd = ax25.frameBufWr;

	for(i = 0; i < (bufidx); i++)
	{
		if(buf[i] & 1)
			break; //look for path end bit
	}


	SendKiss(buf, bufidx); //send KISS frames if ports available


	if(((USBmode == MODE_MONITOR) || (uart1.mode == MODE_MONITOR) || (uart2.mode == MODE_MONITOR)))
	{
		common_toTNC2(buf, bufidx, bufto); //convert to TNC2 format

		//in general, the RMS of the frame is calculated (excluding preamble!)
		//it it calculated from samples ranging from -4095 to 4095 (amplitude of 4095)
		//that should give a RMS of around 2900 for pure sine wave
		//for pure square wave it should be equal to the amplitude (around 4095)
		//real data contains lots of imperfections (especially mark/space amplitude imbalance) and this value is far smaller than 2900 for standard frames
		//division by 9 was selected by trial and error to provide a value of 100(%) when the input signal had peak-peak voltage of 3.3V
		//this probably should be done in a different way, like some peak amplitude tracing
		ax25.sLvl /= 9;

		if(ax25.sLvl > 100)
		{
			term_sendMonitor((uint8_t*)"\r\nInput level too high! Please reduce so most stations are around 50-70%.\r\n", 0);
		}

		if(ax25.sLvl < 10)
		{
			term_sendMonitor((uint8_t*)"\r\nInput level too low! Please increase so most stations are around 50-70%.\r\n", 0);
		}

		term_sendMonitor((uint8_t*)"(AX.25) Frame received [", 0); //show which modem received the frame: [FP] (flat and preemphasized), [FD] (flat and deemphasized - in flat audio input mode)
																   //[F_] (only flat), [_P] (only preemphasized) or [_D] (only deemphasized - in flat audio input mode)
		uint8_t t[2] = {0};
		if(modemReceived & 1)
		{
			t[0] = 'F';
		}
		else
			t[0] = '_';
		if(modemReceived & 2)
		{
			if(afskCfg.flatAudioIn)
				t[1] = 'D';
			else
				t[1] = 'P';
		}
		else
			t[1] = '_';

		term_sendMonitor(t, 2);
		term_sendMonitor((uint8_t*)"], signal level ", 0);
		term_sendMonitorNumber(ax25.sLvl);
		term_sendMonitor((uint8_t*)"%: ", 0);

		term_sendMonitor(bufto, 0);
		term_sendMonitor((uint8_t*)"\r\n", 0);

	}


	if(digi.enable)
	{
		Digi_digipeat(buf, bufidx);
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

  
  SysTick_init();

#ifdef FEATURE_USB_SUPPORT
  //force usb re-enumeration after reset
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN; //pull D+ to ground for a moment
  GPIOA->CRH |= GPIO_CRH_MODE12_1;
  GPIOA->CRH &= ~GPIO_CRH_CNF12;
  GPIOA->BSRR = GPIO_BSRR_BR12;
  uint32_t t = ticks + 10;
  while(t > ticks);;
  GPIOA->CRH &= ~GPIO_CRH_MODE12;
  GPIOA->CRH |= GPIO_CRH_CNF12_0;
  
  #endif /* FEATURE_USB_SUPPORT */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_DAC1_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */

  

	Wdog_init(); //initialize watchdog


	//set some initial values in case there is no configuration saved in memory
	uart1.baudrate = 38400;
	uart2.baudrate = 38400;
	afskCfg.usePWM = 1; // 1 = use PWM, 0 = use R2R
	afskCfg.flatAudioIn = 0;
	ax25Cfg.quietTime = 300;
	ax25Cfg.txDelayLength = 300;
	ax25Cfg.txTailLength = 30;
	digi.dupeTime = 30;

	Config_read();

	Ax25_init();

	uart_init(&uart1, USART1, uart1.baudrate);
	uart_init(&uart2, USART2, uart2.baudrate);

	// added: simple print UART
	uint8_t msg1[255];
	sprintf(msg1, "Booting vp-digi on L432.\n");
	HAL_UART_Transmit(&huart2, msg1, strlen(msg1), 100);

	uart_config(&uart1, 1);
	uart_config(&uart2, 1);

	Afsk_init();
	Beacon_init();


	autoResetTimer = autoReset * 360000;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  static uint32_t usbKissTimer = 0;
  uint16_t main_loop_counter = 0;

  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET); // flash LED
	  HAL_Delay(250);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET); // flash LED
	  HAL_Delay(250);
	  // added: simple print UART
	  	uint8_t msg1[255];
	  	sprintf(msg1, "Running vp-digi loop on L432, #%d\n", main_loop_counter++);
	  	HAL_UART_Transmit(&huart2, msg1, strlen(msg1), 100);


    	  Wdog_reset();

	  if(ax25.frameReceived)
		  handleFrame();

	  Digi_viscousRefresh(); //refresh viscous-delay buffers


	  Ax25_transmitBuffer(); //transmit buffer (will return if nothing to be transmitted)

	  Ax25_transmitCheck(); //check for pending transmission request

#ifdef FEATURE_USB_SUPPORT
	  if(USBint) //USB "interrupt"
	  {
		  USBint = 0; //clear

		  if(USBmode == MODE_KISS) //is USB in KISS mode?
			  usbKissTimer = ticks + 500; //set timeout to 5s

		  term_handleSpecial(TERM_USB); //handle special characters (e.g. backspace)
		  if((usbcdcdata[0] == 0xc0) && /*(usbcdcdata[1] == 0x00) &&*/ (usbcdcdata[usbcdcidx - 1] == 0xc0)) //probably a KISS frame
		  {
			  USBrcvd = DATA_KISS;
			  usbKissTimer = 0;
		  }

		  if(((usbcdcdata[usbcdcidx - 1] == '\r') || (usbcdcdata[usbcdcidx - 1] == '\n'))) //proabably a command
		  {
			  USBrcvd = DATA_TERM;
			  usbKissTimer = 0;
		  }
	  }

	  if((usbKissTimer > 0) && (ticks >= usbKissTimer)) //USB KISS timer timeout
	  {
		  usbcdcidx = 0;
		  memset(usbcdcdata, 0, UARTBUFLEN);
		  usbKissTimer = 0;
	  }


	  if(USBrcvd != DATA_NOTHING)
	  {
		  term_parse(usbcdcdata, usbcdcidx, TERM_USB, USBrcvd, USBmode);
		  USBrcvd = DATA_NOTHING;
		  usbcdcidx = 0;
		  memset(usbcdcdata, 0, UARTBUFLEN);
	  }
#endif /* FEATURE_USB_SUPPORT */

	  if(uart1.rxflag != DATA_NOTHING)
	  {
		  term_parse(uart1.bufrx, uart1.bufrxidx, TERM_UART1, uart1.rxflag, uart1.mode);
		  uart1.rxflag = DATA_NOTHING;
		  uart1.bufrxidx = 0;
		  memset(uart1.bufrx, 0, UARTBUFLEN);
	  }
	  if(uart2.rxflag != DATA_NOTHING)
	  {
		  term_parse(uart2.bufrx, uart2.bufrxidx, TERM_UART2, uart2.rxflag, uart2.mode);
		  uart2.rxflag = DATA_NOTHING;
		  uart2.bufrxidx = 0;
		  memset(uart2.bufrx, 0, UARTBUFLEN);
	  }

	  Beacon_check(); //check beacons


	  if(((autoResetTimer != 0) && (ticks > autoResetTimer)) || (ticks > 4294960000))
		  NVIC_SystemReset();


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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV6;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 0;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 65535;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

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
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA8 PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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

#ifdef  USE_FULL_ASSERT
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
