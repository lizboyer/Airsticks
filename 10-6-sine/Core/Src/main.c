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
#ifndef __MAIN_C
#define __MAIN_C
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MY_CS43L22.h"
#include <math.h>
#include "adpcm.h"

//including in the sound file
#include "afnm.h"
#include "cowbell.h"
#include "floortom.h"
#include "hat.h"
#include "ride.h"
#include "snare.h"
//#endif
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PI 3.14159f

#define F_SAMPLE			50000.0f
#define F_OUT				300.0f



//playback defines -db
tTwoByte newSample;
static AudioElement AudioFile;
static uint8_t AudioFileToPlay = 0; //new -db, temporarily choosing to play new file
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac;

I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi3_tx;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

float mySinVal;
float sample_dt;
uint16_t sample_N;
uint16_t i_t;

uint32_t myDacVal;

int16_t I2S_dummy[4]; //dummy byte for the i2s
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
	sample_dt = F_OUT/F_SAMPLE;
	sample_N = F_SAMPLE/F_OUT;
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
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_DAC_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  CS43_Init(hi2c1);//, MODE_ANALOG);
  CS43_SetVolume(50);
  CS43_Enable_RightLeft(CS43_RIGHT_LEFT);
  CS43_Start();

  HAL_I2S_Transmit_DMA(&hi2s3, (uint16_t *)I2S_dummy, 4);
  HAL_DAC_Start(&hdac,DAC_CHANNEL_1);

  HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	AudioFile.AudioFiles[0] = (uint32_t)&afnm;
	AudioFile.AudioSize[0] = NELEMS(afnm);

	//new -db
	AudioFile.AudioFiles[1] = (uint32_t)&cowbell;
	AudioFile.AudioSize[1] = NELEMS(cowbell);

	AudioFile.AudioFiles[2] = (uint32_t)&floortom;
	AudioFile.AudioSize[2] = NELEMS(floortom);

	AudioFile.AudioFiles[3] = (uint32_t)&hat;
	AudioFile.AudioSize[3] = NELEMS(hat);

	AudioFile.AudioFiles[4] = (uint32_t)&ride;
	AudioFile.AudioSize[4] = NELEMS(ride);

	AudioFile.AudioFiles[5] = (uint32_t)&snare;
	AudioFile.AudioSize[5] = NELEMS(snare);

  while (1)
  {
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

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
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 255;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15
                           PD4 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the __HAL_TIM_PeriodElapsedCallback could be implemented in the user file
   */

	if(htim->Instance == TIM2)
	{
	  /* USER CODE BEGIN TIM3_IRQn 0 */
		uint8_t  adpcmSample;
		static uint16_t pcmSample;
		static uint8_t nibble = 1;
		static uint8_t repetition = 0;
		static uint16_t sample_position = 0;
		static unsigned char *RawAudio;
		static uint8_t PrevAudioFileToPlay = 0xFF;

		static uint8_t play_sound = 0;
		static uint8_t end_of_file = 1;
		static uint8_t retrigger = 0;

		static GPIO_PinState pin8;
		static GPIO_PinState pin9;
		static GPIO_PinState pin10;
		static GPIO_PinState pin11;

		pin8 = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_8);
		pin9 = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_9);
		pin10 = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10);
		pin11 = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_11);

		if(pin8 == GPIO_PIN_SET || pin9 == GPIO_PIN_SET
		|| pin10 == GPIO_PIN_SET || pin11== GPIO_PIN_SET
		|| end_of_file == 0)
		{
			play_sound = 1;
			end_of_file = 0;

		} else {
			play_sound = 0;
		}


		if(play_sound == 0)
		{
			PrevAudioFileToPlay = AudioFileToPlay;
			nibble = 1;
			repetition = 0;
			sample_position = 0;
			RawAudio = (unsigned char *)AudioFile.AudioFiles[AudioFileToPlay];
		}



		if ((HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_8) == GPIO_PIN_SET) && retrigger == 1){
			AudioFileToPlay = 5; //snare
			retrigger = 0;
			PrevAudioFileToPlay = -1;
		} else if ((HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_9) == GPIO_PIN_SET) && retrigger == 1){
			AudioFileToPlay = 4; //ride
			retrigger = 0;
			PrevAudioFileToPlay = -1;
		} else if ((HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10) == GPIO_PIN_SET) && retrigger == 1){
			AudioFileToPlay = 2; //floortom
			retrigger = 0;
			PrevAudioFileToPlay = -1;
		} else if ((HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_11) == GPIO_PIN_SET) && retrigger == 1){
			AudioFileToPlay = 3; //hat
			retrigger = 0;
			PrevAudioFileToPlay = -1;
		} else if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_8) != GPIO_PIN_SET ||
				HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_9) != GPIO_PIN_SET ||
				HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10) != GPIO_PIN_SET ||
				HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_11) != GPIO_PIN_SET) {
			retrigger = 1;
		}



		//picking file to play!!!
		if(PrevAudioFileToPlay != AudioFileToPlay)
		{
			PrevAudioFileToPlay = AudioFileToPlay;
			nibble = 1;
			repetition = 0;
			sample_position = 0;
			RawAudio = (unsigned char *)AudioFile.AudioFiles[AudioFileToPlay];
		}


			if (sample_position >= AudioFile.AudioSize[AudioFileToPlay] && sample_position != 0)
			{
				end_of_file = 1;
				PrevAudioFileToPlay = -1; //allow retrigger if end of sample reached
				sample_position = 0;
			}


			else if ((repetition==0) & (sample_position < AudioFile.AudioSize[AudioFileToPlay])
					&& play_sound == 1 && end_of_file == 0)
			{  // new sample is generated
				repetition = 7;	// reinitialize repetition down counter
				if (nibble)
				{   // first 4 bits of the ADPCM byte decoded
					adpcmSample = (uint8_t)(RawAudio[sample_position] >> 4);
				}
				else
				{   // last 4 bits of the ADPCM byte decoded
					adpcmSample = (uint8_t)(RawAudio[sample_position] & 0x0F);
					sample_position++ ;
				}

				nibble = (uint8_t)(!nibble);/* indicator inverted mean next interrupt will handle
																						 the second part of the byte.  */
				pcmSample = ADPCM_Decode(adpcmSample);

				// update sample
				newSample.uShort = (uint16_t)32768 + pcmSample;
				TIM2->CCR2 = newSample.uBytes[0]; //LSB
				TIM2->CCR1 = newSample.uBytes[1]; //MSB
				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (newSample.uShort)>>4);
			}
			else if (sample_position < AudioFile.AudioSize[AudioFileToPlay] && play_sound == 1 && end_of_file == 0)
			{  // repetition 7 more times of the PWM period before new sample, total of times the same value is repeated = 8
				repetition--;

				// reload Timer with the actual sample value
				newSample.uShort = (uint16_t)32768 + pcmSample;
				TIM2->CCR2 = newSample.uBytes[0]; //LSB
				TIM2->CCR1 = newSample.uBytes[1]; //MSB

				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (newSample.uShort)>>4);

			}

	  /* USER CODE END TIM3_IRQn 0 */
	  /* USER CODE BEGIN TIM3_IRQn 1 */
	}
	  /* USER CODE END TIM3_IRQn 1 */
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
#endif
