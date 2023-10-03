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




/************************** DEVICE ADDRESSES **************************/
#define ACC0_W_ADDR			0xd4
#define ACC0_R_ADDR			0xd5
#define ACC1_W_ADDR			0xd6
#define ACC1_R_ADDR			0xd7


/************************** DEVICE REGISTERS **************************/
#define REG_CTRL1_XL		0x10
#define REG_CTRL2_G			0x11
#define REG_INT1_CTRL 		0x0d
#define REG_INT2_CTRL 		0x0e


/************************** REGISTER CONFIGS **************************/
#define ACC_104HZ_2G		0x40
#define GYRO_OFF			0x00
#define DATA_RDY			0x01

#define OUTX_L_A			0x28
#define OUTX_H_A			0x29
#define OUTY_L_A			0x2a
#define OUTY_H_A			0x2b
#define OUTZ_L_A			0x2c
#define OUTZ_H_A			0x2d


/**************************** TYPEDEFS ********************************/
I2C_HandleTypeDef hi2c1;
typedef enum axis {
	X_AXIS,
	Y_AXIS,
	Z_AXIS,
	ALL_AXIS
} axis_t;

typedef struct accelerometer_t {
	  int16_t x_measurement;
	  int16_t y_measurement;
	  int16_t z_measurement;
	  int8_t  slave_r_addr;
	  int8_t  slave_w_addr;
} accelerometer_t;


/************************** GLOBAL VARIABLES ***************************/
I2C_HandleTypeDef hi2c1;
static struct accelerometer_t xl_r = {.slave_r_addr = ACC0_R_ADDR, .slave_w_addr = ACC0_W_ADDR};
static struct accelerometer_t xl_l = {.slave_r_addr = ACC1_R_ADDR, .slave_w_addr = ACC1_W_ADDR};




/************************ FUNCTION PROTOTYPES **************************/
I2C_HandleTypeDef hi2c1;
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);


/*
 * read_axis
 * inputs:
 * 		- axis_t axis: enumerator representing the axis to read
 * returns:
 * 		- a signed 16-bit integer representing +-MAX_ACCELERATION at each of
 * 		  the respective bounds
 **/
static void read_axis(accelerometer_t* acc, axis_t axis)
{
	  static uint8_t read_buffer[] = { 0 };

	  switch(axis){
	  	  case ALL_AXIS:
	  	  case X_AXIS:
	  		  HAL_I2C_Mem_Read(&hi2c1, acc->slave_r_addr, OUTX_H_A, I2C_MEMADD_SIZE_8BIT, read_buffer, sizeof(read_buffer), HAL_MAX_DELAY);
	  		  acc->x_measurement = *read_buffer << 8;
			  HAL_I2C_Mem_Read(&hi2c1, acc->slave_r_addr, OUTX_L_A, I2C_MEMADD_SIZE_8BIT, read_buffer, sizeof(read_buffer), HAL_MAX_DELAY);
			  acc->x_measurement = acc->x_measurement + *read_buffer;
			  if(axis != ALL_AXIS) break;
	  	  case Y_AXIS:
	  		  HAL_I2C_Mem_Read(&hi2c1, acc->slave_r_addr, OUTY_H_A, I2C_MEMADD_SIZE_8BIT, read_buffer, sizeof(read_buffer), HAL_MAX_DELAY);
	  		  acc->y_measurement = *read_buffer << 8;
	  		  HAL_I2C_Mem_Read(&hi2c1, acc->slave_r_addr, OUTY_L_A, I2C_MEMADD_SIZE_8BIT, read_buffer, sizeof(read_buffer), HAL_MAX_DELAY);
	  		  acc->y_measurement = acc->y_measurement + *read_buffer;
	  		  if(axis != ALL_AXIS) break;
	  	  case Z_AXIS:
	  		  HAL_I2C_Mem_Read(&hi2c1, acc->slave_r_addr, OUTZ_H_A, I2C_MEMADD_SIZE_8BIT, read_buffer, sizeof(read_buffer), HAL_MAX_DELAY);
	  		  acc->z_measurement = *read_buffer << 8;
	  		  HAL_I2C_Mem_Read(&hi2c1, acc->slave_r_addr, OUTZ_L_A, I2C_MEMADD_SIZE_8BIT, read_buffer, sizeof(read_buffer), HAL_MAX_DELAY);
	  		  acc->z_measurement = acc->z_measurement + *read_buffer;
	  		  break;
	  }

	  return;
}

static void accelerometer_write(accelerometer_t* acc, int8_t reg, int8_t data)
{
	static uint8_t write_buffer[] = { 0 };

	*write_buffer = data;
	HAL_I2C_Mem_Write(&hi2c1, acc->slave_w_addr, reg, I2C_MEMADD_SIZE_8BIT, write_buffer, sizeof(write_buffer), HAL_MAX_DELAY);
}


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* HAL and peripheral initialization */
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();


  // configure the accelerometer to 104Hz
  accelerometer_write(&xl_r, REG_CTRL1_XL, ACC_104HZ_2G);
  accelerometer_write(&xl_l, REG_CTRL1_XL, ACC_104HZ_2G);


  // turn the gyroscope off
  accelerometer_write(&xl_r, REG_CTRL2_G, GYRO_OFF);
  accelerometer_write(&xl_l, REG_CTRL2_G, GYRO_OFF);


  // enable interrupts on new data on accelerometer INT2
  accelerometer_write(&xl_r, REG_INT2_CTRL, DATA_RDY);
  accelerometer_write(&xl_l, REG_INT2_CTRL, DATA_RDY);

  int total = 0;
  while (1)
  {
	  // poll the accelerometer

	  read_axis(&xl_r, ALL_AXIS);

	  total = xl_r.x_measurement + xl_r.y_measurement + xl_r.z_measurement;

	  HAL_Delay(250);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hi2c1.Init.Timing = 0x00000708;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

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
