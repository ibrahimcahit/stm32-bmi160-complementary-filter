/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdint.h"
#include "stdio.h"
#include "math.h"
#include "bmi160_wrapper.h"
#include "micros.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
BMI160_t imu_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float aX_f32, aY_f32, aZ_f32;
float gX_f32, gY_f32, gZ_f32;

float accel_roll_f32, accel_pitch_f32;
float gyro_roll_f32, gyro_pitch_f32;

float acc_total_vector_f32 = 0;

float yaw_f32, pitch_f32, roll_f32;

uint32_t loopHz_u64, loopTime_u64;
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
  MX_LPUART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  DWT_Init();

  while (BMI160_init(imu_t) == 1);

  if (imu_t.INIT_OK_i8 != TRUE)
  {
	   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);
  }

  uint64_t timer_u64 = 0;
  uint64_t lastTime_u64 = 0;

  uint8_t set_gyro_angles_u8 = 0;

  float acc_total_vector_f32 = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  timer_u64 = micros();

	  // Read an process data at 1000 Hz rate

	  if ( ((timer_u64 - lastTime_u64) >= 1000) && (imu_t.INIT_OK_i8 != TRUE) ) // && (PIN_LOW == 1))
	  {
		  bmi160ReadAccelGyro(&imu_t);

		  aX_f32 = imu_t.BMI160_Ax_f32; // Read RAW and unscaled acceleration values from all 3 axes, unit: (g)
		  aY_f32 = imu_t.BMI160_Ay_f32; //
		  aZ_f32 = imu_t.BMI160_Az_f32; //

		  gX_f32 = imu_t.BMI160_Gx_f32 * 0.001f; // Read scaled gyro values from all 3 axes, unit: (deg/s)
		  gY_f32 = imu_t.BMI160_Gy_f32 * 0.001f; // 0.001 is 1 ms whic is represents 1000 Hz rate
		  gZ_f32 = imu_t.BMI160_Gz_f32 * 0.001f; // Multiply readings with calculation period for integration


		  gyro_pitch_f32 += gX_f32; // integrate gyro readings for pitch and roll calculation
		  gyro_roll_f32 += gY_f32;

		  gyro_pitch_f32 += gyro_roll_f32 * sin(gZ_f32 * 0.01745329f); // correct X andy Y axis readings with respect to Z axis readings
		  gyro_roll_f32 -= gyro_pitch_f32 * sin(gZ_f32 * 0.01745329f); // sin function accepts radians, 0.01745329 = pi / 180

		  acc_total_vector_f32 = sqrt((aX_f32*aX_f32)+(aY_f32*aY_f32)+(aZ_f32*aZ_f32)); // Calculate total acceleration vector

		  accel_pitch_f32 = asin(aY_f32/acc_total_vector_f32) * 57.296f; // Calculate pitch and roll respect to acceleration readings
		  accel_roll_f32 = asin(aX_f32/acc_total_vector_f32) * -57.296f;

		  accel_pitch_f32 -= 0.0f; // Corrections or acceleration calculations.
		  accel_roll_f32 -= 0.0f; // Leave 0 if accel values are ~0.0 when resting

		  // initial pitch and roll readings should be aceel-based
		  if(set_gyro_angles_u8)
		  {
			  gyro_pitch_f32 = gyro_pitch_f32 * 0.999f + accel_pitch_f32 * 0.001f; // to calculate final pitch and roll, we get most of
			  gyro_roll_f32 = gyro_roll_f32 * 0.999f + accel_roll_f32 * 0.001f;    // gyro readings and small amount of accel readings
		  }
		  else
		  {
			  gyro_pitch_f32 = accel_pitch_f32;
			  gyro_roll_f32 = accel_roll_f32;
			  set_gyro_angles_u8 = 1;
		  }

		  // integrate calculated pitch and roll with previous values
		  pitch_f32 = pitch_f32 * 0.75f + gyro_pitch_f32 * 0.25f;
		  roll_f32 = roll_f32 * 0.75f + gyro_roll_f32 * 0.25f;


//		  printf("%f, %f, %f, %f, %f, %f\r\n", aX_f32, aY_f32, aZ_f32, gX_f32, gY_f32, gZ_f32);

		  lastTime_u64 = micros();

	  }

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV6;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM8 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM8) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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

