/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "stdio.h"
#include "string.h"
#include "ssd1306.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern I2C_HandleTypeDef hi2c1;

#define MPU_ADDR 0xD0
#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H 0x3B
//#define TEMP_OUT_H 0x41
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int16_t aX_RAW, aY_RAW, aZ_RAW, adcRAW[2], aX0, aY0, aZ0, tempRAW;
float aX, aY, aZ, aRoll, aPitch, tempMPU, aX1, aY1, aZ1;
uint8_t AdcConvCmplt;

double temperature;
double VtmpSens;
double VrefInt;
double newTemperature;
double newRoll;
double newPitch;

int calibcount, n = 3000;
uint16_t blinkCount;

#define VREFINT 1.21
#define ADCMAX 4095.0

#define V25 0.76
#define AVG_SLOPE 0.0025
#define alpha 0.1
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
 uint8_t checkI2C_MPU9250;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void MPU_Init ()
{
	uint8_t check, data;
	checkI2C_MPU9250 = &check;

	HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, WHO_AM_I_REG, 1, &check, 1, HAL_MAX_DELAY);

//	if (check == 104)
//
//	{
		data = 0;
		HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, PWR_MGMT_1_REG, 1, &data, 1, HAL_MAX_DELAY);

		data = 0x07;
		HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, SMPLRT_DIV_REG, 1, &data, 1, HAL_MAX_DELAY);

		data = 0b00001100;
		HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, ACCEL_CONFIG_REG, 1, &data, 1, HAL_MAX_DELAY);

//	}
}
//void MPU_Read_TEMP ()
//{
//	uint8_t RecData [2];
//	int16_t tempRAW;
//
//	HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, TEMP_OUT_H, 1, RecData, 2, HAL_MAX_DELAY);
//	tempRAW = (int16_t) (RecData[0]<<8 | RecData [1]);
//
//	tempMPU = (tempRAW/340 + 36.53)/2;
//}
void MPU_Read_ACCEL ()
{
	uint8_t recData [8];

	HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, ACCEL_XOUT_H, 1, recData, 8, HAL_MAX_DELAY);
	aX_RAW = (int16_t)(recData[0]<<8 | recData [1]);
	aY_RAW = (int16_t)(recData[2]<<8 | recData [3]);
	aZ_RAW = (int16_t)(recData[4]<<8 | recData [5]);
	tempRAW = (int16_t)(recData[6]<<8 | recData [7]);

	tempMPU = (tempRAW/340 + 36.53)/2;

	aX = (float)(aX_RAW+(-1*aX1)/2048.00);
	aY = (float)(aY_RAW+(-1*aY1)/2048.00);
	aZ = (float)(aZ_RAW+(2048.00 - aZ1)/2048.00);

	aRoll = (atan2 (aX,aZ))*57.2957795;
	aPitch = (atan2 (aY,aZ))*57.2957795;

	//	newRoll = (alpha*aRoll)+((1-alpha)*newRoll);
	//	newPitch = (alpha*aPitch)+(1-alpha)*newPitch;
}

void MPU_Calib_ACCEL ()
{
	uint8_t RECDATA [6];
	ssd1306_SetCursor(0, 0);
	ssd1306_WriteString("Calib", Font_7x10, White);
	for (calibcount = 0; calibcount<n; calibcount++)
	{
		HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, ACCEL_XOUT_H, 1, RECDATA, 6, HAL_MAX_DELAY);
		aX0 = (int16_t)(RECDATA[0]<<8 | RECDATA [1]);
		aY0 = (int16_t)(RECDATA[2]<<8 | RECDATA [3]);
		aZ0 = (int16_t)(RECDATA[4]<<8 | RECDATA [5]);

		aX1 += aX0;
		aY1 += aY0;
		aZ1 += aZ0;

		if (calibcount==n-1)
		{
			aX1/=n;
			aY1/=n;
			aZ1/=n;
			break;
		}
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
	MX_DMA_Init();
	MX_I2C1_Init();
	MX_TIM1_Init();
	MX_ADC1_Init();
	MX_I2C2_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	/* USER CODE BEGIN 2 */
	HAL_ADC_Start_DMA(&hadc1,(uint32_t *) adcRAW, 2);
	HAL_TIM_Base_Start(&htim3);
	ssd1306_Init(&hi2c2);
	//	if (ssd1306_Init(&hi2c2)!=0)
	//	{
	//		Error_Handler();
	//	}
	//	HAL_Delay(1000);
	//
	ssd1306_UpdateScreen(&hi2c2);
	ssd1306_SetCursor(15, 15);
	ssd1306_WriteString("Calib", Font_16x26, White);
	ssd1306_UpdateScreen(&hi2c2);
//	MPU_Calib_ACCEL();
	MPU_Init();
	HAL_Delay(1000);
	ssd1306_UpdateScreen(&hi2c2);
//	MPU_Calib_ACCEL();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		uint32_t blinkTime = HAL_GetTick();
		if ((blinkTime % 2) == 0)
		{
			blinkTime = 0;
			blinkCount++;
		}
		char buff [32];
		snprintf(buff, sizeof(buff), "%.2f, %.2f, %.2f, %.2f\r\n",aRoll, aPitch, tempMPU, newTemperature);
		//		HAL_UART_Transmit(&huart1, (uint8_t*) buff , strlen(buff), HAL_MAX_DELAY);

		if (AdcConvCmplt)
		{
			MPU_Read_ACCEL();
			//		MPU_Read_TEMP ();
			VrefInt = (VREFINT*ADCMAX)/adcRAW[0];
			VtmpSens = (VrefInt*adcRAW[1])/ADCMAX;

			temperature =(VtmpSens - V25)/(AVG_SLOPE)+ 25.0;
			newTemperature = (alpha*temperature) + ((1-alpha)*newTemperature);
			AdcConvCmplt = 0;
		}
		//
		char buff1[32];char buff2[32];char buff3[32];char buff4[32];
		//				char buff_send [32];
		//				snprintf(buff_send, sizeof(buff_send), "%.2f,%.2f,%.2f,%.2f", aPitch, aRoll, tempMPU, temperature);
		snprintf(buff1, sizeof(buff1), "Roll: %.2f", aRoll);
		snprintf(buff2, sizeof(buff2), "Pitch: %.2f", aPitch);
		snprintf(buff3, sizeof(buff3), "Temp MPU: %.2f", tempMPU);
		snprintf(buff4, sizeof(buff4), "Temp STM: %.2f", newTemperature);
		if (blinkCount >= 0 && blinkCount < 1)
		{
			ssd1306_SetCursor(0, 0);
			ssd1306_WriteString(buff1, Font_7x10, White);
			ssd1306_SetCursor(0, 15);
			ssd1306_WriteString(buff2, Font_7x10, White);
			ssd1306_SetCursor(0, 30);
			ssd1306_WriteString(buff3, Font_7x10, White);
			ssd1306_SetCursor(0, 45);
			ssd1306_WriteString(buff4, Font_7x10, White);
			ssd1306_UpdateScreen(&hi2c2);
		}
		else if (blinkCount>=1&&blinkCount<2)
		{
			ssd1306_UpdateScreen(&hi2c2);
			ssd1306_Fill(Black);
			ssd1306_UpdateScreen(&hi2c2);
		}
		else if (blinkCount ==2)
		{
			blinkCount = 0;
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
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 25;
	RCC_OscInitStruct.PLL.PLLN = 168;
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

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef* hadc)
{
	if (hadc->Instance == ADC1)
	{
		AdcConvCmplt =255;
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
