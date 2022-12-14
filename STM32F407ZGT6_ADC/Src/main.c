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
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "fsmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE BEGIN PV */
uint8_t buffer[5];
float adc_value;
uint32_t ADC_Value;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
//	HAL_GPIO_TogglePin(GPIOF,GPIO_PIN_10);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin==Key_Pin)
	{
		if(HAL_GPIO_ReadPin(Key_GPIO_Port,Key_Pin)==0)
		{
			HAL_Delay(100);
			while(HAL_GPIO_ReadPin(Key_GPIO_Port,Key_Pin)==0);
			HAL_Delay(100);
//			HAL_TIM_Base_Stop_IT(&htim2);
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_UART_Transmit_DMA(&huart1,buffer,3);
	HAL_UART_Receive_DMA(&huart1,buffer,3);
}

void PWM()
{
	uint16_t pwmVal=0;  
	while (pwmVal< 1000)
		  {
			  pwmVal++;
			  __HAL_TIM_SetCompare(&htim14, TIM_CHANNEL_1, pwmVal);  
			  HAL_Delay(1);
		  }
		  while (pwmVal)
		  {
			  pwmVal--;
			  __HAL_TIM_SetCompare(&htim14, TIM_CHANNEL_1, pwmVal);    

			  HAL_Delay(1);
		  }
		  HAL_Delay(200);
}

void rtp_test(void)
{
//	uint8_t key;
	uint8_t i=0;	  
	while(1)
	{
		
		
		tp_dev.scan(0); 	
		
		if(tp_dev.sta&TP_PRES_DOWN)			//????????????
		{	
			
		 	if(tp_dev.x[0]<lcddev.width&&tp_dev.y[0]<lcddev.height)
			{	

				if(tp_dev.x[0]>(lcddev.width-24)&&tp_dev.y[0]<16)
					 LCD_Clear(GREEN);//????
				else 
				{
					TP_Draw_Big_Point(tp_dev.x[0],tp_dev.y[0],RED);		//????	  		
					HAL_GPIO_TogglePin(GPIOF,GPIO_PIN_10);//LED????
				}
			}
		}
		else HAL_Delay(10);	//?????????????????? 	  		
		i++;
		if(i%20==0)
		{
//		  HAL_GPIO_TogglePin(GPIOF,GPIO_PIN_10);//LED????????????????????
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
  MX_USART1_UART_Init();
  MX_FSMC_Init();
  MX_TIM2_Init();
  MX_TIM14_Init();
  MX_TIM7_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_PWM_Start(&htim14,TIM_CHANNEL_1);
  HAL_UART_Receive_DMA(&huart1,buffer,3);
  TFTLCD_Init();	
  W25QXX_Init();
  TP_Init();
//  LCD_Clear(GREEN);
  LCD_ShowString(30,40,210,24,24,"What a nice day!");	
//  LCD_ShowString(30,70,200,16,16,"TFTLCD TEST");
//  LCD_ShowString(30,90,200,16,16,"2020/7/29"); 

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */


    /* USER CODE BEGIN 3 */
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 50);
	if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC))
	{
		ADC_Value = HAL_ADC_GetValue(&hadc1);
		LCD_ShowString(30,70,200,16,16,"TFTLCD TEST");
		LCD_ShowxNum(30,100,ADC_Value*FACTOR_ADC,6,24,0);
//		printf("PA5 True Voltage value : %.4f \r\n",ADC_Value*3.3f/4096);
		HAL_Delay(500);
	}

//	adc_value = get_adc_value();
//	HAL_Delay(0);
//	LCD_ShowString(30,40,210,24,24,"What a nice day!");	
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
