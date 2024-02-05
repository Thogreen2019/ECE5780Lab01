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
#include "stm32f072xb.h"

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  HAL_Init(); // Reset of all peripherals, init the Flash and Systick
	SystemClock_Config(); //Configure the system clock
	
	RCC -> AHBENR |= 1<<19; //Enables the clock on IO Port C, which is AHBENR bit 19.
	
	//Convention: GPIOx_Register = 0b100 -> x = Peripherial (Px), Register = the register (MODER, OSPEEDR, etc), 
	//bits represent which pin is being set to what. For example, the third bit would part of the 1st pin (starting from 0)
	//Need pins 6 and 7 -> 7(xx) 6(xx) 5(00) 4(00) 3(00) 2(00) 1(00) 0(00) -> 0bxxxx000000000000
	
	//General Purpose Output (01) of Red and Blue pin (PC6 and PC7) using MODER register
	GPIOC -> MODER = 0b0101000000000000; //Sets PC6 and PC7 (Red and Blue LEDs) to general outputs.
	//Push Pull (0) on LEDs using OTYPER register (note, each pin is only represented by one bit here, only need 8 bits)
	GPIOC -> OTYPER = 0b00000000;
	//Make LEDs Lowspeed (x0) using OSPEEDR register (the high bit can be anything, for simplicity I am simply using 0).
	GPIOC -> OSPEEDR = 0b0000000000000000;
	//Remove Pull-Up/Down (00) on LEDs using PUPDR register
	GPIOC -> PUPDR = 0b0000000000000000;
	
	//Set button pin (PA0) to input (00) using MODER register, Pin 0 is first two bits
	GPIOA -> MODER = 0b00;
	//Set button pin to low speed (x0) using OSPEEDR register
	GPIOA -> OSPEEDR = 0b00;
	//Set button pin to Pull-down (10) using PUPDR register
	GPIOA -> PUPDR = 0b10;
	
	uint32_t debouncer = 0; //debouncer signal
	//Initialize LED values (one high, one low)
	GPIOC -> ODR = 0b01000000;
	while (1) {
		//Toggle LEDs on and off with timer (to be commented out if doing button press)
		HAL_Delay(200); // Delay 200ms, the only HAL Method we are aloud to use.
		if(GPIOC -> ODR == 0b10000000){ //If Red LED is on
			GPIOC -> ODR = GPIOC -> ODR >> 1; //Switch Red to off and Blue to on
		}else{ //Blue LED must be on
			GPIOC -> ODR = GPIOC -> ODR <<1; //Switch Blue to off and Red to on
		}
		
		//Toggle LEDs on button press (to be commented out if doing timer)
		/*
		debouncer = (debouncer << 1); // Always shift every loop iteration
		if (GPIOA -> IDR & 0b1) { // If input signal is set/high
			debouncer |= 0x01; // Set lowest bit of bit-vector
		}
		if (debouncer == 0xFFFFFFFF) {
			// This code triggers repeatedly when button is steady high!
		}
		if (debouncer == 0x00000000) {
			// This code triggers repeatedly when button is steady low!
		}
		if (debouncer == 0x7FFFFFFF) {
			// This code triggers only once when transitioning to steady high!
			if(GPIOC -> ODR == 0b10000000){ //If Red LED is on
				GPIOC -> ODR = GPIOC -> ODR >> 1; //Switch Red to off and Blue to on
			}else{ //Blue LED must be on
				GPIOC -> ODR = GPIOC -> ODR <<1; //Switch Blue to off and Red to on
			}
			HAL_Delay(200);
	}
		
		*/
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
