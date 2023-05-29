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

#include "stm32f4xx.h"


volatile uint8_t player1_pressed = 0;
volatile uint8_t player2_pressed = 0;

void EXTI0_IRQHandler(void) {
    if (EXTI->PR & EXTI_PR_PR0) {
        EXTI->PR |= EXTI_PR_PR0;
        player1_pressed = 1;
    }
}

void EXTI2_IRQHandler(void) {
    if (EXTI->PR & EXTI_PR_PR2) {
        EXTI->PR |= EXTI_PR_PR2;
        player2_pressed = 1;
    }
}

void init_GPIO() {
    // Enable GPIO clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // Configure player 1 button pin as input with pull-up
    GPIOA->MODER &= ~GPIO_MODER_MODER0;
    GPIOA->PUPDR |= GPIO_PUPDR_PUPDR0_0;

    // Configure player 2 button pin as input with pull-up
    GPIOA->MODER &= ~GPIO_MODER_MODER2;
    GPIOA->PUPDR |= GPIO_PUPDR_PUPDR2_0;

    // Configure player 1 LED pin as output
    GPIOA->MODER &= ~GPIO_MODER_MODER1;
    GPIOA->MODER |= GPIO_MODER_MODER1_0;

    // Configure player 2 LED pin as output
    GPIOA->MODER &= ~GPIO_MODER_MODER3;
    GPIOA->MODER |= GPIO_MODER_MODER3_0;
}

void init_EXTI() {
    // Enable SYSCFG clock
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    // Connect EXTI0 to player 1 button pin
    SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0_Msk;

    // Connect EXTI2 to player 2 button pin
    SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI2_Msk;
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI2_PA;

    // Configure EXTI0 to trigger on rising edge
    EXTI->RTSR |= EXTI_RTSR_TR0;

    // Configure EXTI2 to trigger on rising edge
    EXTI->RTSR |= EXTI_RTSR_TR2;

    // Enable EXTI0 interrupt
    EXTI->IMR |= EXTI_IMR_MR0;

    // Enable EXTI2 interrupt
    EXTI->IMR |= EXTI_IMR_MR2;

    // Set EXTI0 interrupt priority to the highest
    NVIC_SetPriority(EXTI0_IRQn, 0);

    // Set EXTI2 interrupt priority to the highest
    NVIC_SetPriority(EXTI2_IRQn, 0);

    // Enable EXTI0 interrupt in NVIC
    NVIC_EnableIRQ(EXTI0_IRQn);

    // Enable EXTI2 interrupt in NVIC
    NVIC_EnableIRQ(EXTI2_IRQn);
}

void delay(uint32_t ms) {
    for (volatile uint32_t i = 0; i < ms * 1000; i++) {
        __NOP();
    }
}

int main(void) {
    // Initialize GPIO and EXTI
    init_GPIO();
    init_EXTI();

    while (1) {
        player1_pressed = 0;
        player2_pressed = 0;

        // Wait for both players to press their buttons
        while (!(player1_pressed && player2_pressed));

        // Determine the winner based on who pressed the button first
        if (player1_pressed) {
            GPIOA->BSRR = LED1_Pin;
            delay(1000);
            GPIOA->BSRR = LED1_Pin << 16;
        }

        if (player2_pressed) {
            GPIOA->BSRR = LED2_Pin;
            delay(1000);
            GPIOA->BSRR = LED2_Pin << 16;
        }

        delay(2000);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

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
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

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
