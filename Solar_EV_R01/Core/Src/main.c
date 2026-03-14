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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "car.h"
#include "direction.h"
#include "speed.h"
#include "delay.h"
#include "stdio.h"
//#include "ledbar.h"
#include "ultrasonic.h"
#include "statemachine.h"
#include "ina219.h"
#include "power_control.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* 센서 업데이트 / 디버그 출력 주기 */
#define SENSOR_UPDATE_PERIOD_MS   10U
#define DEBUG_PRINT_PERIOD_MS    500U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
    if (ch == '\n')
    {
        HAL_UART_Transmit(&huart2, (uint8_t*)"\r", 1, 0xFFFF);
    }
    HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 0xFFFF);
    return ch;
}

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

INA219_t solar_sensor;      // I2C2
INA219_t battery_sensor;    // I2C3
PowerControl_t p_ctrl;

/* 1ms scheduler flags */
volatile uint8_t g_flag_ctrl_1ms   = 0U;
volatile uint8_t g_flag_sense_10ms = 0U;
volatile uint8_t g_flag_dbg_500ms  = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

static void App_PrintStatus(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void App_PrintStatus(void)
{
    printf("[MODE:%s] "
           "Vin=%.2fV Iin=%.0fmA Pin=%.3fW | "
           "Vbat=%.3fV Ichg=%.0fmA | "
           "Duty=%.3f | Iref=%.3fA Icv=%.3fA Imppt=%.3fA\n",
           PowerControl_ModeString(p_ctrl.mode),
           solar_sensor.voltage_v,
           solar_sensor.current_ma,
           solar_sensor.power_w,
           battery_sensor.voltage_v,
           battery_sensor.current_ma,
           p_ctrl.duty_out,
           p_ctrl.i_ref_last,
           p_ctrl.i_cv_last,
           p_ctrl.mppt_i_ref);
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

  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM11_Init();
  MX_USART1_UART_Init();
  MX_I2C3_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM10_Init();

  /* USER CODE BEGIN 2 */

  /* PWM start */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  __HAL_TIM_MOE_ENABLE(&htim1);

  /* base timers */
  HAL_TIM_Base_Start_IT(&htim10);   // 1ms scheduler
  HAL_TIM_Base_Start(&htim11);      // for delay_us if used elsewhere

  /* optional existing project init */
  STMACHINE_Init();

  /* INA219 init */
  INA219_Init(&solar_sensor, &hi2c2, INA219_ADDR_7BIT);
  INA219_Init(&battery_sensor, &hi2c3, INA219_ADDR_7BIT);

  /* Power control init */
  PowerControl_Init(&p_ctrl);

  printf("System start.\n");
  printf("Solar charger control init done.\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      /* 10ms: sensor update in main loop (not in ISR) */
      if (g_flag_sense_10ms)
      {
          g_flag_sense_10ms = 0U;

          INA219_Update(&solar_sensor);
          INA219_Update(&battery_sensor);

          /* battery side only filtered */
          INA219_BatteryFilterUpdate(&battery_sensor);
      }

      /* 500ms: debug print */
      if (g_flag_dbg_500ms)
      {
          g_flag_dbg_500ms = 0U;
          App_PrintStatus();
      }

      /* other app tasks if needed */
      // SHOW_UART2();
      // ST_MACHINE();

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

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

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

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK
                              | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1
                              | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM10)
    {
        static uint16_t cnt_10ms  = 0U;
        static uint16_t cnt_500ms = 0U;

        /* 1ms control task */
        g_flag_ctrl_1ms = 1U;

        /* run control immediately with latest sensor values */
        if (g_flag_ctrl_1ms)
        {
            g_flag_ctrl_1ms = 0U;
            PowerControl_Run(&p_ctrl, &solar_sensor, &battery_sensor);
        }

        /* 10ms sensor flag */
        cnt_10ms++;
        if (cnt_10ms >= SENSOR_UPDATE_PERIOD_MS)
        {
            cnt_10ms = 0U;
            g_flag_sense_10ms = 1U;
        }

        /* 500ms debug flag */
        cnt_500ms++;
        if (cnt_500ms >= DEBUG_PRINT_PERIOD_MS)
        {
            cnt_500ms = 0U;
            g_flag_dbg_500ms = 1U;
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
