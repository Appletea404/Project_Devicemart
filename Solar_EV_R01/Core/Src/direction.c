/*
 * direction.c
 *
 *  Created on: Feb 24, 2026
 *      Author: lenovo
 */


#include "direction.h"
#include "gpio.h"



/* 핀 매핑 */
#define IN1_GPIO_Port GPIOA
#define IN1_Pin       GPIO_PIN_4

#define IN2_GPIO_Port GPIOB
#define IN2_Pin       GPIO_PIN_0

#define IN3_GPIO_Port GPIOC
#define IN3_Pin       GPIO_PIN_1

#define IN4_GPIO_Port GPIOC
#define IN4_Pin       GPIO_PIN_0


/* ============================= */
/* 내부 모터 동작 함수           */
/* ============================= */






static void Left_Front(void)
{
  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);
}

static void Left_Back(void)
{
  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
}

static void Left_Stop(void)
{
  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
}

static void Right_Front(void)
{
  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_SET);
}

static void Right_Back(void)
{
  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);
}

static void Right_Stop(void)
{
  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);
}


/* ============================= */
/* 외부 API                      */
/* ============================= */

void Direction_Init(void)
{
  Direction_Set(DIR_STOP);
}

void Direction_Set(dir_state_t st)
{
  switch (st)
  {
    case DIR_STOP:
      Left_Stop();
      Right_Stop();
      break;

    case DIR_FRONT:
      Left_Front();
      Right_Front();
      break;

    case DIR_BACK:
      Left_Back();
      Right_Back();
      break;

    /* 제자리 회전 */
    case DIR_LEFT:
      Left_Stop();
      Right_Front();
      break;

    case DIR_RIGHT:
      Left_Front();
      Right_Stop();
      break;

    /* 대각 전진 */
    case DIR_LEFTFRONT:
      Left_Front();
      Right_Front();
      break;

    case DIR_RIGHTFRONT:
      Left_Front();
      Right_Front();
      break;

    /* 대각 후진 */
    case DIR_LEFTBACK:
      Left_Back();
      Right_Back();
      break;

    case DIR_RIGHTBACK:
      Left_Back();
      Right_Back();
      break;

    default:
      Left_Stop();
      Right_Stop();
      break;
  }
}





