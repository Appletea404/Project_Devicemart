/*
 * ledbar.c
 *
 *  Created on: Feb 27, 2026
 *      Author: lenovo
 */


#include "ledbar.h"

void dataOut_LED(uint8_t data)
{
    /*
     * i = 7 → 0
     * MSB부터 LSB까지 순차적으로 전송
     */
	for(int i = 7; i >= 0; i--)
	{
        /*
         * 현재 전송할 비트를 SER 핀에 출력
         * data의 i번째 비트가 1이면 HIGH, 0이면 LOW
         *
         * 이 시점에서는 아직 74HC595 내부로 들어가지 않고
         * 단순히 입력 핀에 값만 세팅된 상태
         */
		if(data & (1 << i))
		{
			HAL_GPIO_WritePin(SER_PORT_LED, SER_PIN_LED, GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(SER_PORT_LED, SER_PIN_LED, GPIO_PIN_RESET);
		}

        /*
         * SRCLK(Shift Clock) 펄스 생성
         *
         * - SRCLK의 상승엣지에서 SER 값이
         *   내부 쉬프트 레지스터로 이동
         * - 이후 비트들은 한 칸씩 밀려남
         *
         * delay_us()는 setup/hold time 확보용 여유
         */
		HAL_GPIO_WritePin(SRCLK_PORT_LED, SRCLK_PIN_LED, GPIO_PIN_SET);
		delay_us(5);

		HAL_GPIO_WritePin(SRCLK_PORT_LED, SRCLK_PIN_LED, GPIO_PIN_RESET);
		delay_us(5);
	}

    /*
     * RCLK(Latch Clock) 펄스 생성
     *
     * - 지금까지 쉬프트 레지스터에 쌓인 8비트 데이터를
     *   출력 래치로 복사
     * - QA~QH 핀이 이 순간에만 갱신됨
     *
     * 장점:
     * - 데이터 전송 중에는 출력이 바뀌지 않음
     * - LED/FND 사용 시 중간 상태가 보이지 않음
     */
	HAL_GPIO_WritePin(RCLK_PORT_LED, RCLK_PIN_LED, GPIO_PIN_SET);
	delay_us(10);
	HAL_GPIO_WritePin(RCLK_PORT_LED, RCLK_PIN_LED, GPIO_PIN_RESET);
}






static uint8_t Pattern = 0;

static uint32_t prevTick = 0;
const static uint32_t waitTick = 200;
static uint8_t role1 = 0;
static uint8_t role2 = 0;
static int idx1 = 3;
static int idx2 = 4;


static bool Flag = 0;		//외부에서 버튼이 바꿔주는 인자




void LED_BAR_UP()			//엘리베이터가 오는동안 위로 표시
{
	if(Flag != 0)
	{
		dataOut_LED(0xF0);
	}
	else
	{
		dataOut_LED(0x00);

	}
}

void LED_BAR_DOWN()			//엘리베이터가 오는동안 아래로 표시
{
	if(Flag != 0)
	{
		dataOut_LED(0x0F);
	}
	else
	{
		dataOut_LED(0x00);

	}
}


void LED_BAR_Reset(void)
{
    Pattern = 0;
    dataOut_LED(0x00);

    // UP 영역(0~3)
    idx1 = 3;
    role1 = 0;

    // DOWN 영역(4~7)
    idx2 = 4;
    role2 = 0;

    prevTick = HAL_GetTick(); // 바로 다음 tick 폭주 방지(선택)
}

void LED_BAR_OFF(void)
{
    Pattern = 0;
    dataOut_LED(0x00);
}


void LED_BAR_UP_ing()		//위로이동시
{

	uint32_t currentTick = HAL_GetTick();		//Non-blocking
	if ((currentTick - prevTick) < waitTick) return; // 200ms 아직 안 됨
	prevTick = currentTick;

	if(role1 != 1)
	{
		Pattern |= (1 << (uint8_t)idx1);
		dataOut_LED(Pattern);

		idx1 = idx1 - 1;
		if(idx1 < 0)
		{
			idx1 = 3;
			role1 = 1;
		}
	}
	else
	{
		Pattern = Pattern & ~(1 << (uint8_t)idx1);
		dataOut_LED(Pattern);
		idx1 = idx1 - 1;

		if(idx1 < 0)
		{
			idx1 = 3;
			role1 = 0;
		}
	}

}

void LED_BAR_DOWN_ing()				//아래로 이동시
{


	uint32_t currentTick = HAL_GetTick();
	if ((currentTick - prevTick) < waitTick) return; // 200ms 아직 안 됨
	prevTick = currentTick;

	if(role2 != 1)
	{
		Pattern |= (1 << (uint8_t)idx2);
		dataOut_LED(Pattern);

		idx2 = idx2 + 1;
		if(idx2 > 7)
		{
			idx2 = 4;
			role2 = 1;
		}
	}
	else
	{
		Pattern = Pattern & ~(1 << (uint8_t)idx2);
		dataOut_LED(Pattern);
		idx2 = idx2 + 1;

		if(idx2 > 7)
		{
			idx2 = 4;
			role2 = 0;
		}
	}

}
