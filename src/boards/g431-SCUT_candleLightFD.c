/*

The MIT License (MIT)

Copyright (c) 2024 Quantulum Ltd,
              Phil Greenland <phil@quantulum.co.uk>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

*/

#include "board.h"
#include "config.h"
#include "device.h"
#include "gpio.h"
#include "led.h"
#include "usbd_gs_can.h"

#define LEDRX_GPIO_Port	  GPIOA
#define LEDRX_Pin		  GPIO_PIN_9
#define LEDRX_Mode		  GPIO_MODE_OUTPUT_OD
#define LEDRX_Active_High 0

#define LEDTX_GPIO_Port	  GPIOA
#define LEDTX_Pin		  GPIO_PIN_8
#define LEDTX_Mode		  GPIO_MODE_OUTPUT_OD
#define LEDTX_Active_High 0

#define CAN_TERM_Pin GPIO_PIN_3
#define CAN_TERM_GPIO_Port GPIOA

#define CAN_EN_Pin GPIO_PIN_4
#define CAN_EN_GPIO_Port GPIOA

static void SCUT_candleLightFD_setup(USBD_GS_CAN_HandleTypeDef *hcan)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	UNUSED(hcan);

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	/* GPIOs */

	HAL_GPIO_WritePin(GPIOA, CAN_TERM_Pin|CAN_EN_Pin|LEDTX_Pin|LEDRX_Pin, GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = CAN_TERM_Pin|CAN_EN_Pin|LEDTX_Pin|LEDRX_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* FDCAN */

	RCC_PeriphCLKInitTypeDef PeriphClkInit = {
		.PeriphClockSelection = RCC_PERIPHCLK_FDCAN,
		.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL,
	};

	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
	__HAL_RCC_FDCAN_CLK_ENABLE();

	/* FDCAN1_RX, FDCAN1_TX */
	GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

static void SCUT_candleLightFD_phy_power_set(can_data_t *channel, bool enable)
{
	UNUSED(channel);

	HAL_GPIO_WritePin(CAN_EN_GPIO_Port, CAN_EN_Pin, enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void SCUT_candleLightFD_termination_set(can_data_t *channel, enum gs_can_termination_state enable)
{
	UNUSED(channel);

	HAL_GPIO_WritePin(CAN_TERM_GPIO_Port, CAN_TERM_Pin, enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

const struct BoardConfig config = {
	.setup = SCUT_candleLightFD_setup,
	.phy_power_set = SCUT_candleLightFD_phy_power_set,
	.termination_set = SCUT_candleLightFD_termination_set,
	.channels[0] = {
		.interface = FDCAN1,
		.leds = {
			[LED_RX] = {
				.port = LEDRX_GPIO_Port,
				.pin = LEDRX_Pin,
				.active_high = LEDRX_Active_High,
			},
			[LED_TX] = {
				.port = LEDTX_GPIO_Port,
				.pin = LEDTX_Pin,
				.active_high = LEDTX_Active_High,
			},
		},
	}
};
