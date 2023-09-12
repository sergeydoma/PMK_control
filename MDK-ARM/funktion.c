#include "main.h"

extern I2C_HandleTypeDef hi2c2;

extern TIM_HandleTypeDef htim7;

void us_delay(uint16_t d)
{
	uint16_t x = htim7.Instance->CNT;

	while((htim7.Instance->CNT - x) < d);
}

void HEX_digit(int digit, uint16_t pin)
{
	static uint8_t LED7[16] = {
				  0x3f, 0x06, 0x5B, 0x4F,
				  0x66, 0x6D, 0x7D, 0x07,
				  0x7F, 0x6F, 0x77, 0x7C,
				  0x39, 0x5E, 0x79, 0x71
		  };
	uint8_t leds = LED7[digit & 15];

	GPIOC->BRR = 0xff;
	GPIOC->BSRR = (~leds) & 0xff;

	HAL_GPIO_WritePin(GPIOC, pin, 1);

	while(leds) {
		us_delay(200);
		leds &= (leds - 1);
	}

	HAL_GPIO_WritePin(GPIOC, pin, 0);
}