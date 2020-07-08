/*
 * led_appl.c
 *
 *  Created on: Jul 4, 2020
 *      Author: sanka
 */

#include"led_appl.h"


#define BUTTON_PRESSED 0
#define BUTTON_NOT_PRESSED 1

static void delay(uint32_t factor);

static void delay(uint32_t factor)
{
	for(uint32_t i = 0 ; i < (250000*factor) ; i ++);
}

void led_function(void)
{
	GPIO_Handle_t  led_GPIO;
	GPIO_Handle_t  button_GPIO;

	memset(&led_GPIO,0, sizeof(led_GPIO));
	memset(&button_GPIO,0, sizeof(button_GPIO));


/*	led_GPIO.pGPIOx = GPIOA;
	led_GPIO.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	led_GPIO.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	led_GPIO.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	led_GPIO.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	led_GPIO.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	GPIO_Init(&led_GPIO);*/


	led_GPIO.pGPIOx = GPIOA;
	led_GPIO.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	led_GPIO.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
	led_GPIO.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	led_GPIO.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	led_GPIO.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	GPIO_Init(&led_GPIO);


	button_GPIO.pGPIOx = GPIOC;
	button_GPIO.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	button_GPIO.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	button_GPIO.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	button_GPIO.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	GPIO_Init(&button_GPIO);

	while(1)
	{
		if(BUTTON_PRESSED==GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13))
		{
			delay(2);

			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_4);
		}

	}



}
