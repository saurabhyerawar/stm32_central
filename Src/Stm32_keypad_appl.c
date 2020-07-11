/*
 * Stm32_keypad_appl.c
 *
 *  Created on: Jul 5, 2020
 *      Author: sanka
 */

#include "Stm32_keypad_appl.h"

static void kepad_appl_init(void);

static void delay(uint32_t factor);

static void delay(uint32_t factor)
{
	for(uint32_t i = 0 ; i < (250000*factor) ; i ++);
}


static bool my_switch[12];

void kepad_function(void)
{
	Keypad_WiresTypeDef mykepad;


	memset(&mykepad,0, sizeof(mykepad));

	kepad_appl_init();


	mykepad.IN0_Port = GPIOB;
	mykepad.IN1_Port = GPIOB;
	mykepad.IN2_Port = GPIOB;
	mykepad.IN3_Port = GPIOB;

	mykepad.OUT0_Port = GPIOC;
	mykepad.OUT1_Port = GPIOC;
	mykepad.OUT2_Port = GPIOC;


	mykepad.IN0pin = GPIO_PIN_NO_14;
	mykepad.IN1pin = GPIO_PIN_NO_15;
	mykepad.IN2pin = GPIO_PIN_NO_1;
	mykepad.IN3pin = GPIO_PIN_NO_2;

	mykepad.OUT0pin = GPIO_PIN_NO_8;
	mykepad.OUT1pin = GPIO_PIN_NO_6;
	mykepad.OUT2pin = GPIO_PIN_NO_5;

	Keypad4x3_Init(&mykepad);

	printf("Press Key to see pressed key number\n");



   while(1)
   {

	   Keypad4x3_ReadKeypad(my_switch);


	   for(uint8_t i=0; i<12; i++)
	   {
		   if(my_switch[i]==0)
		  {
			   delay(3);
			   printf("Key pressed is %d\n",(uint8_t)++i);
			   break;
		   }
	   }


   }





}


static void kepad_appl_init(void)
{
	GPIO_Handle_t  GPIO_col_1;
	GPIO_Handle_t  GPIO_col_2;
	GPIO_Handle_t  GPIO_col_3;

	GPIO_Handle_t  GPIO_row_1;
	GPIO_Handle_t  GPIO_row_2;
	GPIO_Handle_t  GPIO_row_3;
	GPIO_Handle_t  GPIO_row_4;

	memset(&GPIO_col_1,0, sizeof(GPIO_col_1));
	memset(&GPIO_col_2,0, sizeof(GPIO_col_2));
	memset(&GPIO_col_3,0, sizeof(GPIO_col_3));

	memset(&GPIO_row_1,0, sizeof(GPIO_row_1));
	memset(&GPIO_row_2,0, sizeof(GPIO_row_2));
	memset(&GPIO_row_3,0, sizeof(GPIO_row_3));
	memset(&GPIO_row_4,0, sizeof(GPIO_row_4));



	GPIO_col_1.pGPIOx = GPIOC;
	GPIO_col_1.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIO_col_1.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	GPIO_col_1.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIO_col_1.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_col_1.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GPIO_Init(&GPIO_col_1);

	GPIO_col_2.pGPIOx = GPIOC;
	GPIO_col_2.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIO_col_2.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_col_2.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIO_col_2.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_col_2.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GPIO_Init(&GPIO_col_2);

	GPIO_col_3.pGPIOx = GPIOC;
	GPIO_col_3.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIO_col_3.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIO_col_3.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIO_col_3.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_col_3.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GPIO_Init(&GPIO_col_3);

    GPIO_row_1.pGPIOx = GPIOB;
    GPIO_row_1.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    GPIO_row_1.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
    GPIO_row_1.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
    GPIO_row_1.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GPIO_Init(&GPIO_row_1);

    GPIO_row_2.pGPIOx = GPIOB;
    GPIO_row_2.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    GPIO_row_2.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_1;
    GPIO_row_2.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
    GPIO_row_2.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GPIO_Init(&GPIO_row_2);

    GPIO_row_3.pGPIOx = GPIOB;
    GPIO_row_3.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    GPIO_row_3.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
    GPIO_row_3.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
    GPIO_row_3.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GPIO_Init(&GPIO_row_3);

    GPIO_row_4.pGPIOx = GPIOB;
    GPIO_row_4.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    GPIO_row_4.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
    GPIO_row_4.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
    GPIO_row_4.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GPIO_Init(&GPIO_row_4);


}


