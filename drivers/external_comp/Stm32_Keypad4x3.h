/*
Library:						4x3 Keypad drive for STM32 MCUs
Written by:					Mohamed Yaqoob
Date written:				03/04/2018
Description:				The MY_Keypad4x3 library consists of the following public functions
										Function(1)- Keypad4x3_Init
										Function(2)- Keypad4x3_ReadKeypad
										Function(3)- Keypad4x3_GetChar
*/

//Header files
#include "stm32f446xx_gpio_driver.h"
#include <stdbool.h>


//***** Constant variables and typedefs *****//
typedef struct
{
	GPIO_RegDef_t* IN0_Port;
	GPIO_RegDef_t* IN1_Port;
	GPIO_RegDef_t* IN2_Port;
	GPIO_RegDef_t* IN3_Port;
	GPIO_RegDef_t* OUT0_Port;
	GPIO_RegDef_t* OUT1_Port;
	GPIO_RegDef_t* OUT2_Port;

	
	uint8_t	IN0pin;
	uint8_t	IN1pin;
	uint8_t	IN2pin;
	uint8_t	IN3pin;
	uint8_t	OUT0pin;
	uint8_t	OUT1pin;
	uint8_t	OUT2pin;

}Keypad_WiresTypeDef;

//List of keys as chars


//***** Functions prototype *****//
//Function(1): Set Keypad pins and ports
void Keypad4x3_Init(Keypad_WiresTypeDef  *KeypadWiringStruct);

//Function(4): Read active keypad button
void Keypad4x3_ReadKeypad(bool keys[12]);

//Function(5): Get character
char* Keypad4x3_GetChar(uint8_t keypadSw);



