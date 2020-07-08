/*
Library:						4x3 Keypad drive for STM32 MCUs
Written by:					Mohamed Yaqoob
Date written:				03/04/2018
Description:				The MY_Keypad4x3 library consists of the following public functions
										Function(1)- Keypad4x3_Init
										Function(2)- Keypad4x3_ReadKeypad
										Function(3)- Keypad4x3_GetChar
*/

//***** Header files *****//
#include "Stm32_Keypad4x3.h"



//***** Library variables *****//
//1. Keypad pinout variable
static Keypad_WiresTypeDef KeypadStruct;
//2. OUT pins position, or pin number in decimal for use in colomn change function
static uint8_t OutPositions[3];
//
static char *Keypad_keys[12] =
{
	"1",
	"2",
	"3",
	"4",
	"5",
	"6",
	"7",
	"8",
	"9",
	"*",
	"0",
	"#"
};

//Function(2): Get pin positions for colomn change use, only for out pins
static void Keypad4x3_FindPins_positions(void);
//Function(3): Change colomn number
static void Keypad4x3_ChangeColomn(uint8_t colNum_0_to_2);

//***** Functions definition *****//
//Function(1): Set Keypad pins and ports
void Keypad4x3_Init(Keypad_WiresTypeDef  *KeypadWiringStruct)
{
	//Step(1): Copy the Keypad wirings to the library
	KeypadStruct = *KeypadWiringStruct;
	//Step(2): Find the positions of the 4 OUT pins
	Keypad4x3_FindPins_positions();

	/*//Step(3): set  all pins to output drain..so as to disable output
	KeypadStruct.OUT0_Port->OTYPER |= (1UL << OutPositions[0]);
	KeypadStruct.OUT1_Port->OTYPER |= (1UL << OutPositions[1]);
	KeypadStruct.OUT2_Port->OTYPER |= (1UL << OutPositions[2]);*/

	
	GPIO_WriteToOutputPin(KeypadStruct.OUT0_Port, KeypadStruct.OUT0pin, GPIO_PIN_SET);
	GPIO_WriteToOutputPin(KeypadStruct.OUT1_Port, KeypadStruct.OUT1pin, GPIO_PIN_SET);
	GPIO_WriteToOutputPin(KeypadStruct.OUT2_Port, KeypadStruct.OUT2pin, GPIO_PIN_SET);

}
//Function(2): Get pin positions for colomn change use, only for out pins
static void Keypad4x3_FindPins_positions(void)
{

	OutPositions[0] = 8;
	OutPositions[1] = 6;
	OutPositions[2] = 5;

/*	uint8_t idx=0;
	for(idx=0; idx<16; idx++)
	{
		if(((KeypadStruct.OUT0pin>>idx)&0x0001) == 0x0001)
		{
			OutPositions[0] = idx;
		}
		if(((KeypadStruct.OUT1pin>>idx)&0x0001) == 0x0001)
		{
			OutPositions[1] = idx;
		}
		if(((KeypadStruct.OUT2pin>>idx)&0x0001) == 0x0001)
		{
			OutPositions[2] = idx;
		}

	}*/
}
//Function(3): Change colomn number
static void Keypad4x3_ChangeColomn(uint8_t colNum_0_to_2)
{
	if(colNum_0_to_2==0)
	{

		    GPIO_WriteToOutputPin(KeypadStruct.OUT0_Port, KeypadStruct.OUT0pin, GPIO_PIN_RESET);
			GPIO_WriteToOutputPin(KeypadStruct.OUT1_Port, KeypadStruct.OUT1pin, GPIO_PIN_SET);
			GPIO_WriteToOutputPin(KeypadStruct.OUT2_Port, KeypadStruct.OUT2pin, GPIO_PIN_SET);
		/*//Set selected colomn
		KeypadStruct.OUT0_Port->OTYPER &= ~(1UL << OutPositions[0]);
		
		//Make other colomns floating
		KeypadStruct.OUT1_Port->OTYPER |= (1UL << OutPositions[1]);
		KeypadStruct.OUT2_Port->OTYPER |= (1UL << OutPositions[2]);
		*/
	}
	else if(colNum_0_to_2==1)
	{
		GPIO_WriteToOutputPin(KeypadStruct.OUT0_Port, KeypadStruct.OUT0pin, GPIO_PIN_SET);
		GPIO_WriteToOutputPin(KeypadStruct.OUT1_Port, KeypadStruct.OUT1pin, GPIO_PIN_RESET);
		GPIO_WriteToOutputPin(KeypadStruct.OUT2_Port, KeypadStruct.OUT2pin, GPIO_PIN_SET);
		/*//Set selected colomn
		KeypadStruct.OUT1_Port->OTYPER &= ~(1UL << OutPositions[1]);
		
		//Make other colomns floating
		KeypadStruct.OUT0_Port->OTYPER |= (1UL << OutPositions[0]);
		KeypadStruct.OUT2_Port->OTYPER |= (1UL << OutPositions[2]);*/

	}
	else if(colNum_0_to_2==2)
	{
		GPIO_WriteToOutputPin(KeypadStruct.OUT0_Port, KeypadStruct.OUT0pin, GPIO_PIN_SET);
		GPIO_WriteToOutputPin(KeypadStruct.OUT1_Port, KeypadStruct.OUT1pin, GPIO_PIN_SET);
		GPIO_WriteToOutputPin(KeypadStruct.OUT2_Port, KeypadStruct.OUT2pin, GPIO_PIN_RESET);
		/*//Set selected colomn
		KeypadStruct.OUT2_Port->OTYPER &= ~(1UL << OutPositions[2]);
		
		//Make other colomns floating
		KeypadStruct.OUT0_Port->OTYPER |= (1UL << OutPositions[0]);
		KeypadStruct.OUT1_Port->OTYPER |= (1UL << OutPositions[1]);*/
		
	}

}

//Function(4): Read active keypad button
void Keypad4x3_ReadKeypad(bool keys[12])
{
	//Step(1): Make Col0 High and check the rows
	Keypad4x3_ChangeColomn(0);
	keys[2] = GPIO_ReadFromInputPin(KeypadStruct.IN0_Port, KeypadStruct.IN0pin);
	keys[5] = GPIO_ReadFromInputPin(KeypadStruct.IN1_Port, KeypadStruct.IN1pin);
	keys[8] = GPIO_ReadFromInputPin(KeypadStruct.IN2_Port, KeypadStruct.IN2pin);
	keys[11] = GPIO_ReadFromInputPin(KeypadStruct.IN3_Port, KeypadStruct.IN3pin);

	
	//Step(2): Make Col1 High and check the rows
	Keypad4x3_ChangeColomn(1);
	keys[1] = GPIO_ReadFromInputPin(KeypadStruct.IN0_Port, KeypadStruct.IN0pin);
	keys[4] = GPIO_ReadFromInputPin(KeypadStruct.IN1_Port, KeypadStruct.IN1pin);
	keys[7] = GPIO_ReadFromInputPin(KeypadStruct.IN2_Port, KeypadStruct.IN2pin);
	keys[10] = GPIO_ReadFromInputPin(KeypadStruct.IN3_Port, KeypadStruct.IN3pin);
	
	//Step(3): Make Col2 High and check the rows
	Keypad4x3_ChangeColomn(2);
	keys[0] = GPIO_ReadFromInputPin(KeypadStruct.IN0_Port, KeypadStruct.IN0pin);
	keys[3] = GPIO_ReadFromInputPin(KeypadStruct.IN1_Port, KeypadStruct.IN1pin);
	keys[6] = GPIO_ReadFromInputPin(KeypadStruct.IN2_Port, KeypadStruct.IN2pin);
	keys[9] = GPIO_ReadFromInputPin(KeypadStruct.IN3_Port, KeypadStruct.IN3pin);

}	
//Function(5): Get character
char* Keypad4x3_GetChar(uint8_t keypadSw)
{
	return Keypad_keys[keypadSw];
}

