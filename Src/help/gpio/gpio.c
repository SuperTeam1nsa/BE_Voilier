#include "gpio.h"

void GPIO_enable(GPIO_TypeDef* Port)
{
	if (Port == GPIOA)
		RCC->APB2ENR |=	RCC_APB2ENR_IOPAEN;
	else if (Port == GPIOB)
		RCC->APB2ENR |=  RCC_APB2ENR_IOPBEN;
	else if (Port == GPIOC)
		RCC->APB2ENR |=  RCC_APB2ENR_IOPCEN;
	else if (Port == GPIOD)
		RCC->APB2ENR |=  RCC_APB2ENR_IOPDEN;
}	
	
// TODO : initialiser la clock des périph ici
char Port_IO_Init_Output(GPIO_TypeDef * Port, u8 Broche)
{
	GPIO_enable(Port);
	
	if (Broche > 7) 
	{
		Broche -= 8;
		Port->CRH = (Port->CRH & ~(0xF << Broche*4))| (0b0010 << Broche*4);
	}
	else 
	{
		Port->CRL = (Port->CRL & ~(0xF << Broche*4))| (0b0010 << Broche*4);
	}
	return 1;
}

/**
 * @brief Configure la broche du port donné en mode Output push-pull (alternate function).
 */
char Port_IO_Init_Output_Alt(GPIO_TypeDef * Port, u8 Broche)
{
	GPIO_enable(Port);
	
	if (Broche > 7) 
	{
		Broche -= 8;
		Port->CRH = (Port->CRH & ~(0xF << Broche*4))| (0b1010 << Broche*4);
	}
	else 
	{
		Port->CRL = (Port->CRL & ~(0xF << Broche*4))| (0b1010 << Broche*4);
	}
	return 1;
	
}

char Port_IO_Init_Input(GPIO_TypeDef * Port, u8 Broche)
{
	GPIO_enable(Port);
	
	if (Broche > 7) 
	{
		Broche -= 8;
		Port->CRH = (Port->CRH & ~0xF) | (0b01 << 2) | (0b00);
	}
	else 
	{
		Port->CRL = (Port->CRL & ~0xF) | (0b01 << 2) | (0b00);
	}
	return 1;
}

void Port_IO_Set(GPIO_TypeDef * Port, u8 Broche)
{
	Port->ODR = Port->ODR | (0x01 << Broche);
}


void Port_IO_Reset(GPIO_TypeDef * Port, u8 Broche)
{
	Port->ODR = Port->ODR & (~(0x1 << Broche));
}

void Port_IO_Blink(GPIO_TypeDef * Port, u8 Broche)
{
	Port->ODR = Port->ODR ^ (0x1 << Broche);
}

unsigned int Port_IO_Read(GPIO_TypeDef * Port, u8 Broche)
{
	return (Port->IDR  >> Broche) & (0b1) ; 
}


void Port_IO_SetValue(GPIO_TypeDef * Port, u8 Broche, u8 Value) 
{
	if(Value)
		return Port_IO_Set(Port, Broche);
	else
		return Port_IO_Reset(Port, Broche);
}
