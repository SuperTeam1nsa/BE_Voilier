#include "check_battery.h"

#include <stm32f1xx_ll_usart.h>
#include <stm32f1xx_ll_gpio.h>
#include "stm32f1xx_ll_adc.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*===========USART=========*/

void Usart_Init() {

	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	
	LL_GPIO_InitTypeDef gpio;
	
	gpio.Pin = LL_GPIO_PIN_11;
	gpio.Mode = LL_GPIO_MODE_OUTPUT;
	gpio.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
	gpio.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	gpio.Pull = LL_GPIO_PULL_DOWN;
	
	LL_GPIO_Init(GPIOA, &gpio);
	
	gpio.Pin = LL_GPIO_PIN_9;
	gpio.Mode = LL_GPIO_MODE_ALTERNATE;
	gpio.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
	gpio.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	gpio.Pull = LL_GPIO_PULL_DOWN;
	LL_GPIO_Init(GPIOA, &gpio);
	
	LL_USART_InitTypeDef USART_InitStruct;
	
	LL_USART_Enable(USART1);

 	LL_USART_StructInit(&USART_InitStruct); 
	
	LL_USART_SetBaudRate(USART1,72000000,9600);
	LL_USART_Init (USART1, &USART_InitStruct);
	LL_USART_EnableDirectionTx (USART1);
	
}

//linked with USART
void s_char(char a){
	
	while(LL_USART_IsActiveFlag_TXE(USART1))
		LL_USART_TransmitData8(USART1,a);
	
	while(!LL_USART_IsActiveFlag_TC(USART1));
}	

void send_msg (char * msg){
	int i = 0;
	while( i < strlen(msg)) {
		s_char(msg[i]);
		i++;
	}
}

/*===========BATTERY=========*/

void init_battery(){
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
	LL_GPIO_SetPinMode(GPIOC, 2, LL_GPIO_MODE_FLOATING);
	ADC2->CR2 |= ADC_CR2_ADON;
}

//return a boolean
int battery(){
	ADC2->SQR3 = 0 ;
	ADC2->SQR3 |= (1<<3) | (1<< 2);
	ADC2->CR2 |= ADC_CR2_ADON;
	ADC2->SMPR1 |= (7<<6) ;
	while((ADC2->SR & ADC_SR_EOC) == 1){}
	
	unsigned int level = ADC2->DR;;	
	return (level < 0x0700);
}


