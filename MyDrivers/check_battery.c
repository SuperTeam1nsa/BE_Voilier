#include "check_battery.h"

/*===========Batterie=========*/
bool Loop_MyBattery_Is_Low(void) 
{ 
		float niveau_battery = (ADC2->DR << ADC_DR_DATA_Pos);
		return (niveau_battery < 0.74); 
	// max 12v avec 0.92V qui rentrent dans l'ADC 
	// trigger à 9.6V -> 0.74 V qui rentrent dans l'ADC
}
void MyADC_init_battery(ADC_TypeDef * ADC){
	// On va utiliser l'ADC 2 Pour la batterie 
	
	//Mettre en mode adc
	RCC->APB2ENR |= RCC_APB2ENR_ADC2EN ;
	ADC->CR2 |= ADC->CR2 | ADC_CR2_ADON ;
	
	LL_RCC_SetADCClockSource(1);
	
	ADC-> CR2 |= ADC_CR2_EXTTRIG ;  
}
void Usart_Init(USART_TypeDef *ll_usart) {
	//Clock Enabled
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

	LL_USART_InitTypeDef ll_usart1;
	LL_USART_EnableSCLKOutput(ll_usart) ; 
	LL_USART_StructInit(&ll_usart1);
	LL_USART_Init(ll_usart,&ll_usart1);

	// Port Pa9, entrée de l'USART 
	LL_GPIO_InitTypeDef ll_PA9;
	ll_PA9.Mode = LL_GPIO_MODE_ALTERNATE ; 
	ll_PA9.OutputType = LL_GPIO_OUTPUT_PUSHPULL  ; 
	ll_PA9.Pin = LL_GPIO_PIN_9 ; 
	ll_PA9.Pull = LL_GPIO_PULL_DOWN ; 
	ll_PA9.Speed = LL_GPIO_MODE_OUTPUT_2MHz;
	LL_GPIO_Init(GPIOA,&ll_PA9);  	
}

void Usart_Transmit_Low_Battery(USART_TypeDef *ll_usart){
	//test : On veut transmettre un 'A' 
	LL_USART_TransmitData8(ll_usart, 0x41);
}

void Usart_Transmit_High_Battery(USART_TypeDef *ll_usart){
	//test : On veut transmettre un 'B' 
	LL_USART_TransmitData8(ll_usart, 0x42);
}

