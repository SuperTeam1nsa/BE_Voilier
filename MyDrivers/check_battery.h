  
#ifndef _CHECK_BATTERY_H_
#define _CHECK_BATTERY_H_
#include <stdbool.h.>
#include "stm32f1xx_ll_usart.h"
#include "stm32f1xx_ll_tim.h" 
#include "stm32f1xx_ll_bus.h" // Pour l'activation des horloges
#include "stm32f1xx_ll_gpio.h" 
#include "stm32f1xx_ll_rcc.h"


bool Loop_MyBattery_Is_Low(void);
void MyADC_init_battery(ADC_TypeDef * ADC);
void Usart_Init(USART_TypeDef *ll_usart);
void Usart_Transmit_Low_Battery(USART_TypeDef *ll_usart);
void Usart_Transmit_High_Battery(USART_TypeDef *ll_usart);
#endif
