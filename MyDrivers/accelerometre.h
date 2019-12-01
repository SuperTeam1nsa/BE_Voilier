  
#ifndef ACCELERO_H
#define ACCELERO_H

#include "stm32f1xx_ll_gpio.h" 
#include "stm32f1xx_ll_tim.h" 
#include "stm32f1xx_ll_rcc.h"

void MyADC_init_accelero(ADC_TypeDef * ADC);
float MyAccelero_ADC(void) ;

#endif
