  
#ifndef CC_H
#define CC_H


#include "stm32f1xx_ll_gpio.h" 
#include "stm32f1xx_ll_bus.h" // Pour l'activation des horloges
#include "stm32f1xx_ll_tim.h" 
#include "MyTimer.h"
void CC(float rate);
void Pwm_Configure_CC(void);



#endif
