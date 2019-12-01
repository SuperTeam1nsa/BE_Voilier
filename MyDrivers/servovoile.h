  
#ifndef SERVOVOILE_H
#define SERVOVOILE_H


#include "stm32f1xx_ll_gpio.h" 
#include "accelerometre.h"
#include "stm32f1xx_ll_bus.h" // Pour l'activation des horloges
#include "MyTimer.h"
extern int accelero_droit;
extern void servo_voile(void);
void Pwm_Configure_Servoile(void);

#endif
