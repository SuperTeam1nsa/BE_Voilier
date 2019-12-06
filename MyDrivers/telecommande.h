  
#ifndef TELECOMMANDE_H
#define TELECOMMANDE_H


#include "stm32f1xx_ll_gpio.h" 
extern float position_centrale_telecommande;
float Loop_Pwm(void);
void MyTimer_PWM_Init_Input(void);
float Raw_Pwm(void);

#endif
