  
#ifndef _CHECK_BATTERY_H_
#define _CHECK_BATTERY_H_
#include <stdbool.h.>
#include "stm32f1xx_ll_usart.h"
#include "stm32f1xx_ll_tim.h" 
#include "stm32f1xx_ll_bus.h" // Pour l'activation des horloges
#include "stm32f1xx_ll_gpio.h" 
#include "stm32f1xx_ll_rcc.h"


void Usart_Init(void);
void s_char(char a);
void send_msg (char * msg);
void init_battery(void);
int battery(void);
#endif
