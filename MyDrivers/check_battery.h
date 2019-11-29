  
#ifndef _CHECK_BATTERY_H_
#define _CHECK_BATTERY_H_

#include <stdio.h>
#include "stm32f1xx_hal.h"
#include "UART.h"

/*=================================================================================
Software : Keil uVision 5
Processor : STM32F103RB
===================================================================================*/
#define VMAX 104
void init_PC2(void);
void init_ADC1(ADC_HandleTypeDef* ADC_InitHandle);
unsigned int read_battery(ADC_HandleTypeDef* hadc);
void transmit_lvl_battery(ADC_HandleTypeDef* hadc, UART_HandleTypeDef* huart);

#endif