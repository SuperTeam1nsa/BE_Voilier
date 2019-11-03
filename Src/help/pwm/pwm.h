#ifndef __PWM_H
#define __PWM_H
#include "stm32f10x.h"
#include "timer_1234.h"
#include "gpio.h"

/**
 * @brief Configure la PWM sur le timer Tim, avec la période donnée.
 * @param Tim timer sur lequel configurer la PWM.
 * Broches GPIO correspondantes : 
 * - TIM2 -> GPIOA channel 0
 * - TIM3 -> GPIOA channel 6
 * - TIM4 -> GPIOB channel 6 
 * @param Période de la PWM en microsecondes.
 */
int Pwm_Configure(TIM_TypeDef* Tim, float period_us);

/**
 * @brief Configure le rapport cyclique d'une PWM.
 * @param Tim timer sur lequel est relié la PWM
 * @param percent rapport cyclique à utiliser (valeur comprise entre 0 et 1)
 */
int Pwm_Cyclic_RateF(TIM_TypeDef* Tim, float rate);
#endif