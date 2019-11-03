#include "stm32f10x.h"

#ifndef __TIMER_1234_H
#define __TIMER_1234_H

/** 
* Configure les Timers 1, 2, 3 et 4
* @param Tim Pointeur vers le jeu de registres (de type TIM_TypeDef) du 
* timer considéré
* @param Period_us Intervalle de temps exprimé en µs entre
* deux débordements successifs
* @return Le durée véritable qui a été configurée (en microsecondes)
**/
float Timer_1234_Init(TIM_TypeDef* Tim, float Period_us);

/**
 * @brief Retourne vrai si le timer a récemment subi un débordement.
 * Si c'est le cas, les prochains appels retourneront false jusqu'au prochain débordement.
 * @param Tim Pointeur vers le jeu de registres (TIM_TypeDef*) du timer considéré.
 */
int Timer_1234_Poll(TIM_TypeDef* Tim);

/** 
* Configure la routine d'interruption d'un Timer
* @param Timer Pointeur vers le jeu de registres (de type TIM_TypeDef) du 
* timer considéré
* @param Priority Niveau de priorité de l'interruption
* @param IT_function Pointeur sur la fonction qui sera exécutée dans le routine
d'interruption
**/
void Timer_Active_IT(TIM_TypeDef * Timer, u8 Priority, void (*IT_function) (void));

#endif /*__TIMER_1234_H */ 