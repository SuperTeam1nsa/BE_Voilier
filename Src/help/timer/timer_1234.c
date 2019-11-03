#include "timer_1234.h"


void Timer_1234_enable(TIM_TypeDef* Tim)
{
	if (Tim == TIM1)
		RCC->APB2ENR |=  RCC_APB2ENR_TIM1EN;
	else if (Tim == TIM2)
		RCC->APB1ENR |=  RCC_APB1ENR_TIM2EN;
	else if (Tim == TIM3)
		RCC->APB1ENR |=  RCC_APB1ENR_TIM3EN;
	else if (Tim == TIM4)
		RCC->APB1ENR |=  RCC_APB1ENR_TIM4EN;
}
	
u8 Timer_1234_ID(TIM_TypeDef * Tim)
{
	if (Tim == TIM2)
		return 28;
	else if (Tim == TIM3)
		return 29;
	else if (Tim == TIM4)
		return 30;
	
	return -1;
}


/**
 * Fonctions callback des timers.
 */
void (* Timer_Function[3])(void);

/** 
* Configure la routine d'interruption d'un Timer
* @param Timer Pointeur vers le jeu de registres (de type TIM_TypeDef) du 
* timer considéré
* @param Priority Niveau de priorité de l'interruption
* @param IT_function Pointeur sur la fonction qui sera exécutée dans le routine
d'interruption
**/
void Timer_Active_IT(TIM_TypeDef * Timer, u8 Priority, void (*IT_function) (void))
{
	Timer_1234_enable(Timer);
	u8 idTimer = Timer_1234_ID(Timer);
	u8 idTimerBase0 = idTimer - Timer_1234_ID(TIM2);
	
	// On donne la fonction d'interruption.
	Timer_Function[idTimerBase0] = IT_function;
	
	// On active les interruptions
	Timer->DIER |= 1;
	
	// On set la priorité.
	NVIC->IP[idTimer] = Priority << 4;
	NVIC->ISER[0] = 0x01 << idTimer;
}

void TIM2_IRQHandler(void)
{
	// On clear le bit de status (qui indique le débordement)
  // pour éviter que l'IT soit rappelé systématiquement.
	TIM2->SR = TIM2->SR & ~(TIM_SR_UIF);
	Timer_Function[0]();
}

void TIM3_IRQHandler(void)
{
	TIM3->SR = TIM3->SR & ~(TIM_SR_UIF);
	Timer_Function[1]();
}

void TIM4_IRQHandler(void)
{
	TIM4->SR = TIM4->SR & ~(TIM_SR_UIF);
	Timer_Function[2]();
}

/** 
* @brief Configure les Timers 1, 2, 3 et 4
* @param Tim Pointeur vers le jeu de registres (de type TIM_TypeDef) du 
* timer considéré
* @param Period_us Intervalle de temps exprimé en µs entre
* deux débordements successifs
* @return Le durée véritable qui a été configurée (en microsecondes)
**/
float Timer_1234_Init(TIM_TypeDef* Tim, float Period_us)
{
	float PSC, ARR;
	
	Timer_1234_enable(Tim);
	// Met le bit CEN à 1 (autorise le comptage de l'horloge) 
	Tim->CR1 |= 1; 
	
	// Période de l'entrée de l'horloge.
	float tin = 1.0 / 72.0;
	float rap = Period_us / tin;
	
	// On divise par la valeur max de arr pour avoir la valeur min de psc	
	PSC = (rap / 65536.0f); 
	PSC = (uint16_t)PSC + 1;
	ARR = (rap / (float)PSC);
	
	// On affecte PSC et ARR
	Tim->PSC = (uint16_t)PSC - 1;
	Tim->ARR = (uint16_t)ARR - 1;
	
	// On calcule la période réelle à laquelle est lancé le timer (en usec).
	float real_period = tin * (Tim->PSC+1) * (Tim->ARR+1);
	
	return real_period;
}

/**
 * @brief Retourne vrai si le timer a récemment subi un débordement.
 * Si c'est le cas, les prochains appels retourneront false jusqu'au prochain débordement.
 * @param Tim Pointeur vers le jeu de registres (TIM_TypeDef*) du timer considéré.
 */
int Timer_1234_Poll(TIM_TypeDef* Tim)
{
	if ((Tim->SR & 1) == 1) //verfifie le flag uif (bit d'interruption -> la periode est écoulée)
	{
		Tim->SR = Tim->SR & ~ 1;
		return 1;
	}
	return 0;
}