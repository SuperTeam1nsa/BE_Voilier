#include "pwm.h"


/* -------------------------------------------------------
 * 1. Configuration des E/S timer (Ocy) en sortie.
 * -> Dans CCMRx bits CCyS pour conf en output
 * 2. Contrôle de la polarité de la sortie y (output control)
 * -> Dans CCER -> CCxP : output polarity
 * 3. Validation de la sortie
 * -> Dans CCER -> CCxE : output enable
 * 4. mettre la pwm en mode 1 
 * Dans CCMR1 -> OCxM mode PWM mode 1
 * -----------------------------------------------------*/
void Configure_Gpio(TIM_TypeDef* Tim)
{
	if (Tim == TIM2)
		Port_IO_Init_Output_Alt(GPIOA, 0);
	else if (Tim == TIM3)
		Port_IO_Init_Output_Alt(GPIOA, 6);
	else if (Tim == TIM4)
		Port_IO_Init_Output_Alt(GPIOB, 6);
}

int Pwm_Configure(TIM_TypeDef* Tim, float period_us)
{
	Timer_1234_Init(Tim, period_us);
	// Configuration des GPÏO en alternate function.
	Configure_Gpio(Tim);
	// configuration de la sortie du CC1 du timer donné en output sur le channel numéro 1  
	Tim->CCMR1 &= ~TIM_CCMR1_CC1S;
	// mise de la polarité à 0 (logique normale) tjrs sur le numéro 1 
	Tim->CCER &= ~TIM_CCER_CC1P;
	// enable la sortie du channel 1 du timer 
	Tim->CCER |= TIM_CCER_CC1E;
	// mettre la pwm en mode 1 
	Tim->CCMR1 &= ~TIM_CCMR1_OC1M;
	Tim->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
	
	return 0;
}

int Pwm_Cyclic_RateF(TIM_TypeDef* Tim, float rate) 
{
	if(rate < 0 || rate > 1)
		return 1;
	
	uint16_t arr = Tim->ARR;
	uint16_t ccr = (uint16_t)(arr * rate);
	Tim->CCR1 = ccr;
	
	return 0;
}