#include "telecommande.h"
#include "MyTimer.h"

float Loop_Pwm(void) {
		//extraire angle	
		 float duree_imp=TIM4->CCR2;
		 float period = TIM4->CCR1 ;
		 float ratio=duree_imp/period; 
	
	//position neutre si télécommande au milieu, absence de signal ou valeur aberrante
	//ratio entre  0.050 et 0.1 en normal
	//ne tourne pas 
			if ((ratio > 0.070 && ratio <0.080) || (ratio <0.040) || (ratio >0.11)) {
			ratio = 0.075;
		}
		return ratio ;
	
}

//Paramétrer TIM4 Ch1 en PWM input
void MyTimer_PWM_Init_Input(void) {
// on clocke le Tim4
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN ;
	 MyTimer_Conf(TIM4,1439,1499);	

	//
	TIM4->CCMR1 |= TIM4->CCMR1 | TIM_CCMR1_CC1S_0 ;
	TIM4->CCMR1 &= TIM4->CCMR1 | ~TIM_CCMR1_CC1S_1 ;
	
	//Select the active polarity for TI1FP1
	TIM4->CCER &= TIM4->CCER | ~TIM_CCER_CC1P;
	
	//CEN à 1
	TIM4->CR1 |= TIM4->CR1 | TIM_CR1_CEN;
	
	//Select the active input for TIMx_CCR2
	TIM4->CCMR1 &= TIM4->CCMR1 | ~TIM_CCMR1_CC2S_0;
	TIM4->CCMR1 |= TIM4->CCMR1 | TIM_CCMR1_CC2S_1;
	
	//Select the active polarity for TI1FP2
	TIM4->CCER |= TIM4->CCER | TIM_CCER_CC2P;

	//Select the valid trigger input
	TIM4->SMCR |= TIM4->SMCR | TIM_SMCR_TS_0;
	TIM4->SMCR &= TIM4->SMCR | ~TIM_SMCR_TS_1;
	TIM4->SMCR |= TIM4->SMCR | TIM_SMCR_TS_2;

	//Configure the slave mode controller in reset mode
	TIM4->SMCR &= TIM4->SMCR | ~TIM_SMCR_SMS_0;
	TIM4->SMCR &= TIM4->SMCR | ~TIM_SMCR_SMS_1;
	TIM4->SMCR |= TIM4->SMCR |  TIM_SMCR_SMS_2;

	//Enable the captures
	TIM4->CCER |= TIM4->CCER | TIM_CCER_CC1E;
	TIM4->CCER |= TIM4->CCER | TIM_CCER_CC2E;

	TIM4->CCR1 &= TIM4->CCR1 | ~TIM_CCR1_CCR1;
	TIM4->CCR1 |= TIM4->CCR1 | (11000000 << TIM_CCR1_CCR1_Pos) ;; 
	
	TIM4->CCR2 &= TIM4->CCR2 | ~TIM_CCR2_CCR2;
}
