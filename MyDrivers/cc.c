#include "cc.h"

void CC(float rate)
{
	//rq: PA2 est aussi TIM2_CH3 (ne pas s'inquieter si TIM_CH3 s'allume)
			//max 60% moteur (*0.6)
	//marge 0.005 au centre
	if(rate <0.070)
	{LL_GPIO_SetPinPull(GPIOA,LL_GPIO_PIN_2,LL_GPIO_PULL_UP);
		rate=(rate*0.9)/0.070;
	}
	else if( rate >0.080){
		LL_GPIO_SetPinPull(GPIOA,LL_GPIO_PIN_2,LL_GPIO_PULL_DOWN);
	rate=(rate*0.9)/0.1;
	}
	if(rate>=0.072 && rate<=0.078)
		LL_TIM_OC_SetCompareCH2(TIM2,0); 
	else{

	LL_TIM_OC_SetCompareCH2(TIM2,TIM2->ARR*rate); // varie de 0.05 à 0.1 
	}
}
void Pwm_Configure_CC(void) //TIM2 CH2
{
  //rq: cf photo pour LL
  RCC->APB2ENR|=RCC_APB2ENR_IOPAEN;
	// on clocke le Tim2
	//RCC->APB1ENR |= RCC_APB1ENR_TIM2EN ;
	
	//MyTimer_Conf(TIM2,999,71);//180
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
	// Configuration des GPÏO en alternate function.
	//PB8 en alternate push pull
	LL_GPIO_SetPinMode(GPIOA,LL_GPIO_PIN_1,LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinOutputType(GPIOA,LL_GPIO_PIN_1,LL_GPIO_OUTPUT_PUSHPULL );

	LL_GPIO_SetPinMode(GPIOA,LL_GPIO_PIN_2,LL_GPIO_MODE_OUTPUT);
	//init dej faite pwm input (ici pour test)
	int ARR=19999; //conçu pour avoir un kilo check doc pour freq servo voile //49
	int PSC=71;
	LL_TIM_InitTypeDef init;
	init.Prescaler= PSC;
	init.CounterMode=LL_TIM_COUNTERMODE_UP;
	init.Autoreload=ARR;
	init.ClockDivision=LL_TIM_CLOCKDIVISION_DIV1;
	init.RepetitionCounter=(uint8_t)0x00;
	LL_TIM_Init(TIM2,&init);
	//fin init
	
	
	LL_TIM_CC_EnableChannel(TIM2,LL_TIM_CHANNEL_CH2);
	LL_TIM_OC_SetMode(TIM2,LL_TIM_CHANNEL_CH2,LL_TIM_OCMODE_PWM1);
	LL_TIM_OC_SetCompareCH2(TIM2,0);//ARR/10c'est le ratio entre 10 et 90 %
	LL_TIM_EnableAllOutputs(TIM2);
	LL_TIM_EnableCounter(TIM2);
	MyTimer_Start(TIM2);
}
