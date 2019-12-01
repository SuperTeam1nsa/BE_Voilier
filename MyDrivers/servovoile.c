#include "servovoile.h"
int angle = 0;
int accelero_droit=0;
int arret_urgence=0;
float time_up=10;

void servo_voile(void){

int inclinaison =MyAccelero_ADC();
	//amplitude de 400 et valeur en absolue dépendent du capteur
	//relever un peu les voiles avant de repartir normalement
	//double trigger
	if( (inclinaison > (accelero_droit-150) && arret_urgence==0) || (inclinaison > (accelero_droit-110) && arret_urgence==1)){
	angle=TIM3->CNT;
	arret_urgence=0;
	if(angle>=315 || angle<=45)
		time_up=20;
	else if(angle<180)
		time_up=20.00+(angle-45)*(-10.00/135.00);
	else
		time_up=10.00+(angle-180)*(10.00/135.00); //__  __
	//com : courbe de cette forme :        \/      avec des coupure à 45° 180° et 315°. Si on veut une courbe puremment affine on peut mettre 180 en dénominateur (pas le cas ici)
	LL_TIM_OC_SetCompareCH1(TIM1,(int)(19999/time_up));//49=ARRc'est le ratio rq: de 10 à 20 pour avoir de 1 à 2 ms
}
	else{
		//bordage urgence (on lache les voiles) 
		arret_urgence=1;
		LL_TIM_OC_SetCompareCH1(TIM1,(int)(19999.0/20.0));
	}
}

void Pwm_Configure_Servoile(void) //TIM1
{
	//rq: TIM4 CH3 => arr, psc et cnt commun avec l'input (tim4 ch1) 

	//rq: cf photo pour LL
  RCC->APB2ENR|=RCC_APB2ENR_IOPAEN;
	//MyTimer_Conf(TIM1,999,71);//180
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);
	// Configuration des GPÏO en alternate function.
	//PB8 en alternate push pull
	LL_GPIO_SetPinMode(GPIOA,LL_GPIO_PIN_8,LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinOutputType(GPIOA,LL_GPIO_PIN_8,LL_GPIO_OUTPUT_PUSHPULL );

	
	int ARR=19999; //conçu pour avoir un kilo check doc pour freq servo voile //49
	int PSC=71;
	LL_TIM_InitTypeDef init;
	init.Prescaler= PSC;
	init.CounterMode=LL_TIM_COUNTERMODE_UP;
	init.Autoreload=ARR;
	init.ClockDivision=LL_TIM_CLOCKDIVISION_DIV1;
	init.RepetitionCounter=(uint8_t)0x00;

	LL_TIM_Init(TIM1,&init);
	LL_TIM_CC_EnableChannel(TIM1,LL_TIM_CHANNEL_CH1);
	LL_TIM_OC_SetMode(TIM1,LL_TIM_CHANNEL_CH1,LL_TIM_OCMODE_PWM1);
	LL_TIM_OC_SetCompareCH1(TIM1,ARR/10);//c'est le ratio entre 10 et 20
	LL_TIM_EnableAllOutputs(TIM1);
	LL_TIM_EnableCounter(TIM1);
	MyTimer_Start(TIM1);
}
