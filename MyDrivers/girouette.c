#include "girouette.h"
void config_gpio_girouette(void){ // rq: keil 5 ne permet plus de faire marcher le compteur en titillant les channels 1 et 2 de la simulation.
	//active l'horloge gpioA
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN ;
	//Depending on the sequence the counter counts up or down, the DIR bit in the TIMx_CR1 register is modified by hardware accordingly
	//valeur de l'angle direcctement dasn TIM3->CNT cf :
	//This means that the counter just counts continuously between 0 and the auto-reload value in the TIMx_ARR register (0 to ARR or ARR down to 0 depending on the direction)
	MyTimer_Conf(TIM3,0x167,0x1);//167=359 (0 et 360 confondus) PSC à 1 car on prend qu'un changement d'edge (et pas les 2 )
	//MyTimer_IT_Conf(TIM3, update_position_girouette,8);
	//MyTimer_IT_Enable(TIM3);

	//on compte que sur 1 seul edge p328
	TIM3->SMCR=0x001;
	//PA6 et pA7 => CH1 et CH2 TIM3 en input
	TIM3->CCMR1=0x0101;
	//l'user tourne à la maoin la girouette jusqu'à ce qu'elle repasse devant l'index, à ce moment là elle est calibrée
	//rq: vrai système on ecrit la valeur à l'arretdu systeme pour éviter recalibrage à chaque démarrage

 	while(!LL_GPIO_IsInputPinSet(GPIOA,LL_GPIO_PIN_5))//GPIOA->IDR & (1<<5) == (1<<5))
		{
	}
	//TIM3->CNT=180; //car quand vent de face, 0 et quand arrière : 180 degrés
	 // calibrée
		MyTimer_Start(TIM3);
}
