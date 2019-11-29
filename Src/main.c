/**
  ******************************************************************************
  * @file    Templates_LL/Src/main.c
  * @author  MCD Application Team
  * @brief   Main program body through the LL API
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
#include "stm32f1xx_ll_rcc.h" // utile dans la fonction SystemClock_Config
#include "stm32f1xx_ll_utils.h"   // utile dans la fonction SystemClock_Config
#include "stm32f1xx_ll_system.h" // utile dans la fonction SystemClock_Config
#include "stm32f1xx_ll_gpio.h" 
//#include "Chrono.h"
#include "MyTimer.h"
#include "stdbool.h"
#include "stm32f1xx_ll_usart.h"
#include "stm32f1xx_ll_tim.h" 
#include "stm32f1xx_ll_bus.h" // Pour l'activation des horloges
//#include "check_battery.h"
//#include "accelerometre.h"

int angle = 0;
float time_up=10;
void  SystemClock_Config(void);


//variable telecommande et accelero et battery
int accelero_droit=0;
float angleG ;
float duree_imp;
float period;
float ratio ; 
float acc;
float acc_G;
float y;
int arret_urgence=0;
bool B_battery;
float niveau_battery;
/*=========Telecommande==========*/
//fonction a looper pour le servomoteur
float Loop_Pwm(void) {
		//extraire angle	
		 duree_imp=TIM4->CCR2;
		 period = TIM4->CCR1 ;
		 ratio=duree_imp/period; 
	
	//position neutre si télécommande au milieu, absence de signal ou valeur aberrante
	//ratio entre  0.050 et 0.1 en normal
	//ne tourne pas 
			if ((ratio > 0.070 && ratio <0.080) || (ratio <0.040) || (ratio >0.11)) {
			ratio = 0.075;
		}
		return ratio ;
	
}

//Paramétrer TIM4 Ch1 en PWM input
void MyTimer_PWM_Init_Input() {
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

/*===========Accelerometre=========*/
float MyAccelero_ADC(void) 
{
		y= ADC1->DR << ADC_DR_DATA_Pos ; 
		//max 1.90V
		//min 1.45 V
		
		//return valeur y* (delta voltage reçu) / 90 degré
		//acc= (float) y*0.45/90.0;
	
	//return y -> min (bateau couché) = 1650; max (bateau normal) = 2150
	// à faire dans le main : si y<1900, ne plus border les voiles
		return y ; 
}

void MyADC_init_accelero(ADC_TypeDef * ADC){
	// On va utiliser l'ADC 1 pour l'accéléro 
	
	//Mettre en mode adc
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN ;
	ADC->CR2 |= ADC->CR2 | ADC_CR2_ADON ;
	//----------METTRE PSC A 1----------changer clock !!!!

	LL_RCC_SetAPB2Prescaler(4)	;
	
	ADC->SQR3 = 11;
	ADC-> CR2 |= ADC_CR2_EXTTRIG ; 
	
}



/*===========Batterie=========*/
bool Loop_MyBattery_Is_Low() 
{ 
		niveau_battery = (ADC2->DR << ADC_DR_DATA_Pos);
		return ((ADC2->DR << ADC_DR_DATA_Pos) < 0.74); 
	// max 12v avec 0.92V qui rentrent dans l'ADC 
	// trigger à 9.6V -> 0.74 V qui rentrent dans l'ADC
}
void MyADC_init_battery(ADC_TypeDef * ADC){
	// On va utiliser l'ADC 2 Pour la batterie 
	
	//Mettre en mode adc
	RCC->APB2ENR |= RCC_APB2ENR_ADC2EN ;
	ADC->CR2 |= ADC->CR2 | ADC_CR2_ADON ;
	
	LL_RCC_SetADCClockSource(1);
	
	ADC-> CR2 |= ADC_CR2_EXTTRIG ;  
}
void Usart_Init(USART_TypeDef *ll_usart) {
	//Clock Enabled
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

	LL_USART_InitTypeDef ll_usart1;
	LL_USART_EnableSCLKOutput(ll_usart) ; 
	LL_USART_StructInit(&ll_usart1);
	LL_USART_Init(ll_usart,&ll_usart1);

	// Port Pa9, entrée de l'USART 
	LL_GPIO_InitTypeDef ll_PA9;
	ll_PA9.Mode = LL_GPIO_MODE_ALTERNATE ; 
	ll_PA9.OutputType = LL_GPIO_OUTPUT_PUSHPULL  ; 
	ll_PA9.Pin = LL_GPIO_PIN_9 ; 
	ll_PA9.Pull = LL_GPIO_PULL_DOWN ; 
	ll_PA9.Speed = LL_GPIO_MODE_OUTPUT_2MHz;
	LL_GPIO_Init(GPIOA,&ll_PA9);  	
}

void Usart_Transmit_Low_Battery(USART_TypeDef *ll_usart){
	//test : On veut transmettre un 'A' 
	LL_USART_TransmitData8(ll_usart, 0x41);
}

void Usart_Transmit_High_Battery(USART_TypeDef *ll_usart){
	//test : On veut transmettre un 'B' 
	LL_USART_TransmitData8(ll_usart, 0x42);
}
//
/*
void update_position_girouette(void)
{
	// rabaisser le flag d'IT
	LL_TIM_ClearFlag_UPDATE(TIM3ccc);
	//lecture: TIM3-CNT
	//TIM3->CNT;
	//(*Ptr_ItFct_TIM3)(); //update position
}	*/

/* Private functions ---------------------------------------------------------*/
void config_gpio_girouette(void){ // on pense que ça marche mais c'est à tester car keil 5 ne permet plus de faire marcher le compteur en titillant les channels 1 et 2 de la simulation.
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
	
	//lance le compteur
	/* reecriture exemple p329 en assembleur...
	TIM3->CCMR1 |= TIM3->CCMR1 | TIM_CCMR1_CC1S_0 ;
	TIM3->CCMR1 &= TIM3->CCMR1 | ~TIM_CCMR1_CC1S_1 ;
	
	*/
	/*
	//active l'interruption 
	//TIM3->DIER |= 1;
	LL_TIM_EnableIT_UPDATE(TIM3);
	
	//set priorite
	NVIC->ISER[0] = 0x01;
	
	//en floating input 
	//PA5 en floating input (index)
	LL_GPIO_SetPinMode(GPIOA,LL_GPIO_PIN_5,LL_GPIO_MODE_FLOATING);
	
	
	///timer en mode incremental encode: 
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);
	//active l'horloge
	//TIM3->CR1=0x1;
	LL_TIM_InitTypeDef My_LL_Tim_Init_Struct;
	My_LL_Tim_Init_Struct.Autoreload=0xB4;//180
	My_LL_Tim_Init_Struct.Prescaler=0x1;
	My_LL_Tim_Init_Struct.ClockDivision=LL_TIM_CLOCKDIVISION_DIV1;
	//LL_TIM_IC_Config
	My_LL_Tim_Init_Struct.RepetitionCounter=0;
	LL_TIM_Init(TIM3,&My_LL_Tim_Init_Struct);
	*/
}

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
void CC(float rate)
{
	//rq: PA2 est aussi TIM2_CH3 (ne pas s'inquieter si TIM_CH3 s'allume)
			//max 60% moteur (*0.6)
	//marge 0.005 au centre
	if(rate <0.070)
	{LL_GPIO_SetPinPull(GPIOA,LL_GPIO_PIN_2,LL_GPIO_PULL_UP);
		rate=(rate*0.6)/0.070;
	}
	else if( rate >0.080){
		LL_GPIO_SetPinPull(GPIOA,LL_GPIO_PIN_2,LL_GPIO_PULL_DOWN);
	rate=(rate*0.6)/0.1;
	}
	if(rate>=0.070 && rate<=0.080)
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
	
	
	/* avec les registres ça ne marche pas... 
	// configuration de la sortie du CC1 du timer donné en output sur le channel numéro 1  
	TIM1->CCMR1 &= ~TIM_CCMR1_CC1S;
	// mise de la polarité à 0 (logique normale) tjrs sur le numéro 1 
	TIM1->CCER &= ~TIM_CCER_CC1P;
	// enable la sortie du channel 1 du timer 
	TIM1->CCER |= TIM_CCER_CC1E;
	// mettre la pwm en mode 1 
	TIM1->CCMR1 &= ~TIM_CCMR1_OC1M;
	TIM1->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
	TIM1->CCR1 = TIM1->ARR*rate;
	*/
	

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
	/*
void send(char x){
	 LL_USART_TransmitData8(USART2,x);
		while(LL_USART_IsActiveFlag_TXE(USART2)!=1); //réativité du bouton non nécesaire #1ms
}
void Chrono_Background(void){
	//Time *ct=Chrono_Read();
	if(LL_GPIO_IsInputPinSet(GPIOC,LL_GPIO_PIN_6)){
		Chrono_Stop();
	}
		if(LL_GPIO_IsInputPinSet(GPIOC,LL_GPIO_PIN_8)){
			Chrono_Start();
		}
		//B1 USER: the user button is connected to the I/O PC13 (pin 2) of the STM32
//microcontroller. (altternative logic)
		if(!LL_GPIO_IsInputPinSet(GPIOC,LL_GPIO_PIN_13)){
			Chrono_Reset();
		}
}
//peut prendre usart en arg (need modif pin pour gpio)
void configUSART(void ){
	
	//uart config 
	LL_USART_Enable(USART2);
	LL_USART_EnableDirectionTx(USART2);
	LL_USART_ConfigAsyncMode(USART2);
	
	LL_USART_InitTypeDef uart;
	LL_USART_StructInit(&uart);
	uart.BaudRate=9.6;
	uart.DataWidth=LL_USART_DATAWIDTH_8B;
	uart.HardwareFlowControl=LL_USART_HWCONTROL_NONE;
	uart.Parity=LL_USART_PARITY_NONE;
	uart.StopBits=LL_USART_STOPBITS_1;
	uart.TransferDirection=LL_USART_DIRECTION_TX;

	LL_USART_Init(USART2,&uart);
	//probleme link struct resolu en ajoutant le .c de ll_usart.h dans le projet.         A FAIRE
	//																																										par fonctions: mauvais baud rate 
	//																																										non enable
  LL_USART_Enable(USART2);
	LL_USART_EnableDirectionTx(USART2);
	LL_USART_ConfigAsyncMode(USART2);
	
	LL_USART_SetTransferDirection(USART2,LL_USART_DIRECTION_TX);
	LL_USART_SetParity(USART2,LL_USART_PARITY_NONE);
	LL_USART_SetDataWidth(USART2,LL_USART_DATAWIDTH_8B);
	LL_USART_SetStopBitsLength(USART2,LL_USART_STOPBITS_1);
	LL_USART_SetHWFlowCtrl(USART2,LL_USART_HWCONTROL_NONE);
	LL_USART_SetBaudRate(USART2,72000000,9600); //systemcoreclock vraiment ?
	LL_USART_ConfigAsyncMode(USART2);
	
	//pas de HWD controle
	//transmit data(UART1, 'a');
	 rq: strcuture uart,
	initStructure UART
	config champs
	• 9600 (baudrate)
	• 8 bits
	• 1 bit stop
	• pas de parite
	• pas de HWD controle
	initUART
	uart enable
	
	//GPIO:
	//active l'horloge sur gpioA
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN ;
	//PA2 en alternate push pull
	LL_GPIO_SetPinMode(GPIOA,LL_GPIO_PIN_2,LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinPull(GPIOA,LL_GPIO_PIN_2,LL_GPIO_OUTPUT_PUSHPULL );

}
*/

//GPIO index
int main(void)
{
	
	// Configure the system clock to 72 MHz 
  SystemClock_Config();
	
	//initialisation pwm télécommande
	MyTimer_PWM_Init_Input();
	//initialisation adc accelero
	MyADC_init_accelero(ADC1);

	MyADC_init_battery(ADC2);
	Usart_Init(USART1);
	
  Pwm_Configure_Servoile();
	Pwm_Configure_CC();
	
	accelero_droit= MyAccelero_ADC();
  //config_gpio_girouette();
	/* Infinite loop */
  while (1)
  {
			servo_voile();
			CC(Loop_Pwm());
			if(Loop_MyBattery_Is_Low()){
				Usart_Transmit_Low_Battery(USART1);
			}
			else
				Usart_Transmit_High_Battery(USART1);
	}
}





/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 72000000
  *            HCLK(Hz)                       = 72000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            HSE Frequency(Hz)              = 8000000
  *            PLLMUL                         = 9
  *            Flash Latency(WS)              = 2
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  /* Set FLASH latency */
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);

  /* Enable HSE oscillator */
	// ********* Commenter la ligne ci-dessous pour MCBSTM32 *****************
	// ********* Conserver la ligne si Nucléo*********************************
  //LL_RCC_HSE_EnableBypass(); //à commenter si bateau
  LL_RCC_HSE_Enable();
  while(LL_RCC_HSE_IsReady() != 1)
  {
  };

  /* Main PLL configuration and activation */
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);

  LL_RCC_PLL_Enable();
  while(LL_RCC_PLL_IsReady() != 1)
  {
  };

  /* Sysclk activation on the main PLL */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  };

  /* Set APB1 & APB2 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

  /* Set systick to 1ms in using frequency set to 72MHz */
  LL_Init1msTick(72000000); // utile lorsqu'on utilise la fonction LL_mDelay

  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(72000000);
}



/* ==============   BOARD SPECIFIC CONFIGURATION CODE END      ============== */

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
