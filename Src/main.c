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
#include "stm32f1xx_ll_usart.h"
#include "stm32f1xx_ll_tim.h" 
#include "stm32f1xx_ll_bus.h" // Pour l'activation des horloges

int angle=0;
void  SystemClock_Config(void);
void update_position_girouette(void)
{
	// rabaisser le flag d'IT
	LL_TIM_ClearFlag_UPDATE(TIM3);
	//lecture: TIM3-CNT
	//TIM3->CNT;
	//(*Ptr_ItFct_TIM3)(); //update position
}	

/* Private functions ---------------------------------------------------------*/
void config_gpio_girouette(void){ // on pense que ça marche mais c'est à tester car keil 5 ne permet plus de faire marcher le compteur en titillant les channels 1 et 2 de la simulation.
	//active l'horloge gpioA
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN ;
	

	//Depending on the sequence the counter counts up or down, the DIR bit in the TIMx_CR1 register is modified by hardware accordingly
	//valeur de l'angle direcctement dasn TIM3->CNT cf :
	//This means that the counter just counts continuously between 0 and the auto-reload value in the TIMx_ARR register (0 to ARR or ARR down to 0 depending on the direction)
	MyTimer_Conf(TIM3,0xB4,0x1);//180
	MyTimer_IT_Conf(TIM3, update_position_girouette,8);
	MyTimer_IT_Enable(TIM3);
		
	//on compte que sur 1 seul edge p328
	TIM3->SMCR=0x001;
	//PA6 et pA7 => CH1 et CH2 TIM3 en input 
	TIM3->CCMR1=0x0101;
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

void Pwm_Configure( float rate)
{
	//rq: TIM4 CH3 => arr, psc et cnt commun avec l'input (tim4 ch1) 
	//rq: cf photo pour LL
	MyTimer_Conf(TIM1,999,71);//180
	//MyTimer_IT_Conf(TIM1, update_position_girouette,8);
	//MyTimer_IT_Enable(TIM3);
	
	// Configuration des GPÏO en alternate function.
	//PB8 en alternate push pull
	LL_GPIO_SetPinMode(GPIOA,LL_GPIO_PIN_8,LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinOutputType(GPIOA,LL_GPIO_PIN_8,LL_GPIO_OUTPUT_PUSHPULL );
	
	int ARR=999; //conçu pour avoir un kilo check doc pour freq servo voile
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
	LL_TIM_OC_SetCompareCH1(TIM1,ARR/4);//c'est le ratio
	LL_TIM_EnableAllOutputs(TIM1);
	LL_TIM_EnableCounter(TIM1);
	
	
	
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
	
	MyTimer_Start(TIM1);
}
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
	
	//rq:add configUsart dans chrono.c et lève flag RXNE dans task10ms
	//configUSART();

  // Configure the system clock to 72 MHz 
  SystemClock_Config();

  // Add your application code here 

  config_gpio_girouette();
  Pwm_Configure(0.75);
	
  /* Infinite loop */
  while (1)
  {
			
		/*USART
		if(LL_USART_IsActiveFlag_RXNE(USART2)){ //flag : on recoit l'ordre d'ecrire #custom 
			  Time *ct=Chrono_Read();
				send(ct->Min);
				send(':');
				send(ct->Sec);
				send(':');
				send(ct->Hund);
				send(':');
				send(0x0D);//0x0D =retour chariot
				LL_USART_ClearFlag_RXNE(USART2);
				}
		Chrono_Background();
  }*/
		

	
	};

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
  LL_RCC_HSE_EnableBypass();
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
