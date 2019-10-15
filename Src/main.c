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
#include "Chrono.h"
#include "stm32f1xx_ll_usart.h"



void  SystemClock_Config(void);

/* Private functions ---------------------------------------------------------*/

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
	/*
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
	LL_GPIO_SetPinMode(GPIOA,LL_GPIO_PIN_2,LL_GPIO_MODE_ALTERNATE|LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinMode(GPIOA,LL_GPIO_PIN_2,LL_GPIO_OUTPUT_PUSHPULL );
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
