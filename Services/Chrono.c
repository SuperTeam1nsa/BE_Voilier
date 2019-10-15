// A COMPLETER

/*
Service permettant de chornom�trer jusqu'� 59mn 59s 99 1/100
Utilise un timer au choix (TIMER1 � TIMER4).
Utilise la lib MyTimers.h /.c
*/



#include "Chrono.h"
#include "MyTimer.h"
#include "stm32f1xx_ll_gpio.h" 
#include "stm32f1xx_ll_usart.h"
static char ledOn=0;

// variable priv�e de type Time qui m�morise la dur�e mesur�e
static Time Chrono_Time; // rem : static rend la visibilit� de la variable Chrono_Time limit�e � ce fichier 

// variable priv�e qui m�morise pour le module le timer utilis� par le module
static TIM_TypeDef * Chrono_Timer=TIM1; // init par d�faut au cas o� l'utilisateur ne lance pas Chrono_Conf avant toute autre fct.

// d�claration callback appel� toute les 10ms
void Chrono_Task_10ms(void);
void Chrono_Conf_IO(void);
/**
	* @brief  Configure le chronom�tre. 
  * @note   A lancer avant toute autre fonction.
	* @param  Timer : indique le timer � utiliser par le chronom�tre, TIM1, TIM2, TIM3 ou TIM4
  * @retval None
  */
void Chrono_Conf(TIM_TypeDef * Timer)
{
	// Reset Time
	Chrono_Time.Hund=0;
	Chrono_Time.Sec=0;
	Chrono_Time.Min=0;
	
	// Fixation du Timer
	Chrono_Timer=Timer;

	// R�glage Timer pour un d�bordement � 10ms
	MyTimer_Conf(Chrono_Timer,999, 719);
	
	// R�glage interruption du Timer avec callback : Chrono_Task_10ms()
	MyTimer_IT_Conf(Chrono_Timer, Chrono_Task_10ms,3);
	
	// Validation IT
	MyTimer_IT_Enable(Chrono_Timer);
	
	Chrono_Conf_IO();
	
}
void Chrono_Conf_IO(void){
		//active l'horloge
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN ;
	
 /* LL_GPIO_InitTypeDef GPIO_InitStruct;
	LL_GPIO_StructInit(&GPIO_InitStruct);
	//on remplit initStruct puis on l'applique � GPIOC
	GPIO_InitStruct.Mode=LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull=LL_GPIO_PULL_DOWN;
	GPIO_InitStruct.Pin=LL_GPIO_PIN_13;
	LL_GPIO_Init(GPIOC,&GPIO_InitStruct);
	*/
  //B1 user (reset) en pulldown cable sur PC13 sur la nucleo
	LL_GPIO_SetPinMode(GPIOC,LL_GPIO_PIN_13,LL_GPIO_MODE_INPUT);
	LL_GPIO_SetPinPull(GPIOC,LL_GPIO_PIN_13,LL_GPIO_PULL_DOWN);
	
	//PC10 en pushpull
	LL_GPIO_SetPinMode(GPIOC,LL_GPIO_PIN_10,LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinOutputType(GPIOC,LL_GPIO_PIN_10,LL_GPIO_OUTPUT_PUSHPULL);
	//PC8 en floating input
	LL_GPIO_SetPinMode(GPIOC,LL_GPIO_PIN_8,LL_GPIO_MODE_FLOATING);
		//stop en pulldown PC6
	LL_GPIO_SetPinMode(GPIOC,LL_GPIO_PIN_6,LL_GPIO_MODE_INPUT);
	LL_GPIO_SetPinPull(GPIOC,LL_GPIO_PIN_6,LL_GPIO_PULL_DOWN);
}

/**
	* @brief  D�marre le chronom�tre. 
  * @note   si la dur�e d�passe 59mn 59sec 99 Hund, elle est remise � z�ro et repart
	* @param  Aucun
  * @retval Aucun
  */
void Chrono_Start(void)
{
	MyTimer_Start(Chrono_Timer);
}


/**
	* @brief  Arr�te le chronom�tre. 
  * @note   
	* @param  Aucun
  * @retval Aucun
  */
void Chrono_Stop(void)
{
	MyTimer_Stop(Chrono_Timer);
}


/**
	* @brief  Remet le chronom�tre � 0 
  * @note   
	* @param  Aucun
  * @retval Aucun
  */
void Chrono_Reset(void)
{
  // Arr�t Chrono
	MyTimer_Stop(Chrono_Timer);

	// Reset Time
	Chrono_Time.Hund=0;
	Chrono_Time.Sec=0;
	Chrono_Time.Min=0;
	LL_GPIO_ResetOutputPin(GPIOC,LL_GPIO_PIN_10);
		ledOn=0;
}


/**
	* @brief  Renvoie l'adresse de la variable Time priv�e g�r�e dans le module Chrono.c
  * @note   
	* @param  Aucun
  * @retval adresse de la variable Time
  */
Time * Chrono_Read(void)
{
	return &Chrono_Time;
}



/**
	* @brief  incr�mente la variable priv�e Chron_Time modulo 60mn 
  * @note   
	* @param  Aucun
  * @retval Aucun
  */
void Chrono_Task_10ms(void)
{ 
	//mise � 1 du flag RXNE
	USART2->SR |= LL_USART_SR_RXNE;
	if(!ledOn){
	LL_GPIO_SetOutputPin(GPIOC,LL_GPIO_PIN_10);
		ledOn=1;
	}else{
		LL_GPIO_ResetOutputPin(GPIOC,LL_GPIO_PIN_10);
		ledOn=0;
	}
	Chrono_Time.Hund++;
	if (Chrono_Time.Hund==100)
	{
		Chrono_Time.Sec++;
		Chrono_Time.Hund=0;
	}
	if (Chrono_Time.Sec==60)
	{
		Chrono_Time.Min++;
		Chrono_Time.Sec=0;
	}
	if (Chrono_Time.Min==60)
	{
		Chrono_Time.Hund=0;
	}
	
}


