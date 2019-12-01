#include "accelerometre.h"
float MyAccelero_ADC(void) 
{
		float y= ADC1->DR << ADC_DR_DATA_Pos ; 
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

	LL_RCC_SetAPB2Prescaler(4);
	
	ADC->SQR3 = 11;
	ADC-> CR2 |= ADC_CR2_EXTTRIG ; 
	
}
