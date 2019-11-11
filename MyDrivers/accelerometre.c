#include "accelerometre.h"
#include <math.h>

/*A vérifier. HSN*/

int init_accelero() {
	// Enable clock sur GPIOC
	(RCC->APB2ENR)|= RCC_APB2ENR_IOPCEN;
	// Analog input pour PC0 et PC1
	GPIOC->CRL &= ~(0xF);
	GPIOC->CRL &= ~(0xF<<4);
	//power on ADC
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	ADC1->CR2 |= ADC_CR2_ADON;
	// L = 1
    ADC1->SQR3 |= 0xa;  // qs1 = ch10
    
    // Calibration
    ADC1->CR2 |= ADC_CR2_ADON; // lancement conversion adc
	while((ADC1->SR & ADC_SR_EOC) == 0) {
		// bloquant
	}
	// baisse le flag
	ADC1->SR &= ~ADC_SR_EOC;
	return ADC1->DR & ADC_DR_DATA;
}

int inclinaison_critique(int zero) {
	ADC1->CR2 |= ADC_CR2_ADON; // lancement conversion adc
	while((ADC1->SR & ADC_SR_EOC) == 0) {
		// bloquant
	}
	// baisse le flag
	ADC1->SR &= ~ADC_SR_EOC;
	float x = ADC1->DR & ADC_DR_DATA;
	
    // 0.48V = 595
	return (x < (zero - 297) || x > (zero + 297));
}
