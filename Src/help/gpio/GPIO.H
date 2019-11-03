#ifndef __GPIO_H
#define __GPIO_H
#include "stm32f10x.h"

// TODO : mettre des exemples.

/**
 * @brief Configure la broche du port donné en mode Output push-pull.
 */
char Port_IO_Init_Output( GPIO_TypeDef * Port, u8 Broche);
/**
 * @brief Configure la broche du port donné en mode Output push-pull (alternate function).
 */
char Port_IO_Init_Output_Alt( GPIO_TypeDef * Port, u8 Broche);
/**
 * @brief Configure la broche du port donné en mode floating input.
 */
char Port_IO_Init_Input( GPIO_TypeDef * Port, u8 Broche);

/**
 * @brief Met à 1 la valeur d'une proche préalablement configurée en mode output.
 */
void Port_IO_Set(GPIO_TypeDef * Port, u8 Broche);

/**
 * @brief Met à 0 la valeur d'une proche préalablement configurée en mode output.
 */
void Port_IO_Reset(GPIO_TypeDef * Port, u8 Broche);

/**
 * @brief Inverse la valeur d'une proche préalablement configurée en mode output.
 */
void Port_IO_Blink(GPIO_TypeDef * Port, u8 Broche);

/**
 * @brief Définit la valeur d'une proche préalablement configurée en mode output.
 */
void Port_IO_SetValue(GPIO_TypeDef * Port, u8 Broche, u8 Value);
/**
 * @brief Lis la valeur d'une broche préabalement configurée en mode input.
 */
unsigned int Port_IO_Read(GPIO_TypeDef * Port, u8 Broche);

#endif /* __GPIO_H */
