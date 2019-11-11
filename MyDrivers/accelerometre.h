  
#ifndef ACCELERO_H
#define ACCELERO_H

#include "stm32f10x.h"

/**
* Permet d'initialiser l'acceleromètre et de lancer l'aquisition de valeurs.
* @return La valeur 0 de l'acceleromètre
**/
int init_accelero();

/**
* Permet de déterminer si l'inclinaison du bateau est critique ou non.
* @param zero : La valeur 0 de l'acceleromètre renvoyé par l'intialisation.
* @return 1 si l'inclinaison est critique, 0 sinon
**/
int inclinaison_critique(int zero);

#endif