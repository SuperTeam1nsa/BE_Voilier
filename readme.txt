============= Génération de ce projet, T.Rocacher ===========================================
Le projet est basé sur CubeF1/STM32Cube_FW_F1_V1.8.0/Projects/STM32F103RB-Nucleo/Templates_LL
La seconde partie de ce readme est l'original qu'on peut trouver dans /Templates_LL

NB: curieusement ds le main, la fonction void SystemClock_Config(void) est différente des exemples LL.
Elle est donc modifiée pour coller à tout les exemples LL, en particulier, elle utilise la lib utils.


Cette partie du readme explique les modifications apportées à Templates_LL
=============================================================================================

*********************** LES MODIFICATIONS    ************************************************

#############  Structuration du projet, du répertoire (modif par rapport au template ########

La philosophie générale est de partir du template et de sortir du projet tout ce qui n'est pas utile
en terme de lib LL. On a tout de même besoin de RCC et de utils (les deux pour la clock conf)
Chaque projet est donc "self contained" et contient les lib LL dont il a besoin.


Structure physique des répertoire
/Inc (h "systeme")
/src (main et system_stm32f1xx.c = systeminit au boot startup)
/LLDrivers : src et inc des drivers LL utiles, ici rcc et utils
/MDK-ARM : le projet KEIL
/Services: vide pour l'instant...

Structure de groupes de KEIL en deux grandes parties :
Partie sources user
/User Applications
/User Services
/MyDriver    les drivers à faire à la main ...
/Drivers/STM32f1xx_LL_Driver
/Doc

Partie système
/Drivers/CMSIS (l'init system depuis boot)
/Example/MDK-ARM contient le startup


###### La récupération des IT     ##############
La philosophie LL est de récupérer les IT dans un fichier spécifique stm32f1xx_it.c.
Il est supprimé de manière à ce que les handlers soient placés dans les divers modules que les
étudiants construiront.


###### Divers main    ##############
main.h enlevé
Remarque ligne 72   LL_RCC_HSE_EnableBypass();, si on devait travailler sur MCBSTM32, il faudrait commenter la ligne
Ligne 108 commentée, il s'agit de l'activation systick inutile mais qui active les IT non récupérée ... donc otée





/**
  @page Templates_LL  Description of the Templates_LL example
  
  @verbatim
  ******************** (C) COPYRIGHT 2017 STMicroelectronics *******************
  * @file    Templates_LL/readme.txt 
  * @author  MCD Application Team
  * @brief   Description of the Templates_LL example.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  @endverbatim

@par Example Description

This projects provides a reference template through the LL API that can be used to build any firmware application.

This project LL template provides:
 - Inclusion of all LL drivers (include files in "main.h" and LL sources files in IDE environment, with option "USE_FULL_LL_DRIVER" in IDE environment)
   Note: If optimization is needed afterwards, user can perform a cleanup by removing unused drivers.
 - Definition of LEDs and user button (file: main.h)
   Note: User button name printed on board may differ from naming "user button" in code: "key button", ...
 - Clock configuration (file: main.c)

This project LL template does not provide:
 - Functions to initialize and control LED and user button
 - Functions to manage IRQ handler of user button

To port a LL example to the targeted board:
1. Select the LL example to port.
   To find the board on which LL examples are deployed, refer to LL examples list in "STM32CubeProjectsList.html", table section "Examples_LL"
   or AN4724: STM32Cube firmware examples for STM32F1 Series

2. Replace source files of the LL template by the ones of the LL example, except code specific to board.
   Note: Code specific to board is specified between tags:
         /* ==============   BOARD SPECIFIC CONFIGURATION CODE BEGIN    ============== */
         /* ==============   BOARD SPECIFIC CONFIGURATION CODE END      ============== */
         
   - Replace file main.h, with updates:
     - Keep LED and user button definition of the LL template under tags
    
   - Replace file main.c, with updates:
     - Keep clock configuration of the LL template: function "SystemClock_Config()"
     - Depending of LED availability, replace LEDx_PIN by another LEDx (number) available in file main.h
     
   - Replace file stm32f1xx_it.h
   - Replace file stm32f1xx_it.c

@par Keywords

Reference, Templates_LL

@par Directory contents 

  - Templates_LL/Inc/stm32f1xx_it.h          Interrupt handlers header file
  - Templates_LL/Inc/main.h                  Header for main.c module
  - Templates_LL/Inc/stm32_assert.h          Template file to include assert_failed function
  - Templates_LL/Src/stm32f1xx_it.c          Interrupt handlers
  - Templates_LL/Src/main.c                  Main program
  - Templates_LL/Src/system_stm32f1xx.c      STM32f1x system source file


@par Hardware and Software environment

  - This template runs on STM32F103xB devices.
    
  - This template has been tested with STM32F103RB-Nucleo RevC board and can be
    easily tailored to any other supported device and development board.


@par How to use it ? 

In order to make the program work, you must do the following :
 - Open your preferred toolchain
 - Rebuild all files and load your image into target memory
 - Run the example

 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
