/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LIGHTCONTROLL_H
#define __LIGHTCONTROLL_H


#include "stm32f7xx.h"                  // Device header
#include "stm32746g_discovery.h"        // Keil.STM32F746G-Discovery::Board Support:Drivers:Basic I/O

void send_Message_Bin(short add[8], short data[8]);
void main_light(void);

#endif /* __LIGHTCONTROLL_H */


