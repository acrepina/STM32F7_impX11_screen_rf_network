
#include "stm32f7xx.h"                  // Device header
#include "stm32746g_discovery.h"        // Keil.STM32F746G-Discovery::Board Support:Drivers:Basic I/O
#include "stm32f7xx_hal.h"
#include "lightControll.h"

static void initMessage(void);
static void sendZero(void);
static void sendOne(void);



void main_light(void){
	while(1){
	}
}

 void sendZero(void)
 {
	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
	 HAL_Delay(22); //550us
	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
	 HAL_Delay(23);//575us
 }
 
 void sendOne(void)
 {
	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
	 HAL_Delay(45); //9 ms
	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
	 HAL_Delay(45);//4.5ms zob
 }

	void initMessage(void)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
		HAL_Delay(360); //9 ms
		
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
		
		HAL_Delay(180);//4.5ms zob
	}	


void send_Message_Bin(short add[8], short data[8]) {
	short i;
	short j;
	
	initMessage(); 
	for(j=0;j<6;j++)
	{
		for(i=0; i<8; i++){
			if(add[i]==1){
				sendOne();
			}else{
				sendZero();
			}
		}
			
		for(i=0; i<8; i++){
			if(add[i]==1){
				sendZero();
			}else{
				sendOne();
			}	
		}
		
		for(i=0; i<8; i++){
			if(data[i]==1){
				sendOne();
			}else{
				sendZero();
			}
		}
		
			for(i=0; i<8; i++){
			if(data[i]==1){
				sendZero();
			}else{
				sendOne();
			}	
		}
			sendOne();
			HAL_Delay(220);
	}
	
	
}

