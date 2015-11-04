/**
  ******************************************************************************
  * @file    Templates/Src/main.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    25-June-2015
  * @brief   STM32F7xx HAL API Template project (with modifications from ARM)
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/*
 * This file contains modifications by ARM to provide it as User Code Template
 * within the STM32 Device Family Pack.
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f7xx.h"                  // Device header
#include "stm32746g_discovery.h"        // Keil.STM32F746G-Discovery::Board Support:Drivers:Basic I/O
#include "Board_Buttons.h"              // ::Board Support:Buttons
#include "stm32f7xx_hal.h"
#include "stm32746g_discovery_lcd.h"
#include "stm32746g_discovery_ts.h"		//Biblio touchscreen


/* Private macro */
#define MESSAGE1   "              |               "
#define MESSAGE2   "              |               "
#define MESSAGE3   "              |               "
#define MESSAGE4   "     ON       |      OFF      "
#define MESSAGE5   "              |               "
#define MESSAGE6   "              |               "
#define MESSAGE7   "         LEY CREPIN           "


#ifdef _RTE_
#include "RTE_Components.h"             // Component selection
#endif
#ifdef RTE_CMSIS_RTOS                   // when RTE component CMSIS RTOS is used
#include "cmsis_os.h"                   // CMSIS RTOS header file
#endif

#ifdef RTE_CMSIS_RTOS_RTX
extern uint32_t os_time;

uint32_t HAL_GetTick(void) { 
  return os_time; 
}
#endif

/** @addtogroup STM32F7xx_HAL_Examples
  * @{
  */

/** @addtogroup Templates
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);
static void MPU_Config(void);
static void CPU_CACHE_Enable(void);
static void initMessage(void);
static void sendZero(void);
static void sendOne(void);
static void send_Message_Bin(short add[8], short data[8]);
int LCDinit(void);
void TouchScreenInit(void);



int m_nCurrentLine = 0;

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  mPort_Initialize program
  * @param  None
  * @retval None
  */
	
	void TouchScreenInit(void)
	{
		BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize());
		//BSP_TS_ITConfig();
		
	}
	
	void TS_IRQHandler(void)
	{
		TS_StateTypeDef * state;
		
		if(BSP_TS_ITGetStatus() != RESET)
		{
			BSP_TS_GetState(state);
			BSP_TS_ITClear();
		}
	}
	
	
	void initMessage(void)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
		HAL_Delay(360); //9 ms
		
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
		
		HAL_Delay(180);//4.5ms zob
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

int LCDinit(void) {
	/* Hardware initialization */
	BSP_LCD_Init();

	/* LCD Initialization */
	BSP_LCD_LayerDefaultInit(0, LCD_FB_START_ADDRESS);
	BSP_LCD_LayerDefaultInit(1, LCD_FB_START_ADDRESS+(BSP_LCD_GetXSize()*BSP_LCD_GetYSize()*4));
	BSP_LCD_SetLayerVisible(1, DISABLE);



	/* Select the LCD Background Layer  */
	BSP_LCD_SelectLayer(0);
	

	/* Clear the Background Layer */
	BSP_LCD_Clear(LCD_COLOR_ORANGE);
	BSP_LCD_SetTransparency(0, 0);
	//BSP_LCD_SetBackColor(LCD_COLOR_ORANGE);
	BSP_LCD_SetLayerVisible(0, ENABLE);
	

	/* Select the LCD Foreground Layer  */
	BSP_LCD_SelectLayer(1);

	/* Clear the Foreground Layer */
	BSP_LCD_Clear(LCD_COLOR_ORANGE);

	/* Configure the transparency for foreground and background : Increase the transparency */
	
	BSP_LCD_SetTransparency(1, 100);

	// Display a startup message
		BSP_LCD_SetBackColor(LCD_COLOR_LIGHTGREEN);
    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);

    BSP_LCD_DisplayStringAtLine(1, (uint8_t*)MESSAGE1);
    BSP_LCD_DisplayStringAtLine(2, (uint8_t*)MESSAGE2);
    BSP_LCD_DisplayStringAtLine(3, (uint8_t*)MESSAGE3);
    BSP_LCD_DisplayStringAtLine(4, (uint8_t*)MESSAGE4);
    BSP_LCD_DisplayStringAtLine(5, (uint8_t*)MESSAGE5);
    BSP_LCD_DisplayStringAtLine(6, (uint8_t*)MESSAGE6);
		BSP_LCD_DisplayStringAtLine(7, (uint8_t*)MESSAGE7);
		BSP_LCD_SetLayerVisible(1, ENABLE);

	m_nCurrentLine = 0;
	
		/* Enable the LCD */
	BSP_LCD_DisplayOn();

	return 1;
}

void	mPort_Initialize(void){
  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Configure GPIO pin: PI1 (LD1) */
  GPIO_InitStruct.Pin   = GPIO_PIN_9;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
}
//int8_t trad_x10ToBin(char* mess){//char[12] mess
////	|    Byte 1     |    Byte 2     |
////|X|X|X|X|0|X|0|0|X|X|X|X|X|0|0|0|
////|1 2 3 4 5 6 7 8|1 2 3 4 5 6 7 8|

////Byte 1
////1: One for house codes E-L
////2: One for house codes A-D & I-L
////3: One for house codes A,B,G,H,I,J,O,P
////4: One for house codes B,D,F,H,J,L,N,P
////5: Always zero
////6: One for units 9-16
////7: Always zero
////8: Always zero

////Byte 2
////1: One for BRIGHT or DIM
////2: One for unit codes 5-8 & 13-16
////3: One for OFF (Zero for ON or BRIGHT/DIM)
////4: One for unit codes 2,4,6,8,10,12,14,16 & DIM command
////5: One for unit codes 3,4,7,8,11,12,15,16 & BRIGHT & DIM commands
////6: Always zero
////7: Always zero
////8: Always zero


//	int8_t bintab=0;
//	if{
//			
//	}
//	
//	return bintab;
//}


/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
	
	short ADD[8] = { 0,1,1,0,0,1,0,0};
  short DATA_allume[8] = { 0,0,0,0,1,0,0,0 };
	short DATA_eteindre[8] = { 0,0,1,0,1,0,0,0 };
	TS_StateTypeDef * state;

  /* This project template calls firstly two functions in order to configure MPU feature 
     and to enable the CPU Cache, respectively MPU_Config() and CPU_CACHE_Enable().
     These functions are provided as template implementation that User may integrate 
     in his application, to enhance the performance in case of use of AXI interface 
     with several masters. */ 
  
  /* Configure the MPU attributes as Write Through */
  MPU_Config();

  /* Enable the CPU Cache */
  CPU_CACHE_Enable();

#ifdef RTE_CMSIS_RTOS                   // when using CMSIS RTOS
  osKernelInitialize();                 // initialize CMSIS-RTOS
#endif

  /* STM32F7xx HAL library initialization:
       - Configure the Flash ART accelerator on ITCM interface
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
  HAL_Init();

  /* Configure the System clock to have a frequency of 216 MHz */
  SystemClock_Config();
	mPort_Initialize();
	Buttons_Initialize ();
	 /* Configure LCD : Only one layer is used */
 LCDinit();
 TouchScreenInit();

  /* Add your application code here
     */

#ifdef RTE_CMSIS_RTOS                   // when using CMSIS RTOS
  // create 'thread' functions that start executing,
  // example: tid_name = osThreadCreate (osThread(name), NULL);

  osKernelStart();                      // start thread execution 
#endif


//HAL_GPIO_WritePin(GPIOI, GPIO_PIN_1, GPIO_PIN_SET);

	
		/* Infinite loop */
 HAL_Delay(220); //550us
 
 while (1)
 {
	BSP_TS_GetState(state);
	 
		if (state->touchDetected !=0)
		{
			if(state->touchX[0] < ((uint16_t)0xf0))
			{
				BSP_TS_ResetTouchData(state);
				send_Message_Bin(ADD,DATA_allume);
			}
			else
			{
				BSP_TS_ResetTouchData(state);
				send_Message_Bin(ADD,DATA_eteindre);
			}
		}
 }
	
	 
//  while (1)
//  {
//		do
//	 {
//	 }while(Buttons_GetState() !=1);
//	
//	
//		send_Message_Bin(ADD,DATA_allume);
//	 
//	 	do
//	 {
//	 }while(Buttons_GetState() !=1);
//	 
//	
//		send_Message_Bin(ADD,DATA_eteindre);
//	}
  
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 216000000
  *            HCLK(Hz)                       = 216000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 25000000
  *            PLL_M                          = 25
  *            PLL_N                          = 432
  *            PLL_P                          = 2
  *            PLL_Q                          = 9
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 7
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;  
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* activate the OverDrive to reach the 216 Mhz Frequency */
  if(HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* User may add here some code to deal with this error */
  while(1)
  {
  }
}

/**
  * @brief  Configure the MPU attributes as Write Through for SRAM1/2.
  * @note   The Base Address is 0x20010000 since this memory interface is the AXI.
  *         The Region Size is 256KB, it is related to SRAM1 and SRAM2  memory size.
  * @param  None
  * @retval None
  */
static void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct;
  
  /* Disable the MPU */
  HAL_MPU_Disable();

  /* Configure the MPU attributes as WT for SRAM */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = 0x20010000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_256KB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Enable the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**
  * @brief  CPU L1-Cache enable.
  * @param  None
  * @retval None
  */
static void CPU_CACHE_Enable(void)
{
  /* Enable I-Cache */
  SCB_EnableICache();

  /* Enable D-Cache */
  SCB_EnableDCache();
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

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
