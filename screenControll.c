
//#include "screenControll.h"
//#include "stm32746g_discovery_lcd.h"



/////* Private macro */
//#define MESSAGE1   "              |               "
//#define MESSAGE2   "              |               "
//#define MESSAGE3   "              |               "
//#define MESSAGE4   "     ON       |      OFF      "
//#define MESSAGE5   "              |               "
//#define MESSAGE6   "              |               "
//#define MESSAGE7   "         LEY CREPIN           "




//void TouchScreenInit(void)
//	{
//		BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize());
//		//BSP_TS_ITConfig();
//		
//	}
//	

//	
//	
//	
//	
//	





//int LCDinit(void) {
//	
//	/* Hardware initialization */
//	BSP_LCD_Init();

//	/* LCD Initialization */
//	BSP_LCD_LayerDefaultInit(0, LCD_FB_START_ADDRESS);
//	BSP_LCD_LayerDefaultInit(1, LCD_FB_START_ADDRESS+(BSP_LCD_GetXSize()*BSP_LCD_GetYSize()*4));
//	BSP_LCD_SetLayerVisible(1, DISABLE);



//	/* Select the LCD Background Layer  */
//	BSP_LCD_SelectLayer(0);
//	

//	/* Clear the Background Layer */
//	BSP_LCD_Clear(LCD_COLOR_ORANGE);
//	BSP_LCD_SetTransparency(0, 0);
//	//BSP_LCD_SetBackColor(LCD_COLOR_ORANGE);
//	BSP_LCD_SetLayerVisible(0, ENABLE);
//	

//	/* Select the LCD Foreground Layer  */
//	BSP_LCD_SelectLayer(1);

//	/* Clear the Foreground Layer */
//	BSP_LCD_Clear(LCD_COLOR_ORANGE);

//	/* Configure the transparency for foreground and background : Increase the transparency */
//	
//	BSP_LCD_SetTransparency(1, 100);

//	// Display a startup message
//		BSP_LCD_SetBackColor(LCD_COLOR_LIGHTGREEN);
//    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);

//    BSP_LCD_DisplayStringAtLine(1, (uint8_t*)MESSAGE1);
//    BSP_LCD_DisplayStringAtLine(2, (uint8_t*)MESSAGE2);
//    BSP_LCD_DisplayStringAtLine(3, (uint8_t*)MESSAGE3);
//    BSP_LCD_DisplayStringAtLine(4, (uint8_t*)MESSAGE4);
//    BSP_LCD_DisplayStringAtLine(5, (uint8_t*)MESSAGE5);
//    BSP_LCD_DisplayStringAtLine(6, (uint8_t*)MESSAGE6);
//		BSP_LCD_DisplayStringAtLine(7, (uint8_t*)MESSAGE7);
//		BSP_LCD_SetLayerVisible(1, ENABLE);


//	
//		/* Enable the LCD */
//	BSP_LCD_DisplayOn();

//	return 1;
//}



