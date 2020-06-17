#ifndef __LED_H
#define __LED_H	 
#include "sys.h"

//....................................
#define 	CXX_LED_P		GPIOB
#define 	CXX_LED			GPIO_Pin_0

#define		CXX_LED_ON	  GPIO_SetBits(CXX_LED_P,CXX_LED)	  
#define		CXX_LED_OFF		GPIO_ResetBits(CXX_LED_P,CXX_LED)	 //讯灯 LED控制

//....................................
#define		LK_LEDCTL_P		GPIOB
#define		LK_LEDCTL		GPIO_Pin_1	

#define 	LK_LEDCTL_ON	GPIO_SetBits(LK_LEDCTL_P,LK_LEDCTL)	//指示灯 LED亮
#define 	LK_LEDCTL_OFF	GPIO_ResetBits(LK_LEDCTL_P,LK_LEDCTL)

//....................................



void LED_Init(void);//初始化

		 				    
#endif
