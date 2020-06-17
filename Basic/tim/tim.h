#ifndef  __PWM_H
#define  __PWM_H
#include "sys.h"

extern vu32 Millis;//系统毫秒计时器  T3中断内部计数

extern u8  T3_CXX_IDLE;
extern u8  T3_CXX_RXOK;

void TIM3_Init(u16 arr,u16 psc);
void TIM3_NVIC_Init (void);
void TIM3_Set_Ti(u16 ti_ms);  //  重新设置延时

void CXX_RCV_Ti(u16 ti_ms);		// cxx接受 完成判断延时
void CXX_IDLE_Ti(u16 ti_ms);	// cxx接受 完成判断延时



#endif
