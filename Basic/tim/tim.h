#ifndef  __PWM_H
#define  __PWM_H
#include "sys.h"

extern vu32 Millis;//ϵͳ�����ʱ��  T3�ж��ڲ�����

extern u8  T3_CXX_IDLE;
extern u8  T3_CXX_RXOK;

void TIM3_Init(u16 arr,u16 psc);
void TIM3_NVIC_Init (void);
void TIM3_Set_Ti(u16 ti_ms);  //  ����������ʱ

void CXX_RCV_Ti(u16 ti_ms);		// cxx���� ����ж���ʱ
void CXX_IDLE_Ti(u16 ti_ms);	// cxx���� ����ж���ʱ



#endif
