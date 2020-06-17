#ifndef __IWDG_H
#define __IWDG_H	 
#include "sys.h"

//���Ź���ʱʱ����㹫ʽ:Tout=(Ԥ��Ƶֵ*��װ��ֵ)/40 (��λ��ms)

//��ǰpreΪ64��rlrΪ625������õ�Toutʱ��Ϊ1�루���ֵ����

//72M   3S
//#define pre		IWDG_Prescaler_64 //��Ƶֵ��Χ��4,8,16,32,64,128,256
//#define rlr		1875 //��װ��ֵ��Χ��0��0xFFF��4095��	 3S

//24M   4S
//#define pre		IWDG_Prescaler_128 //��Ƶֵ��Χ��4,8,16,32,64,128,256
//#define rlr		3750 //��װ��ֵ��Χ��0��0xFFF��4095��	 

//16M   2S
#define pre		IWDG_Prescaler_32 //��Ƶֵ��Χ��4,8,16,32,64,128,256
#define rlr		625 //��װ��ֵ��Χ��0��0xFFF��4095��

void IWDG_Init(void);
void IWDG_Feed(void);
		 				    
#endif
