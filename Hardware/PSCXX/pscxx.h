#ifndef __PSCXX_H
#define __PSCXX_H	 
#include "sys.h"



//==========���߼�ͣ�������ذ� �˿ڶ���=====================




//================================

extern u8 	B_CXX_STA; //B_CXX_STA=0;��ʼ��״̬

extern u8 	B_SB_ID ;	//������豸IDʶ���
extern u8 	B_SB_NUM ;	//������豸��ַ
extern u8 	B_SB_DATA ;  //��ѯ���豸���� BIT4-��ͣ���� ,

extern u8 	cxx_fx1  ;
extern u8 	cxx_fx2  ;
extern u8	RX1_STS_IN;
extern u8	RX2_STS_IN;

extern u8 can_led_dly;
extern u8 cxx_led_dly;
extern u8 Task_ok;
extern u8 B_BX_ID;	
extern u8 B_MC_ID;   
extern u16 W_BX_NUM ;	

extern u8 Data_update_always_flag;
extern u8 Data_update_temp_flag;				//�������ݸ��±�־��1 �������У�0����������
extern u8 Data_update_run_falg;				//���ݸ����Ƿ�ִ�У�1 �Ѿ����У�0 δ����
extern u8 Output_pulse_h_flag; 				//������������Ϊ1����Ϊ0
//===========================



//=======��������=========

void Pscxx_Main(void);   // Ƥ������ϵͳ ��ѯ�ߴ���	������
void Cxx_IO_Init(void);   //IO��ʼ��




		 				    
#endif
