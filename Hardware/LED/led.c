
#include "led.h"

void LED_Init(void){ //LED�ƵĽӿڳ�ʼ��
	GPIO_InitTypeDef  GPIO_InitStructure; 	
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB,ENABLE);
	
	//  ��ѯ��LED��
    GPIO_InitStructure.GPIO_Pin =  CXX_LED;                        
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //ѡ��IO�ӿڹ�����ʽ   �������       
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //����IO�ӿ��ٶȣ�2/10/50MHz�� 
	GPIO_Init(CXX_LED_P, &GPIO_InitStructure);


	//  ��ͣ������ʾ LED��
    GPIO_InitStructure.GPIO_Pin = LK_LEDCTL; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		//ѡ��IO�ӿڹ�����ʽ   ������� 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 		//����IO�ӿ��ٶȣ�2/10/50MHz�� 	                   
	GPIO_Init(LK_LEDCTL_P, &GPIO_InitStructure);

	CXX_LED_OFF;	
	LK_LEDCTL_OFF;
			
}
 

/*
ѡ��IO�ӿڹ�����ʽ��
GPIO_Mode_AIN ģ������
GPIO_Mode_IN_FLOATING ��������
GPIO_Mode_IPD ��������
GPIO_Mode_IPU ��������
GPIO_Mode_Out_PP �������
GPIO_Mode_Out_OD ��©���
GPIO_Mode_AF_PP �����������
GPIO_Mode_AF_OD ���ÿ�©���
*/
