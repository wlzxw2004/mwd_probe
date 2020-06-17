

#include "tim.h"
#include "led.h"
#include "pscxx.h"
vu32 Millis=0;//ϵͳ�����ʱ��  T3�ж��ڲ�����

u8 can_tx_dly;

extern vu16 ADC_DMA_IN[2]; //�����ⲿ����
extern vu32 adc_sum[2];   //ADת�����ۼ�
extern vu16 adc_cnt;	  //AD�ۼӵĴ���
extern u32  P_delay_time;
//extern vu16 CXX_RXOVER_TS;
//extern vu16 USART_CXX_RX_TS;

u8 T3_set;
u8 T3_CXX_IDLE=1;

u8  T3_CXX_RXOK=0;

u16 CXX_TX_DLY=200;

//��ʱ��ʱ����㹫ʽTout = ((��װ��ֵ+1)*(Ԥ��Ƶϵ��+1))/ʱ��Ƶ��;
//���磺1���붨ʱ����װ��ֵ=99��Ԥ��Ƶϵ��=719


//============================================================
void TIM3_NVIC_Init (void){ //����TIM3�ж�����
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x2;	//������ռ�������ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
//============================================================
void TIM3_Init(u16 arr,u16 psc){  //TIM3 ��ʼ�� arr��װ��ֵ pscԤ��Ƶϵ��
    TIM_TimeBaseInitTypeDef     TIM_TimeBaseInitStrue;
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);//ʹ��TIM3
    TIM3_NVIC_Init (); //����TIM3�ж�����
	      
    TIM_TimeBaseInitStrue.TIM_Period=arr; //�����Զ���װ��ֵ
    TIM_TimeBaseInitStrue.TIM_Prescaler=psc; //Ԥ��Ƶϵ��

    TIM_TimeBaseInitStrue.TIM_CounterMode=	TIM_CounterMode_Up; //�������������

    TIM_TimeBaseInitStrue.TIM_ClockDivision=TIM_CKD_DIV1; //ʱ�ӵķ�Ƶ���ӣ�����һ������ʱ���ã�һ����ΪTIM_CKD_DIV1
    TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStrue); //TIM3��ʼ������
 
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);//ʹ��TIM3�ж�    
    TIM_Cmd(TIM3,ENABLE); //ʹ��TIM3
}



//============================================================

void TIM3_IRQHandler(void){ //TIM3�жϴ�����
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET){	//�ж��Ƿ���TIM3�ж�
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

        //�˴�д���û��Լ��Ĵ������
	     Millis++ ; // ���� + 1


		if(CXX_TX_DLY)		    //CXX������ʱ
		 		CXX_TX_DLY--;

		if(P_delay_time)
			P_delay_time--;


    }
}

//============================================================

