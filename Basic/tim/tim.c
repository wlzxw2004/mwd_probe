

#include "tim.h"
#include "led.h"
#include "pscxx.h"
vu32 Millis=0;//系统毫秒计时器  T3中断内部计数

u8 can_tx_dly;

extern vu16 ADC_DMA_IN[2]; //声明外部变量
extern vu32 adc_sum[2];   //AD转换的累加
extern vu16 adc_cnt;	  //AD累加的次数
extern u32  P_delay_time;
//extern vu16 CXX_RXOVER_TS;
//extern vu16 USART_CXX_RX_TS;

u8 T3_set;
u8 T3_CXX_IDLE=1;

u8  T3_CXX_RXOK=0;

u16 CXX_TX_DLY=200;

//定时器时间计算公式Tout = ((重装载值+1)*(预分频系数+1))/时钟频率;
//例如：1毫秒定时，重装载值=99，预分频系数=719


//============================================================
void TIM3_NVIC_Init (void){ //开启TIM3中断向量
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x2;	//设置抢占和子优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
//============================================================
void TIM3_Init(u16 arr,u16 psc){  //TIM3 初始化 arr重装载值 psc预分频系数
    TIM_TimeBaseInitTypeDef     TIM_TimeBaseInitStrue;
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);//使能TIM3
    TIM3_NVIC_Init (); //开启TIM3中断向量
	      
    TIM_TimeBaseInitStrue.TIM_Period=arr; //设置自动重装载值
    TIM_TimeBaseInitStrue.TIM_Prescaler=psc; //预分频系数

    TIM_TimeBaseInitStrue.TIM_CounterMode=	TIM_CounterMode_Up; //计数器向上溢出

    TIM_TimeBaseInitStrue.TIM_ClockDivision=TIM_CKD_DIV1; //时钟的分频因子，起到了一点点的延时作用，一般设为TIM_CKD_DIV1
    TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStrue); //TIM3初始化设置
 
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);//使能TIM3中断    
    TIM_Cmd(TIM3,ENABLE); //使能TIM3
}



//============================================================

void TIM3_IRQHandler(void){ //TIM3中断处理函数
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET){	//判断是否是TIM3中断
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

        //此处写入用户自己的处理程序
	     Millis++ ; // 毫秒 + 1


		if(CXX_TX_DLY)		    //CXX发送延时
		 		CXX_TX_DLY--;

		if(P_delay_time)
			P_delay_time--;


    }
}

//============================================================

