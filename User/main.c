/*********************************************************************************************
模板制作：  
程序名：	皮带保护系统 拉线急停闭锁开关板
编写人：	刘永忠	
编写时间：	2020年3月20日
硬件支持：	STM32F103C8   外部晶振8MHz RCC函数设置主频16MHz　  


*********************************************************************************************/

#include "stm32f10x.h" //STM32头文件

#include "sys.h"    

#include "delay.h"
#include "iwdg.h"
#include "led.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "pscxx.h"
#include "modbus.h"
#include "key.h"
#include "exti.h"


#define  SQUE_6_C      6     		//序列6发送次数
extern vu16 ADC_DMA_IN[2]; 			//声明外部变量
extern vu32 adc_sum[2];   			//AD转换的累加
extern vu16 adc_cnt;	  				//AD累加的次数
extern vu16 adc_val[2];   			//工程值

extern u8 cxx_led_dly;


u8 Sque_6_count=0;							//序列6发送次数
u8 Sque_4_flag=0;
u8 Sque_2_flag=0;
u8 Pressure_state=0;
u8 Pressure_low_flag=0;
u8 Sque_4_clear_flag=1;
u8 Sque_6_clear_flag=0;
u8 Vibration_flag=0;
u8 Gamma_connect_flag=0;


void send_data_init(u8 sq);
void data_update(u8 sq);
void vibration_sta_judge(void);

int main (void)									//主程序
{

	u8 i;
	delay_ms(100); 								//上电时等待其他器件就绪

	RCC_Configuration(); 					//系统时钟初始化


	LED_Init();

	delay_ms(300);

	NVIC_Configuration();					//嵌套中断向量控制器 的设置
	USART1_Init(9600); 						//串口1初始化并启动
	USART_CXX_Init(9600); 				//查询线用串口，波特率9600
 //	USART_MBUS_Init(115200); 		//串口初始化（参数是波特率）
	USART2_Init(9600);						//串口2初始化
	USART3_Init(9600);						//串口3初始化
	TIM3_Init(99,179);						//定时器初始化,定时时间0.25ms
	KEY_Init();
	EXTIX_Init();
	IWDG_Init(4,2000) ;
	for(i=0;i<10;i++)
	{
		delay_s(2);
	}

	//Vibration_flag=1;
	//gamma连接判断
	if(Gamma_data_receive_flag)
	{
		Gamma_data_receive_flag=0;
		Gamma_connect_flag=1;
	}
	for(i=0;i<30;i++)
	{
		Send_buf[i]=0;
		Rdy_buf[i]=0;
	}	
	if(Gamma_connect_flag)
	{
		data_copy_dynamic(usart_director_data_dynamic,usart_director_data,D_DATA_SIZE);				//	
		data_copy_dynamic(usart_gamma_data_dynamic,usart_gamma_data,G_DATA_SIZE);				//	
		Data_Calculate_g(8,0);
		data_copy();		
	}
	else
	{
		data_copy_dynamic(usart_director_data_dynamic,usart_director_data,D_DATA_SIZE);				//	
		Data_Calculate(8,0);
		data_copy();	
	}
	
	while(1)					//发送一组序列8后跳出循环
	{
			IWDG_Feed();
			if(Task_ok==1)
			{
				Task_ok=0;
				Sque_6_count=SQUE_6_C;				//序列8发送完成后，准备发送序列6
				Sque_6_clear_flag=1;
					for(i=0;i<30;i++)
					{
						Send_buf[i]=0;
						Rdy_buf[i]=0;
					}	
				break;
			}
			else
			{
				Cxx_IO_Init();	
			}
	}
	//以上代码只执行一次
/*****************************************************************************************/
	while(1)						//主循环
	{
			IWDG_Feed();
			vibration_sta_judge();	
			if(Director_data_receive_flag)					//一组数据接收完成后，复制到动态数组中
			{
				Director_data_receive_flag=0;
				data_copy_dynamic(usart_director_data_dynamic,usart_director_data,D_DATA_SIZE);				//	
			}
			if(Gamma_data_receive_flag)
			{
				Gamma_data_receive_flag=0;
				data_copy_dynamic(usart_gamma_data_dynamic,usart_gamma_data,G_DATA_SIZE);	
			}
////////////////////////////////////////////////////////////////////////////////////////////
//数据计算和发送

			if(Sque_4_flag)							//发送序列4
			{			
				if(Task_ok==1)
				{
					Task_ok=0;
					Sque_4_flag=0;				
						for(i=0;i<30;i++)
					{
						Send_buf[i]=0;
						Rdy_buf[i]=0;
					}		
																		
					Sque_6_count=SQUE_6_C;
					Sque_6_clear_flag=1;					
				}
				else
				{
					if(Sque_4_clear_flag)				//振动中断后，设置标志位，将数据发送的中间变量全部清零，从头开始发送序列4
					{
						 Sque_4_clear_flag=0;						
						send_data_init(4);

					}
					Cxx_IO_Init();
				}
			}
			if((Sque_6_count)&&(Sque_4_flag==0))			//发送序列6
			{
				data_update(6);				
				if(Task_ok==1)
				{
					Task_ok=0;
					Sque_6_count--;
						LK_LEDCTL_OFF;
						delay_s(10);							
					if(Sque_6_count==0)
					{
						Sque_2_flag=1;
						for(i=0;i<30;i++)
						{
							Send_buf[i]=0;
							Rdy_buf[i]=0;
						}		
					}
//					send_data_init(6);			
				}
				else
				{
					if(Sque_6_clear_flag)
					{			
						send_data_init(6);						
						
					}
					Cxx_IO_Init();
				}
				

			}
			else if((Sque_2_flag)&&(Sque_4_flag==0))				//发送序列2
			{

				data_update(2);				
				if(Task_ok==1)
				{
					Task_ok=0;
					Sque_2_flag=0;
					send_data_init(2);								
				}
				else
				{
					Cxx_IO_Init();	
				}
			}
			
     //--------------------------------------

	}//while(1)
} //MAIN()


/*

【变量定义】
u32     a; //定义32位无符号变量a
u16     a; //定义16位无符号变量a
u8     a; //定义8位无符号变量a
vu32     a; //定义易变的32位无符号变量a
vu16     a; //定义易变的 16位无符号变量a
vu8     a; //定义易变的 8位无符号变量a
uc32     a; //定义只读的32位无符号变量a
uc16     a; //定义只读 的16位无符号变量a
uc8     a; //定义只读 的8位无符号变量a

#define ONE  1   //宏定义

delay_us(1); //延时1微秒
delay_ms(1); //延时1毫秒
delay_s(1); //延时1秒

GPIO_WriteBit(LEDPORT,LED1,(BitAction)(1)); //LED控制

*/

////-----------------------------------------------------------------------------
//函数名称： vibration_sta_judge
//函数功能：读取端口电平，结合外部中断，判断仪器状态
//入口参数：无
//出口参数：无
//备注：
////
////-----------------------------------------------------------------------------
void vibration_sta_judge(void)
{
	u8 i;
	
		Pressure_state=GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0);		//读取振动判断端口PA0的状态		
		if(Pressure_state==Bit_RESET)
		{

			if(Sque_4_flag)
			{
				Vibration_flag=0;
				Sque_4_flag=0;
				Sque_6_count=SQUE_6_C;	
				Sque_6_clear_flag=1;			
			
			}
			else
			{
				Vibration_flag=0;
				Sque_4_flag=0;	
				if(Director_data_receive_flag)		
				{
					Director_data_receive_flag=0;
					for(i=0;i<D_DATA_SIZE;i++)
					{
						usart_director_data_sta[i]=usart_director_data[i];			//静止状态下，当接收完成一组数据后，更新静态数据
					}						
				}
			
			}			
		}
		
		if(Vibration_flag)																					//是否有上升沿
		{
			delay_ms(500);																							//延时消抖
			Pressure_state=GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0);		//读取压力判断端口PA0的状态
			if(Pressure_state==Bit_SET)
			{
				Vibration_flag=0;
				Sque_4_flag=1;
				Sque_4_clear_flag=1;			
			}
			else
			{
				Vibration_flag=0;
			}
		}

}

////-----------------------------------------------------------------------------
//函数名称： send_data_init
//函数功能：每组序列发送前初始化，将中间变量清零
//入口参数：序列号
//出口参数：无
//备注：
////
////-----------------------------------------------------------------------------

void send_data_init(u8 sq)
{
	u8 i;
	switch(sq)
	{
		case 2:
//			LK_LEDCTL_OFF;
//			delay_s(10);
			Sque_6_count=SQUE_6_C;				
			Sque_6_clear_flag=1;		
			for(i=0;i<30;i++)
			{
			Send_buf[i]=0;
			Rdy_buf[i]=0;
			}
			break;
		case 4:
			for(i=0;i<30;i++)
			{
				Send_buf[i]=0;
				Rdy_buf[i]=0;
			}	
			if(Gamma_connect_flag)
			{
				Data_Calculate_sta_g(4);
				data_copy();					
			}
			else
			{
				Data_Calculate_sta(4);	
				data_copy();	
			}
								
			B_CXX_STA=0;
			B_BX_ID = 0;
			B_MC_ID = 0;
			W_BX_NUM = 0;	
			Sque_6_count=0;
			LK_LEDCTL_OFF;
			delay_s(30);					//序列4发送之前的等待时间
			LK_LEDCTL_ON;
			delay_ms(1500);
			LK_LEDCTL_OFF;
			delay_s(10);					//测试脉冲后的延时时间
			break;
		case 6:
			B_CXX_STA=0;
			B_BX_ID = 0;
			B_MC_ID = 0;
			W_BX_NUM = 0;
			Sque_6_clear_flag=0;
			Sque_6_count=SQUE_6_C;
			LK_LEDCTL_OFF;
			delay_s(10);		
			for(i=0;i<30;i++)
			{
				Send_buf[i]=0;
				Rdy_buf[i]=0;
			}		
			
			break;		
	}
}
////-----------------------------------------------------------------------------
//函数名称： data_update
//函数功能：对于序列2和序列6，需要实时更新数据，选择在合适的时间点更新数据
//入口参数：序列号
//出口参数：无
//备注：
////
////-----------------------------------------------------------------------------
void data_update(u8 sq)
{
	if(Gamma_connect_flag)
	{
		switch(sq)
		{
			case 2:
				if(Data_update_always_flag)
				{
					Data_Calculate_g(2,1);
				}
				if((B_CXX_STA)&&(B_MC_ID==2))
				{
					Data_Calculate_g(2,2);
				}
				if((B_CXX_STA)&&(B_MC_ID==4))
				{
					Data_Calculate_g(2,3);
				}				
				if((B_CXX_STA)&&(B_MC_ID==7))
				{
					Data_Calculate_g(2,4);
				}			
				if((B_CXX_STA)&&(B_MC_ID==9))
				{
					Data_Calculate_g(2,5);
				}					
				break;
			case 6:
				if(Data_update_always_flag)					//发送同步头时随时可以更新数据
				{
					Data_Calculate_g(6,1);
				}
				if((B_CXX_STA)&&(B_MC_ID==2))
				{
					Data_Calculate_g(6,2);
				}
				if((B_CXX_STA)&&(B_MC_ID==4))
				{
					Data_Calculate_g(6,3);
				}				
				if((B_CXX_STA)&&(B_MC_ID==6))
				{
					Data_Calculate_g(6,4);
				}							
				if((B_CXX_STA)&&(B_MC_ID==8))
				{
					Data_Calculate_g(6,5);
				}					
		}		
	}
	else
	{
		switch(sq)
		{
			case 2:
				if(Data_update_always_flag)
				{
					Data_Calculate(2,1);
				}
				if((B_CXX_STA)&&(B_MC_ID==2))
				{
					Data_Calculate(2,2);
				}
				if((B_CXX_STA)&&(B_MC_ID==5))
				{
					Data_Calculate(2,3);
				}				
				if((B_CXX_STA)&&(B_MC_ID==7))
				{
					Data_Calculate(2,4);
				}			
				break;
			case 6:
				if(Data_update_always_flag)					//发送同步头时随时可以更新数据
				{
					Data_Calculate(6,1);
				}
				if((B_CXX_STA)&&(B_MC_ID==2))
				{
					Data_Calculate(6,2);
				}
				if((B_CXX_STA)&&(B_MC_ID==4))
				{
					Data_Calculate(6,3);
				}				
				if((B_CXX_STA)&&(B_MC_ID==6))
				{
					Data_Calculate(6,4);
				}							
			
		}	
	}

}



