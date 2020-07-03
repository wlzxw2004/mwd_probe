/*********************************************************************************************
ģ��������  
��������	Ƥ������ϵͳ ���߼�ͣ�������ذ�
��д�ˣ�	������	
��дʱ�䣺	2020��3��20��
Ӳ��֧�֣�	STM32F103C8   �ⲿ����8MHz RCC����������Ƶ16MHz��  


*********************************************************************************************/

#include "stm32f10x.h" //STM32ͷ�ļ�

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


#define  SQUE_6_C      6     		//����6���ʹ���
extern vu16 ADC_DMA_IN[2]; 			//�����ⲿ����
extern vu32 adc_sum[2];   			//ADת�����ۼ�
extern vu16 adc_cnt;	  				//AD�ۼӵĴ���
extern vu16 adc_val[2];   			//����ֵ

extern u8 cxx_led_dly;


u8 Sque_6_count=0;							//����6���ʹ���
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

int main (void)									//������
{

	u8 i;
	delay_ms(100); 								//�ϵ�ʱ�ȴ�������������

	RCC_Configuration(); 					//ϵͳʱ�ӳ�ʼ��


	LED_Init();

	delay_ms(300);

	NVIC_Configuration();					//Ƕ���ж����������� ������
	USART1_Init(9600); 						//����1��ʼ��������
	USART_CXX_Init(9600); 				//��ѯ���ô��ڣ�������9600
 //	USART_MBUS_Init(115200); 		//���ڳ�ʼ���������ǲ����ʣ�
	USART2_Init(9600);						//����2��ʼ��
	USART3_Init(9600);						//����3��ʼ��
	TIM3_Init(99,179);						//��ʱ����ʼ��,��ʱʱ��0.25ms
	KEY_Init();
	EXTIX_Init();
	IWDG_Init(4,2000) ;
	for(i=0;i<10;i++)
	{
		delay_s(2);
	}

	//Vibration_flag=1;
	//gamma�����ж�
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
	
	while(1)					//����һ������8������ѭ��
	{
			IWDG_Feed();
			if(Task_ok==1)
			{
				Task_ok=0;
				Sque_6_count=SQUE_6_C;				//����8������ɺ�׼����������6
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
	//���ϴ���ִֻ��һ��
/*****************************************************************************************/
	while(1)						//��ѭ��
	{
			IWDG_Feed();
			vibration_sta_judge();	
			if(Director_data_receive_flag)					//һ�����ݽ�����ɺ󣬸��Ƶ���̬������
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
//���ݼ���ͷ���

			if(Sque_4_flag)							//��������4
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
					if(Sque_4_clear_flag)				//���жϺ����ñ�־λ�������ݷ��͵��м����ȫ�����㣬��ͷ��ʼ��������4
					{
						 Sque_4_clear_flag=0;						
						send_data_init(4);

					}
					Cxx_IO_Init();
				}
			}
			if((Sque_6_count)&&(Sque_4_flag==0))			//��������6
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
			else if((Sque_2_flag)&&(Sque_4_flag==0))				//��������2
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

���������塿
u32     a; //����32λ�޷��ű���a
u16     a; //����16λ�޷��ű���a
u8     a; //����8λ�޷��ű���a
vu32     a; //�����ױ��32λ�޷��ű���a
vu16     a; //�����ױ�� 16λ�޷��ű���a
vu8     a; //�����ױ�� 8λ�޷��ű���a
uc32     a; //����ֻ����32λ�޷��ű���a
uc16     a; //����ֻ�� ��16λ�޷��ű���a
uc8     a; //����ֻ�� ��8λ�޷��ű���a

#define ONE  1   //�궨��

delay_us(1); //��ʱ1΢��
delay_ms(1); //��ʱ1����
delay_s(1); //��ʱ1��

GPIO_WriteBit(LEDPORT,LED1,(BitAction)(1)); //LED����

*/

////-----------------------------------------------------------------------------
//�������ƣ� vibration_sta_judge
//�������ܣ���ȡ�˿ڵ�ƽ������ⲿ�жϣ��ж�����״̬
//��ڲ�������
//���ڲ�������
//��ע��
////
////-----------------------------------------------------------------------------
void vibration_sta_judge(void)
{
	u8 i;
	
		Pressure_state=GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0);		//��ȡ���ж϶˿�PA0��״̬		
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
						usart_director_data_sta[i]=usart_director_data[i];			//��ֹ״̬�£����������һ�����ݺ󣬸��¾�̬����
					}						
				}
			
			}			
		}
		
		if(Vibration_flag)																					//�Ƿ���������
		{
			delay_ms(500);																							//��ʱ����
			Pressure_state=GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0);		//��ȡѹ���ж϶˿�PA0��״̬
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
//�������ƣ� send_data_init
//�������ܣ�ÿ�����з���ǰ��ʼ�������м��������
//��ڲ��������к�
//���ڲ�������
//��ע��
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
			delay_s(30);					//����4����֮ǰ�ĵȴ�ʱ��
			LK_LEDCTL_ON;
			delay_ms(1500);
			LK_LEDCTL_OFF;
			delay_s(10);					//������������ʱʱ��
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
//�������ƣ� data_update
//�������ܣ���������2������6����Ҫʵʱ�������ݣ�ѡ���ں��ʵ�ʱ����������
//��ڲ��������к�
//���ڲ�������
//��ע��
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
				if(Data_update_always_flag)					//����ͬ��ͷʱ��ʱ���Ը�������
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
				if(Data_update_always_flag)					//����ͬ��ͷʱ��ʱ���Ը�������
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



