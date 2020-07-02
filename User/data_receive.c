#include "data_receive.h"

u8 usart_director_data[D_DATA_SIZE];
u8 usart_gamma_data[G_DATA_SIZE];
u8 usart_director_data_sta[D_DATA_SIZE];
u8 usart_director_data_dynamic[D_DATA_SIZE];
u8 Director_data_receive_flag=0;
u8 Gamma_data_receive_flag=0;
u32  P_delay_time=0;
////-----------------------------------------------------------------------------
//�������ƣ� directional_data_receive
//�������ܣ��������ݣ�����һ�����ݵ�ͷ����ȷ����������һ������
//��ڲ��������ֽ� data_usart
//���ڲ�������
//��ע��һ�����ݵ�ͷ��060203������β��0604���˳���δ�ж�����β
////
////-----------------------------------------------------------------------------
void directional_data_receive(uint8_t data_usart)
{
	static uint8_t director_sync_state=0;
	static uint8_t receive_director_data_len=0;
		
	if(director_sync_state==0)
	{
			if(data_usart==0x06)
					director_sync_state=1;
			else
					director_sync_state=0;

	}
	else if(director_sync_state==1)
	{
			if(data_usart==0x02)
					director_sync_state=2;
			else
					director_sync_state=0;
	}
	 else if(director_sync_state==2)
	{
			if(data_usart==0x03)
					director_sync_state=3;
			else
					director_sync_state=0;
	}
	else if(director_sync_state==3|director_sync_state==4)
	{
			usart_director_data[receive_director_data_len++]=data_usart;          //���ݱ���
			if(receive_director_data_len==D_DATA_SIZE)
			{
					receive_director_data_len=0;
					director_sync_state=5;
			}
					
			else
					director_sync_state=4;
	}
	else if(director_sync_state==5)
	{
					director_sync_state=0;
					Director_data_receive_flag=1;               
	}
	else
					director_sync_state=0;
		
}

////-----------------------------------------------------------------------------
//�������ƣ� gamma_data_receive
//�������ܣ��������ݣ�����һ�����ݵ�ͷ����ȷ����������һ������
//��ڲ��������ֽ� data_usart
//���ڲ�������
//��ע��һ�����ݵ�ͷ��06 31������β��0604���˳���δ�ж�����β
////
////-----------------------------------------------------------------------------
void gamma_data_receive(u8 data_usartg)
{
		static uint8_t gamma_sync_state=0;
		static uint8_t receive_gamma_data_len=0;
    if(gamma_sync_state==0)
    {
        if(data_usartg==0x06)
        {
            gamma_sync_state=1;          
        }

        else
            gamma_sync_state=0;

    }
    else if(gamma_sync_state==1)
    {
        if(data_usartg==0x31)
        {
            gamma_sync_state=2;
        }
            
        else
        {
            gamma_sync_state=0;
        }
            
    }
    else if(gamma_sync_state==2|gamma_sync_state==3)
    {
        usart_gamma_data[receive_gamma_data_len++]=data_usartg;          //���ݱ���
        if(receive_gamma_data_len==G_DATA_SIZE)
        {
            receive_gamma_data_len=0;
            gamma_sync_state=4;            
        }

        else
        {
            gamma_sync_state=3;
        }
            
    }
    else if(gamma_sync_state==4)
    {
        if(data_usartg==0x06)
        {
            gamma_sync_state=5;
        }
            
        else
        {
            gamma_sync_state=0;
        }
            
    }
    else if(gamma_sync_state==5)
    {
        if(data_usartg==0x04)
        {
            gamma_sync_state=0;
            Gamma_data_receive_flag=1;

        }
        else
        {
            gamma_sync_state=0;
        }
            
    }

}

/*****************************************************************************************/

