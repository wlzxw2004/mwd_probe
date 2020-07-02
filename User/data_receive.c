#include "data_receive.h"

u8 usart_director_data[D_DATA_SIZE];
u8 usart_gamma_data[G_DATA_SIZE];
u8 usart_director_data_sta[D_DATA_SIZE];
u8 usart_director_data_dynamic[D_DATA_SIZE];
u8 Director_data_receive_flag=0;
u8 Gamma_data_receive_flag=0;
u32  P_delay_time=0;
////-----------------------------------------------------------------------------
//函数名称： directional_data_receive
//函数功能：接收数据，查找一组数据的头，来确认完整接收一组数据
//入口参数：单字节 data_usart
//出口参数：无
//备注：一组数据的头是060203，数据尾是0604，此程序未判断数据尾
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
			usart_director_data[receive_director_data_len++]=data_usart;          //数据保存
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
//函数名称： gamma_data_receive
//函数功能：接收数据，查找一组数据的头，来确认完整接收一组数据
//入口参数：单字节 data_usart
//出口参数：无
//备注：一组数据的头是06 31，数据尾是0604，此程序未判断数据尾
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
        usart_gamma_data[receive_gamma_data_len++]=data_usartg;          //数据保存
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

