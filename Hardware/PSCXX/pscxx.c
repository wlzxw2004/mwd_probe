

#include "sys.h"
#include "usart.h"
#include "pscxx.h"
#include "tim.h"
#include "modbus.h"
#include "delay.h"


#include "led.h"


extern u16 CXX_TX_DLY;
extern u8 Sque_6_count;			//����6����4��

u8 CXX_RXNUM_BAK ;
u8 CXX_TXNUM ;
u8 Data_update_always_flag=1;
u8 Data_update_temp_flag=0;				//�������ݸ��±�־��1 �������У�0����������
u8 Data_update_run_falg=0;				//���ݸ����Ƿ�ִ�У�1 �Ѿ����У�0 δ����
u8 Output_pulse_h_flag=0; 				//������������Ϊ1����Ϊ0
			
u8  CXX_FUNC_BUF[250]=	{0X03,0X7D,0X00,0X00,0X00,0X00,0X00,0X0,
						0X15,0x3F,0X41,0X21,0X41,0X21,0X41,0X21,0X33,0x41,0x21,0x11,0X35,0x41,0x21,0X1A,0X70};
						


u8  CXX_MC[60] = {0x2F,0x5D,0x78,0x88,0x90,0x94,0x97,0x98,0x99,0x99,
                  0x99,0x99,0x99,0x99,0x99,0x99,0x99,0x99,0x99,0x99,
				  0x99,0x99,0x99,0x99,0x99,0x99,0x99,0x99,0x99,0x99,
				  0x99,0x99,0x99,0x99,0x99,0x99,0x99,0x99,0x99,0x99,
				  0x99,0x99,0x99,0x99,0x99,0x88,0x78,0x5D,0x22,0x11};	

u8  CXX_DATOUT[20] = {0x7F,0xFF,0xFF,0x10,0x00,0x00,0x00,0x00,0x00,0x00,
                      0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0D,0x0A};	


u8 B_CXX_STA=0; 

u8 	B_BX_ID=0 ;	  		//���ε�ID��
u8 	B_BX_NUM ;	  		//���ε�ID�ĵڸ��Σ�0�����ж�Σ�1����ξ�һ����
u8 	B_BX_SUM ;

u8 	B_MC_ID =0;    		//	H L ���εĵڼ�����
u8  B_MC_SUM ;		//����㣬���ٴ�����

u8  B_MC_TYPE;	  //��ǰ�εĲ������� 


u16 W_BX_NUM=0 ;	  		//���ε�ID�ĵڸ��Σ�0�����ж�Σ�1����ξ�һ����
u16	W_BX_SUM ;
u16	W_BX_0SUM ;

u8 	CXX_tx_en=0; 
u8 Task_ok=0;

//==============��ѯ�߶Ͽ���ʼ��=========================


void Cxx_IO_Init(void){ //��ѯ�߽ӿڳ�ʼ��

//u8 i,b1;
//u16 w1;
//	CXX_FUNC_BUF[0]=0;
//	B_CXX_STA=0;
//	B_BX_ID = 0;
//	B_MC_ID = 0;
//	W_BX_NUM = 0;

	
	 //------------------------------------
		if(CXX_TX_DLY==0){

			CXX_TX_DLY = 0x7d;							//125*0.25=31.25ms

			//------------------------------
			if(B_CXX_STA==0){	  //��ʼ����

			  W_BX_SUM = 96;								
			  if(W_BX_NUM>=W_BX_SUM) //���岨����������31.25*96=3000ms
			  {
				B_MC_ID++;
				W_BX_NUM = 0;
			  }

			  if(B_MC_ID<3 )
				{
					if(W_BX_NUM<48)						//31.25*48=1500ms��������1.5s
					  	LK_LEDCTL_ON;
					else
						LK_LEDCTL_OFF;

					//��������3�������ź�
			  }	//......................................
			  else
			  {	  // 3����ͷ�������

			   	B_CXX_STA = 1;
			   	B_MC_ID = 0;
					W_BX_NUM = 0;

				//B_BX_ID = 0;

					W_BX_SUM = 0;
				if(Send_buf[B_MC_ID]&0x40)//�е�
					W_BX_SUM = (Send_buf[B_MC_ID]&0x1f)*24;
				
					W_BX_0SUM = W_BX_SUM;
				
				if(Send_buf[B_MC_ID]&0x80)//�и�
					W_BX_SUM += 96;

				//���㲻ͬ��ͬ��ͷ����ʱʱ��
				
			  }

				
				//�������ݸ��±�־
				Data_update_always_flag=1;
			}
			//----------------------------------
			else if	 (B_CXX_STA==1)
				{
					Data_update_always_flag=0;

				if(W_BX_NUM>=W_BX_SUM)
				{
			   	B_MC_ID ++;
					W_BX_NUM = 0;
					

					if(Send_buf[B_MC_ID]==0)
					{  //�κ�ֵ=0

						Task_ok = 1;        // ������ɡ�
					//		CXX_FUNC_BUF[0]=0;
							B_CXX_STA=0;
							B_BX_ID = 0;
							B_MC_ID = 0;
							W_BX_NUM = 0;

					}
					else
					{
						W_BX_SUM = 0;
						if(Send_buf[B_MC_ID]&0x40)//�е�
						{

							W_BX_SUM = (Send_buf[B_MC_ID]&0x1f);
							W_BX_0SUM = W_BX_SUM;

							if(Send_buf[B_MC_ID]&0x20)
							W_BX_SUM  = 0x20-W_BX_SUM;

						    W_BX_0SUM =  W_BX_SUM*24;
							W_BX_SUM  =  W_BX_0SUM;


						}
						if(Send_buf[B_MC_ID]&0x80)//�и�
							W_BX_SUM += 96;

					}

				}
				//.........................................
				if(Task_ok==0)
				{
					if(Send_buf[B_MC_ID]&0x80)
					{	 //�и������

						if(  (W_BX_NUM >=W_BX_0SUM)&&( W_BX_NUM< (W_BX_0SUM+48)) )
						{
							LK_LEDCTL_ON;
				//			Output_pulse_h_flag=1;
						}
					  		
						else
						{
							LK_LEDCTL_OFF; 
			//				Output_pulse_h_flag=0;
							//�ж�ID��
							
						}
								
								

					}
						
				} 

			}
			//----------------------------------			
			W_BX_NUM ++;		
		}

	//----------------------------------

	

}


//===========��ѯ�ߴ���������====================
void Pscxx_Main(void){ 

		  
	//---------------------------------------------------------------------------------

			if(CXX_tx_en){					   	//---������Ҫ����
	 			if((!USART_CXX_TXSTA))	{  

		   			USART_CXX_Txstart(CXX_TXNUM);	//��ʼ����2�ֽ�

					CXX_TXNUM = 0 ;
		   			CXX_tx_en = 0;

				}
			}


		if(USART_CXX_TXSTA&0x80){ 			//�����������

			USART_CXX_TXSTA =0;

		}
	//-------------------------------------------------------------------------------



}



//*********************************************************************************************
