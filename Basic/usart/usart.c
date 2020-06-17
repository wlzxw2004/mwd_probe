
/*
���޸���־��
1-201708271933 ����������ʱ������


*/
#include "stm32f10x_usart.h"

#include "sys.h"
#include "usart.h"
#include "pscxx.h"
#include "modbus.h" 	 
#include "led.h"
#include "tim.h"

extern vu32 Millis;//ϵͳ�����ʱ��  T3�ж��ڲ�����

//-----------------------------------------------------------


u8  USART_MBUS_BUF[MBUS_MAXBUF_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.
u8  USART_MBUS_RXBUF[MBUS_MAXBUF_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.


u8 USART_MBUS_RXNUM=0;         	 	//�Ѿ����յ�����
vu32 USART_MBUS_RX_TS=0;         	//�������ݵ�ʱ���
	
u8 USART_MBUS_TXSUM=0;         		//��Ҫ���͵ĳ���	
u8 USART_MBUS_SEND_NUM=0;         	//�Ѿ����͵ĳ���

u8 USART_MBUS_RXSTA=0;         		//����״̬��� //bit7��������ɱ�־  //bit6����ʼ����   //bit5�����ջ��������
u8 USART_MBUS_TXSTA=0;         		//����״̬��� //bit7��������� //bit6��������Ҫ����

//-----------------------------------------------------------

u8  USART_CXX_RX_BUF[CXX_MAXRX_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.
u8  USART_CXX_TX_BUF[CXX_MAXTX_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.

u8 USART_CXX_RXNUM=0;         	 	//�Ѿ����յ�����

vu16 USART_CXX_RX_TS=0;         	//�������ݵ�ʱ���
vu16 CXX_RXOVER_TS;
	
u8 USART_CXX_TXSUM=0;         		//��Ҫ���͵ĳ���	
u8 USART_CXX_SEND_NUM=0;         	//�Ѿ����͵ĳ���

u8 USART_CXX_RXSTA=0;         		//����״̬��� //bit7��������ɱ�־  //bit6����ʼ����   //bit5�����ջ��������
u8 USART_CXX_TXSTA=0;         		//����״̬��� //bit7��������� //bit6��������Ҫ����

//-----------------------------------------------------------
void USART_CXX_Init(u32 bound){ //����1��ʼ��������

//	USART1_Init(bound);
//	USART2_Init(bound);
	USART3_Init(bound);
}

//-----------------------------------------------------------
void USART_MBUS_Init(u32 bound){ //����1��ʼ��������

	USART1_Init(bound);
//	USART2_Init(bound);
//	USART3_Init(bound);
}
//-----------------------------------------------------------
/*
USART1������س���
*/

#if EN_USART1   //USART1ʹ��������ѡ��

u8  USART1_RX_BUF[USART1_MAXRX_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.
u8  USART1_TX_BUF[USART1_MAXTX_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.

u16 USART1_RXNUM=0;         		//�Ѿ����յ�����
vu32 USART1_RX_TS=0;         		//�������ݵ�ʱ���
	
u16 USART1_TXNUM=0;         		//��Ҫ���͵ĳ���	
u16 USART1_SEND_NUM=0;         	//�Ѿ����͵ĳ���

u8 USART1_RXSTA=0;         		//����״̬���


u8 USART1_TXSTA=0;         		//����״̬���

void USART1_Init(u32 bound){ //����1��ʼ��������
    //GPIO�˿�����
    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
		 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//ʹ��USART1��GPIOAʱ��


     //USART1_TX   PA.9
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
    GPIO_Init(GPIOA, &GPIO_InitStructure);  
    //USART1_RX	  PA.10
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
    GPIO_Init(GPIOA, &GPIO_InitStructure); 

   //Usart1 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ��� 

   //USART ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//һ������Ϊ9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
    USART_Init(USART1, &USART_InitStructure); //��ʼ������


    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//�����жϣ�����ENABLE/�ر�DISABLE�ж�
  //  USART_ITConfig(USART1, USART_IT_TXE, ENABLE);//�����жϣ�����ENABLE/�ر�DISABLE�ж�

    USART_Cmd(USART1, ENABLE);                    //ʹ�ܴ��� 
}

//-------------------------------------------
void USART1_IRQHandler(void){ //����1�жϷ�����򣨹̶��ĺ����������޸ģ�	

	static u8 R_temp;
	
	//	MBUS_IRQHandler(USART1);
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //�����ж�
		{
			R_temp =USART_ReceiveData(USART1);	//��ȡ���յ�������
			if(R_temp==0x55)
				CXX_LED_ON;
			if(R_temp==0xaa)
				CXX_LED_OFF;

		}
		
	USART_ClearITPendingBit(USART1,USART_IT_RXNE);
			
} 


#endif	



//===========================================================

/*
USART2������س���
*/
#if EN_USART2   //USART2ʹ��������ѡ��


 //===========================================================
void USART2_Init(u32 bound){ //����2��ʼ��������
    //GPIO�˿�����
    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
		 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE); //ʹ��UART2����GPIOA��ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); //ʹ�ܴ��ڵ�RCCʱ��

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; //����USART2��RX�ӿ���PA3
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
    GPIO_Init(GPIOA, &GPIO_InitStructure); 

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //����USART2��TX�ӿ���PA2
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
    GPIO_Init(GPIOA, &GPIO_InitStructure);  

   //USART2 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//һ������Ϊ9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
    USART_Init(USART2, &USART_InitStructure); //��ʼ������

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//����ENABLE/�ر�DISABLE�ж�
    USART_Cmd(USART2, ENABLE);                    //ʹ�ܴ��� 

   //Usart2 NVIC ����

    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		//���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ��� 
}


//=========================================================================
void USART2_IRQHandler(void){ //����2�жϷ�����򣨹̶��ĺ����������޸ģ�	

	static u8 R_temp;
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //�����ж�
		{
			R_temp =USART_ReceiveData(USART2);	//��ȡ���յ�������
			directional_data_receive(R_temp);
			
     } 
	
}
 
#endif	



//===========================================================
#if EN_USART3   //���ʹ���˽���
u8 USART3_RX_BUF[USART3_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
u16 USART3_RX_STA=0;       //����״̬���	  

/*
USART3ר�õ�printf����
��ͬʱ����2�����ϴ���ʱ��printf����ֻ����������֮һ����������Ҫ�Դ�������printf����
���÷�����USART3_printf("123"); //��USART3�����ַ�123
*/

//����3��ʼ��


void USART3_Init(u32 BaudRate){ //USART3��ʼ��������
   GPIO_InitTypeDef GPIO_InitStructure;
   USART_InitTypeDef USART_InitStructure;
   NVIC_InitTypeDef NVIC_InitStructure; 

   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE); //ʹ��UART3����GPIOB��ʱ��
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); //ʹ�ܴ��ڵ�RCCʱ��

   //����ʹ�õ�GPIO������
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//����USART3��RX�ӿ���PB11
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//�ӿ�ģʽ ��������
   GPIO_Init(GPIOB, &GPIO_InitStructure);

   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//����USART3��TX�ӿ���PB10
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//����ٶ�50MHz
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//�ӿ�ģʽ �����������
   GPIO_Init(GPIOB, &GPIO_InitStructure);

   //���ô���
   USART_InitStructure.USART_BaudRate = BaudRate;
   USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
   USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
   USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
   USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������


   USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

 //  USART_InitStructure.USART_Mode =  USART_Mode_Tx;	//ֻ���� ģʽ

   USART_Init(USART3, &USART_InitStructure);//���ô���3

   USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//ʹ�ܴ��ڽ����ж�  
   USART_ITConfig(USART3, USART_IT_TXE, DISABLE);//���ڷ����ж��ڷ�������ʱ����

   USART_Cmd(USART3, ENABLE);//ʹ�ܴ���3


   //�����ж�����
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; 	 //���ȵȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;			 //��Ӧ�ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ��� 
}




//==================================================
//����3�жϷ�����򣨹̶��ĺ����������޸ģ�
void USART3_IRQHandler(void){ 	

//	CXX_IRQHandler(USART3);
		static u8 R_temp;
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //�����ж�
		{
			R_temp =USART_ReceiveData(USART3);	//��ȡ���յ�������
			gamma_data_receive(R_temp);		
     } 
		USART_ClearITPendingBit(USART3,USART_IT_RXNE);
}
#endif	


//======================================================
void CXX_IRQHandler(USART_TypeDef* USARTx){ //��ѯ�� �����жϷ������
	u8 Res;

	if(USART_GetFlagStatus(USARTx,USART_FLAG_ORE)==SET)	//��������־λ
	{
		USART_ClearFlag(USARTx,USART_FLAG_ORE); 
		USART_ReceiveData(USARTx);    		 		//��ȡDR
	}


	//----------�����жϴ���-------------------
	if(USART_GetITStatus(USARTx, USART_IT_RXNE) == SET){  //�����ж�
			
		Res =USART_ReceiveData(USARTx);//(USART1->DR);	//��ȡ���յ�������

        USART_CXX_ITRX(Res);

	} 

	//-------�����ж�----------------------------
	if(USART_GetITStatus(USARTx, USART_IT_TXE)==SET)	//�������ݼĴ����ձ�־λ
	{
		USART_ClearITPendingBit(USARTx,USART_IT_TXE);
		USART_ITConfig(USARTx,USART_IT_TXE,DISABLE);

	}	

	//-------�����ж�----------------------------
	if(USART_GetITStatus(USARTx, USART_IT_TC) == SET){  //�����ж�

		USART_CXX_ITTX();

//		USART_ITConfig(USART1, USART_IT_TC, DISABLE);//�����жϣ��ر�DISABLE�ж�


	}   
	//-----------------------------------


} 

//======================================================
//��ѯ�ߴ��ڽ����ж��ӳ���
void USART_CXX_ITRX(u8 Res){//���ڵ��жϽ���

		USART_CXX_RX_TS = 0;
		CXX_RXOVER_TS = 0;


		if(!(USART_CXX_RXSTA&0x40)){//���ܵ��ĵ�1������
		   USART_CXX_RXSTA =0x40	;  //�����Ѿ���ʼ���ܵı�־
		   USART_CXX_RX_BUF[0] = Res;	//���ܵ��ĵ�1������
		   USART_CXX_RXNUM = 1;
		}
		else if(USART_CXX_RXNUM<CXX_MAXRX_LEN){ 			//���ܻ�����û�����
		        USART_CXX_RX_BUF[USART_CXX_RXNUM] = Res;	//���ܵ�������
		        USART_CXX_RXNUM++;							//���ܳ���+1
	      	 }
		     else  USART_CXX_RXSTA |=0x20; //bit5 ���ܻ����������־


}


//======================================================
//��ѯ�ߴ��� ��������жϣ�������ɷ���1
u8 USART_CXX_Recok(vu16 ts){  //����1��������ж�,ts�Ǻ��뵥λ
u8 ret=0;

     if(USART_CXX_RXSTA&0x40){//���ܵ��ĵ�1������

	  	if( USART_CXX_RX_TS>= ts)	 {
			USART_CXX_RXSTA |=0x80;
		 	ret = 1; 
	  	} 
	 }
   return ret;
}


//======================================================
//��ѯ�ߴ��� ��ʼ����
void USART_CXX_Txstart(u8 txnum){

	if(!(USART_CXX_TXSTA&0x40)){ //û�з�������

	 //�ڷ��͵�ʱ��رս����ж�
		USARTx_MODE(USART_CXX,USART_Mode_Tx); 				//�رս��ܹ���
		USART_ITConfig(USART_CXX, USART_IT_RXNE, DISABLE);	//�ر�DISABLE�ж�
		USART_ReceiveData(USART_CXX);						//���ܵ�USARTx->DR������


		USART_SendData(USART_CXX ,USART_CXX_TX_BUF[0]);//���͵�1���ֽ�
		USART_CXX_TXSTA =0x40;
		USART_CXX_SEND_NUM = 0;
		USART_CXX_TXSUM = txnum;
		USART_ITConfig(USART_CXX, USART_IT_TC, ENABLE);// ����ENABLE	�����ж�

	}	

}


//======================================================
//��ѯ�ߴ��ڷ����ж��ӳ���
void USART_CXX_ITTX(void){//���ڵ��жϽ���
u8 tx_end=0;

	  if(USART_CXX_TXSTA&0x40){ //�з�������

	    	USART_CXX_SEND_NUM++;   //�Ѿ����͵ĳ���
	    	if(USART_CXX_SEND_NUM<USART_CXX_TXSUM){  ////û�з�����ɣ���������
  	       		USART_SendData(USART_CXX, USART_CXX_TX_BUF[USART_CXX_SEND_NUM]);
			}
			else 
			{
	  	      	USART_CXX_TXSTA |=0x80;//���ͽ�����־
				tx_end = 1 ;
		     }

	   } 
	   else tx_end = 1 ;

	//------------------------------
	//��������	   
	 if(tx_end){

			USART_ITConfig(USART_CXX, USART_IT_TC, DISABLE);		//�ط����ж�

			//USARTx_MODE(USART_CXX,USART_Mode_Rx|USART_Mode_Tx); 	// �򿪽��ܹ���
			//USART_ReceiveData(USART_CXX);							//���ܵ�USARTx->DR������
  	        //USART_ITConfig(USART_CXX, USART_IT_RXNE, ENABLE);		//����ENABLE
	 }

}


//==============================================================
void MBUS_IRQHandler(USART_TypeDef* USARTx){ 	

u8 Res;

	if(USART_GetFlagStatus(USARTx,USART_FLAG_ORE)==SET)	//��������־λ
	{
		USART_ClearFlag(USARTx,USART_FLAG_ORE); 
		USART_ReceiveData(USARTx);    		 		//��ȡDR
	}

	//----------�����жϴ���-------------------
	if(USART_GetITStatus(USARTx, USART_IT_RXNE) == SET){  //�����ж�
			
		USART_ClearITPendingBit(USARTx,USART_IT_RXNE);
		Res =USART_ReceiveData(USARTx);					//(USARTx->DR);	//��ȡ���յ�������

        USART_MBUS_ITRX(Res);

	}

	//-------�����ж�----------------------------
	if(USART_GetITStatus(USARTx, USART_IT_TXE)==SET)	//�������ݼĴ����ձ�־λ
	{
		USART_ClearITPendingBit(USARTx,USART_IT_TXE);
		USART_ITConfig(USARTx,USART_IT_TXE,DISABLE);

	}	
	 
	//-------�����ж�----------------------------
	if(USART_GetITStatus(USARTx, USART_IT_TC) == SET){  //������� �ж�

		USART_ClearITPendingBit(USARTx,USART_IT_TC);
		USART_MBUS_ITTX();

	 }


}





//======================================================
//MODBUS �����ж��ӳ���
void USART_MBUS_ITRX(u8 Res){//���ڵ��жϽ���

		USART_MBUS_RX_TS = Millis;

		if(!(USART_MBUS_RXSTA & 0x40)){		//���ܵ��ĵ�1������
		   USART_MBUS_RXSTA |=0x40	;  		//�����Ѿ���ʼ���ܵı�־
		   USART_MBUS_RXBUF[0] = Res;			//���ܵ��ĵ�1������
		   USART_MBUS_RXNUM = 1;
		}
		else if(USART_MBUS_RXNUM < MBUS_MAXBUF_LEN){ 		//���ܻ�����û�����
		        USART_MBUS_RXBUF[USART_MBUS_RXNUM] = Res;	//���ܵ�������
		        USART_MBUS_RXNUM++;							//���ܳ���+1
	      	 }
		     else  USART_MBUS_RXSTA |=0x20; 				//bit5 ���ܻ����������־


}


//======================================================
//MODBUS���� ��������жϣ�������ɷ���1
u8 USART_MBUS_Recok(vu32 ts){  //����1��������ж�,ts�Ǻ��뵥λ

u8 retv=0;
     if((USART_MBUS_RXSTA&0xC0)==0X40){//���ܵ��ĵ�1������

	  	if((Millis - USART_MBUS_RX_TS)>= ts){
		 	USART_MBUS_RXSTA |=0x80;
		 	USART_MBUS_RX_TS = Millis;
		 	retv = 1; 
	  	}
	 }

	 
    return retv; //1-������ɷ���  ��0-û�����

}

//======================================================
//MODBUS���� ��ʼ����
void USART_MBUS_Txstart(u8 txnum){

  if(!(USART_MBUS_TXSTA&0x40)){ //û�з�������

	 //�ڷ��͵�ʱ��رս����ж�
	 USART_ITConfig(MBUS_USARTn, USART_IT_RXNE, DISABLE);//����ENABLE/�ر�DISABLE�ж�
	 USART_ReceiveData(MBUS_USARTn);

     USART_MBUS_SEND_NUM = 0;
	 USART_MBUS_TXSUM = txnum;
	 USART_MBUS_TXSTA =0x40;

	 USART_SendData(MBUS_USARTn ,USART_MBUS_BUF[0]);//���͵�1���ֽ�
     USART_ITConfig(MBUS_USARTn, USART_IT_TC, ENABLE);// ����ENABLE	�����ж�

  }

}

//======================================================
//MODBUS���ڷ����ж��ӳ���
void USART_MBUS_ITTX(void){//���ڵ��жϽ���

	  if(USART_MBUS_TXSTA & 0x40){ //�з�������

	    USART_MBUS_SEND_NUM++;   //�Ѿ����͵ĳ���
	    if(USART_MBUS_SEND_NUM < USART_MBUS_TXSUM){  ////û�з�����ɣ���������
  	       USART_SendData(MBUS_USARTn, USART_MBUS_BUF[USART_MBUS_SEND_NUM]);
		}
		else {
	  	      USART_MBUS_TXSTA |=0x80;//���ͽ�����־
		      USART_ITConfig(MBUS_USARTn, USART_IT_TC, DISABLE);//�ط����ж�

			  USART_ClearITPendingBit(MBUS_USARTn,USART_IT_RXNE); // �����־
			  USART_ReceiveData(MBUS_USARTn);//���ܵ�USARTx->DR������
  	          USART_ITConfig(MBUS_USARTn, USART_IT_RXNE, ENABLE);//����ENABLE�ж�

		     }
	   } else USART_ITConfig(MBUS_USARTn, USART_IT_TC, DISABLE);//�ط����ж�


}



//================================================================
// USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
//�������� �շ�ģʽ
void USARTx_MODE(USART_TypeDef* USARTx, uint32_t USART_Mode)
{
  uint32_t tmpreg = 0x00;

/*---------------------------- USART CR1 Configuration -----------------------*/
  tmpreg = USARTx->CR1;

  tmpreg &= 0xfffffff3;
  tmpreg |= USART_Mode;
  USARTx->CR1 = (uint16_t)tmpreg;
}









