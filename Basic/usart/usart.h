#ifndef __USART_H
#define __USART_H

#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include "stdio.h"	
#include "sys.h" 
#include "data_receive.h"

//-------------------------------------------------------------------------------

#define MODBUS_EN		1  //MODBUS  ���ܿ��أ�0-�أ�1-��

#define MBUS_USARTn		USART1  //MODBUS �˿�

#define MBUS_MAXBUF_LEN  	256  	//�����������ֽ���

extern u8  USART_MBUS_BUF[MBUS_MAXBUF_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.
extern u8  USART_MBUS_RXBUF[MBUS_MAXBUF_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.


extern u8 USART_MBUS_RXNUM;         		//����״̬���	
extern u8 USART_MBUS_TXSUM;         		//����״̬���	

extern u8 USART_MBUS_RXSTA;         		//����״̬���	
extern u8 USART_MBUS_TXSTA;         		//����״̬���



void USART_MBUS_ITRX(u8 Res);			//���ڵ��жϽ���
void USART_MBUS_ITTX(void);				//���ڵ��жϽ���

u8 USART_MBUS_Recok(vu32 ts);			//���ڽ�������ж�
void USART_MBUS_Txstart(u8 txnum);			//���ڿ�ʼ����

void USART_MBUS_Init(u32 bound);
void MBUS_IRQHandler(USART_TypeDef* USARTx);

//-------------------------------------------------------------------------------
#define EN_CXX_USART1  1

#define USART_CXX  USART3  			//����Ƥ������ģ�飬��ѯ�ߵĴ���


#define CXX_MAXRX_LEN  			60  	//����USART1�������ֽ���
#define CXX_MAXTX_LEN  			60  	//����USART1������ֽ���
	  	
extern u8  USART_CXX_RX_BUF[CXX_MAXRX_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.
extern u8  USART_CXX_TX_BUF[CXX_MAXTX_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.

extern u8 USART_CXX_RXNUM;         		//����״̬���	
extern u8 USART_CXX_TXSUM;         		//����״̬���	

extern u8 USART_CXX_RXSTA;         		//����״̬���	
extern u8 USART_CXX_TXSTA;         		//����״̬���

void USART_CXX_ITRX(u8 Res);			//���ڵ��жϽ���
void USART_CXX_ITTX(void);				//���ڵ��жϽ���

u8 USART_CXX_Recok(vu16 ts);			//����1��������ж�
void USART_CXX_Txstart(u8 txnum);			//����1��ʼ����

void USART_CXX_Init(u32 bound);
void CXX_IRQHandler(USART_TypeDef* USARTx);

//---------------------------------------------------------------------------------
#define EN_USART1 			1	

#define USART1_MAXRX_LEN  			100  	//����USART1�������ֽ���
#define USART1_MAXTX_LEN  			100  	//����USART1������ֽ���
	  	


void USART1_Init(u32 bound);//��ʼ��������


//--------------------------------------------------------------------------
#define EN_USART2 			1		

#define USART2_REC_LEN  			10  	//����USART2�������ֽ���
extern u8  USART2_RX_BUF[USART2_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.
extern u16 USART2_RX_STA;         		//����״̬���

void USART2_Init(u32 bound);//��ʼ��������	

//--------------------------------------------------------------------------
#define EN_USART3 			1	

#define USART3_REC_LEN  			10  	//����USART3�������ֽ���
extern u8  USART3_RX_BUF[USART3_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.
extern u16 USART3_RX_STA;         		//����״̬���

void USART3_Init(u32 bound);//��ʼ��������
void USART2_Init(u32 bound);//����2��ʼ��������



void USARTx_MODE(USART_TypeDef* USARTx, uint32_t USART_Mode);



#endif


