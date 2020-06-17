#ifndef __USART_H
#define __USART_H

#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include "stdio.h"	
#include "sys.h" 
#include "data_receive.h"

//-------------------------------------------------------------------------------

#define MODBUS_EN		1  //MODBUS  功能开关，0-关，1-开

#define MBUS_USARTn		USART1  //MODBUS 端口

#define MBUS_MAXBUF_LEN  	256  	//定义最大接收字节数

extern u8  USART_MBUS_BUF[MBUS_MAXBUF_LEN]; //接收缓冲,最大USART_REC_LEN个字节.
extern u8  USART_MBUS_RXBUF[MBUS_MAXBUF_LEN]; //接收缓冲,最大USART_REC_LEN个字节.


extern u8 USART_MBUS_RXNUM;         		//接收状态标记	
extern u8 USART_MBUS_TXSUM;         		//接收状态标记	

extern u8 USART_MBUS_RXSTA;         		//接受状态标记	
extern u8 USART_MBUS_TXSTA;         		//发送状态标记



void USART_MBUS_ITRX(u8 Res);			//串口的中断接受
void USART_MBUS_ITTX(void);				//串口的中断接受

u8 USART_MBUS_Recok(vu32 ts);			//串口接受完成判断
void USART_MBUS_Txstart(u8 txnum);			//串口开始发送

void USART_MBUS_Init(u32 bound);
void MBUS_IRQHandler(USART_TypeDef* USARTx);

//-------------------------------------------------------------------------------
#define EN_CXX_USART1  1

#define USART_CXX  USART3  			//定义皮带保护模块，查询线的串口


#define CXX_MAXRX_LEN  			60  	//定义USART1最大接收字节数
#define CXX_MAXTX_LEN  			60  	//定义USART1最大发送字节数
	  	
extern u8  USART_CXX_RX_BUF[CXX_MAXRX_LEN]; //接收缓冲,最大USART_REC_LEN个字节.
extern u8  USART_CXX_TX_BUF[CXX_MAXTX_LEN]; //接收缓冲,最大USART_REC_LEN个字节.

extern u8 USART_CXX_RXNUM;         		//接收状态标记	
extern u8 USART_CXX_TXSUM;         		//接收状态标记	

extern u8 USART_CXX_RXSTA;         		//接受状态标记	
extern u8 USART_CXX_TXSTA;         		//发送状态标记

void USART_CXX_ITRX(u8 Res);			//串口的中断接受
void USART_CXX_ITTX(void);				//串口的中断接受

u8 USART_CXX_Recok(vu16 ts);			//串口1接受完成判断
void USART_CXX_Txstart(u8 txnum);			//串口1开始发送

void USART_CXX_Init(u32 bound);
void CXX_IRQHandler(USART_TypeDef* USARTx);

//---------------------------------------------------------------------------------
#define EN_USART1 			1	

#define USART1_MAXRX_LEN  			100  	//定义USART1最大接收字节数
#define USART1_MAXTX_LEN  			100  	//定义USART1最大发送字节数
	  	


void USART1_Init(u32 bound);//初始化并启动


//--------------------------------------------------------------------------
#define EN_USART2 			1		

#define USART2_REC_LEN  			10  	//定义USART2最大接收字节数
extern u8  USART2_RX_BUF[USART2_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.
extern u16 USART2_RX_STA;         		//接收状态标记

void USART2_Init(u32 bound);//初始化并启动	

//--------------------------------------------------------------------------
#define EN_USART3 			1	

#define USART3_REC_LEN  			10  	//定义USART3最大接收字节数
extern u8  USART3_RX_BUF[USART3_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.
extern u16 USART3_RX_STA;         		//接收状态标记

void USART3_Init(u32 bound);//初始化并启动
void USART2_Init(u32 bound);//串口2初始化并启动



void USARTx_MODE(USART_TypeDef* USARTx, uint32_t USART_Mode);



#endif


