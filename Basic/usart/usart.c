
/*
《修改日志》
1-201708271933 加入了秒延时函数。


*/
#include "stm32f10x_usart.h"

#include "sys.h"
#include "usart.h"
#include "pscxx.h"
#include "modbus.h" 	 
#include "led.h"
#include "tim.h"

extern vu32 Millis;//系统毫秒计时器  T3中断内部计数

//-----------------------------------------------------------


u8  USART_MBUS_BUF[MBUS_MAXBUF_LEN]; //接收缓冲,最大USART_REC_LEN个字节.
u8  USART_MBUS_RXBUF[MBUS_MAXBUF_LEN]; //接收缓冲,最大USART_REC_LEN个字节.


u8 USART_MBUS_RXNUM=0;         	 	//已经接收的数据
vu32 USART_MBUS_RX_TS=0;         	//接收数据的时间点
	
u8 USART_MBUS_TXSUM=0;         		//需要发送的长度	
u8 USART_MBUS_SEND_NUM=0;         	//已经发送的长度

u8 USART_MBUS_RXSTA=0;         		//接受状态标记 //bit7，接收完成标志  //bit6，开始接收   //bit5，接收缓冲区溢出
u8 USART_MBUS_TXSTA=0;         		//发送状态标记 //bit7，发送完成 //bit6，有数据要发送

//-----------------------------------------------------------

u8  USART_CXX_RX_BUF[CXX_MAXRX_LEN]; //接收缓冲,最大USART_REC_LEN个字节.
u8  USART_CXX_TX_BUF[CXX_MAXTX_LEN]; //接收缓冲,最大USART_REC_LEN个字节.

u8 USART_CXX_RXNUM=0;         	 	//已经接收的数据

vu16 USART_CXX_RX_TS=0;         	//接收数据的时间点
vu16 CXX_RXOVER_TS;
	
u8 USART_CXX_TXSUM=0;         		//需要发送的长度	
u8 USART_CXX_SEND_NUM=0;         	//已经发送的长度

u8 USART_CXX_RXSTA=0;         		//接受状态标记 //bit7，接收完成标志  //bit6，开始接收   //bit5，接收缓冲区溢出
u8 USART_CXX_TXSTA=0;         		//发送状态标记 //bit7，发送完成 //bit6，有数据要发送

//-----------------------------------------------------------
void USART_CXX_Init(u32 bound){ //串口1初始化并启动

//	USART1_Init(bound);
//	USART2_Init(bound);
	USART3_Init(bound);
}

//-----------------------------------------------------------
void USART_MBUS_Init(u32 bound){ //串口1初始化并启动

	USART1_Init(bound);
//	USART2_Init(bound);
//	USART3_Init(bound);
}
//-----------------------------------------------------------
/*
USART1串口相关程序
*/

#if EN_USART1   //USART1使用与屏蔽选择

u8  USART1_RX_BUF[USART1_MAXRX_LEN]; //接收缓冲,最大USART_REC_LEN个字节.
u8  USART1_TX_BUF[USART1_MAXTX_LEN]; //接收缓冲,最大USART_REC_LEN个字节.

u16 USART1_RXNUM=0;         		//已经接收的数据
vu32 USART1_RX_TS=0;         		//接收数据的时间点
	
u16 USART1_TXNUM=0;         		//需要发送的长度	
u16 USART1_SEND_NUM=0;         	//已经发送的长度

u8 USART1_RXSTA=0;         		//接受状态标记


u8 USART1_TXSTA=0;         		//发送状态标记

void USART1_Init(u32 bound){ //串口1初始化并启动
    //GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
		 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//使能USART1，GPIOA时钟


     //USART1_TX   PA.9
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
    GPIO_Init(GPIOA, &GPIO_InitStructure);  
    //USART1_RX	  PA.10
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
    GPIO_Init(GPIOA, &GPIO_InitStructure); 

   //Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器 

   //USART 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//一般设置为9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
    USART_Init(USART1, &USART_InitStructure); //初始化串口


    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//接受中断，开启ENABLE/关闭DISABLE中断
  //  USART_ITConfig(USART1, USART_IT_TXE, ENABLE);//发送中断，开启ENABLE/关闭DISABLE中断

    USART_Cmd(USART1, ENABLE);                    //使能串口 
}

//-------------------------------------------
void USART1_IRQHandler(void){ //串口1中断服务程序（固定的函数名不能修改）	

	static u8 R_temp;
	
	//	MBUS_IRQHandler(USART1);
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断
		{
			R_temp =USART_ReceiveData(USART1);	//读取接收到的数据
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
USART2串口相关程序
*/
#if EN_USART2   //USART2使用与屏蔽选择


 //===========================================================
void USART2_Init(u32 bound){ //串口2初始化并启动
    //GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
		 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE); //使能UART2所在GPIOA的时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); //使能串口的RCC时钟

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; //设置USART2的RX接口是PA3
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
    GPIO_Init(GPIOA, &GPIO_InitStructure); 

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //设置USART2的TX接口是PA2
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
    GPIO_Init(GPIOA, &GPIO_InitStructure);  

   //USART2 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//一般设置为9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
    USART_Init(USART2, &USART_InitStructure); //初始化串口

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启ENABLE/关闭DISABLE中断
    USART_Cmd(USART2, ENABLE);                    //使能串口 

   //Usart2 NVIC 配置

    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		//优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器 
}


//=========================================================================
void USART2_IRQHandler(void){ //串口2中断服务程序（固定的函数名不能修改）	

	static u8 R_temp;
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //接收中断
		{
			R_temp =USART_ReceiveData(USART2);	//读取接收到的数据
			directional_data_receive(R_temp);
			
     } 
	
}
 
#endif	



//===========================================================
#if EN_USART3   //如果使能了接收
u8 USART3_RX_BUF[USART3_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART3_RX_STA=0;       //接收状态标记	  

/*
USART3专用的printf函数
当同时开启2个以上串口时，printf函数只能用于其中之一，其他串口要自创独立的printf函数
调用方法：USART3_printf("123"); //向USART3发送字符123
*/

//串口3初始化


void USART3_Init(u32 BaudRate){ //USART3初始化并启动
   GPIO_InitTypeDef GPIO_InitStructure;
   USART_InitTypeDef USART_InitStructure;
   NVIC_InitTypeDef NVIC_InitStructure; 

   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE); //使能UART3所在GPIOB的时钟
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); //使能串口的RCC时钟

   //串口使用的GPIO口配置
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//设置USART3的RX接口是PB11
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//接口模式 浮空输入
   GPIO_Init(GPIOB, &GPIO_InitStructure);

   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//设置USART3的TX接口是PB10
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//输出速度50MHz
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//接口模式 复用推挽输出
   GPIO_Init(GPIOB, &GPIO_InitStructure);

   //配置串口
   USART_InitStructure.USART_BaudRate = BaudRate;
   USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
   USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
   USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
   USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制


   USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

 //  USART_InitStructure.USART_Mode =  USART_Mode_Tx;	//只发送 模式

   USART_Init(USART3, &USART_InitStructure);//配置串口3

   USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//使能串口接收中断  
   USART_ITConfig(USART3, USART_IT_TXE, DISABLE);//串口发送中断在发送数据时开启

   USART_Cmd(USART3, ENABLE);//使能串口3


   //串口中断配置
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; 	 //抢先等级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;			 //响应等级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器 
}




//==================================================
//串口3中断服务程序（固定的函数名不能修改）
void USART3_IRQHandler(void){ 	

//	CXX_IRQHandler(USART3);
		static u8 R_temp;
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //接收中断
		{
			R_temp =USART_ReceiveData(USART3);	//读取接收到的数据
			gamma_data_receive(R_temp);		
     } 
		USART_ClearITPendingBit(USART3,USART_IT_RXNE);
}
#endif	


//======================================================
void CXX_IRQHandler(USART_TypeDef* USARTx){ //查询线 串口中断服务程序
	u8 Res;

	if(USART_GetFlagStatus(USARTx,USART_FLAG_ORE)==SET)	//溢出错误标志位
	{
		USART_ClearFlag(USARTx,USART_FLAG_ORE); 
		USART_ReceiveData(USARTx);    		 		//读取DR
	}


	//----------接受中断处理-------------------
	if(USART_GetITStatus(USARTx, USART_IT_RXNE) == SET){  //接收中断
			
		Res =USART_ReceiveData(USARTx);//(USART1->DR);	//读取接收到的数据

        USART_CXX_ITRX(Res);

	} 

	//-------发送中断----------------------------
	if(USART_GetITStatus(USARTx, USART_IT_TXE)==SET)	//发送数据寄存器空标志位
	{
		USART_ClearITPendingBit(USARTx,USART_IT_TXE);
		USART_ITConfig(USARTx,USART_IT_TXE,DISABLE);

	}	

	//-------发送中断----------------------------
	if(USART_GetITStatus(USARTx, USART_IT_TC) == SET){  //发送中断

		USART_CXX_ITTX();

//		USART_ITConfig(USART1, USART_IT_TC, DISABLE);//发送中断，关闭DISABLE中断


	}   
	//-----------------------------------


} 

//======================================================
//查询线串口接受中断子程序
void USART_CXX_ITRX(u8 Res){//串口的中断接受

		USART_CXX_RX_TS = 0;
		CXX_RXOVER_TS = 0;


		if(!(USART_CXX_RXSTA&0x40)){//接受到的第1个数据
		   USART_CXX_RXSTA =0x40	;  //设置已经开始接受的标志
		   USART_CXX_RX_BUF[0] = Res;	//接受到的第1个数据
		   USART_CXX_RXNUM = 1;
		}
		else if(USART_CXX_RXNUM<CXX_MAXRX_LEN){ 			//接受缓冲区没有溢出
		        USART_CXX_RX_BUF[USART_CXX_RXNUM] = Res;	//接受到缓冲区
		        USART_CXX_RXNUM++;							//接受长度+1
	      	 }
		     else  USART_CXX_RXSTA |=0x20; //bit5 接受缓冲区溢出标志


}


//======================================================
//查询线串口 接受完成判断，接受完成返回1
u8 USART_CXX_Recok(vu16 ts){  //串口1接受完成判断,ts是毫秒单位
u8 ret=0;

     if(USART_CXX_RXSTA&0x40){//接受到的第1个数据

	  	if( USART_CXX_RX_TS>= ts)	 {
			USART_CXX_RXSTA |=0x80;
		 	ret = 1; 
	  	} 
	 }
   return ret;
}


//======================================================
//查询线串口 开始发送
void USART_CXX_Txstart(u8 txnum){

	if(!(USART_CXX_TXSTA&0x40)){ //没有发送任务

	 //在发送的时候关闭接受中断
		USARTx_MODE(USART_CXX,USART_Mode_Tx); 				//关闭接受功能
		USART_ITConfig(USART_CXX, USART_IT_RXNE, DISABLE);	//关闭DISABLE中断
		USART_ReceiveData(USART_CXX);						//接受掉USARTx->DR的数据


		USART_SendData(USART_CXX ,USART_CXX_TX_BUF[0]);//发送第1个字节
		USART_CXX_TXSTA =0x40;
		USART_CXX_SEND_NUM = 0;
		USART_CXX_TXSUM = txnum;
		USART_ITConfig(USART_CXX, USART_IT_TC, ENABLE);// 开启ENABLE	发送中断

	}	

}


//======================================================
//查询线串口发送中断子程序
void USART_CXX_ITTX(void){//串口的中断接受
u8 tx_end=0;

	  if(USART_CXX_TXSTA&0x40){ //有发送任务

	    	USART_CXX_SEND_NUM++;   //已经发送的长度
	    	if(USART_CXX_SEND_NUM<USART_CXX_TXSUM){  ////没有发送完成，继续发送
  	       		USART_SendData(USART_CXX, USART_CXX_TX_BUF[USART_CXX_SEND_NUM]);
			}
			else 
			{
	  	      	USART_CXX_TXSTA |=0x80;//发送结束标志
				tx_end = 1 ;
		     }

	   } 
	   else tx_end = 1 ;

	//------------------------------
	//结束发送	   
	 if(tx_end){

			USART_ITConfig(USART_CXX, USART_IT_TC, DISABLE);		//关发送中断

			//USARTx_MODE(USART_CXX,USART_Mode_Rx|USART_Mode_Tx); 	// 打开接受功能
			//USART_ReceiveData(USART_CXX);							//接受掉USARTx->DR的数据
  	        //USART_ITConfig(USART_CXX, USART_IT_RXNE, ENABLE);		//开启ENABLE
	 }

}


//==============================================================
void MBUS_IRQHandler(USART_TypeDef* USARTx){ 	

u8 Res;

	if(USART_GetFlagStatus(USARTx,USART_FLAG_ORE)==SET)	//溢出错误标志位
	{
		USART_ClearFlag(USARTx,USART_FLAG_ORE); 
		USART_ReceiveData(USARTx);    		 		//读取DR
	}

	//----------接受中断处理-------------------
	if(USART_GetITStatus(USARTx, USART_IT_RXNE) == SET){  //接收中断
			
		USART_ClearITPendingBit(USARTx,USART_IT_RXNE);
		Res =USART_ReceiveData(USARTx);					//(USARTx->DR);	//读取接收到的数据

        USART_MBUS_ITRX(Res);

	}

	//-------发送中断----------------------------
	if(USART_GetITStatus(USARTx, USART_IT_TXE)==SET)	//发送数据寄存器空标志位
	{
		USART_ClearITPendingBit(USARTx,USART_IT_TXE);
		USART_ITConfig(USARTx,USART_IT_TXE,DISABLE);

	}	
	 
	//-------发送中断----------------------------
	if(USART_GetITStatus(USARTx, USART_IT_TC) == SET){  //发送完成 中断

		USART_ClearITPendingBit(USARTx,USART_IT_TC);
		USART_MBUS_ITTX();

	 }


}





//======================================================
//MODBUS 接受中断子程序
void USART_MBUS_ITRX(u8 Res){//串口的中断接受

		USART_MBUS_RX_TS = Millis;

		if(!(USART_MBUS_RXSTA & 0x40)){		//接受到的第1个数据
		   USART_MBUS_RXSTA |=0x40	;  		//设置已经开始接受的标志
		   USART_MBUS_RXBUF[0] = Res;			//接受到的第1个数据
		   USART_MBUS_RXNUM = 1;
		}
		else if(USART_MBUS_RXNUM < MBUS_MAXBUF_LEN){ 		//接受缓冲区没有溢出
		        USART_MBUS_RXBUF[USART_MBUS_RXNUM] = Res;	//接受到缓冲区
		        USART_MBUS_RXNUM++;							//接受长度+1
	      	 }
		     else  USART_MBUS_RXSTA |=0x20; 				//bit5 接受缓冲区溢出标志


}


//======================================================
//MODBUS串口 接受完成判断，接受完成返回1
u8 USART_MBUS_Recok(vu32 ts){  //串口1接受完成判断,ts是毫秒单位

u8 retv=0;
     if((USART_MBUS_RXSTA&0xC0)==0X40){//接受到的第1个数据

	  	if((Millis - USART_MBUS_RX_TS)>= ts){
		 	USART_MBUS_RXSTA |=0x80;
		 	USART_MBUS_RX_TS = Millis;
		 	retv = 1; 
	  	}
	 }

	 
    return retv; //1-接受完成返回  ，0-没有完成

}

//======================================================
//MODBUS串口 开始发送
void USART_MBUS_Txstart(u8 txnum){

  if(!(USART_MBUS_TXSTA&0x40)){ //没有发送任务

	 //在发送的时候关闭接受中断
	 USART_ITConfig(MBUS_USARTn, USART_IT_RXNE, DISABLE);//开启ENABLE/关闭DISABLE中断
	 USART_ReceiveData(MBUS_USARTn);

     USART_MBUS_SEND_NUM = 0;
	 USART_MBUS_TXSUM = txnum;
	 USART_MBUS_TXSTA =0x40;

	 USART_SendData(MBUS_USARTn ,USART_MBUS_BUF[0]);//发送第1个字节
     USART_ITConfig(MBUS_USARTn, USART_IT_TC, ENABLE);// 开启ENABLE	发送中断

  }

}

//======================================================
//MODBUS串口发送中断子程序
void USART_MBUS_ITTX(void){//串口的中断接受

	  if(USART_MBUS_TXSTA & 0x40){ //有发送任务

	    USART_MBUS_SEND_NUM++;   //已经发送的长度
	    if(USART_MBUS_SEND_NUM < USART_MBUS_TXSUM){  ////没有发送完成，继续发送
  	       USART_SendData(MBUS_USARTn, USART_MBUS_BUF[USART_MBUS_SEND_NUM]);
		}
		else {
	  	      USART_MBUS_TXSTA |=0x80;//发送结束标志
		      USART_ITConfig(MBUS_USARTn, USART_IT_TC, DISABLE);//关发送中断

			  USART_ClearITPendingBit(MBUS_USARTn,USART_IT_RXNE); // 清除标志
			  USART_ReceiveData(MBUS_USARTn);//接受掉USARTx->DR的数据
  	          USART_ITConfig(MBUS_USARTn, USART_IT_RXNE, ENABLE);//开启ENABLE中断

		     }
	   } else USART_ITConfig(MBUS_USARTn, USART_IT_TC, DISABLE);//关发送中断


}



//================================================================
// USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
//单独设置 收发模式
void USARTx_MODE(USART_TypeDef* USARTx, uint32_t USART_Mode)
{
  uint32_t tmpreg = 0x00;

/*---------------------------- USART CR1 Configuration -----------------------*/
  tmpreg = USARTx->CR1;

  tmpreg &= 0xfffffff3;
  tmpreg |= USART_Mode;
  USARTx->CR1 = (uint16_t)tmpreg;
}









