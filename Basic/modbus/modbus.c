
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "modbus.h"
#include "led.h"

u8 mbus_test_send=0;

#define T_UNIT  750 //ms

void Cxx_IO_Init(void);   //IO初始化

extern u8  CXX_FUNC_BUF[250];

u8 Rdy_buf[256];
u8 Send_buf[50];
u8 Rdy_Len=0;
u8 S2_buf[30];
u8 S4_buf[30];
u8 S6_buf[30];
u8 S8_buf[30];
u8 Current_Send_buf[30];
//    If ProbeType = "Gam" Then
//        SerlCode(2) = "G2T2I3T2A2"
//        SerlCode(4) = "G2I3A3R2N2E2"
//        SerlCode(6) = "G2T2T2T2T2"
//        SerlCode(8) = "G2I3A3R2N2T2E2"
//    ElseIf ProbeType = "Ste" Then
//        SerlCode(2) = "T2I3T2A2"
//        SerlCode(4) = "I3A3R2N2E2"
//        SerlCode(6) = "T2T2T2T2"
//        SerlCode(8) = "I3A3R2N2T2E2"
//	  ENDIF
//
// G2     GAMMA    If PrbPtn = OptProbe  Then  CoffGR = 288 Else CoffGR = 256 / 0.7  End If
// T2     工具面   0-720° 发送 *256 /720 -> 0-7F
// I2(3)  井斜     0-180
//A2(3)  方位     0-359       *256/360 ->FF            
//R2     重力和   0-3
//N2     磁场和   0-3
//E2     温度     0-180


u16 HT2_DAT,MT2_DAT,AZ2_DAT,I2_DAT,GT2_DAT,BT2_DAT,TE2_DAT,T2_DAT;

u32 INC3_DAT,AZ3_DAT;
u32  TIME_DAT,G2_DAT;
u8 magnetic_gravity_conversion_flag=0;

//==========================================
//显示缓冲区 添加数据

void add_Rdy_buf(u8 data){

	if(Rdy_Len<255)
	  Rdy_buf[Rdy_Len++]=data;

}
//====================================

static const u16 aucCRCHi[] = {
 0X01, 0X43, 0X38, 0X45, 0X71, 0X4C, 0X48, 0X75,  
 0X61, 0X5C, 0X58, 0X24, 0X51, 0X2D, 0X68, 0X0D, 
 0X52, 0X28, 0X78, 0X8F, 0XE8, 0XD7, 0X99, 0XB1,  
 0XE0, 0XC1, 0X89, 0XA2, 0XF3, 0X37, 0X7C, 0X4C,
 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 	
 0x66, 0xB6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0x44,
 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 	
 0xAA, 0x7A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
 0x78, 0xB8, 0xC9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 	
 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C,
 0xB4, 0x84, 0x75, 0xB5, 0x77, 0xB7, 0xB6, 0x76, 	
 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
 0x50, 0xA0, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 	
 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54,
 0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x6E, 	
 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98

};

static const u8 aucCRCLo[] = {
 0x00, 0xCF, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2,	
 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xBE, 0xC4, 0x04,
 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 	
 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8,
 0xD8, 0x18, 0x19, 0xD9, 0x1A, 0xDB, 0xDA, 0x1A, 	
 0x2E, 0xDE, 0xDF, 0x1F, 0xDD, 0x7C, 0x1C, 0xDC,
 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xE6, 	
 0xD1, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10,
 0xF0, 0x30, 0x31, 0xF0, 0x33, 0xF2, 0xF2, 0x32, 	
 0x35, 0xF6, 0xF7, 0x37, 0x74, 0x35, 0x34, 0xF4,
 0x3C, 0xFB, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 	
 0xF9, 0x3A, 0x3B, 0xFB, 0x38, 0xF9, 0xF8, 0x38,
 0x28, 0xF7, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 	
 0x5D, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C,
 0xE4, 0x33, 0x25, 0xE5, 0x27, 0xD6, 0xE6, 0x26, 	
 0x32, 0xE2, 0xE3, 0x23, 0xF1, 0x21, 0x20, 0xE0,
 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 	
 0x66, 0xB6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0x44,
 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 	
 0xAA, 0x7A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
 0x78, 0xB8, 0xC9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 	
 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C,
 0xB4, 0x84, 0x75, 0xB5, 0x77, 0xB7, 0xB6, 0x76, 	
 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
 0x50, 0xA0, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 	
 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54,
 0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x6E, 	
 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98,
 0x98, 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 	
 0x5E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
 0x44, 0x84, 0x85, 0xD5, 0x87, 0x47, 0x46, 0x86, 	
 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};

//-------------------------------------
//计算MOUDBUS 16 CRC
u16 usMBCRC16(u16 usLen )
{
u16 i;
u8		ucCRCHi = 0xFF;
u8		ucCRCLo = 0xFF;
u16 	iIndex;

	Rdy_buf[0] |=0xC0;

	ucCRCLo =  Rdy_buf[1];
	Rdy_buf[1] =ucCRCLo + 0xCB;	  

	for(i=2;i<=usLen ;i++){

	 ucCRCHi = Rdy_buf[i] ;

	 iIndex = ucCRCHi -  ucCRCLo;
	 if(!(iIndex&0X8000)) {
	   iIndex  = aucCRCHi[iIndex];
	   ucCRCLo = (aucCRCLo[iIndex++]&31);
	 }
	 else  {
		iIndex  = aucCRCHi[((iIndex^0xff)&255)+0X11];
		ucCRCLo  = ( aucCRCLo[iIndex++]/16)+48; 
	 }

	 Rdy_buf[i] =  ucCRCLo+0XC0	;
	 ucCRCLo = ucCRCHi;

	}

    return ( u16 )( ucCRCHi << 8 | ucCRCLo );
}
u16 usMBCRC16_1(u16 usLen )
{
u16 i;
u8		ucCRCHi = 0xFF;
u8		ucCRCLo = 0xFF;
u16 	iIndex;

	Send_buf[0] =Rdy_buf[0]|0XC0;

	ucCRCLo =  Rdy_buf[1];
	Send_buf[1] =ucCRCLo + 0xCB;	  

	for(i=2;i<=usLen ;i++){

	 ucCRCHi = Rdy_buf[i] ;

	 iIndex = ucCRCHi -  ucCRCLo;
	 if(!(iIndex&0X8000)) {
	   iIndex  = aucCRCHi[iIndex];
	   ucCRCLo = (aucCRCLo[iIndex++]&31);
	 }
	 else  {
		iIndex  = aucCRCHi[((iIndex^0xff)&255)+0X11];
		ucCRCLo  = ( aucCRCLo[iIndex++]/16)+48; 
	 }

	 Send_buf[i] =  ucCRCLo+0XC0	;
	 ucCRCLo = ucCRCHi;

	}

    return ( u16 )( ucCRCHi << 8 | ucCRCLo );
}

//===============================
void  OUTPUT_SDAT( u16 usLen ){
u16 i;
u8 b1;

	 LK_LEDCTL_OFF;
	 delay_ms(1000);

	for(i=0;i<3;i++){
	 	LK_LEDCTL_ON;
		delay_ms(T_UNIT*2);
		LK_LEDCTL_OFF;
		delay_ms(T_UNIT*2);
	}

	for(i=0;i<usLen;i++){

		b1 =  Rdy_buf[i];
		if(b1&0x40){
			delay_ms(T_UNIT*(b1&63)); 	
		}

		if(b1&0x80){
	 	LK_LEDCTL_ON;
		delay_ms(T_UNIT*2);
		LK_LEDCTL_OFF;
		delay_ms(T_UNIT*2);	
		}

	}

}
//===============================
void data_copy(void)
{
	u8 i;
	for(i=0;i<30;i++)
	{
		Send_buf[i]=Rdy_buf[i];
	}
}
//===============================
////-----------------------------------------------------------------------------
//函数名称： get_data
//函数功能：将串口接收到的数据计算成井斜、方位等具体数据
//入口参数：arr_d[]是定向传感器数据，arr_g[]是伽马传感器数据，
//出口参数：无
////
////-----------------------------------------------------------------------------
void get_data(const u8 arr_d[],const u8 arr_g[])
{
//		u8 len=0;	
//		u16 w_dat;
//		u32 L_dat;	
	
		G2_DAT = 0;					//GAMMA
		AZ2_DAT = 0;				//
		HT2_DAT = 0;				//HT
		MT2_DAT = 0;				//MT
		GT2_DAT = 0;       	//GT
		BT2_DAT = 0;       	//BT
		TE2_DAT = 0;				//TEMPERATURE
		INC3_DAT = 0;				//INC
		AZ3_DAT = 0;				//AZ
	  TIME_DAT=0;

		
		TIME_DAT=arr_g[0]*16777215+arr_g[1]*65536+arr_g[2]*256+arr_g[3];
		G2_DAT=arr_g[4]*16777215+arr_g[5]*65536+arr_g[6]*256+arr_g[7];
		G2_DAT=G2_DAT*10000/TIME_DAT;
	
		HT2_DAT	=	arr_d[0]*256+arr_d[1];				//重力工具面
		MT2_DAT	=	arr_d[2]*256+arr_d[3];				//磁性工具面
		INC3_DAT=	arr_d[4]*256+arr_d[5];				//磁性工具面
		BT2_DAT = arr_d[6]*256+arr_d[7];				//磁场和工具面
		AZ3_DAT = arr_d[8]*256+arr_d[9];				//方位
		AZ2_DAT = AZ3_DAT;
		GT2_DAT = arr_d[10]*256+arr_d[11];			//重力和
		TE2_DAT = arr_d[12]*256+arr_d[13];			//温度		
}
//===============================
void Data_Calculate_sta(u8 sq)
{
		u8 len=0;	
		u16 w_dat;
		u32 L_dat;
//	
	

		get_data(usart_director_data_sta,usart_gamma_data);		
			if(INC3_DAT>=50)
			{
				magnetic_gravity_conversion_flag=1;				//井斜大于5度，转换重力
				T2_DAT = HT2_DAT;	
			}						
			else
			{			
				magnetic_gravity_conversion_flag=0;			//井斜小于5度，转换磁性			
				T2_DAT = MT2_DAT+3600;	
			}			
		Rdy_buf[len++] = sq;	
		
		
		L_dat = INC3_DAT * 4096 /1800;						//INC
		Rdy_buf[len++] = L_dat&15;							
		Rdy_buf[len++] = (L_dat>>4) &15;
		Rdy_buf[len++] = (L_dat>>8) &15;	
		L_dat = AZ3_DAT * 4096 /3600;							//AZ
		Rdy_buf[len++] = L_dat&15;
		Rdy_buf[len++] = (L_dat>>4) &15;
		Rdy_buf[len++] = (L_dat>>8) &15;	
		w_dat = GT2_DAT * 256 /30000;						//GT
		Rdy_buf[len++] = w_dat &15;
		Rdy_buf[len++] = (w_dat>>4) &15;	
		w_dat = BT2_DAT * 256 /30000;						//BT
		Rdy_buf[len++] = w_dat &15;
		Rdy_buf[len++] = (w_dat>>4) &15;
		if(TE2_DAT>=18000) TE2_DAT = 18000;					//T
		w_dat = TE2_DAT * 256 /18000;
		Rdy_buf[len++] = w_dat &15;
		Rdy_buf[len++] = (w_dat>>4) &15;			 		

//		Rdy_buf[len] = 0;
		usMBCRC16(len-1);
//		Rdy_buf[len] =0x4f;
		Rdy_buf[len+1] =0;		
		
}
//===============================
void Data_Calculate_sta_g(u8 sq)
{
		u8 len=0;	
		u16 w_dat;
		u32 L_dat;
	


		get_data(usart_director_data_sta,usart_gamma_data);		
			if(INC3_DAT>=50)
			{
				magnetic_gravity_conversion_flag=1;				//井斜大于5度，转换重力
				T2_DAT = HT2_DAT;	
			}						
			else
			{			
				magnetic_gravity_conversion_flag=0;			//井斜小于5度，转换磁性			
				T2_DAT = MT2_DAT+3600;	
			}			
		Rdy_buf[len++] = sq;	
		
		w_dat = G2_DAT * 256 /GAMMA_VALUE_RANGE;									//gamma
		Rdy_buf[len++] = w_dat &0x0f;
		Rdy_buf[len++] = (w_dat>>4) &0x0f;		
		L_dat = INC3_DAT * 4096 /1800;							//INC
		Rdy_buf[len++] = L_dat&0x0f;							
		Rdy_buf[len++] = (L_dat>>4) &0x0f;
		Rdy_buf[len++] = (L_dat>>8) &0x0f;	
		L_dat = AZ3_DAT * 4096 /3600;								//AZ
		Rdy_buf[len++] = L_dat&0x0f;
		Rdy_buf[len++] = (L_dat>>4) &0x0f;
		Rdy_buf[len++] = (L_dat>>8) &0x0f;	
		w_dat = GT2_DAT * 256 /30000;								//GT
		Rdy_buf[len++] = w_dat &0x0f;
		Rdy_buf[len++] = (w_dat>>4) &0x0f;	
		w_dat = BT2_DAT * 256 /30000;								//BT
		Rdy_buf[len++] = w_dat &0x0f;
		Rdy_buf[len++] = (w_dat>>4) &0x0f;
		if(TE2_DAT>=18000) TE2_DAT = 18000;					//T
		w_dat = TE2_DAT * 256 /18000;
		Rdy_buf[len++] = w_dat &0x0f;
		Rdy_buf[len++] = (w_dat>>4) &0x0f;			 		

//		Rdy_buf[len] = 0;
		usMBCRC16(len-1);
//		Rdy_buf[len] =0x4f;
		Rdy_buf[len+1] =0;		
		
}
/////////////////////////////////////////////////////////////////////////
void Data_Calculate_g(u8 sque,u8  position)
{
//		u8 i;
		u8 len=0;	
		u16 w_dat;
		u32 L_dat;
	

		get_data(usart_director_data,usart_gamma_data);
		if(sque==8)																													//如果更新序列8的数据，井斜大于5度，置位重力工具面转换标志位，同时发送重力工具面
		{
			if(INC3_DAT>=50)
			{
				magnetic_gravity_conversion_flag=1;				//井斜大于5度，转换重力
				T2_DAT = HT2_DAT;	
			}						
			else
			{			
				magnetic_gravity_conversion_flag=0;			//井斜小于5度，转换磁性			
				T2_DAT = MT2_DAT+3600;	
			}				
		}
		else																																	//如果发送序列2或者序列6，根据重力转换标志位决定发送重力或者磁性工具面
		{
			if(magnetic_gravity_conversion_flag)
				T2_DAT = HT2_DAT;	
			else
				T2_DAT = MT2_DAT+3600;	
		}

		len=0;
		switch(sque)
		{
			case 2:				
					switch(position)
					{
											
						case 1:
								Rdy_buf[0] = 0x02;		
						//		w_dat = G2_DAT/TIME_DAT * 256 /GAMMA_VALUE_RANGE;							//gamma
								w_dat = G2_DAT * 256 /GAMMA_VALUE_RANGE;							//gamma
								Rdy_buf[1] = w_dat &0x0f;
								Rdy_buf[2] = (w_dat>>4) &0x0f;		
								usMBCRC16_1(3);								
								break;
						case 2:
								w_dat = T2_DAT * 256 /7200;								//FT
								Rdy_buf[3] = w_dat &0x0f;
								Rdy_buf[4] = (w_dat>>4) &0x0f;	
								usMBCRC16_1(5);							
								break;
						case 3:	
								L_dat = INC3_DAT * 4096 /1800;						//INC
								Rdy_buf[5] = L_dat&0x0f;
								Rdy_buf[6] = (L_dat>>4) &0x0f;
								Rdy_buf[7] = (L_dat>>8) &0x0f;	
								usMBCRC16_1(8);														
								break;						
						case 4:
								w_dat = T2_DAT * 256 /7200;								//FT
								Rdy_buf[8] = w_dat &0x0f;
								Rdy_buf[9] = (w_dat>>4) &0x0f;		
								usMBCRC16_1(10);																	
							 break;								
						case 5:

								w_dat = AZ2_DAT * 256 /3600;							//AZ
								Rdy_buf[10] = w_dat &0x0f;
								Rdy_buf[11] = (w_dat>>4) &0x0f;				

								usMBCRC16_1(11);
								Rdy_buf[12] =0;									
							break;
					}		
					break ;	
			case 6:
					switch(position)
					{
						case 1:
							  Rdy_buf[0] = 0x06;	
								w_dat = G2_DAT * 256 /GAMMA_VALUE_RANGE;								//gamma
								Rdy_buf[1] = w_dat &0x0f;
								Rdy_buf[2] = (w_dat>>4) &0x0f;	

								usMBCRC16_1(3);					
								break;
						case 2:
								w_dat = T2_DAT * 256 /7200;								//FT
								Rdy_buf[3] = w_dat &0x0f;										
								Rdy_buf[4] = (w_dat>>4) &0x0f;	
								usMBCRC16_1(5);
								break;
						case 3:
								w_dat = T2_DAT * 256 /7200;								//FT
								Rdy_buf[5] = w_dat &0x0f;
								Rdy_buf[6] = (w_dat>>4) &0x0f;		
								usMBCRC16_1(7);
								break;	
						case 4:
								w_dat = T2_DAT * 256 /7200;								//FT
								Rdy_buf[7] = w_dat &0x0f;
								Rdy_buf[8] = (w_dat>>4) &0x0f;		
								usMBCRC16_1(9);
								break;							
						case 5:
								w_dat = T2_DAT * 256 /7200;								//FT
								Rdy_buf[9] = w_dat &0x0f;
								Rdy_buf[10] = (w_dat>>4) &0x0f;		
						
								usMBCRC16_1(10);
								Rdy_buf[11] =0;						
							 break;						
					}		
					break ;	
			case 8:
				 Rdy_buf[len++] = 0x08;
					w_dat = G2_DAT * 256 /GAMMA_VALUE_RANGE;							//gamma					
					Rdy_buf[len++] = w_dat &0x0f;
					Rdy_buf[len++] = (w_dat>>4) &0x0f;
				 L_dat = INC3_DAT * 4096 /1800;						
				 Rdy_buf[len++] = L_dat&15;							
				 Rdy_buf[len++] = (L_dat>>4) &15;
				 Rdy_buf[len++] = (L_dat>>8) &15;	
				 L_dat = AZ3_DAT * 4096 /3600;
				 Rdy_buf[len++] = L_dat&15;
				 Rdy_buf[len++] = (L_dat>>4) &15;
				 Rdy_buf[len++] = (L_dat>>8) &15;	
				 w_dat = GT2_DAT * 256 /30000;						//GT
				 Rdy_buf[len++] = w_dat &15;
				 Rdy_buf[len++] = (w_dat>>4) &15;	
				 w_dat = BT2_DAT * 256 /30000;						//BT
				 Rdy_buf[len++] = w_dat &15;
				 Rdy_buf[len++] = (w_dat>>4) &15;
				 w_dat = T2_DAT * 256 /7200;							//FT
				 Rdy_buf[len++] = w_dat &15;
				 Rdy_buf[len++] = (w_dat>>4) &15;	
				 if(TE2_DAT>=18000) TE2_DAT = 18000;					//T
				 w_dat = TE2_DAT * 256 /18000;
				 Rdy_buf[len++] = w_dat &15;
				 Rdy_buf[len++] = (w_dat>>4) &15;

				 Rdy_buf[len] = 0;
				 usMBCRC16(len-1);
				 Rdy_buf[len] =0x4f;
				 Rdy_buf[len+1] =0;						 			 
				break;
		}		

}


void Data_Calculate(u8 sque,u8  position)
{
//		u8 i;
		u8 len=0;	
		u16 w_dat;
		u32 L_dat;
	


		get_data(usart_director_data,usart_gamma_data);
		if(sque==8)																													//如果更新序列8的数据，井斜大于5度，置位重力工具面转换标志位，同时发送重力工具面
		{
			if(INC3_DAT>=50)
			{
				magnetic_gravity_conversion_flag=1;				//井斜大于5度，转换重力
				T2_DAT = HT2_DAT;	
			}						
			else
			{			
				magnetic_gravity_conversion_flag=0;			//井斜小于5度，转换磁性			
				T2_DAT = MT2_DAT+3600;	
			}				
		}
		else																																	//如果发送序列2或者序列6，根据重力转换标志位决定发送重力或者磁性工具面
		{
			if(magnetic_gravity_conversion_flag)
				T2_DAT = HT2_DAT;	
			else
				T2_DAT = MT2_DAT+3600;	
		}
//		len=0;
		switch(sque)
		{
			case 2:				
					switch(position)
					{
						case 1:
							 Rdy_buf[0] = 0x02;	
								w_dat = T2_DAT * 256 /7200;								//FT
								Rdy_buf[1] = w_dat &15;
								Rdy_buf[2] = (w_dat>>4) &15;	

								usMBCRC16_1(3);					
								break;
						case 2:
								L_dat = INC3_DAT * 4096 /1800;						//INC
								Rdy_buf[3] = L_dat&15;
								Rdy_buf[4] = (L_dat>>4) &15;
								Rdy_buf[5] = (L_dat>>8) &15;	
								usMBCRC16_1(6);
								break;
						case 3:
								w_dat = T2_DAT * 256 /7200;								//FT
								Rdy_buf[6] = w_dat &15;
								Rdy_buf[7] = (w_dat>>4) &15;		
								usMBCRC16_1(8);
								break;						
						case 4:
								w_dat = AZ2_DAT * 256 /3600;							//AZ
								Rdy_buf[8] = w_dat &15;
								Rdy_buf[9] = (w_dat>>4) &15;		

								usMBCRC16_1(9);
								Rdy_buf[10] =0;						
							 break;						
					}		
					break ;	
			case 6:
					switch(position)
					{
						case 1:
							 Rdy_buf[0] = 0x06;	
								w_dat = T2_DAT * 256 /7200;								//FT
								Rdy_buf[1] = w_dat &15;
								Rdy_buf[2] = (w_dat>>4) &15;	
								usMBCRC16_1(3);					
								break;
						case 2:
								w_dat = T2_DAT * 256 /7200;								//FT
								Rdy_buf[3] = w_dat &15;
								Rdy_buf[4] = (w_dat>>4) &15;	
								usMBCRC16_1(5);
								break;
						case 3:
								w_dat = T2_DAT * 256 /7200;								//FT
								Rdy_buf[5] = w_dat &15;
								Rdy_buf[6] = (w_dat>>4) &15;		
								usMBCRC16_1(7);
								break;						
						case 4:
								w_dat = T2_DAT * 256 /7200;								//FT
								Rdy_buf[7] = w_dat &15;
								Rdy_buf[8] = (w_dat>>4) &15;		
						
								usMBCRC16_1(8);
								Rdy_buf[9] =0;						
							 break;						
					}		
					break ;	
			case 8:
				 Rdy_buf[len++] = 8;
				 L_dat = INC3_DAT * 4096 /1800;						
				 Rdy_buf[len++] = L_dat&15;							
				 Rdy_buf[len++] = (L_dat>>4) &15;
				 Rdy_buf[len++] = (L_dat>>8) &15;	
				 L_dat = AZ3_DAT * 4096 /3600;
				 Rdy_buf[len++] = L_dat&15;
				 Rdy_buf[len++] = (L_dat>>4) &15;
				 Rdy_buf[len++] = (L_dat>>8) &15;	
				 w_dat = GT2_DAT * 256 /30000;						//GT
				 Rdy_buf[len++] = w_dat &15;
				 Rdy_buf[len++] = (w_dat>>4) &15;	
				 w_dat = BT2_DAT * 256 /30000;						//BT
				 Rdy_buf[len++] = w_dat &15;
				 Rdy_buf[len++] = (w_dat>>4) &15;
				 w_dat = T2_DAT * 256 /7200;							//FT
				 Rdy_buf[len++] = w_dat &15;
				 Rdy_buf[len++] = (w_dat>>4) &15;	
				 if(TE2_DAT>=18000) TE2_DAT = 18000;					//T
				 w_dat = TE2_DAT * 256 /18000;
				 Rdy_buf[len++] = w_dat &15;
				 Rdy_buf[len++] = (w_dat>>4) &15;

				 Rdy_buf[len] = 0;
				 usMBCRC16(len-1);
				 Rdy_buf[len] =0x4f;
				 Rdy_buf[len+1] =0;						 			 
				break;
		}		

}
////===============================
//void MMD_Main(void){
//u8 i;
//u16 usLen;
//u8 sn_type;
//u16 w_dat;
//u32 L_dat;

//	G2_DAT = 0;					//GAMMA
//	AZ2_DAT = 0;
//	HT2_DAT = 0;				//HT
//	MT2_DAT = 0;				//MT
//	I2_DAT = 0;	
//	GT2_DAT = 0;       	//GT
//	BT2_DAT = 0;       	//BT
//	TE2_DAT = 0;				//TEMPERATURE
//	INC3_DAT = 0;				//INC
//	AZ3_DAT = 0;				//AZ

//	while(1){

//		if(Director_data_receive_flag==1)
//		{
//			
//			HT2_DAT	=	usart_director_data[0]*256+usart_director_data[1];				//重力工具面
//			MT2_DAT	=	usart_director_data[2]*256+usart_director_data[3];				//磁性工具面
//			INC3_DAT=	usart_director_data[4]*256+usart_director_data[5];				//磁性工具面
//			BT2_DAT = usart_director_data[6]*256+usart_director_data[7];			//磁场和工具面
//			AZ3_DAT = usart_director_data[8]*256+usart_director_data[9];			//方位
//			AZ2_DAT = AZ3_DAT;
//			GT2_DAT = usart_director_data[10]*256+usart_director_data[11];		//重力和
//			TE2_DAT = usart_director_data[12]*256+usart_director_data[13];		//温度			
//			if(INC3_DAT>=50)
//			{
//				T2_DAT = HT2_DAT;	
//			}						
//			else
//			{
//				T2_DAT = MT2_DAT+3600;		
//			}			
//										
//			Director_data_receive_flag=0;
//		}


////		for (i=0;i<10;i++)
////		{
////		 	CXX_LED_ON;
////			delay_ms(100);
////		 	CXX_LED_OFF;
////			delay_ms(100);
////		}
////		 	CXX_LED_ON;

//		//-----------------------------------------
//		//SerlCode(8) = "I3A3R2N2T2E2"
//		   usLen=0;
//		   sn_type = 8;
//		   Rdy_buf[usLen++] = sn_type;

//	//	   INC3_DAT += 18;
//		   if(INC3_DAT>=1800) INC3_DAT = 0;
//		   L_dat = INC3_DAT * 4096 /1800;						//数据计算正确，发送正确
//		   Rdy_buf[usLen++] = L_dat&15;							
//		   Rdy_buf[usLen++] = (L_dat>>4) &15;
//		   Rdy_buf[usLen++] = (L_dat>>8) &15;

//	//	   AZ3_DAT += 18;
//		   if(AZ3_DAT>=3600) AZ3_DAT = 0;
//		   L_dat = AZ3_DAT * 4096 /3600;
//		   Rdy_buf[usLen++] = L_dat&15;
//		   Rdy_buf[usLen++] = (L_dat>>4) &15;
//		   Rdy_buf[usLen++] = (L_dat>>8) &15;


//		//   GT2_DAT += 10;
//	//	   if(GT2_DAT>=30000) GT2_DAT = 0;
//		   w_dat = GT2_DAT * 256 /30000;
//		   Rdy_buf[usLen++] = w_dat &15;
//		   Rdy_buf[usLen++] = (w_dat>>4) &15;

//		//   BT2_DAT = 10;
//	//	   if(BT2_DAT>=300) BT2_DAT = 0;
//		   w_dat = BT2_DAT * 256 /30000;
//		   Rdy_buf[usLen++] = w_dat &15;
//		   Rdy_buf[usLen++] = (w_dat>>4) &15;
//				
//				
//				
//		//		 HT2_DAT += 9;
//		//		 if(T2_DAT>=7200) T2_DAT = 0;
//				 w_dat = T2_DAT * 256 /7200;
//				 Rdy_buf[usLen++] = w_dat &15;
//				 Rdy_buf[usLen++] = (w_dat>>4) &15;				
//				



//	//	   TE2_DAT += 10;
//		   if(TE2_DAT>=18000) TE2_DAT = 0;
//		   w_dat = TE2_DAT * 256 /18000;
//		   Rdy_buf[usLen++] = w_dat &15;
//		   Rdy_buf[usLen++] = (w_dat>>4) &15;

//		   Rdy_buf[usLen] = 0;
//		   usMBCRC16(usLen-1);
//		   Rdy_buf[usLen] =0x4f;
//		   Rdy_buf[usLen+1] =0;

//		   //OUTPUT_SDAT(usLen);

//		   Cxx_IO_Init();

//		//-----------------------------------------		  		
// 		//SerlCode(6) = "T2T2T2T2"
//		for(i=0;i<5;i++){
//		   usLen=0;
//		   sn_type = 6;
//		   Rdy_buf[usLen++] = sn_type;

//		//   T2_DAT += 9;
//		   if(T2_DAT>=7200) T2_DAT = 0;
//		   w_dat = T2_DAT * 256 /7200;
//		   Rdy_buf[usLen++] = w_dat &15;
//		   Rdy_buf[usLen++] = (w_dat>>4) &15;

//	//	   T2_DAT += 9;
//		   if(T2_DAT>=7200) T2_DAT = 0;
//		   w_dat = T2_DAT * 256 /7200;
//		   Rdy_buf[usLen++] = w_dat &15;
//		   Rdy_buf[usLen++] = (w_dat>>4) &15;

//	//	   T2_DAT += 9;
//		   if(T2_DAT>=7200) T2_DAT = 0;
//		   w_dat = T2_DAT * 256 /7200;
//		   Rdy_buf[usLen++] = w_dat &15;
//		   Rdy_buf[usLen++] = (w_dat>>4) &15;

//	//	   T2_DAT += 9;
//		   if(T2_DAT>=7200) T2_DAT = 0;
//		   w_dat = T2_DAT * 256 /7200;
//		   Rdy_buf[usLen++] = w_dat &15;
//		   Rdy_buf[usLen++] = (w_dat>>4) &15;

//		   Rdy_buf[usLen] = 0;
//		   usMBCRC16(usLen-1);
//		   Rdy_buf[usLen] =0x4f;
//		   Rdy_buf[usLen+1] =0;

//		   //OUTPUT_SDAT(usLen);

//		   Cxx_IO_Init();

//		}


//		//-----------------------------------------
//		//SerlCode(2) = "T2I3T2A2"
//		   usLen=0;
//		   sn_type = 2;
//		   Rdy_buf[usLen++] = sn_type;

//	//	   T2_DAT += 9;
//		   if(T2_DAT>=7200) T2_DAT = 0;
//		   w_dat = T2_DAT * 256 /7200;
//		   Rdy_buf[usLen++] = w_dat &15;
//		   Rdy_buf[usLen++] = (w_dat>>4) &15;

//	//	   INC3_DAT += 18;
//		   if(INC3_DAT>=1800) INC3_DAT = 0;
//		   L_dat = INC3_DAT * 4096 /1800;
//		   Rdy_buf[usLen++] = L_dat&15;
//		   Rdy_buf[usLen++] = (L_dat>>4) &15;
//		   Rdy_buf[usLen++] = (L_dat>>8) &15;

//	//	   T2_DAT += 9;
//		   if(T2_DAT>=7200) T2_DAT = 0;
//		   w_dat = T2_DAT * 256 /7200;
//		   Rdy_buf[usLen++] = w_dat &15;
//		   Rdy_buf[usLen++] = (w_dat>>4) &15;

//	//	   A2_DAT += 18;
//		   if(AZ2_DAT>=3600) AZ2_DAT = 0;
//		   w_dat = AZ2_DAT * 256 /3600;
//		   Rdy_buf[usLen++] = w_dat &15;
//		   Rdy_buf[usLen++] = (w_dat>>4) &15;

//		   Rdy_buf[usLen] = 0;
//		   usMBCRC16(usLen-1);
//		   Rdy_buf[usLen] =0x4f;
//		   Rdy_buf[usLen+1] =0;

//		   //OUTPUT_SDAT(usLen);

//		   Cxx_IO_Init();
//		//-----------------------------------------





//	}

////---------------------------------------------

//} 

//======================================================

void MBUS_Main(void){

u8 i;
		 if (USART_MBUS_TXSTA & 0x80) //发送完成 	
		     USART_MBUS_TXSTA = 0;

//------------------------------------------------
		// modbus 串口接受处理
		if (USART_MBUS_Recok(10)&&(USART_MBUS_TXSTA == 0))  {
		
			USART_MBUS_RXSTA = 0;

				for(i=0;i<USART_MBUS_RXNUM;i++)	{
					USART_MBUS_BUF[i]= USART_MBUS_RXBUF[i];
					CXX_FUNC_BUF[i]	=  USART_MBUS_RXBUF[i];
				}
	
				USART_MBUS_TXSTA = 0;
				USART_MBUS_Txstart(USART_MBUS_RXNUM);

		}    

} 
//===================================================


