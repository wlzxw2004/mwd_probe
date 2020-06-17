#ifndef __PSCXX_H
#define __PSCXX_H	 
#include "sys.h"



//==========拉线急停闭锁开关板 端口定义=====================




//================================

extern u8 	B_CXX_STA; //B_CXX_STA=0;初始化状态

extern u8 	B_SB_ID ;	//分配的设备ID识别号
extern u8 	B_SB_NUM ;	//分配的设备地址
extern u8 	B_SB_DATA ;  //查询的设备数据 BIT4-急停动作 ,

extern u8 	cxx_fx1  ;
extern u8 	cxx_fx2  ;
extern u8	RX1_STS_IN;
extern u8	RX2_STS_IN;

extern u8 can_led_dly;
extern u8 cxx_led_dly;
extern u8 Task_ok;
extern u8 B_BX_ID;	
extern u8 B_MC_ID;   
extern u16 W_BX_NUM ;	

extern u8 Data_update_always_flag;
extern u8 Data_update_temp_flag;				//单次数据更新标志，1 允许运行，0不允许运行
extern u8 Data_update_run_falg;				//数据更新是否执行，1 已经运行，0 未运行
extern u8 Output_pulse_h_flag; 				//如果脉冲输出高为1，低为0
//===========================



//=======函数声明=========

void Pscxx_Main(void);   // 皮带保护系统 查询线处理	主函数
void Cxx_IO_Init(void);   //IO初始化




		 				    
#endif
