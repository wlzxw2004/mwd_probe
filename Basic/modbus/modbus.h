#ifndef __MODBUS_H
#define __MODBUS_H
	 
#include "sys.h"

#define GAMMA_VALUE_RANGE   365


void MBUS_Main(void);	  //MODBUS Ö÷³ÌÐò
void MMD_Main(void);
void Data_Calculate(u8 sque,u8 position);
void Data_Calculate_g(u8 sque,u8  position);
void Data_Calculate_sta(u8 sq);
void Data_Calculate_sta_g(u8 sq);
void data_copy(void);
void get_data(const u8 arr_d[],const u8 arr_g[]);

extern	u8 Rdy_buf[256];
extern u8 Send_buf[50];
extern	u8 Rdy_Len;
extern u8 Gamma_connect_flag;
#endif  //__MODBUS_H
