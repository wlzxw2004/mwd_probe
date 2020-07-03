#ifndef __DATA_RECEIVE_H
#define __DATA_RECEIVE_H	 
#include "sys.h"
#include "led.h"
#include "modbus.h"
//....................................

//....................................

#define D_DATA_SIZE  18
#define G_DATA_SIZE  12

extern u8 usart_director_data[];
extern u8 usart_gamma_data[];
extern u8 usart_gamma_data_dynamic[];
extern u8 usart_director_data_sta[];
extern u8 usart_director_data_dynamic[];
extern u8 Director_data_receive_flag;
extern u8 Gamma_data_receive_flag;

void directional_data_receive(uint8_t data_usart);
void gamma_data_receive(u8 data_usartg);

		 				    
#endif
