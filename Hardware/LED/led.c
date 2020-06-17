
#include "led.h"

void LED_Init(void){ //LED灯的接口初始化
	GPIO_InitTypeDef  GPIO_InitStructure; 	
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB,ENABLE);
	
	//  查询线LED灯
    GPIO_InitStructure.GPIO_Pin =  CXX_LED;                        
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //选择IO接口工作方式   推挽输出       
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //设置IO接口速度（2/10/50MHz） 
	GPIO_Init(CXX_LED_P, &GPIO_InitStructure);


	//  急停闭锁显示 LED灯
    GPIO_InitStructure.GPIO_Pin = LK_LEDCTL; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		//选择IO接口工作方式   推挽输出 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 		//设置IO接口速度（2/10/50MHz） 	                   
	GPIO_Init(LK_LEDCTL_P, &GPIO_InitStructure);

	CXX_LED_OFF;	
	LK_LEDCTL_OFF;
			
}
 

/*
选择IO接口工作方式：
GPIO_Mode_AIN 模拟输入
GPIO_Mode_IN_FLOATING 浮空输入
GPIO_Mode_IPD 下拉输入
GPIO_Mode_IPU 上拉输入
GPIO_Mode_Out_PP 推挽输出
GPIO_Mode_Out_OD 开漏输出
GPIO_Mode_AF_PP 复用推挽输出
GPIO_Mode_AF_OD 复用开漏输出
*/
