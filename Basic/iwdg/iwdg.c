


#include "iwdg.h"


void IWDG_Init(void){ //��ʼ���������Ź�
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); //ʹ�ܶԼĴ���IWDG_PR��IWDG_RLR��д����
    IWDG_SetPrescaler(pre); //����IWDGԤ��Ƶֵ
    IWDG_SetReload(rlr); //����IWDG��װ��ֵ
    IWDG_ReloadCounter(); //����IWDG��װ�ؼĴ�����ֵ��װ��IWDG������
    IWDG_Enable(); //ʹ��IWDG
}

void IWDG_Feed(void){ //ι������
    IWDG_ReloadCounter();//�̼����ι������
}

