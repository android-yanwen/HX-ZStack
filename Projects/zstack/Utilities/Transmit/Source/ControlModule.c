#include "ControlModule.h"
//            ����ܶ��� 0     1     2     3     4     5     6     7     8     9
const uint8 smgCode[] = {0x3f, 0x30, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f};
//�����ת
const uint8 motorForewardCode[] = {0x08, 0x0c, 0x04, 0x06, 0x02, 0x03, 0x01, 0x09};
//�����ת
const uint8 motorReversalCode[] = {0x09, 0x01, 0x03, 0x02, 0x06, 0x04, 0x0c, 0x08};
/**
* �������ʾnum����
*/
void DisplaySmg(uint8 num){
    HC595IOInit();
    HC595StoreData(smgCode[num%10]);
    HC595StoreData(0x00);
    HC595StoreData(0x00);
    HC595StoreData(0x00);
    HC595StoreData(0x00);
    HC595SendData();
}

/**
* Desc:���Ƶ������ת
* Param: 0x00��ת
*        0xff��ת
*/
void ControlStepMotor(uint8 flg){
    uint8 u_StepNum, i;
    HC595IOInit();
    for(i = 0; i<100;i++){
        for(u_StepNum = 0; u_StepNum < 8; u_StepNum++){
            HC595StoreData(0x00);
            HC595StoreData(0x00);
            HC595StoreData(0x00);
            HC595StoreData(0x00);
            if(flg == 0x00)
                HC595StoreData(~motorForewardCode[u_StepNum] & 0x0f);
            else 
                HC595StoreData(~motorReversalCode[u_StepNum] & 0x0f);
            Onboard_wait(4500);
            HC595SendData();
        }
    }
    
}

/**
* ��ʼ��HC595��IO��
*/
void HC595IOInit(void){
    IO_DIR_PORT_PIN(0, 0, IO_OUT);
    IO_DIR_PORT_PIN(0, 1, IO_OUT);
    IO_DIR_PORT_PIN(0, 4, IO_OUT);
    IO_DIR_PORT_PIN(1, 2, IO_OUT);
    EN = 0;
}

/**
* ��val���ݴ���595������
*/
void HC595StoreData(uint8 val){
    uint8 i;
    for(i = 0; i < 8; i++){
        if((val << i) & 0x80){
            DI = 1;
        } else {
            DI = 0;
        }
        SCLK = 0;
        SCLK = 1;
    }
}

/**
* ���������е����ݲ���������ͷ�����
*/
void HC595SendData(void){
    RCLK = 0;
    RCLK = 1;
}

