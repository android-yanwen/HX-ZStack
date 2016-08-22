#include "ControlModule.h"
//            ����ܶ��� 0     1     2     3     4     5     6     7     8     9
const uint8 smgCode[] = {0x3f, 0x30, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f};
//�����ת
const uint8 motorForewardCode[] = {0x08, 0x0c, 0x04, 0x06, 0x02, 0x03, 0x01, 0x09};
//�����ת
const uint8 motorReversalCode[] = {0x09, 0x01, 0x03, 0x02, 0x06, 0x04, 0x0c, 0x08};
//LED�������0~9
const uint8 ledRow[][8] = {
    0x38,0x44,0x44,0x44,0x44,0x44,0x44,0x38,//0
    0x18,0x28,0x08,0x08,0x08,0x08,0x08,0x3e,//1
    0x3e,0x02,0x02,0x3e,0x20,0x20,0x20,0x3e,//2
    0x3e,0x02,0x02,0x3e,0x02,0x02,0x02,0x3e,//3
    0x48,0x48,0x48,0x48,0x7e,0x08,0x08,0x08,//4
    0x7c,0x40,0x40,0x78,0x04,0x04,0x44,0x38,//5
    0x38,0x44,0x40,0x78,0x44,0x44,0x44,0x38,//6
    0x7c,0x04,0x04,0x08,0x10,0x10,0x10,0x10,//7
    0x38,0x44,0x44,0x38,0x44,0x44,0x44,0x38,//8
    0x38,0x44,0x44,0x44,0x3c,0x04,0x44,0x38//9
    
};
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
            if(flg == 0x00) //��ת
                HC595StoreData(~motorForewardCode[u_StepNum] & 0x0f);
            else 
                HC595StoreData(~motorReversalCode[u_StepNum] & 0x0f);
            Onboard_wait(4500);
            HC595SendData();
        }
    }
    HC595StoreData(0x00);
    HC595StoreData(0x00);
    HC595StoreData(0x00);
    HC595StoreData(0x00);
    HC595StoreData(0x00);
    HC595SendData();
}
/**
* Desc: �������ݵ�595�����Ʒ������Ŀ���
* Param: flg 0x00 ��
*            0xff �ر�
*/
void ControlBeep(uint8 flg){
    uint8 u_StepNum, i;
    HC595IOInit();
    HC595StoreData(0x00);
    HC595StoreData(0x00);
    HC595StoreData(0x00);
    HC595StoreData(0x00);
    if(flg == 0x00)//��������
        HC595StoreData(0x20);
    else 
        HC595StoreData(0x00);
    HC595SendData();
}
/**
* Desc: �������ݵ�595�����Ƽ̵����Ŀ���
* Param: flg 0x00 ��
*            0xff �ر�
*/
void ControlRelay(uint8 flg){
    uint8 u_StepNum, i;
    HC595IOInit();
    HC595StoreData(0x00);
    HC595StoreData(0x00);
    HC595StoreData(0x00);
    HC595StoreData(0x00);
    if(flg == 0x00)//�̵������رպ�
        HC595StoreData(0x10);
    else 
        HC595StoreData(0x00);
    HC595SendData();
}
/**
* Desc: �������ݵ�595������LED�������ʾ
* Param: num%10  ��ʾ0~9��������
*            
*/
void ControlLeds(uint8 num){
    uint8 col, row;
    uint8 colCode;
    HC595IOInit();
    colCode = 0xff;
    for(row = 0; row < 8; row++){
        HC595StoreData(0x00);
        colCode = ~(0x01 << row);
        HC595StoreData(colCode);
        HC595StoreData(ledRow[num%10][row]);
        HC595StoreData(0x00);
        HC595StoreData(0x00);
        HC595SendData();
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

