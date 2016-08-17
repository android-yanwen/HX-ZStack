#include "DS18B20.h"

unsigned char sensor_data_values[2];

void Delay_nus( uint16 s ){
    while ( s-- ) {
        asm("NOP");
        asm("NOP");
        asm("NOP");
    }
}


void write_1820 ( unsigned char x ){
    unsigned char m;
    SET_OUT;
    for(m = 0; m < 8; m++){
        CL_DQ;
        if(x&(1<<m)){
            SET_DQ;
        } else {
            CL_DQ;
        }
        Delay_nus(55);   //50~60us
        SET_DQ;
    }
}

unsigned char read_1820(void){
    unsigned char temp, k, n;
    temp = 0;
    for(n = 0; n < 8; n++){
        CL_DQ;
        SET_DQ;
        SET_IN;
        k = IN_DQ;  //读数据从低位开始
        if(k){
            temp |= (1<<n);
        } else {
            temp &= ~(1 << n);
        }
        Delay_nus(70);
        SET_OUT;
    }
    return (temp);
}

bool init_1820(void){
    int i = 0;
    IO_DIR_PORT_PIN(0, 1, IO_OUT);
//    P0_1=1;
//    P0_1=0;
    SET_OUT;
    SET_DQ;
    CL_DQ;
//    Onboard_wait(550);
    Delay_nus(700); //拉低一段时间
//    P0_1 = 1;
//    IO_DIR_PORT_PIN(0, 1, IO_IN);
    //for(i = 0; i < 10 ; i++);
    //   asm("nop");
    //Delay_nus(200);
    //Delay_nus(200);
    SET_DQ;//释放
    SET_IN;//输入
    Delay_nus(40);//释放总线后等待15~240us
    while(IN_DQ){;}//等待18b20回复
    Delay_nus(240);//回复的低电平在60到240us
    SET_OUT;
    SET_DQ;//回到初始DQ=1
    return 1;
}

void read_data(void){
    unsigned int j;
    unsigned char temph, templ;
    unsigned char a, b, c;
    init_1820();
    write_1820(0xcc);
    write_1820(0x44);
    for(j = 20; j > 1; j--){
        Delay_nus(50000);
    }
    init_1820();
    write_1820(0xcc);
    write_1820(0xbe);
    
    templ = read_1820();
    temph = read_1820();
    a = temph << 4;
    a+=(templ&0xf0) >> 4;
    b = templ & 0x0f;
    temph = a;
    templ = b & 0x00ff;
    sensor_data_values[0] = templ;
    sensor_data_values[1] = temph;
}

uint8 ch[2];
uint8* DataChange(void){
    unsigned char temph, templ;
    unsigned int num;
//    unsigned i;
    templ = sensor_data_values[0];
    temph = sensor_data_values[1];
    ch[0] = temph;
    num = templ*625;    //小数部分的取值每位代表0.0625（精度）
    ch[1] = num/1000;
    /*if(temph/100==0){
        ch[0] = 1;
    } else {
        ch[0] = temph/100+0x30;
    }
    if((temph%100/10==0)){
        ch[1] = 0;
    } else {
        ch[1] = temph%100/10 + 0x30;
    }
    ch[2] = temph%10 + 0x30;
    ch[3] = '.';
    ch[4] = num/1000 + 0x30;
    ch[5] = '\0';*/
    return ch;
}









