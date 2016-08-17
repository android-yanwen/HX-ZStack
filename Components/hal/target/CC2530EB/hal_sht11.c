/*******************************************************************************
* �� �� ����SHT11.c
* ��    �ܣ�SHT11��ʪ�ȴ�����������
* Ӳ�����ӣ�SHT11��CC2530��Ӳ�����ӹ�ϵ������ʾ��
*
*           SHT11                  CC2530
*           DATA                    p0.1//P0.7
*           SCK                     p1.7//P0.0
*******************************************************************************/

#include "hal_types.h"
#include "OSAL.h"
#include "OnBoard.h"
#include "hal_sht11.h"
#include "math.h"









/*******************************************************************************
 * �������ƣ�SHT11_PIN_Init
 *
 * ����������CC2430��SHT11�����ӵ����ų�ʼ��
 *
 * ��    ������
 *
 * �� �� ֵ����
 ******************************************************************************/
void Hal_SHT11_Init(void)
{
  P1SEL = P1SEL & 0x7f;
  IO_DIR_PORT_PIN(1, 7, IO_OUT);      //p1.7 // P0.0����Ϊ���
  IO_DIR_PORT_PIN(0, 1, IO_OUT);      //p0.1// P0.7����Ϊ���
  DATA = 1;
  SCK = 0;
}


/*******************************************************************************
 * �������ƣ�s_write_byte
 *
 * ������������SHT11д1���ֽڲ����SHT11�Ƿ�Ӧ��
 *
 * ��    ����value  Ҫд���1�ֽ�����
 *
 * �� �� ֵ��error=1����SHT11δӦ��
 ******************************************************************************/
char s_write_byte(unsigned char value)
{
  static unsigned char i,j,error=0;

  IO_DIR_PORT_PIN(0, 1, IO_OUT);      //p0.1// P0.7����Ϊ���

  /* �Ӹߵ�����λ���� */
  for (i=0x80;i>0;i/=2)               // ��λ����
  {
    if(i & value)
      DATA = 1;
    else
      DATA = 0;

    SCK = 1;
    for(j=0;j<50;j++ )  asm("nop");     	
    SCK = 0;
    for(j=0;j<50;j++ )  asm("nop");  
  }

  DATA=1;                              // �ͷ�DATA��
  SCK=1;                               // ��9��SCK
  IO_DIR_PORT_PIN(0, 1, IO_IN);        //p0.1// P0.7����Ϊ����
  for(j=0;j<50;j++ )  asm("nop");    
  error=DATA;                          // ���Ӧ�� (SHT11������DATA��ΪӦ��)
  SCK=0;
  return error;                        // error=1����SHT11δӦ��
}


/*******************************************************************************
 * �������ƣ�s_read_byte
 *
 * ������������SHT11��1���ֽڲ����������ack=1ʱ����Ӧ��
 *
 * ��    ����ack  Ӧ���־
 *
 * �� �� ֵ��error=1����SHT11δӦ��
 ******************************************************************************/
unsigned char s_read_byte(unsigned char ack)
{
  unsigned char i,j,val=0;

  IO_DIR_PORT_PIN(0, 1, IO_OUT);      //p0.1// P0.7����Ϊ���

  DATA=1;                               // �ͷ�DATA��

  IO_DIR_PORT_PIN(0, 1, IO_IN);         //p0.1// P0.7����Ϊ����
  /* �Ӹߵ�����λ��ȡ */
  for (i=0x80;i>0;i/=2)                 // ��λ����
  {
    SCK = 1;
    for(j=0;j<50;j++ )  asm("nop"); 
    if (DATA) val=(val | i);
     
    SCK = 0;  	
    for(j=0;j<50;j++ )  asm("nop");     
  }

  IO_DIR_PORT_PIN(0, 1, IO_OUT);         //p0.1// P0.2����Ϊ���
  //DATA = !ack;                           // ��ack=1ʱ����DATA��
  if(ack==1)DATA=0;
  else DATA=1;
  for(j=0;j<50;j++ )  asm("nop");  
  SCK = 1;                               // ��9��SCK
  for(j=0;j<50;j++ )  asm("nop");         	
  SCK = 0;
  for(j=0;j<50;j++ )  asm("nop");  
  DATA = 1;                              // �ͷ�DATA��
  return val;
}


/*******************************************************************************
 * �������ƣ�s_transstart
 *
 * ��������������һ��"��������"����
 *                 _____         ________
 *           DATA:      |_______|
 *                     ___     ___
 *           SCK : ___|   |___|   |______
 *
 * ��    ������
 *
 * �� �� ֵ����
 ******************************************************************************/
void s_transstart(void)
{
   unsigned char j;

   IO_DIR_PORT_PIN(0, 1, IO_OUT);         //p0.1// P0.7����Ϊ���

   DATA = 1; 
   SCK = 0;                     // ��ʼ״̬
   for(j=0;j<100;j++ )  asm("nop");         	
   SCK = 1;
   for(j=0;j<100;j++ )  asm("nop");         	
   DATA = 0;
   for(j=0;j<100;j++ )  asm("nop");         	
   SCK = 0;
   for(j=0;j<100;j++ ) asm("nop");         	
   SCK = 1;
   for(j=0;j<100;j++ )  asm("nop");         	
   DATA = 1;		
   for(j=0;j<100;j++ )  asm("nop");         	
   SCK= 0;		
}


/*******************************************************************************
 * �������ƣ�s_connectionreset
 *
 * ����������ͨ�Ÿ�λ
 *                 _____________________________________________________         ________
 *           DATA:                                                      |_______|
 *                    _    _    _    _    _    _    _    _    _        ___     ___
 *           SCK : __| |__| |__| |__| |__| |__| |__| |__| |__| |______|   |___|   |______
 *
 * ��    ������
 *
 * �� �� ֵ����
 ******************************************************************************/
void s_connectionreset(void)
{
  unsigned char i;

  IO_DIR_PORT_PIN(0, 1, IO_OUT);            //p0.1// P0.2����Ϊ���

  DATA = 1; SCK = 0;                        // ��ʼ״̬

  /* 9��SCK ����*/
  for(i=0;i<9;i++)
  {
    SCK = 1;
    SCK = 0;
  }

  s_transstart();                           // ����һ��"��������"����
}


/*******************************************************************************
 * �������ƣ�s_softreset
 *
 * ���������������λ
 *
 * ��    ������
 *
 * �� �� ֵ������ֵΪ1��ʾSHT11δ��Ӧ
 ******************************************************************************/
char s_softreset(void)
{
  unsigned char error=0;

  s_connectionreset();                       // ͨ�Ÿ�λ
  error+=s_write_byte(RESET);                // ����"��λ"�����SHT11
  return error;                              // error=1��ʾSHT11δ��Ӧ
}


/*******************************************************************************
 * �������ƣ�s_read_statusreg
 *
 * ������������ȡ״̬�Ĵ�����У���
 *
 * ��    ����p_value      ״̬�Ĵ�����ֵ
 *           p_checksum   У���
 *
 * �� �� ֵ������ֵΪ1��ʾSHT11δ��Ӧ
 ******************************************************************************/
char s_read_statusreg(unsigned char *p_value, unsigned char *p_checksum)
{
  unsigned char error=0;

  s_transstart();                             // ����һ��"��������"����
  error=s_write_byte(STATUS_REG_R);           // ����"��״̬�Ĵ���"����
  *p_value=s_read_byte(ACK);                  // ��״̬�Ĵ���
  *p_checksum=s_read_byte(noACK);             // ��У���

  return error;                               // error=1��ʾSHT11δ��Ӧ
}


/*******************************************************************************
 * �������ƣ�s_write_statusreg
 *
 * ����������д״̬�Ĵ���
 *
 * ��    ����p_value      ״̬�Ĵ�����ֵ
 *
 * �� �� ֵ������ֵΪ1��ʾSHT11δ��Ӧ
 ******************************************************************************/
char s_write_statusreg(unsigned char *p_value)
{
  unsigned char error=0;

  s_transstart();                              // ����һ��"��������"����
  error+=s_write_byte(STATUS_REG_W);           // ����"д״̬�Ĵ���"����
  error+=s_write_byte(*p_value);               // д״̬�Ĵ���

  return error;                                // error=1��ʾSHT11δ��Ӧ
}


/*******************************************************************************
 * �������ƣ�s_measure
 *
 * ��������������һ�β���(���ʪ�Ȼ��¶�)
 *
 * ��    ����p_value      ����ֵ
 *           checksum     У���
 *           mode         TEMP��ʾ�����¶Ȳ���
 *                        HUMI��ʾ�������ʪ�Ȳ���
 *
 * �� �� ֵ������ֵΪ1��ʾSHT11δ��Ӧ
 ******************************************************************************/
char s_measure(char *p_value,unsigned char *p_checksum, unsigned char mode)
{
  unsigned char error=0;
  unsigned long i;

  s_transstart();                              // ����һ��"��������"����
  switch(mode)                                 // �����������mode����һ����Ӧ�Ĳ���
  {
    case TEMP	: error+=s_write_byte(MEASURE_TEMP); break;
    case HUMI	: error+=s_write_byte(MEASURE_HUMI); break;
    default     : break;	
  }
  //for(i=0;i<65500;i++ )  asm("nop"); 
  for (i=0;i<665535;i++) if(DATA==0) break;    // �ȴ�SHT11��ɲ���
  if(DATA) error+=1;                           // ��������
  //for(i=0;i<65500;i++ )  asm("nop"); 
  *(p_value+1) = s_read_byte(ACK);             // ����1���ֽ� (MSB)
  //for(i=0;i<65500;i++ )  asm("nop"); 
  *(p_value) = s_read_byte(ACK);               // ����2���ֽ� (LSB)
  //for(i=0;i<65500;i++ )  asm("nop"); 
  *p_checksum =s_read_byte(noACK);             // ��У���

  return error;
}


/*******************************************************************************
 * �������ƣ�calc_sth11
 *
 * �����������������ʪ�Ⱥ��¶�
 *
 * ��    ����p_humidity      SHT11�ɼ��������ʪ��ֵ(������������ת�����ò�������ʵ����������ֵ)
 *           p_temperature   SHT11�ɼ������¶�ֵ(������������ת�����ò�������ʵ����������ֵ)
 *
 * �� �� ֵ����
 ******************************************************************************/
void calc_sth11(float *p_humidity ,float *p_temperature)
{
  const float C1=-4.0;                          // 12λ
  const float C2=+0.0405;                       // 12 Bit
  const float C3=-0.0000028;                    // 12 Bit
  const float T1=+0.01;                         // 14λ 5V
  const float T2=+0.00008;                      // 14λ 5V	

  float rh=*p_humidity;                         // ���ʪ�Ȳɼ�ֵ 12λ
  float t=*p_temperature;                       // �¶Ȳɼ�ֵ 14λ
  float rh_lin;                                 // ���ʪ�ȵķ����Բ���
  float rh_true;                                // ���ʪ��������ֵ
  float t_C;                                    // �¶�������ֵ

  t_C=t*0.01 - 40;                              // �����¶�������ֵ
  rh_lin=C3*rh*rh + C2*rh + C1;                 // �������ʪ�ȵķ����Բ���
  rh_true=(t_C-25)*(T1+T2*rh)+rh_lin;           // �������ʪ��������ֵ

  /* ��������������ʪ��������ֵ����Χ��ض� */
  if(rh_true>100)rh_true=100;
  if(rh_true<0.1)rh_true=0.1;

  *p_temperature=t_C;                            // �����¶�������ֵ
  *p_humidity=rh_true;                           // �������ʪ��������ֵ
}


/*******************************************************************************
 * �������ƣ�calc_dewpoint
 *
 * ��������������¶��
 *
 * ��    ����h      ���ʪ��������ֵ
 *           t      �¶�������ֵ
 *
 * �� �� ֵ��¶��ֵ
 ******************************************************************************/
float calc_dewpoint(float h,float t)
{
  float logEx,dew_point;

  logEx=0.66077+7.5*t/(237.3+t)+(log10(h)-2);
  dew_point = (logEx - 0.66077)*237.3/(0.66077+7.5-logEx);

  return dew_point;
}






















