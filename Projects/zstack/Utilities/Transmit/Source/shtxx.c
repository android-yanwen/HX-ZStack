/********************************************************************
 * INCLUDES
 */
#include "shtxx.h"

/********************************************************************
 * MACROS
 */
#define DELAY_TICK(n)   {tick=(n); while (tick--);}

/********************************************************************
 * CONSTANTS
 */

/********************************************************************
 * LOCAL VARIABLES
 */
uint8 tick;

uint8 m_error = 0;

uint16 m_temperature;
uint16 m_humidity;
uint8 m_checksum;
uint8 *p_value;



/*********************************************************************
 * GLOBAL FUNCTIONS
 */
void readTH(uint8 *pData, uint8 *pStartIndex);

/*********************************************************************
 * LOCAL FUNCTIONS
 */
void reset(void);
void start(void);
void sendByte(uint8 value);
uint8 recvByte(uint8 ack);



/**************************************************************************
 *@fn       readTH
 *
 *@brief    read temperature and humidity sensor data.
 *
 *@param    pData-data buffer.
 *          startIndex- .
 *
 *@return   none
 */
void readTH(uint8 *pData, uint8 *pStartIndex)
{
  /*Reading temperature*/
  reset();
  start();
  sendByte(SHT_MEASURE_TEMP);
  SHT_DATA_IN;

  while (SHT_DATA);

  pData[(*pStartIndex) + 1] = recvByte(SHT_MEASURE_TEMP);
  pData[(*pStartIndex) + 1] &= 0x3f; /*00111111*/
  pData[(*pStartIndex)] = recvByte(SHT_MEASURE_TEMP);
  *pStartIndex += 2;
  (void)recvByte(SHT_MEASURE_TEMP);

  /*Reading humidity*/
  reset();
  start();
  sendByte(SHT_MEASURE_HUMI);
  SHT_DATA_IN;

  while (SHT_DATA);

  pData[(*pStartIndex) + 1] = recvByte(SHT_MEASURE_HUMI);
  pData[(*pStartIndex) + 1] &= 0x0f; /*00001111*/
  pData[(*pStartIndex)] = recvByte(SHT_MEASURE_HUMI);
  *pStartIndex += 2;
  (void)recvByte(SHT_MEASURE_HUMI);
}

void start()
{
  SHT_DATA_OUT;
  SHT_SCK_OUT;

  SHT_DATA = 1;
  SHT_SCK = 0;
  //_nop_();
  tick = 255; while (tick--);
  SHT_SCK = 1;
  //_nop_();
  tick = 255; while (tick--);
  SHT_DATA = 0;
  //_nop_();
  tick = 255; while (tick--);
  SHT_SCK = 0;
  //_nop_();
  //_nop_();
  //_nop_();
  tick = 255; while (tick--);

  SHT_SCK = 1;
  //_nop_();
  tick = 255; while (tick--);
  SHT_DATA = 1;
  //_nop_();
  tick = 255; while (tick--);
  SHT_SCK = 0;
}

void reset()
//----------------------------------------------------------------------------------
// communication reset: DATA-line=1 and at least 9 SCK cycles followed by transstart
//       _____________________________________________________         ________
// DATA:                                                      |_______|
//          _    _    _    _    _    _    _    _    _        ___     ___
// SCK : __| |__| |__| |__| |__| |__| |__| |__| |__| |______|   |___|   |______
{
  uint8 i;

  SHT_DATA_OUT;
  SHT_SCK_OUT;


  SHT_DATA = 1;
  SHT_SCK = 0;                    //Initial state

  for (i=0; i<9; i++)                  //9 SCK cycles
  {
    SHT_SCK = 1;
    tick = 255; while (tick--);

    SHT_SCK = 0;

    tick = 255; while (tick--);

  }

  start();                   //transmission start

}


void sendByte(uint8 value)
{
  uint8 i;


  SHT_DATA_OUT;
  SHT_SCK_OUT;
  //    SHT_DATA = 1;
  //SHT_SCK = 1;//

  for (i=0x80; i>0; i/=2)
  {
    if (i & value)
    {
      SHT_DATA = 1;
    }
    else
    {
      SHT_DATA = 0;
    }

    SHT_SCK = 1;

    tick = 255;
    while (tick--);

    SHT_SCK = 0;

    tick = 255;
    while (tick--);
  }



  SHT_DATA = 1;
  SHT_SCK = 1;


  //tick = 250;

  SHT_DATA_IN;


  while (SHT_DATA);

  SHT_SCK = 0;
}


uint8 recvByte(uint8 ack)
{
  uint8 i,val = 0;

  SHT_SCK_OUT;


  SHT_DATA_OUT;
  SHT_DATA = 1;////////////////////

  SHT_DATA_IN;

  tick=255;
  while (tick--);

  for (i=0x80; i>0; i/=2)
  {
    SHT_SCK = 1;

    tick=255;
    while (tick--);
    if (SHT_DATA)
    {
      val = (val | i);
    }

    SHT_SCK = 0;
    tick=255;
    while (tick--);
  }

  SHT_DATA_OUT;

  SHT_DATA = !ack;                        //in case of "ack==1" pull down DATA-Line
  SHT_SCK = 1;                            //clk #9 for ack
  //_nop_();
  //_nop_();
  //_nop_();          //pulswith approx. 5 us

  tick = 255;
  while (tick--);
  SHT_SCK = 0;
  SHT_DATA = 1;                          //release DATA-line
  return val;
}
