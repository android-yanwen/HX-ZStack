/**************************************************************************************************
    Filename:       TransmitApp.h
    Revised:        $Date: 2007-10-27 17:22:23 -0700 (Sat, 27 Oct 2007) $
    Revision:       $Revision: 15795 $

    Description:    This file contains the Transmit Application definitions.


  Copyright 2004-2007 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED ìAS ISî WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, 
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE, 
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com. 
**************************************************************************************************/

#ifndef TRANSMITAPP_H
#define TRANSMITAPP_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "ZComDef.h"
  #include "hal_uart.h"
#include "DS18B20.h"

/*********************************************************************
 * CONSTANTS
 */

// These constants are only for example and should be changed to the
// device's needs
#define TRANSMITAPP_ENDPOINT           1

#define TRANSMITAPP_PROFID             0x0F05
#define TRANSMITAPP_DEVICEID           0x0001
#define TRANSMITAPP_DEVICE_VERSION     0
#define TRANSMITAPP_FLAGS              0

#define TRANSMITAPP_MAX_CLUSTERS       1
#define TRANSMITAPP_CLUSTERID_TESTMSG  0x00A1

// Application Events (OSAL) - These are bit weighted definitions.
#define TRANSMITAPP_SEND_MSG_EVT       0x0001
#define TRANSMITAPP_RCVTIMER_EVT       0x0002
#define TRANSMITAPP_SEND_ERR_EVT       0x0004
#define TRANSMITAPP_BIND_REQUEST_EVT    0x0008//microdjg
#define TRANSMITAPP_START_REPORT_EVT    0X0010

#define TRANSMITAPP_LEDS_DISPLAY_EVT    0X0020
  
  
  
 //RX BUFFER SIZE 
#define RX_BUF_LEN                        128//UART Ω” ’ª∫≥Â«¯¥Û–°

   /*
 GTA DATA FRAME
*/
#define FRAME_HEAD                              0x7E

#define FUN_CODE_READ_COIL_STATE                0x01//∂¡œﬂ»¶◊¥Ã¨
#define FUN_CODE_READ_BIT_STATE                 0x02//∂¡»° ‰»ÎŒª◊¥Ã¨
#define FUN_CODE_READ_HOLDING_REGISTERS         0x03//∂¡±£≥÷ºƒ¥Ê∆˜
#define FUN_CODE_READ_INPUT_REGISTERS           0x04//∂¡ ‰»Îºƒ¥Ê∆˜
#define FUN_CODE_SET_SINGLE_CTRL_COIL           0x05//«ø÷√µ•œﬂ»¶£®øÿ÷∆ ‰≥ˆ£©DO
#define FUN_CODE_SET_SINGLE_REGISTERS           0x06//‘§÷√µ•ºƒ¥Ê∆˜AO
#define FUN_CODE_SET_MULTI_CTRL_COIL            0x0F//«ø÷√∂‡œﬂ»¶£®øÿ÷∆ ‰≥ˆ£©DO
#define FUN_CODE_SET_MULTI_REGISTERS            0x10//‘§÷√∂‡ºƒ¥Ê∆˜AO
#define FUN_CODE_READ_ADDR                      0x11//∂¡»°…Ë±∏µÿ÷∑
#define FUN_CODE_SET_ADDR                       0x12//…Ë÷√…Ë±∏µÿ÷∑

//#define FUN_CODE_READ_CODE_STATE
//ModBusFrame_t
#define FRAME_CHECK_RIGHT						0x01
#define FRAME_CHECK_HEAD_WRONG					0x02
#define FRAME_CHECK_CMD_WRONG					0x03


//Module ADDRESS Define
#define ADDR_TEMP_HUMI_SENSOR                   0x01
#define ADDR_LIGHT_SENSOR                       0x02
#define ADDR_WATER_SENSOR                       0x03
#define ADDR_INFRARED_SENSOR                    0x04
#define ADDR_VIBRATION_SENSOR                   0x05
#define ADDR_SMOKE_SENSOR                       0x06
#define ADDR_CARBON_MONOXIDE_SENSOR             0x07
#define ADDR_HALL_SENSOR                        0x08
#define ADDR_FLAMMABLE_GAS_SENSOR               0x09
#define ADDR_ULTRASONIC_SENSOR                  0x0A

#define ADDR_SETPMOTOR_CTRL                     0x11
  
#define ADDR_HF_RFID                            0x21
#define ADDR_LF_RFID                            0x22
#define ADDR_UF_RFID                            0x23
  
#define ADDR_BD_GPS_MODULE                      0x31
  
#define	IO_SENSOR          	                P0_6
    
#if defined (HX_WCGSB_R1) || (HX_ALCOHOL_SENSOR) || (HX_SMOKE_SENSOR)
    #define IO_COMBUSTIBLE_GAS_PORT                 P0_1
#endif
    

#define	IO_RELAY1          	                P0_6
#define	IO_RELAY2          	                P0_0

typedef struct
{
  uint8 ADDR;
  uint8 CMD;
  uint8 DATALEN;
  uint8 pData[20];
  uint8 COMFIRM;
}ModBusFrame_t;
/*********************************************************************
 * MACROS
 */

// Define to enable fragmentation transmit test
//#define TRANSMITAPP_FRAGMENTED

/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task Initialization for the Transmit Application
 */
extern void TransmitApp_Init( byte task_id );

/*
 * Task Event Processor for the Transmit Application
 */
extern UINT16 TransmitApp_ProcessEvent( byte task_id, UINT16 events );

extern void TransmitApp_ChangeState( void );
extern void TransmitApp_SetSendEvt( void );
void initUart(halUARTCBack_t pf);
void uartRxCB( uint8 port, uint8 event );
void FrameUnPackage(uint8 *pBuf,ModBusFrame_t *pFrame);
uint16 Crc16Code(uint8 *str, uint8 len);//Modbus Ckeck Code.//LO First
void SentDataInBind(uint8 *pdata,uint16 len);
void SendBindRequest(void);

#if defined(GTA_WMSCB_R1)
void init_GTA_WMSCB(void);
void ctrl_GTA_WMSCB(uint8 port,uint8 data);
void StepMotorRun(uint8 Dir,uint16 StepNum);
void SegDisplay(uint8 num);
#endif
/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* TRANSMITAPP_H */
