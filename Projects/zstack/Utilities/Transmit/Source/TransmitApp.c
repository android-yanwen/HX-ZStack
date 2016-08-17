/**************************************************************************************************
  Filename:       TransmitApp.c
  Revised:        $Date: 2012-03-05 09:54:49 -0800 (Mon, 05 Mar 2012) $
  Revision:       $Revision: 29619 $

  Description:    Transmit Application (no Profile).


  Copyright 2004-2012 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License"). You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product. Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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

/*********************************************************************
  This application will send a data packet to another
  tranmitApp device as fast as it can.  The receiving
  transmitApp device will calculate the following transmit
  rate statistics:
    - Number bytes in the last second
    - Number of seconds running
    - Average number of bytes per second
    - Number of packets received.

  The application will send one message and as soon as it
  receives the confirmation for that message it will send
  the next message.

  If you would like a delay between messages
  define TRANSMITAPP_DELAY_SEND and set the delay amount
  in TRANSMITAPP_SEND_DELAY.

  TransmitApp_MaxDataLength defines the message size

  Set TRANSMITAPP_TX_OPTIONS to AF_MSG_ACK_REQUEST to send
  the message expecting an APS ACK, this will decrease your
  throughput.  Set TRANSMITAPP_TX_OPTIONS to 0 for no
  APS ACK.

  This applications doesn't have a profile, so it handles
  everything directly - itself.

  Key control:
    SW1:  Starts and stops the transmitting
    SW2:  initiates end device binding
    SW3:  Resets the display totals
    SW4:  initiates a match description request

  Notes:

    This application was intended to be used to test the maximum
    throughput between 2 devices in a network - between routers
    coordinators.

    Although not recommended, it can be used between
    an end device and a router (or coordinator), but you must
    enable the delay feature (TRANSMITAPP_DELAY_SEND and
    TRANSMITAPP_SEND_DELAY).  If you don't include the delay, the
    end device can't receive messages because it will stop polling.
    Also, the delay must be greater than RESPONSE_POLL_RATE (default 100 MSec).
*********************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "OSAL.h"
#include "AF.h"
#include "ZDObject.h"
#include "ZDProfile.h"

#include "TransmitApp.h"
#include "OnBoard.h"

#include "DebugTrace.h"

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_uart.h"

#include "OSAL_NV.h"

#include "ZComDef.h"
#include "sapi.h"//

#include "hal_types.h"

#if defined(GTA_WTHSB_R1)
#include "hal_sht11.h"
#endif

#if defined(GTA_WHFRFID_R1)
#include "WHFRFID.h"
#endif
#include <string.h>

#if defined GTA_WMSCB_R1
#include "ControlModule.h"
#endif
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
//#define TRANSMITAPP_RANDOM_LEN

#define TRANSMITAPP_STATE_WAITING 0
#define TRANSMITAPP_STATE_SENDING 1

#if !defined ( RTR_NWK )
  // Use these 2 lines to add a delay between each packet sent
  //  - default for end devices
  #define TRANSMITAPP_DELAY_SEND
  #define TRANSMITAPP_SEND_DELAY    (RESPONSE_POLL_RATE * 2)  // in MSecs
#endif

// Send with or without APS ACKs
//#define TRANSMITAPP_TX_OPTIONS    (AF_DISCV_ROUTE | AF_ACK_REQUEST)
#define TRANSMITAPP_TX_OPTIONS    AF_DISCV_ROUTE

#define TRANSMITAPP_INITIAL_MSG_COUNT  2

#define TRANSMITAPP_TRANSMIT_TIME   4  // 4 MS
#define TRANSMITAPP_DISPLAY_TIMER   (2 * 1000)

#if defined ( TRANSMITAPP_FRAGMENTED )
#define TRANSMITAPP_MAX_DATA_LEN    225
#else
#define TRANSMITAPP_MAX_DATA_LEN    102
#endif

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */


// This is the buffer that is sent out as data.
byte TransmitApp_Msg[ TRANSMITAPP_MAX_DATA_LEN ];

// This is the Cluster ID List and should be filled with Application
// specific cluster IDs.
const cId_t TransmitApp_ClusterList[TRANSMITAPP_MAX_CLUSTERS] =
{
  TRANSMITAPP_CLUSTERID_TESTMSG  // MSG Cluster ID
};

const SimpleDescriptionFormat_t TransmitApp_SimpleDesc =
{
  TRANSMITAPP_ENDPOINT,              //  int    Endpoint;
  TRANSMITAPP_PROFID,                //  uint16 AppProfId[2];
  TRANSMITAPP_DEVICEID,              //  uint16 AppDeviceId[2];
  TRANSMITAPP_DEVICE_VERSION,        //  int    AppDevVer:4;
  TRANSMITAPP_FLAGS,                 //  int    AppFlags:4;
  TRANSMITAPP_MAX_CLUSTERS,          //  byte   AppNumInClusters;
  (cId_t *)TransmitApp_ClusterList,  //  byte   *pAppInClusterList;
  TRANSMITAPP_MAX_CLUSTERS,          //  byte   AppNumInClusters;
  (cId_t *)TransmitApp_ClusterList   //  byte   *pAppInClusterList;
};

// This is the Endpoint/Interface description.  It is defined here, but
// filled-in in TransmitApp_Init().  Another way to go would be to fill
// in the structure here and make it a "const" (in code space).  The
// way it's defined in this sample app it is define in RAM.
endPointDesc_t TransmitApp_epDesc;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// Task ID for event processing - received when TransmitApp_Init() is called.
byte TransmitApp_TaskID;

devStates_t TransmitApp_NwkState;

static byte TransmitApp_TransID;  // This is the unique message ID (counter)

afAddrType_t TransmitApp_DstAddr;
afAddrType_t TestDirectSent_DstAddr;
byte TransmitApp_State;

// Shadow of the OSAL system clock used for calculating actual time expired.
static uint32 clkShdw;
// Running total count of test messages recv/sent since beginning current run.
static uint32 rxTotal, txTotal;
// Running count of test messages recv/sent since last display / update - 1 Hz.
static uint32 rxAccum, txAccum;

static byte timerOn;

static byte timesToSend;

uint16 pktCounter;

// Max Data Request Length
uint16 TransmitApp_MaxDataLength;

uint8   NativeAddr=0x01;

#if defined(GTA_DEBUG)
uint8 Test_Mark=0;
uint8 Test_Buf[15];
#endif

#if defined(GTA_WTHSB_R1)

char temp_val[2];
char humi_val[2];
static uint8 readTempHumi(void);
#endif

#if defined(GTA_WMSCB_R1)

#endif

uint8 Flag_ReportData=0;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
void TransmitApp_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg );
void TransmitApp_HandleKeys( byte shift, byte keys );
void TransmitApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
void TransmitApp_ProcessZDOIncomingMsgs( zdoIncomingMsg_t *inMsg );//microdjg
void TransmitApp_SendTheMessage( void );
void TransmitApp_ChangeState( void );

void ReportData(ModBusFrame_t *pRxFrame);
uint16 FramePackage(uint8 *pFrame,uint8 addr,uint8 cmd,uint8 *data,uint16 datalen);
uint16 ReadADC(uint8 channel);
void SendDataRequest(uint8 *pBuf,uint16 len);

/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */
void TransmitApp_DisplayResults( void );

/*********************************************************************
 * @fn      TransmitApp_Init
 *
 * @brief   Initialization function for the Generic App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void TransmitApp_Init( byte task_id )
{
#if !defined ( TRANSMITAPP_FRAGMENTED )
  afDataReqMTU_t mtu;
#endif
  uint16 i;
  
  #if defined(GTA_WTHSB_R1)
  Hal_SHT11_Init();
  s_connectionreset();
  #endif
/*#if defined (HX_18B20_R1)
  P0SEL &= 0x00;
#endif*/
  
  
  initUart(uartRxCB);//microdjg
  osal_nv_item_init(ZCD_NV_NATIVE_ADDRESS,sizeof(NativeAddr),NULL);
  osal_nv_read(ZCD_NV_NATIVE_ADDRESS,0,sizeof(NativeAddr),&NativeAddr);
  
  TransmitApp_TaskID = task_id;
  TransmitApp_NwkState = DEV_INIT;
  TransmitApp_TransID = 0;

  pktCounter = 0;

  TransmitApp_State = TRANSMITAPP_STATE_WAITING;

  // Device hardware initialization can be added here or in main() (Zmain.c).
  // If the hardware is application specific - add it here.
  // If the hardware is other parts of the device add it in main().

  TransmitApp_DstAddr.addrMode = (afAddrMode_t)AddrNotPresent;//°ó¶¨·¢
  TransmitApp_DstAddr.endPoint = 0;
  TransmitApp_DstAddr.addr.shortAddr = 0;//ÎÞËùÎ½

  // Fill out the endpoint description.
  TransmitApp_epDesc.endPoint = TRANSMITAPP_ENDPOINT;
  TransmitApp_epDesc.task_id = &TransmitApp_TaskID;
  TransmitApp_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&TransmitApp_SimpleDesc;
  TransmitApp_epDesc.latencyReq = noLatencyReqs;
  
  
  
  //if(ZG_DEVICE_COORDINATOR_TYPE)
  //{
    //HalLcdInit();
    //HalLcd_HW_WriteLine(HAL_LCD_LINE_1,"GTA Zigbee Test");
    HalLcdWriteString( "GTA Zigbee", HAL_LCD_LINE_1 );
  //}


  // Register the endpoint/interface description with the AF
  afRegister( &TransmitApp_epDesc );

  // Register for all key events - This app will handle all key events
  RegisterForKeys( TransmitApp_TaskID );

  // Update the display
#if defined ( LCD_SUPPORTED )
  HalLcdWriteString( "TransmitApp", HAL_LCD_LINE_2 );
#endif

  // Set the data length
#if defined ( TRANSMITAPP_FRAGMENTED )
  TransmitApp_MaxDataLength = TRANSMITAPP_MAX_DATA_LEN;
#else
  mtu.kvp        = FALSE;
  mtu.aps.secure = FALSE;
  TransmitApp_MaxDataLength = afDataReqMTU( &mtu );
#endif

  // Generate the data
  for (i=0; i<TransmitApp_MaxDataLength; i++)
  {
    TransmitApp_Msg[i] = (uint8) i;
  }

  ZDO_RegisterForZDOMsg( TransmitApp_TaskID, End_Device_Bind_rsp );
  ZDO_RegisterForZDOMsg( TransmitApp_TaskID, Match_Desc_rsp );
  ZDO_RegisterForZDOMsg( TransmitApp_TaskID,TRANSMITAPP_CLUSTERID_TESTMSG);
  
#if defined (HX_18B20_R1)
  osal_start_timerEx( TransmitApp_TaskID, TRANSMITAPP_START_REPORT_EVT, 1000);
  Flag_ReportData = true;
#endif
#if defined GTA_WMSCB_R1
  NativeAddr = 0x22;  //ÉèÖÃ¿ØÖÆÄ£¿éµÄZigBeeµØÖ·
#endif

}

/*********************************************************************
 * @fn      TransmitApp_ProcessEvent
 *
 * @brief   Generic Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  none
 */
UINT16 TransmitApp_ProcessEvent( byte task_id, UINT16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
  afDataConfirm_t *afDataConfirm;
  (void)task_id;  // Intentionally unreferenced parameter

  // Data Confirmation message fields
  ZStatus_t sentStatus;
  byte sentEP;
  

  
  uint8 Frame[20];//ÁÙÊ±Éè¶¨µÄ´óÐ¡
  uint8 data[10];//
  uint16 i=0,j=0;
  uint8 led=0;
  uint16 temp;
  uint16 FrameLen;
  

  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( TransmitApp_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
        case ZDO_CB_MSG:
          TransmitApp_ProcessZDOMsgs( (zdoIncomingMsg_t *)MSGpkt );
          break;

        case KEY_CHANGE:
          TransmitApp_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;

        case AF_DATA_CONFIRM_CMD:
          // This message is received as a confirmation of a data packet sent.
          // The status is of ZStatus_t type [defined in ZComDef.h]
          // The message fields are defined in AF.h
          afDataConfirm = (afDataConfirm_t *)MSGpkt;
          sentEP = afDataConfirm->endpoint;
          sentStatus = afDataConfirm->hdr.status;

          if ( (ZSuccess == sentStatus) &&
               (TransmitApp_epDesc.endPoint == sentEP) )
          {
#if !defined ( TRANSMITAPP_RANDOM_LEN )
            txAccum += TransmitApp_MaxDataLength;
#endif
            if ( !timerOn )
            {
              osal_start_timerEx( TransmitApp_TaskID,TRANSMITAPP_RCVTIMER_EVT,
                                                     TRANSMITAPP_DISPLAY_TIMER);
              clkShdw = osal_GetSystemClock();
              timerOn = TRUE;
            }
          }

          // Action taken when confirmation is received: Send the next message.
          TransmitApp_SetSendEvt();
          break;

        case AF_INCOMING_MSG_CMD:
          TransmitApp_MessageMSGCB( MSGpkt );
          
          break;

        case ZDO_STATE_CHANGE:
          TransmitApp_NwkState = (devStates_t)(MSGpkt->hdr.status);

          break;

        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );

      // Next
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( TransmitApp_TaskID );
    }

    // Squash compiler warnings until values are used.
    (void)sentStatus;
    (void)sentEP;

    // Return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  // Send a message out
  if ( events & TRANSMITAPP_SEND_MSG_EVT )
  {
    if ( TransmitApp_State == TRANSMITAPP_STATE_SENDING )
    {
      TransmitApp_SendTheMessage();
    }

    // Return unprocessed events
    return (events ^ TRANSMITAPP_SEND_MSG_EVT);
  }

  // Timed wait from error
  if ( events & TRANSMITAPP_SEND_ERR_EVT )
  {
    TransmitApp_SetSendEvt();

    // Return unprocessed events
    return (events ^ TRANSMITAPP_SEND_ERR_EVT);
  }

  // Receive timer
  if ( events & TRANSMITAPP_RCVTIMER_EVT )
  {
    // Setup to display the next result
    osal_start_timerEx( TransmitApp_TaskID, TRANSMITAPP_RCVTIMER_EVT,
                                            TRANSMITAPP_DISPLAY_TIMER );
    TransmitApp_DisplayResults();

    return (events ^ TRANSMITAPP_RCVTIMER_EVT);
  }
  if ( events & TRANSMITAPP_BIND_REQUEST_EVT)//microdjg
  {
    SendBindRequest();//×Ô¶¯¿ªÊ¼°ó¶¨ microdjg
    //osal_start_timerEx( TransmitApp_TaskID, TRANSMITAPP_BIND_REQUEST_EVT,3000 );
    return (events ^ TRANSMITAPP_BIND_REQUEST_EVT);
  }
  
  
  if ( events & TRANSMITAPP_START_REPORT_EVT )
  {
    if(Flag_ReportData)
      {
        osal_start_timerEx( TransmitApp_TaskID, TRANSMITAPP_START_REPORT_EVT, 1000);
      }
    HalLedBlink(HAL_LED_1,1,50,300);
    
    
    
    
    #if defined ( GTA_WWSB_R1 )||(GTA_WFSB_R1) //´«¸ÐÆ÷   
    temp=ReadADC(4);//¶ÁµçÑ¹Öµ
    data[0]=1;
    if(temp>3000)
      data[1]=0x01;
    else
      data[1]=0x00;
    //data[1]=LO_UINT16(temp);
    //data[2]=HI_UINT16(temp);
#if defined GTA_WWSB_R1
    NativeAddr = 0x11;//ÓêµÎ´«¸ÐÆ÷µØÖ·
#elif defined GTA_WFSB_R1
    NativeAddr = 0x16;//»ðÑæ´«¸ÐÆ÷µØÖ·
#endif
    FrameLen=FramePackage(Frame,NativeAddr,0x03,data,2);//´ò°ü
    SendDataRequest(Frame,FrameLen);//·¢ËÍ
    #endif
    
    
    #if defined (GTA_WVSB_R1)||(GTA_WIRSB_R1)
    temp=ReadADC(4);
    data[0]=1;
    if(temp>3000)
      data[1]=0x00;
    else
      data[1]=0x01;
#if defined GTA_WVSB_R1
    NativeAddr = 0x13;//Çã½Ç´«¸ÐÆ÷µØÖ·
#elif defined GTA_WIRSB_R1
    NativeAddr = 0x15;//ºìÍâ´«¸ÐÆ÷µØÖ·
#endif
    FrameLen=FramePackage(Frame,NativeAddr,0x03,data,2);//´ò°ü
    SendDataRequest(Frame,FrameLen);//·¢ËÍ
    #endif
    
    #if defined ( GTA_WUSB_R1 )
    IO_DIR_PORT_PIN(0, 6, IO_IN);
    data[0]=1;
    if(IO_SENSOR)
      data[1]=0x01;
    else
      data[1]=0x00;
    FrameLen=FramePackage(Frame,NativeAddr,0x03,data,2);//´ò°ü
    SendDataRequest(Frame,FrameLen);//·¢ËÍ
    #endif
    
    #if defined (GTA_WISB_R1)
    temp=ReadADC(4);//
    data[0]=2;
    data[1]=LO_UINT16(temp);
    data[2]=HI_UINT16(temp);
    NativeAddr = 0x14;//¹âÕÕ´«¸ÐÆ÷µØÖ·
    FrameLen=FramePackage(Frame,NativeAddr,0x03,data,3);//´ò°ü
    SendDataRequest(Frame,FrameLen);//·¢ËÍ
    #endif
    
    
    #if defined ( GTA_WTHSB_R1 ) //ÎÂÊª¶È´«¸ÐÆ÷  
    
    readTempHumi();
    data[0]=4;
    data[1]=humi_val[0];//L
    data[2]=humi_val[1];//H
    data[3]=temp_val[0];
    data[4]=temp_val[1];
    NativeAddr = 0x12;//ÎÂÊª¶È´«¸ÐÆ÷µØÖ·
    FrameLen=FramePackage(Frame,NativeAddr,0x03,data,5);//´ò°ü
    SendDataRequest(Frame,FrameLen);//·¢ËÍ
    HalUARTWrite(HAL_UART_PORT_0, Frame, FrameLen);
    
    #endif
    
    //¿ÉÈ¼ÆøÌåCombustible gas¡¢¾Æ¾«Alcohol¡¢ÑÌÎíSmoke
#if defined ( HX_WCGSB_R1 ) || (HX_ALCOHOL_SENSOR) || (HX_SMOKE_SENSOR)
    IO_DIR_PORT_PIN(0, 1, IO_IN);
    data[0]=1;
    if(IO_COMBUSTIBLE_GAS_PORT)
      data[1] = 1;
    else
      data[1] = 0;
#if defined HX_SMOKE_SENSOR
    NativeAddr = 0x21;//ÑÌÎí´«¸ÐÆ÷µØÖ·
#elif defined HX_ALCOHOL_SENSOR
    NativeAddr = 0x20;//¾Æ¾«´«¸ÐÆ÷µØÖ·
#elif defined HX_WCGSB_R1
    NativeAddr = 0x18;//¿ÉÈ¼ÆøÌå´«¸ÐÆ÷µØÖ·
#endif
    FrameLen=FramePackage(Frame,NativeAddr,0x03,data,2);//´ò°ü
    SendDataRequest(Frame,FrameLen);//·¢ËÍ
#endif
    
#if defined ( HX_18B20_R1 )
    uint8* pTempData;
    read_data();
    pTempData = DataChange();
    data[0]=4;
    data[1]=*pTempData++;//temperature H
    data[2]=*pTempData;//temperature L
    NativeAddr = 0x19;//ÉèÖÃ18b20µÄZigBeeµØÖ·Îª19h
    FrameLen=FramePackage(Frame,NativeAddr,0x03,data,3);//´ò°ü
    SendDataRequest(Frame,FrameLen);//·¢ËÍ
    HalUARTWrite(HAL_UART_PORT_0, Frame, FrameLen);
#endif
    
    
    return (events ^ TRANSMITAPP_START_REPORT_EVT);
  }
  
  

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * Event Generation Functions
 */
/*********************************************************************
 * @fn      TransmitApp_ProcessZDOMsgs()
 *
 * @brief   Process response messages
 *
 * @param   none
 *
 * @return  none
 */
void TransmitApp_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg )
{
  switch ( inMsg->clusterID )
  {
    case End_Device_Bind_rsp:
      if ( ZDO_ParseBindRsp( inMsg ) == ZSuccess )
      {
        // Light LED
        HalLedSet( HAL_LED_4, HAL_LED_MODE_ON );
        osal_stop_timerEx( TransmitApp_TaskID, TRANSMITAPP_BIND_REQUEST_EVT );

      }
#if defined(BLINK_LEDS)
      else
      {
        // Flash LED to show failure
        //HalLedSet ( HAL_LED_2, HAL_LED_MODE_FLASH );//4
        HalLedBlink(HAL_LED_2,1,50,1000);//Ö¸Ê¾°ó¶¨ÓÐÎó
      }
#endif
      break;

    case Match_Desc_rsp:
      {
        ZDO_ActiveEndpointRsp_t *pRsp = ZDO_ParseEPListRsp( inMsg );
        if ( pRsp )
        {
          if ( pRsp->status == ZSuccess && pRsp->cnt )
          {
            TransmitApp_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
            TransmitApp_DstAddr.addr.shortAddr = pRsp->nwkAddr;
            // Take the first endpoint, Can be changed to search through endpoints
            TransmitApp_DstAddr.endPoint = pRsp->epList[0];

            // Light LED
            HalLedSet( HAL_LED_4, HAL_LED_MODE_ON );
          }
          osal_mem_free( pRsp );
        }
      }
      break;
      
  case TRANSMITAPP_CLUSTERID_TESTMSG://microdjg
    {  
      //osal_set_event( TransmitApp_TaskID, AF_INCOMING_MSG_CMD );///
      TransmitApp_ProcessZDOIncomingMsgs(inMsg);
      HalLedBlink(HAL_LED_4,1,50,500);///
    }
  }
}

/*********************************************************************
 * @fn      TransmitApp_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 EVAL_SW4
 *                 EVAL_SW3
 *                 EVAL_SW2
 *                 EVAL_SW1
 *
 * @return  none
 */
void TransmitApp_HandleKeys( byte shift, byte keys )
{
  //zAddrType_t dstAddr;
      
    if ( keys == HAL_KEY_INT_1)
    {
      #if defined(GTA_DEBUG)
      Test_Mark=0x01;
      Test_Buf[0] = 0x99;
      Test_Buf[1] = 0x88;
      Test_Buf[2] = 0x77;
      Test_Buf[3] = 0x66;
      HalUARTWrite(HAL_UART_PORT_0,Test_Buf,15);
      #endif
      Flag_ReportData=!Flag_ReportData;
      HalLedBlink (HAL_LED_1, 1, 50, 100);
      if(Flag_ReportData)
      {
        //osal_start_timerEx( TransmitApp_TaskID, TRANSMITAPP_START_REPORT_EVT, 2000);
        osal_set_event( TransmitApp_TaskID, TRANSMITAPP_START_REPORT_EVT );
      }
    }
 //}
}

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      TransmitApp_MessageMSGCB
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 *
 * @param   none
 *
 * @return  none
 */
void TransmitApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
  uint16 i;
  uint8 error = FALSE;
//HalLedBlink (HAL_LED_2, 1, 50, 300);
//HalUARTWrite(HAL_UART_PORT_0,pkt->cmd.Data,pkt->cmd.DataLength);//microdjg
  switch ( pkt->clusterId )
  {
    case TRANSMITAPP_CLUSTERID_TESTMSG:
      
      //HalUARTWrite(HAL_UART_PORT_0,pkt->cmd.Data,pkt->cmd.DataLength);//microdjg
      //HalLedBlink ( HAL_LED_2, 1, 50, 300 );//Blink to indicate RX
      break;
  default:break;
  }
}

void TransmitApp_ProcessZDOIncomingMsgs( zdoIncomingMsg_t *inMsg )
{
    ModBusFrame_t RxFrame;
    uint16 temp;
    //uint8 recdatabuf[20];
    HalLedBlink (HAL_LED_2, 1, 50, 300);
    uint8 i;
    //#if defined(GTA_DEBUG)
    //  uint8 i;
    //#endif
    
    //HalUARTWrite(HAL_UART_PORT_0,inMsg->asdu,inMsg->asduLen);//²âÊÔ
    
    if(ZG_DEVICE_COORDINATOR_TYPE)
    {
      
      if(inMsg->asduLen!=0&&inMsg->asdu[0]!=0)
      {/*
        for(i=0;i<inMsg->asduLen;i++)
        {
          recdatabuf[i]=inMsg->asdu[i];
        } 
        HalUARTWrite(HAL_UART_PORT_0,recdatabuf,inMsg->asduLen);
        */
        HalUARTWrite(HAL_UART_PORT_0,inMsg->asdu,inMsg->asduLen);
      }
      
      //HalUARTWrite(HAL_UART_PORT_0,recdatabuf,inMsg->asduLen);
      #if defined(GTA_DEBUG)
      if(Test_Mark==0x01)
      {
        for(i=0;i<15;i++)
        {
          Test_Buf[i]=0;
        }
        for(i=0;i<inMsg->asduLen;i++)
        {
          Test_Buf[i]=inMsg->asdu[i];
        }
        Test_Mark=0;
      }
      
      #endif
    }
    else
    {
      FrameUnPackage(inMsg->asdu,&RxFrame);//check Package
      if(RxFrame.COMFIRM==FRAME_CHECK_RIGHT)
      {
        ReportData(&RxFrame);
        
      }
    } 
    
}

/*********************************************************************
 * @fn      TransmitApp_SendTheMessage
 *
 * @brief   Send "the" message.
 *
 * @param   none
 *
 * @return  none
 */
void TransmitApp_SendTheMessage( void )
{
  uint16 len;
  uint8 tmp;

  do {
    // put the sequence number in the message
    tmp = HI_UINT8( TransmitApp_TransID );
    tmp += (tmp <= 9) ? ('0') : ('A' - 0x0A);
    TransmitApp_Msg[2] = tmp;
    tmp = LO_UINT8( TransmitApp_TransID );
    tmp += (tmp <= 9) ? ('0') : ('A' - 0x0A);
    TransmitApp_Msg[3] = tmp;

    len = TransmitApp_MaxDataLength;

#if defined ( TRANSMITAPP_RANDOM_LEN )
    len = (uint8)(osal_rand() & 0x7F);
    if( len > TransmitApp_MaxDataLength || len == 0 )
      len = TransmitApp_MaxDataLength;
    else if ( len < 4 )
      len = 4;
#endif
	
    tmp = AF_DataRequest( &TransmitApp_DstAddr, &TransmitApp_epDesc,
                           TRANSMITAPP_CLUSTERID_TESTMSG,
                           len, TransmitApp_Msg,
                          &TransmitApp_TransID,
                           TRANSMITAPP_TX_OPTIONS,
                           AF_DEFAULT_RADIUS );
    HalLedBlink ( HAL_LED_3, 1, 50, 200 );//Blink to indicate TX  microdjg

#if defined ( TRANSMITAPP_RANDOM_LEN )
    if ( tmp == afStatus_SUCCESS )
    {
      txAccum += len;
    }
#endif

    if ( timesToSend )
    {
      timesToSend--;
    }
  } while ( (timesToSend != 0) && (afStatus_SUCCESS == tmp) );

  if ( afStatus_SUCCESS == tmp )
  {
    pktCounter++;
  }
  else
  {
    // Error, so wait (10 mSec) and try again.
    osal_start_timerEx( TransmitApp_TaskID, TRANSMITAPP_SEND_ERR_EVT, 10 );
  }
}

/*********************************************************************
 * @fn      TransmitApp_ChangeState
 *
 * @brief   Toggle the Sending/Waiting state flag
 *
 * @param   none
 *
 * @return  none
 */
void TransmitApp_ChangeState( void )
{
  if ( TransmitApp_State == TRANSMITAPP_STATE_WAITING )
  {
    TransmitApp_State = TRANSMITAPP_STATE_SENDING;
    TransmitApp_SetSendEvt();
    timesToSend = TRANSMITAPP_INITIAL_MSG_COUNT;
  }
  else
  {
    TransmitApp_State = TRANSMITAPP_STATE_WAITING;
  }
}

/*********************************************************************
 * @fn      TransmitApp_SetSendEvt
 *
 * @brief   Set the event flag
 *
 * @param   none
 *
 * @return  none
 */
void TransmitApp_SetSendEvt( void )
{
#if defined( TRANSMITAPP_DELAY_SEND )
  // Adds a delay to sending the data
  osal_start_timerEx( TransmitApp_TaskID,
                    TRANSMITAPP_SEND_MSG_EVT, TRANSMITAPP_SEND_DELAY );
#else
  // No Delay - just send the data
  osal_set_event( TransmitApp_TaskID, TRANSMITAPP_SEND_MSG_EVT );
#endif
}

/*********************************************************************
 * @fn      TransmitApp_DisplayResults
 *
 * @brief   Display the results and clear the accumulators
 *
 * @param   none
 *
 * @return  none
 */
void TransmitApp_DisplayResults( void )
{
#ifdef LCD_SUPPORTED
  #define LCD_W  16
  uint32 rxShdw, txShdw, tmp;
  byte lcd_buf[LCD_W+1];
  byte idx;
#endif

  // The OSAL timers are not real-time, so calculate the actual time expired.
  uint32 msecs = osal_GetSystemClock() - clkShdw;
  clkShdw = osal_GetSystemClock();

  rxTotal += rxAccum;
  txTotal += txAccum;

#if defined ( LCD_SUPPORTED )
  rxShdw = (rxAccum * 1000 + msecs/2) / msecs;
  txShdw = (txAccum * 1000 + msecs/2) / msecs;

  osal_memset( lcd_buf, ' ', LCD_W );
  lcd_buf[LCD_W] = NULL;

  idx = 4;
  tmp = (rxShdw >= 100000) ? 99999 : rxShdw;
  do
  {
    lcd_buf[idx--] = (uint8) ('0' + (tmp % 10));
    tmp /= 10;
  } while ( tmp );

  idx = LCD_W-1;
  tmp = rxTotal;
  do
  {
    lcd_buf[idx--] = (uint8) ('0' + (tmp % 10));
    tmp /= 10;
  } while ( tmp );

  HalLcdWriteString( (char*)lcd_buf, HAL_LCD_LINE_1 );
  osal_memset( lcd_buf, ' ', LCD_W );

  idx = 4;
  tmp = (txShdw >= 100000) ? 99999 : txShdw;
  do
  {
    lcd_buf[idx--] = (uint8) ('0' + (tmp % 10));
    tmp /= 10;
  } while ( tmp );

  idx = LCD_W-1;
  tmp = txTotal;
  do
  {
    lcd_buf[idx--] = (uint8) ('0' + (tmp % 10));
    tmp /= 10;
  } while ( tmp );

  HalLcdWriteString( (char*)lcd_buf, HAL_LCD_LINE_2 );

#elif defined( MT_TASK )
  DEBUG_INFO( COMPID_APP, SEVERITY_INFORMATION, 3,
              rxAccum, (uint16)msecs, (uint16)rxTotal );
#else
  (void)msecs;  // Not used when no output
#endif

  if ( (rxAccum == 0) && (txAccum == 0) )
  {
    osal_stop_timerEx( TransmitApp_TaskID, TRANSMITAPP_RCVTIMER_EVT );
    timerOn = FALSE;
  }

  rxAccum = txAccum = 0;
}


void initUart(halUARTCBack_t pf)
{
  halUARTCfg_t uartConfig;
  
  uartConfig.configured           = TRUE;              
  uartConfig.baudRate             = HAL_UART_BR_9600;//HAL_UART_BR_38400;  //HAL_UART_BR_19200;
  uartConfig.flowControl          = FALSE;              
  uartConfig.flowControlThreshold = 48;
  uartConfig.rx.maxBufSize        = RX_BUF_LEN;
  uartConfig.tx.maxBufSize        = 128;
  
  uartConfig.idleTimeout          = 6;   
  uartConfig.intEnable            = TRUE;              
  uartConfig.callBackFunc         = pf;
  
  HalUARTOpen (HAL_UART_PORT_0, &uartConfig);
}


void uartRxCB( uint8 port, uint8 event )
{
  uint8 pBuf[RX_BUF_LEN];
  uint16 len;
  static uint16 destAddr;
  ModBusFrame_t *pRxFrame;
  zAddrType_t dstAddr;
  #if defined (GTA_WHFRFID_R1) 
  RC623Frame_t *pRC623Frame;
  uint8 Frame[15];//ÁÙÊ±Éè¶¨µÄ´óÐ¡  
  uint8 data[10];//
  uint16 FrameLen;
  uint8 i;
  #endif
 
    len = HalUARTRead( HAL_UART_PORT_0, pBuf, RX_BUF_LEN );
    if ( len>0 ) 
    {
       // HalUARTWrite(HAL_UART_PORT_0, pBuf, len);
      //SentDataInBind(pBuf,len);
      if(!ZG_DEVICE_COORDINATOR_TYPE)//·ÇÐ­µ÷Æ÷
      {
        #if defined (GTA_WHFRFID_R1) 
        UnPackRC623Data(pBuf,pRC623Frame);
        if(pRC623Frame->Comfirm==FRAME_CHECK_RIGHT)
        {
          switch(pRC623Frame->Cmd)
          {
              case RFID_CMD_SET_ISOTYPE:

                break;
              case RFID_CMD_SELECT_CARD:

                break;
              case RFID_CMD_GET_CARD_ID:
                data[0]=7;
                data[1]=MODBUS_RFID_CMD_GET_CARD_ID&0x00FF;
                data[2]=(MODBUS_RFID_CMD_GET_CARD_ID>>8)&0x00FF;
                data[3]=pRC623Frame->Data[1];
                data[4]=pRC623Frame->Data[2];
                data[5]=pRC623Frame->Data[3];
                data[6]=pRC623Frame->Data[4];
                //data[4]=pRC623Frame->Data[3];
                FrameLen=FramePackage(Frame,NativeAddr,0x03,data,7);//´ò°ü
                SendDataRequest(Frame,FrameLen);//·¢ËÍ
                break;
              case RFID_CMD_FIND_ISO14443_CARD:
                //for(i=0;i<RC623Frame.DataLen;i++)
                //{
                //  RC623Frame.Data
                //}
                data[0]=5;//2+1+2
                data[1]=MODBUS_RFID_CMD_FIND_ISO14443_CARD&0x00FF;
                data[2]=(MODBUS_RFID_CMD_FIND_ISO14443_CARD>>8)&0x00FF;
                data[3]=pRC623Frame->Data[0];
                data[4]=pRC623Frame->Data[1];
                data[5]=pRC623Frame->Data[2];
                data[6]=pRC623Frame->Data[3];
                FrameLen=FramePackage(Frame,NativeAddr,0x03,data,6);//´ò°ü
                SendDataRequest(Frame,FrameLen);//·¢ËÍ

                break;
              case RFID_CMD_CONFIG_KEY:

                break;
              case RFID_CMD_READ_VAL:

                break;
              default:
                break;
          }
        }
        #else
        TestDirectSent_DstAddr.addrMode=(afAddrMode_t)Addr16Bit;//µØÖ·´«ËÍ
        TestDirectSent_DstAddr.endPoint = 0;
        TestDirectSent_DstAddr.addr.shortAddr = 0x0000;
    
        AF_DataRequest( 
                      &TestDirectSent_DstAddr,               //µØÖ·
                      &TransmitApp_epDesc,                //ÃèÊö
                      TRANSMITAPP_CLUSTERID_TESTMSG,      //×å
                        len,                                  //len
                        pBuf,//TransmitApp_Msg,             //DATA
                      &TransmitApp_TransID,               //
                      TRANSMITAPP_TX_OPTIONS,
                      AF_DEFAULT_RADIUS 
                      );
        HalLedBlink (HAL_LED_3, 1, 50, 200);
        #endif
      }
      else
      {
        //destAddr=BUILD_UINT16(pBuf[1],pBuf[0]);
        if(ZG_DEVICE_COORDINATOR_TYPE)
        {
          TestDirectSent_DstAddr.addrMode=(afAddrMode_t)AddrBroadcast;//Addr16Bit;//µØÖ·´«ËÍ
          TestDirectSent_DstAddr.endPoint = 0;
          TestDirectSent_DstAddr.addr.shortAddr = 0xffff;//destAddr;
          //TestDirectSent_DstAddr.addr.extAddr
          
          AF_DataRequest( 
                      &TestDirectSent_DstAddr,               //µØÖ·
                      &TransmitApp_epDesc,                //ÃèÊö
                      TRANSMITAPP_CLUSTERID_TESTMSG,      //×å
                        len,                                  //len
                        pBuf,//TransmitApp_Msg,             //DATA
                      &TransmitApp_TransID,               //
                      TRANSMITAPP_TX_OPTIONS,
                      AF_DEFAULT_RADIUS 
                      );
          HalLedBlink (HAL_LED_3, 1, 50, 200);
        }
      }
    }

}
/*********************************************************************
*********************************************************************/




void FrameUnPackage(uint8 *pBuf,ModBusFrame_t *pFrame)
{
   uint16 crc;
   uint8 datalen;
   uint8 cmd;
   uint8 i;
   
   if(pBuf[0]==FRAME_HEAD)
   {
   	cmd=pBuf[2];
	switch(cmd)
		{
		case FUN_CODE_READ_COIL_STATE:		
		case FUN_CODE_READ_HOLDING_REGISTERS:
			datalen =4;
			break;
		case 0x05:	
		case 0x06:
			datalen =4;
			break;
		case 0x0F:
			datalen =5+((BUILD_UINT16(pBuf[6],pBuf[5])+7)/8);//2+2+1+(N+7)/8
			break;
		case 0x10:
			datalen=5+((BUILD_UINT16(pBuf[6],pBuf[5])*2));//2+2+1+2N
			break;
		case 0x11:
                        datalen = 0;
                        break;
		case 0x12:
			datalen = 1;	//1+ 6;
			break;
		default:pFrame->COMFIRM=FRAME_CHECK_CMD_WRONG;
			return ;
		}
   	crc=BUILD_UINT16(pBuf[3+datalen],pBuf[3+datalen+1]);
     if(crc==Crc16Code(pBuf+1,datalen+2))//pBuf+1  ²»¼ÆHEAD  ³¤¶ÈÊÇ1+1+1-1
     	{
     		pFrame->CMD=cmd;
		pFrame->DATALEN=datalen;
		pFrame->ADDR=pBuf[1];
		for(i=0;i<datalen;i++)
                    {
                    pFrame->pData[i]=pBuf[3+i];
                    }
                pFrame->COMFIRM=FRAME_CHECK_RIGHT;
                return ;//ÍêÈ«ÕýÈ· ²¢°Ñ°ü·µ»Ø
     	}
	 	
   }
   else
   {
   	 pFrame->COMFIRM=FRAME_CHECK_HEAD_WRONG;
	 return ;
   }
     
}



uint16 Crc16Code(uint8 *str, uint8 len)//Modbus Ckeck Code.//LO First
{
    uint16 code = 0xFFFF;
    uint16 Poly = 0xA001;
// Dnp is 0x3D65 or A6BC

    int flag = 0;
    int j = 0;

    for(j=0; j<len; j++)
    {
		code ^= (uint16) (*str);
		for(int i=0; i<8;i++)
		{
			if ( code & 0x0001 ) flag = 1;
			code = code>>1;
			if ( flag )
			{
				code ^= Poly;
				flag = 0;
			}
		}
		str ++;
    }
    return code;
}

void SentDataInBind(uint8 *pdata,uint16 len)
{
  //uint8 data[]="hello";
     AF_DataRequest( 
                    &TransmitApp_DstAddr,               //µØÖ·
                    &TransmitApp_epDesc,                //ÃèÊö
                    TRANSMITAPP_CLUSTERID_TESTMSG,      //×å
                    len,                                  //len
                    pdata,//TransmitApp_Msg,             //DATA
                    &TransmitApp_TransID,               //
                    TRANSMITAPP_TX_OPTIONS,
                    AF_DEFAULT_RADIUS 
                    );
     HalLedBlink (HAL_LED_3, 1, 50, 300);
}

void SendBindRequest(void)
{
    zAddrType_t dstAddr;
    dstAddr.addrMode = Addr16Bit;
    dstAddr.addr.shortAddr = 0x0000; // Coordinator
    ZDP_EndDeviceBindReq( &dstAddr, NLME_GetShortAddr(),
                  TransmitApp_epDesc.endPoint,
                  TRANSMITAPP_PROFID,
                  TRANSMITAPP_MAX_CLUSTERS, (cId_t *)TransmitApp_ClusterList,
                  TRANSMITAPP_MAX_CLUSTERS, (cId_t *)TransmitApp_ClusterList,
                  FALSE );
    HalLedBlink (HAL_LED_1, 1, 50, 300);
}


void ReportData(ModBusFrame_t *pRxFrame)
{
 /* 
  #if defined ( GTA_WWSB_R1 ) //ÓêµÎ´«¸ÐÆ÷   
  uint8 Frame[10];//ÁÙÊ±Éè¶¨µÄ´óÐ¡
  uint8 data[3];//GTA_WWSB_R1 Êý¾ÝÎª3
  #endif
  #if defined ( GTA_WTHSB_R1 ) //ÎÂÊª¶È´«¸ÐÆ÷   
  uint8 Frame[10];//ÁÙÊ±Éè¶¨µÄ´óÐ¡
  uint8 data[5];//GTA_WWSB_R1 Êý¾ÝÎª3
  #endif
*/

  #if defined (GTA_WHFRFID_R1)
  RC623Frame_t pframe;
  uint16 cmd;
  #endif

  uint8 Frame[20];//ÁÙÊ±Éè¶¨µÄ´óÐ¡
  uint8 data[10];//
  uint16 i=0,j=0;
  uint8 led=0;

  
  uint16 temp;
  uint16 FrameLen;
  switch(pRxFrame->CMD)
      {
      case FUN_CODE_READ_HOLDING_REGISTERS://¶Á¼Ä´æÆ÷Êý¾Ý
        {
          //temp=BUILD_UINT16(pRxFrame->pData[1],pRxFrame->pData[0]);
          //if(temp==NativeAddr)//1 2×Ö½ÚÊÇµØÖ·£¬ÊÇ·ñ±¾»úµØÖ·
          if(pRxFrame->ADDR==NativeAddr)//ÊÇ·ñ±¾»úµØÖ·
          {
              #if defined ( GTA_WWSB_R1 )||(GTA_WFSB_R1) //´«¸ÐÆ÷   
              temp=ReadADC(4);//¶ÁÓêµÎµçÑ¹Öµ,»ðÑæ´«¸ÐÆ÷
              data[0]=1;
              if(temp>3000)
                data[1]=0x01;
              else
                data[1]=0x00;
              //data[1]=LO_UINT16(temp);
              //data[2]=HI_UINT16(temp);
              FrameLen=FramePackage(Frame,NativeAddr,0x03,data,2);//´ò°ü
              SendDataRequest(Frame,FrameLen);//·¢ËÍ
              #endif
              
              #if defined ( GTA_WUSB_R1 )
              IO_DIR_PORT_PIN(0, 6, IO_IN);
              data[0]=1;
              if(IO_SENSOR)
                data[1]=0x01;
              else
                data[1]=0x00;
              FrameLen=FramePackage(Frame,NativeAddr,0x03,data,2);//´ò°ü
              SendDataRequest(Frame,FrameLen);//·¢ËÍ
              #endif

              #if defined(GTA_WRCB_R1)
              IO_DIR_PORT_PIN(0, 6, IO_OUT);
              IO_DIR_PORT_PIN(0, 0, IO_OUT);
              if(pRxFrame->pData[0]==0x01)
                {
                  if(pRxFrame->pData[1]==0x00)
                    IO_RELAY1=0;
                  if(pRxFrame->pData[1]==0x01)
                    IO_RELAY1=1;
                }
              if(pRxFrame->pData[0]==0x02)
                {
                  if(pRxFrame->pData[1]==0x00)
                    IO_RELAY2=0;
                  if(pRxFrame->pData[1]==0x01)
                    IO_RELAY2=1;
                }
              #endif
              
              
              #if defined (GTA_WVSB_R1)||(GTA_WIRSB_R1)
              temp=ReadADC(4);//ºìÍâ´«¸ÐÆ÷
              data[0]=1;
              if(temp>3000)
                data[1]=0x00;
              else
                data[1]=0x01;
              FrameLen=FramePackage(Frame,NativeAddr,0x03,data,2);//´ò°ü
              SendDataRequest(Frame,FrameLen);//·¢ËÍ
              #endif
              
              #if defined (GTA_WISB_R1)
              temp=ReadADC(4);//
              data[0]=2;
              data[1]=LO_UINT16(temp);
              data[2]=HI_UINT16(temp);
              FrameLen=FramePackage(Frame,pRxFrame->ADDR,pRxFrame->CMD,data,3);//´ò°ü
              SendDataRequest(Frame,FrameLen);//·¢ËÍ
              #endif
              
              
              #if defined ( GTA_WTHSB_R1 ) //ÎÂÊª¶È´«¸ÐÆ÷  
              osal_int_disable( INTS_ALL );
              readTempHumi();
              osal_int_enable( INTS_ALL );
              data[0]=4;
              data[1]=humi_val[0];//L
              data[2]=humi_val[1];//H
              data[3]=temp_val[0];
              data[4]=temp_val[1];
              FrameLen=FramePackage(Frame,pRxFrame->ADDR,pRxFrame->CMD,data,5);//´ò°ü
              SendDataRequest(Frame,FrameLen);//·¢ËÍ
              #endif
              
              #if defined (GTA_WHFRFID_R1)
              cmd=BUILD_UINT16(pRxFrame->pData[0],pRxFrame->pData[1]);
                switch(cmd)
                {
                case MODBUS_RFID_CMD_FIND_ISO14443_CARD://7E xx 03 01 0A 00 00 xx
                  FrameLen=PackUpRC623Data(RFID_CMD_FIND_ISO14443_CARD,&(pRxFrame->pData[2]),Frame);//´ò°ü
                   HalUARTWrite(HAL_UART_PORT_0,Frame,FrameLen);
                  //SendDataRequest(Frame,FrameLen);
                  break;
                case MODBUS_RFID_CMD_GET_CARD_ID:
                  FrameLen=PackUpRC623Data(RFID_CMD_GET_CARD_ID,&(pRxFrame->pData[2]),Frame);//´ò°ü
                   HalUARTWrite(HAL_UART_PORT_0,Frame,FrameLen);
                  break;
                case MODBUS_RFID_CMD_READ_VAL:
                  
                  break;
                default:
                  break;
                }
              
              #endif
                
              #if defined(GTA_WMSCB_R1)
               /*
              ctrl_GTA_WMSCB('M',0x80);
              for(j=0;j<1000;j++)
              {
               
               ctrl_GTA_WMSCB('A',led);
               ctrl_GTA_WMSCB('B',led++);
              StepMotorRun(1,4096);
              StepMotorRun(0,4096);
              }
                */
                /*init_GTA_WMSCB();
                ctrl_GTA_WMSCB('M',0x80);
                if(pRxFrame->pData[0]==0x00)
                {
                  //ctrl_GTA_WMSCB('M',0x80);
                  osal_int_disable( INTS_ALL );
                  StepMotorRun(pRxFrame->pData[1],BUILD_UINT16(pRxFrame->pData[2],pRxFrame->pData[3]));
                  osal_int_enable( INTS_ALL );
                }
                if(pRxFrame->pData[0]==0x01)
                {
                  //ctrl_GTA_WMSCB('M',0x80);
                  ctrl_GTA_WMSCB('A',pRxFrame->pData[1]);
                }
                if(pRxFrame->pData[0]==0x02)
                {

                  SegDisplay(pRxFrame->pData[1]);
                }*/
                if(pRxFrame->pData[0] == 0x00){  //ÅÐ¶ÏÊÇµç»úµÄ¿ØÖÆÊý¾Ý
                    ControlStepMotor(pRxFrame->pData[1]);
                } else if(pRxFrame->pData[0] == 0x02){//ÅÐ¶ÏÊÇÊýÂë¹ÜµÄ¿ØÖÆÊý¾Ý
                    DisplaySmg(pRxFrame->pData[1]);
                } else if(pRxFrame->pData[0] == 0x01){//ÅÐ¶ÏÊÇ·äÃùÆ÷
                    ControlBeep(pRxFrame->pData[1]);
                } else if(pRxFrame->pData[0] == 0x03){//ÅÐ¶ÏÊÇ¼ÌµçÆ÷
                    ControlRelay(pRxFrame->pData[1]);
                }
              #endif
              
          }
        }
        break;
      case FUN_CODE_SET_ADDR://ÉèµØÖ·
        {
          if(pRxFrame->ADDR==NativeAddr)//ÊÇ·ñ±¾»úµØÖ·
          {
          NativeAddr=pRxFrame->pData[0];
          osal_nv_item_init(ZCD_NV_NATIVE_ADDRESS,sizeof(NativeAddr),NULL);
          osal_nv_write(ZCD_NV_NATIVE_ADDRESS,0,sizeof(NativeAddr),&NativeAddr);
          FrameLen=FramePackage(Frame,pRxFrame->ADDR,pRxFrame->CMD,pRxFrame->pData,1);//´ò°ü
          SendDataRequest(Frame,FrameLen);//·¢ËÍ
          }
        }
        break;
      case FUN_CODE_READ_ADDR://¶ÁµØÖ·
        {
          FrameLen=FramePackage(Frame,pRxFrame->ADDR,pRxFrame->CMD,&NativeAddr,1);//´ò°ü
          SendDataRequest(Frame,FrameLen);//·¢ËÍ
          
        }
        break;
      }
  
  

}
#if defined(GTA_WMSCB_R1)
void StepMotorRun(uint8 Dir,uint16 StepNum)
{
  uint8 ForwardStep[4]={0x03,0x09,0x0C,0x06};
  uint8 BackStep[4]={0x03,0x06,0x0C,0x09};
  uint16 i=0,j=0,k=0;
  uint8 index=0,l=0;
  for(i=0;i<StepNum;i++)
  {
    if(Dir)
      ctrl_GTA_WMSCB('C',ForwardStep[index]);
    else
      ctrl_GTA_WMSCB('C',BackStep[index]);
    if(i==(k*255-1)||i==0)
    {
      SegDisplay(l);
      if(l++>8)l=0;
      k++;
    }
    ctrl_GTA_WMSCB('A',i);
    
    if(++index>=4)index=0;
    
    for(j=0;j<3000;j++)
    {
      asm("nop");            
    }
    //ctrl_GTA_WMSCB('A',StepNum);
  }
}

void SegDisplay(uint8 num)
{
  uint8 DispNum[10]={0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F};
  //ctrl_GTA_WMSCB('M',0x80);
  if(num<0x10)
    ctrl_GTA_WMSCB('B',DispNum[num]);
}
#endif
uint16 FramePackage(uint8 *pFrame,uint8 addr,uint8 cmd,uint8 *data,uint16 datalen)
{
  uint16 i;
  uint16 crc;
  uint8 *p;
    p=pFrame;
  *p++=FRAME_HEAD;//²»Ã÷bug ÔÙ¼ÓÒ»¸ö×Ö½Ú
  *p++=FRAME_HEAD;
  *p++=addr;
  *p++=cmd;
  for(i=0;i<datalen;i++)
  {
    *p++=*data++;
  }
  //crc=Crc16Code(pFrame+1,datalen+2);//FRAME_HEAD²»¼Æ
   crc=Crc16Code(pFrame+2,datalen+2);//FRAME_HEAD²»¼Æ
  *p++=LO_UINT16(crc);
  *p=HI_UINT16(crc);
  //return datalen+5;
  return datalen+6;//¶à¼ÓÒ»¸öFRAME_HEAD
}

  /*********************************************************************
 * º¯ÊýÃû³Æ£ºReadADC
 * ¹¦    ÄÜ£º¶ÁÈ¡ADCÖµ
 * Èë¿Ú²ÎÊý£ºÍ¨µÀ
 * ³ö¿Ú²ÎÊý£ºÎÞ
 * ·µ »Ø Öµ£ºADÊý¾Ý()
 ********************************************************************/
uint16 ReadADC( uint8 channel )
{
  volatile unsigned char tmp,n;
  signed short adcvalue;
  float voltagevalue_X;

  /* XÖá¼ÓËÙ¶È²É¼¯ */
  /* ÉèÖÃ»ù×¼µçÑ¹¡¢³éÈ¡ÂÊºÍµ¥¶ËÊäÈëÍ¨µÀ */
  ADCCON3 = ((0x02 << 6) |  // ²ÉÓÃAVDD5Òý½ÅÉÏµÄµçÑ¹Îª»ù×¼µçÑ¹
             (0x03 << 4) |  // ³éÈ¡ÂÊÎª512£¬ÏàÓ¦µÄÓÐÐ§Î»Îª12Î»(×î¸ßÎ»Îª·ûºÅÎ»)
             channel);

  /* µÈ´ýÍ¨µÀ×ª»»Íê³É */
  while ((ADCCON1 & 0x80) != 0x80);

  /* ´ÓADCL£¬ADCH¶ÁÈ¡×ª»»Öµ£¬´Ë²Ù×÷»¹ÇåÁãADCCON1.EOC */
  adcvalue = (signed short)ADCL;
  adcvalue |= (signed short)(ADCH << 8); 

  /* ÈôadcvalueÐ¡ÓÚ0£¬¾ÍÈÏÎªËüÎª0 */
  if(adcvalue < 0) adcvalue = 0;
    
  adcvalue >>= 4;  // È¡³ö12Î»ÓÐÐ§Î»
    
  /* ½«×ª»»Öµ»»ËãÎªÊµ¼ÊµçÑ¹Öµ */
  voltagevalue_X = (adcvalue * 3.3) / 2047;  // 2047ÊÇÄ£ÄâÊäÈë´ïµ½VREFÊ±µÄÂúÁ¿³ÌÖµ
                                             // ÓÉÓÚÓÐÐ§Î»ÊÇ12Î»(×î¸ßÎ»Îª·ûºÅÎ»)£¬
                                             // ËùÒÔÕýµÄÂúÁ¿³ÌÖµÎª2047
                                             // ´Ë´¦£¬VREF = 3.3V
  return ((uint16)(voltagevalue_X * 1000));  // µçÑ¹·Å´ó100±¶±ãÓÚ´«Êä¼°ÉÏÎ»»ú´¦Àí
}
void SendDataRequest(uint8 *pBuf,uint16 len)
{
    if(!ZG_DEVICE_COORDINATOR_TYPE)//·ÇÐ­µ÷Æ÷
      {
        TestDirectSent_DstAddr.addrMode=(afAddrMode_t)Addr16Bit;//µØÖ·´«ËÍ
        TestDirectSent_DstAddr.endPoint = 0;
        TestDirectSent_DstAddr.addr.shortAddr = 0x0000;
      }
      else
      {
        if(ZG_DEVICE_COORDINATOR_TYPE)
        {
          TestDirectSent_DstAddr.addrMode=(afAddrMode_t)AddrBroadcast;//Addr16Bit;//µØÖ·´«ËÍ
          TestDirectSent_DstAddr.endPoint = 0;
          TestDirectSent_DstAddr.addr.shortAddr = 0xffff;//destAddr;
          //TestDirectSent_DstAddr.addr.extAddr
        }
      }
     AF_DataRequest(    
              &TestDirectSent_DstAddr,               //µØÖ·
              &TransmitApp_epDesc,                //ÃèÊö
              TRANSMITAPP_CLUSTERID_TESTMSG,      //×å
                len,                                  //len
                pBuf,//TransmitApp_Msg,             //DATA
              &TransmitApp_TransID,               //
              TRANSMITAPP_TX_OPTIONS,
              AF_DEFAULT_RADIUS 
              );
     HalLedBlink (HAL_LED_3, 1, 50, 200);
}

#if defined(GTA_WTHSB_R1)
uint8 readTempHumi(void)
{
 float f_humi=0,f_temp=0; 
 
  uint8 checksum;
 
  /* ¿ØÖÆSHT11½øÐÐÎÂÊª¶È²âÁ¿ */
  
  
  s_measure(temp_val,&checksum,TEMP); // ²âÁ¿ÎÂ¶È
  s_measure(humi_val,&checksum,HUMI); // ²âÁ¿Ïà¶ÔÊª¶È
  f_humi=BUILD_UINT16(humi_val[0],humi_val[1]);
  f_temp=BUILD_UINT16(temp_val[0],temp_val[1]);
  calc_sth11(&f_humi,&f_temp);
  humi_val[0]=f_humi;//&(uint8)f_humi;
  humi_val[1]=(f_humi-humi_val[0])*100;  
  temp_val[0]=f_temp; 
  temp_val[1]=(f_temp-temp_val[0])*100;  
  return TRUE;
}
#endif

#if defined(GTA_WMSCB_R1)
void init_GTA_WMSCB(void)
{
  P0SEL=P0SEL&0x8C;//10001100
  P1SEL=P1SEL&0xF0;
  P2SEL=P2SEL&0xFA;
  //HAL_CONFIG_IO_OUTPUT(0, HAL_LCD_CS_PIN, 1);
  IO_DIR_PORT_PIN(0, 0, IO_OUT);
  IO_DIR_PORT_PIN(0, 1, IO_OUT);
  IO_DIR_PORT_PIN(0, 4, IO_OUT);
  IO_DIR_PORT_PIN(0, 5, IO_OUT);
  IO_DIR_PORT_PIN(0, 6, IO_OUT);
  IO_DIR_PORT_PIN(1, 0, IO_OUT);
  IO_DIR_PORT_PIN(1, 1, IO_OUT);
  IO_DIR_PORT_PIN(1, 2, IO_OUT);
  IO_DIR_PORT_PIN(1, 3, IO_OUT);
  IO_DIR_PORT_PIN(2, 0, IO_OUT);
  IO_DIR_PORT_PIN(2, 2, IO_OUT);
}

void ctrl_GTA_WMSCB(uint8 port,uint8 data)
{
  //uint8 i=0;
  //init_GTA_WMSCB();
  asm("nop"); 
  asm("nop"); 

  T82C55A_WR=1;
  asm("nop"); 
  asm("nop"); 

  switch(port)
  {
  case        'A':
    T82C55A_ADDR0=0;
    T82C55A_ADDR1=0;
    break;
  case        'B':
    T82C55A_ADDR0=1;
    T82C55A_ADDR1=0;
    break;  
  case        'C':
    T82C55A_ADDR0=0;
    T82C55A_ADDR1=1;
    break;
  case        'M':
    T82C55A_ADDR0=1;
    T82C55A_ADDR1=1;
    break;
  default:break;
  }
  asm("nop"); 
  asm("nop"); 

  T82C55A_WR=0;
  asm("nop"); 
  asm("nop"); 

  if(data&0x01)
    T82C55A_D0=1;
  else
    T82C55A_D0=0;
  
  if(data&0x02)
    T82C55A_D1=1;
  else
    T82C55A_D1=0;
  
  if(data&0x04)
    T82C55A_D2=1;
  else
    T82C55A_D2=0;
  
  if(data&0x08)
    T82C55A_D3=1;
  else
    T82C55A_D3=0;
  
  if(data&0x10)
    T82C55A_D4=1;
  else
    T82C55A_D4=0;
  
  if(data&0x20)
    T82C55A_D5=1;
  else
    T82C55A_D5=0;
  
  if(data&0x40)
    T82C55A_D6=1;
  else
    T82C55A_D6=0;
  
  if(data&0x80)
    T82C55A_D7=1;
  else
    T82C55A_D7=0;
  asm("nop"); 
  asm("nop"); 

  T82C55A_WR=1;
  asm("nop"); 
  asm("nop"); 

  T82C55A_WR=0;
  
}

#endif