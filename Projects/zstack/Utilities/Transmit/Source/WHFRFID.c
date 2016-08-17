
#if defined (GTA_WHFRFID_R1)
#include "WHFRFID.h"
//#include "hal_defs.h"
#include "ZComDef.h"
unsigned int PackUpRC623Data(unsigned int cmd,unsigned char *pdata,unsigned char *pframe)    
{
  unsigned int datalen=0;
  unsigned char i=0;
  unsigned char crc=0;
  pframe[0]=0xAA;
  pframe[1]=0xBB;
  pframe[4]=DEV_DESC_H;
  crc^=pframe[4];
  pframe[5]=DEV_DESC_L;
  crc^=pframe[5];
  pframe[6]=LO_UINT16(cmd);
  crc^=pframe[6];
  pframe[7]=HI_UINT16(cmd);
  crc^=pframe[7];
  switch(cmd)
  {
  case RFID_CMD_SET_ISOTYPE:
    datalen=1;
    break;
  case RFID_CMD_SELECT_CARD:
    datalen=4;
    break;
  case RFID_CMD_GET_CARD_ID:
    datalen=1;
    break;
  case RFID_CMD_FIND_ISO14443_CARD:
    datalen=1;
    break;
  case RFID_CMD_CONFIG_KEY:
    datalen=8;
    break;
  case RFID_CMD_READ_VAL:
    datalen=1;
    break;
  default:
    return -1;
  }
  pframe[2]=LO_UINT16(datalen+5);
  pframe[3]=HI_UINT16(datalen+5);//长度
  for(i=0;i<datalen;i++)
  {
   pframe[8+i]= pdata[i];
   crc^=pframe[8+i];
  }//数据
  pframe[8+datalen]=crc;//校验
  return datalen+9;
  
}

void UnPackRC623Data(unsigned char *pbuf,RC623Frame_t *pframe)
{
  unsigned char crc=0;
  unsigned char i=0;
  unsigned int len=0;
  //RC623Frame_t pframe;
  //AA BB 06 00 00 00 08 01 41 48
  if(pbuf[0]==0xAA)
    if(pbuf[1]==0xBB)
    {
      len=BUILD_UINT16(pbuf[2],pbuf[3]);
      for(i=4;i<(4+len-1);i++)//长度算上了CRC 减去1个
        crc^=pbuf[i];
      if(crc==pbuf[4+len-1])
      {
        pframe->Comfirm=FRAME_CHECK_RIGHT;
        pframe->Cmd=BUILD_UINT16(pbuf[6],pbuf[7]);
        pframe->DataLen=len-5;//-cmd-desc-crc
        for(i=0;i<pframe->DataLen;i++)
        {
          pframe->Data[i]=pbuf[8+i];
        }
      }
      else
        pframe->Comfirm=FRAME_CHECK_CRC_WRONG;
    }
  else
    pframe->Comfirm=FRAME_CHECK_HEAD_WRONG;
  //return pframe;
}




              
#endif