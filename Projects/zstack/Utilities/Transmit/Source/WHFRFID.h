#ifndef WHFRFID_H
#define WHFRFID_H

#define DEV_DESC_H      0
#define DEV_DESC_L      0

#define RFID_CMD_SET_ISOTYPE            0x0108
#define RFID_CMD_SELECT_CARD            0x0203
#define RFID_CMD_GET_CARD_ID            0x0202
#define RFID_CMD_FIND_ISO14443_CARD     0x0201
#define RFID_CMD_CONFIG_KEY             0x0207
#define RFID_CMD_READ_VAL               0X020B


#define FRAME_CHECK_RIGHT				        0x01
#define FRAME_CHECK_HEAD_WRONG					0x02
#define FRAME_CHECK_CMD_WRONG					0x03
#define FRAME_CHECK_CRC_WRONG					0x04

#define MODBUS_RFID_CMD_FIND_ISO14443_CARD      0x0A01
#define MODBUS_RFID_CMD_GET_CARD_ID             0x0A02
#define MODBUS_RFID_CMD_READ_VAL                0x0A03

typedef struct
{
  unsigned int Cmd;
  unsigned int DataLen;
  unsigned char Data[20];
  unsigned char Comfirm;
}RC623Frame_t;

unsigned int PackUpRC623Data(unsigned int cmd,unsigned char *pdata,unsigned char *pframe);
//RC623Frame_t UnPackRC623Data(unsigned char *pbuf);
void UnPackRC623Data(unsigned char *pbuf,RC623Frame_t *pframe);
#endif
