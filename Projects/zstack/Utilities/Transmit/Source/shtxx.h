#ifndef SHT_H
#define SHT_H

/***********************************************************************
 * INCLUDES
 */
#include "ZComDef.h"
#include "iocc2530.h"

/***********************************************************************
 * CONSTANTS
 */
#define SHT_NOACK       0
#define SHT_ACK         1

#define SHT_MODE_TEMP   0
#define SHT_MODE_HUMI   1

//adr  command  r/w
#define SHT_STATUS_REG_W        0x06   //000   0011    0
#define SHT_STATUS_REG_R        0x07   //000   0011    1
#define SHT_MEASURE_TEMP        0x03   //000   0001    1
#define SHT_MEASURE_HUMI        0x05   //000   0010    1
#define SHT_RESET               0x1e

/***********************************************************************
 * MACROS
 */

#define SHT_DATA            P0_1
#define SHT_DATA_OUT \
  { \
    P0DIR |= 0x02; /* P0DIR | 0000,0010 */ \
  }
#define SHT_DATA_IN \
  { \
    P0DIR &= 0xfd; /* P0DIR & 1111,1101*/ \
  }

#define SHT_SCK             P1_7
#define SHT_SCK_OUT \
  { \
    P1DIR |= 0x80; /* P1DIR | 1000,0000*/ \
  }
#define SHT_SCK_IN \
  { \
    P1DIR &= 0x7f; /* P1DIR & 0111,1111*/ \
  }

#endif
