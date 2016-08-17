#ifndef DS18B20_H
#define DS18B20_h

#include "ioCC2530.h"
#include "OnBoard.h"
#include "hal_types.h"

#define SET_OUT    IO_DIR_PORT_PIN(0, 1, IO_OUT)
#define SET_IN     IO_DIR_PORT_PIN(0, 1, IO_IN)
#define CL_DQ      P0_1=0
#define SET_DQ     P0_1=1
#define IN_DQ      P0_1

extern void Delay_nus( uint16 s );
extern void write_1820 ( unsigned char x );
extern unsigned char read_1820(void);
extern bool init_1820(void);
extern void read_data(void);
extern uint8* DataChange(void);


#endif


