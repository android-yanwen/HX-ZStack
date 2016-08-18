#ifndef CONTROLMODULE_H
#define CONTROLMODULE_H
#include "hal_board_cfg.h"
#include "hal_types.h"
#include "OnBoard.h"

#define RCLK    P0_4
#define SCLK    P0_1
#define DI      P0_0
#define EN      P1_2
#define MOTOR_FOREWARD  0x00
#define MOTOR_REVERSAL  0xff

extern void HC595IOInit(void);
extern void HC595StoreData(uint8 val);
extern void HC595SendData(void);
extern void DisplaySmg(uint8 num);
extern void ControlStepMotor(uint8 flg);
extern void ControlBeep(uint8 flg);
extern void ControlRelay(uint8 flg);
extern void ControlLeds(uint8 num);
#endif