#ifndef __cmd_H__
#define __cmd_H__

#ifdef __cplusplus
 extern "C" {
#endif

#include "main.h"
#include "cmsis_os.h"
#include "ringbuff/ringbuff.h"


extern osThreadId cmdTaskHandle;
extern ringbuff_t uartrxbuff,uarttxbuff;


#define MotionUP   0x01
#define MotionSTOP 0x00
#define MotionDOWN 0x02

/* 发送命令 */
#define Send_NiujvbothADC  0x01
#define Send_MotoSpeedNow  0x02
#define Send_StepperDisNow 0x03

/* 接收命令 */
#define Rec_StepperDis    0x01
#define Rec_StepperSpeed  0x02
#define Rec_StepperAccel  0x03
#define Rec_StepperDecel  0x04
#define Rec_StepperState  0x05
#define Rec_MotoSpeed     0x06
#define Rec_MotoState     0x07
#define Rec_MotoPI1        0x08  // P  I 参数
#define Rec_MotoD1_Set     0x09  // D 和 PID更新标志
#define Rec_MotoPI2       0x0a  // P  I 参数
#define Rec_MotoD2_Set    0x0b  // D 和 PID更新标志
#define Rec_Motion        0x0c

#define Rec_MCUReset     0xFE  // MCU复位
#define Rec_ConTest     0xFF

void cmd_init(void);

#ifdef __cplusplus
}
#endif

#endif
