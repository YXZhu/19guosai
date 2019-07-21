#ifndef __stepper_H__
#define __stepper_H__

#ifdef __cplusplus
 extern "C" {
#endif

#include "main.h"

typedef struct{
	uint32_t Run;  // 1 开始 0 停止
	uint32_t Cycle;  //设置脉冲频率 Hz
	uint32_t Cycle_old;  //上一次设置的脉冲频率 Hz
	uint32_t Dir;  // 方向
	uint32_t PulseNum; // 设置脉冲数量 绝对数量 初始为0 控制电机正反转
	uint32_t PulseNum_old; // 上一次设置的脉冲数量
	uint32_t PulseNum_now; // 当前已发送脉冲数量
	int32_t PulseCount;
	uint32_t PWMChannel; // PWM 通道
	uint32_t CycleMax; // 最大脉冲频率 Hz
	uint32_t CycleMin; // 最小脉冲频率 Hz
	void (*SetDirPin)(uint8_t);
	void (*SetENPin)(uint8_t);
}stepper_t;	

extern stepper_t stepp_1;

//uint32_t st_init(stepper_t *st);
void st_user_init(void);
void st_update(stepper_t *st);
#ifdef __cplusplus
}
#endif

#endif
