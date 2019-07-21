#ifndef __stepper_H__
#define __stepper_H__

#ifdef __cplusplus
 extern "C" {
#endif

#include "main.h"

typedef struct{
	uint32_t Run;  // 1 ��ʼ 0 ֹͣ
	uint32_t Cycle;  //��������Ƶ�� Hz
	uint32_t Cycle_old;  //��һ�����õ�����Ƶ�� Hz
	uint32_t Dir;  // ����
	uint32_t PulseNum; // ������������ �������� ��ʼΪ0 ���Ƶ������ת
	uint32_t PulseNum_old; // ��һ�����õ���������
	uint32_t PulseNum_now; // ��ǰ�ѷ�����������
	int32_t PulseCount;
	uint32_t PWMChannel; // PWM ͨ��
	uint32_t CycleMax; // �������Ƶ�� Hz
	uint32_t CycleMin; // ��С����Ƶ�� Hz
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
