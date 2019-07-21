#include "stepper.h"
#include "tim.h"

/* ��֪���� �ı�Ƶ����������һ */
stepper_t stepp_1;
#define STEPPER_TIMER (&htim9)

void st_start(stepper_t *st);
void st_stop(stepper_t *st);
void st_stopN(stepper_t *st);
uint32_t st_init(stepper_t *st);

void st1_setDir(uint8_t State)
{
	if(State == 1)
	{
		HAL_GPIO_WritePin(StepMoto_1_Dir_GPIO_Port,StepMoto_1_Dir_Pin,GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(StepMoto_1_Dir_GPIO_Port,StepMoto_1_Dir_Pin,GPIO_PIN_RESET);
	}
}
void st1_setEN(uint8_t State)
{
	if(State == 1)
	{
		HAL_GPIO_WritePin(StepMoto_1_Dir_GPIO_Port,StepMoto_1_Dir_Pin,GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(StepMoto_1_Dir_GPIO_Port,StepMoto_1_Dir_Pin,GPIO_PIN_RESET);
	}
}
/* �û������ʼ�� */
void st_user_init()
{
	stepp_1.Dir = 0;
	stepp_1.Cycle = 300;
	stepp_1.PulseNum = 0;
	stepp_1.PWMChannel = TIM_CHANNEL_1;
	stepp_1.CycleMax = 50000;
	stepp_1.CycleMin = 10000;
	stepp_1.SetDirPin = st1_setDir;
	stepp_1.SetENPin = st1_setEN;
	st_init(&stepp_1);
}

/**********************���²����޸�*****************************/

void st_set_freq(stepper_t *st)
{		
	tim_setfreq(STEPPER_TIMER,st->Cycle);	
	if(st->Run == 1) 
		st_start(st);
	else
		st_stop(st);
}

void st_stopN(stepper_t *st)
{
	__HAL_TIM_SetCompare(STEPPER_TIMER,st->PWMChannel,0);
}

void st_stop(stepper_t *st)
{
	__HAL_TIM_SetCompare(STEPPER_TIMER,st->PWMChannel,65535);
}
/* ����PWM��� */ 
void st_start(stepper_t *st)
{
	//__HAL_TIM_SetCounter(STEPPER_TIMER,0);
	__HAL_TIM_SetCompare(STEPPER_TIMER,st->PWMChannel,__HAL_TIM_GET_AUTORELOAD(STEPPER_TIMER)/2 + 1);
}

uint32_t st_init(stepper_t *st)
{	
	st->PulseNum_now = 0;
	st->PulseNum_old = 0;
	st->Cycle_old = 0;
	st->SetENPin(0);
	
	HAL_TIM_Base_Start_IT(STEPPER_TIMER);
	HAL_TIM_PWM_Start(STEPPER_TIMER,st->PWMChannel);
	
	return 0;
}

/* ��ȡ���������������� */
uint32_t st_get_pulse_num(stepper_t *st)
{
	return st->PulseNum;
}

/* ���ò���������� */
void st_set_dir(stepper_t *st,uint32_t dir)
{
	st->Dir = dir;
	st->SetDirPin(dir);
}

/* ����������� */ 
void st_speedup(stepper_t *st)
{
	if(st->Cycle < st->CycleMax)
	{
		st->Cycle ++;
	}
	else
	{
		st->Cycle = st->CycleMax;
	}
	
}

/* ����������� */ 
void st_speeddown(stepper_t *st)
{
	if(st->Cycle > st->CycleMin)
	{
		st->Cycle --;
	}
	else
	{
		st->Cycle = st->CycleMin;
	}
	
}
/* ������º��� ����һ�����ڽ������ж� */
void st_update(stepper_t *st)
{
	if(st->Cycle !=0)
	{
		/* ��������Ƶ�� */
		if(st->Cycle !=st->Cycle_old)
		{			
			st_set_freq(st);	
			st->Cycle_old = st->Cycle;	
			return; // ֱ���˳������� 	
		}
		
		/* ������������ */
		if(st->PulseNum != st->PulseNum_old)
		{	
			if(st->PulseNum_now > st->PulseNum)
				st_set_dir(st,0);
			else st_set_dir(st,1);
//			if(st->PulseNum > st->PulseNum_old) st_set_dir(st,1);
//			else st_set_dir(st,0);
			st_start(st);
			st->Run = 1;
			st->PulseCount = st->PulseNum - st->PulseNum_now;
			st->PulseNum_old = st->PulseNum;
			//return; // ֱ���˳������� 
		}
		if(st->PulseNum_now == (st->PulseNum))
		{		
			st_stop(st);
			st->Cycle = st->CycleMin;
			st->Run = 0;
		}
		else
		{
			if(st->PulseNum_now <= st->PulseNum)  // ��ת�Ӽ���
			{
				if(st->PulseNum - st->PulseNum_now > ((st->PulseCount *2)/5))
					st_speedup(st);
				else if(st->PulseNum - st->PulseNum_now < (st->PulseCount *3/5))
					st_speeddown(st);
				else
				{
					
				}
			}
			else  // ��ת�Ӽ���
			{
				if(st->PulseNum_now - st->PulseNum < -((st->PulseCount *2)/5))
					st_speeddown(st);
				else if(st->PulseNum_now - st->PulseNum > -((st->PulseCount *3)/5))
					st_speedup(st);
				else
				{
					
				}
			}
			if(st->Dir == 1)
				st->PulseNum_now ++ ;
			else 
				st->PulseNum_now -- ;
		}
	}
	else return;
}

