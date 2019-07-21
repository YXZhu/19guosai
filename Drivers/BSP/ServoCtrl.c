#include "servoctrl.h"
#include <string.h>

#define LIMIT( x,min,max ) ( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ) ) //��С����

#define TEST (0)
ServoCtrl_t ServoCtrlS1;
ServoCtrl_t ServoCtrlS2;
ServoCtrl_t ServoCtrlS3;
ServoCtrl_t ServoCtrlS4; 

osThreadId ServoCtrlHandle;

uint32_t Angle2Pulse(uint8_t angle,uint8_t exchange); 
uint8_t AngleRun(ServoCtrl_t *ServoCtr);
void ServoCtrlTask(void * argument);
void readResu(void);
void catsome(uint8_t state1);
void ServoCtrlFreertosInit(void) // �� ��s1 120 160 s2 50 s3 0 ���� s1 120 170 s2 55 s3 18 ���� s1 100 130 s2 42 s3 18
{
	/* ��ʼ�����*/
	ServoCtrlS1.MinAngleSet = 0;
	ServoCtrlS1.MaxAngleSet = 180;
	ServoCtrlS1.ServoAngleSet = 0; // 115 ƽֱ     0 ����
	ServoCtrlS1.ServoAngleNow = 180;
	ServoCtrlS1.ServoSpeed = 360;
	
	ServoCtrlS2.MinAngleSet = 18;
	ServoCtrlS2.MaxAngleSet = 70; //50
	ServoCtrlS2.ServoAngleSet = 18; //18 //18 ���
	ServoCtrlS2.ServoAngleNow = 18;
	ServoCtrlS2.ServoSpeed = 360;
	
	ServoCtrlS3.MinAngleSet = 15;
	ServoCtrlS3.MaxAngleSet = 180;
	ServoCtrlS3.ServoAngleSet = 95; // ǰ������ ˳ʱ�� Ϊ15 ��ʱ��Ϊ 180
	ServoCtrlS3.ServoAngleNow = 95;
	ServoCtrlS3.ServoSpeed = 360;
	
	ServoCtrlS4.MinAngleSet = 0;
	ServoCtrlS4.MaxAngleSet = 180;
	ServoCtrlS4.ServoAngleSet = 90;  //35 160  // ǰ������ ˳ʱ�� Ϊ15 ��ʱ��Ϊ 180
	ServoCtrlS4.ServoAngleNow = 90;
	ServoCtrlS4.ServoSpeed = 180;	

    const osThreadAttr_t defaultTask_attributes = {
    .name = "ServoCtrlTask",
    .priority = (osPriority_t) osPriorityAboveNormal,
    .stack_size = 256
  };
  ServoCtrlHandle = osThreadNew(ServoCtrlTask, NULL, &defaultTask_attributes);
  
}

#if TEST 
uint32_t plues;
#endif
uint32_t event;
void ServoCtrlTask(void * argument)
{
	
	
	osDelay(100);
	__HAL_TIM_SET_COMPARE(&SxTime,S1_OUT,Angle2Pulse(AngleRun(&ServoCtrlS1),0));
	__HAL_TIM_SET_COMPARE(&SxTime,S2_OUT,Angle2Pulse(AngleRun(&ServoCtrlS2),0));
	__HAL_TIM_SET_COMPARE(&SxTime,S3_OUT,Angle2Pulse(AngleRun(&ServoCtrlS3),0));
	__HAL_TIM_SET_COMPARE(&SxTime,S4_OUT,Angle2Pulse(AngleRun(&ServoCtrlS4),0));
	
	HAL_TIM_PWM_Start_IT(&SxTime,S1_OUT);
	HAL_TIM_PWM_Start_IT(&SxTime,S2_OUT);
	HAL_TIM_PWM_Start_IT(&SxTime,S3_OUT);
	HAL_TIM_PWM_Start_IT(&SxTime,S4_OUT);

  for(;;)
  {  
	  event = osThreadFlagsWait(0xff,osFlagsWaitAny,40);
	  //READ_BIT
	  

 
	  if(event == osFlagsErrorTimeout)
	  {
		  
	  }
	  else
	  {
		  if(READ_BIT(event,0x01))
			  #if TEST 
			  __HAL_TIM_SET_COMPARE(&SxTime,S1_OUT,plues);
			  #else
				__HAL_TIM_SET_COMPARE(&SxTime,S1_OUT,Angle2Pulse(AngleRun(&ServoCtrlS1),0));
			  #endif
		  if(READ_BIT(event,0x02))
				__HAL_TIM_SET_COMPARE(&SxTime,S2_OUT,Angle2Pulse(AngleRun(&ServoCtrlS2),0));
		  if(READ_BIT(event,0x04))
				__HAL_TIM_SET_COMPARE(&SxTime,S3_OUT,Angle2Pulse(AngleRun(&ServoCtrlS3),0));
		  if(READ_BIT(event,0x08))
				__HAL_TIM_SET_COMPARE(&SxTime,S4_OUT,Angle2Pulse(AngleRun(&ServoCtrlS4),0));		  	  
	  }
    //osDelay(5);
  }
}

// ���ö���ٶ� �Ƕ�
void setServo(ServoCtrl_t *ServoCtr,float Speed,uint8_t Angle) // ��ÿ��	
{
	ServoCtr->ServoSpeed = Speed;
	ServoCtr->ServoAngleSet = Angle;
}

//void motionrun(uint16_t *data,
/* angle �Ƕ� exchange ���÷�����Ϊ0�� */
uint32_t Angle2Pulse(uint8_t angle,uint8_t exchange)
{
	#define MaxPulse 2500.f
	#define MinPulse 600.f
	if(angle > 180 ) return 0;
	if(exchange)
		return  (uint32_t)((180 - angle)*((MaxPulse - MinPulse)/180.f)+MinPulse);
	else
		return (uint32_t)(angle*((MaxPulse - MinPulse)/180.f)+MinPulse);
}

/* �Ƕȿ��� �ٶȿ��� */
uint8_t AngleRun(ServoCtrl_t *ServoCtr)
//uint8_t AngleRun(uint8_t angleSet,uint8_t angleSpeed,uint8_t *nowangle,uint32_t *time)
{
	float temp;
	
	if(ServoCtr->ServoAngleSet > ServoCtr->MaxAngleSet) // �ж��ǲ����ڸö�����÷�Χ��
	{
		ServoCtr->ServoAngleSet = ServoCtr->MaxAngleSet;
	}
	else if(ServoCtr->ServoAngleSet < ServoCtr->MinAngleSet)
	{
		ServoCtr->ServoAngleSet = ServoCtr->MinAngleSet;
	}
	
	if(ServoCtr->ServoAngleSet == ServoCtr->ServoAngleNow)
	{
		ServoCtr->TimeLast = GetTime(); // ���浱ǰʱ��
		return ServoCtr->ServoAngleSet;
	}
	else
	{
//		if(ServoCtr->TimeLast == 0) // δ��ȡʱ��
//		{
//			ServoCtr->TimeLast = osKernelSysTick();
//			return ServoCtr->ServoAngleNow;
//		}
		
		temp = GetTime() - ServoCtr->TimeLast; // ��ȡ��ǰʱ��֮��
		temp = 1000.f / temp; //  ���㵱ǰ������ֵ
		ServoCtr->TimeLast = GetTime(); // ���浱ǰʱ��
		//if(temp == 0) return 255; // �����ٶȹ���
		temp = (float)ServoCtr->ServoSpeed / temp;
		
		if(ServoCtr->ServoAngleSet > ServoCtr->ServoAngleNow)
		{	
			temp = ServoCtr->ServoAngleNow + temp;	
			if(temp >= ServoCtr->ServoAngleSet) 
				ServoCtr->ServoAngleNow = ServoCtr->ServoAngleSet;	
			else
				ServoCtr->ServoAngleNow = temp;
		}
		else
		{
			temp = ServoCtr->ServoAngleNow - temp;
			if(temp <= ServoCtr->ServoAngleSet) 
				ServoCtr->ServoAngleNow = ServoCtr->ServoAngleSet;	
			else
				ServoCtr->ServoAngleNow = temp;
		}
		return ServoCtr->ServoAngleNow;
	}
}

// pwm �����ж� ʹ����˶������� 
 void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
 {
	 if(htim == &SxTime)
	 {
		 if(ServoCtrlHandle == NULL) return;
		 if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		 {			 
			 osThreadFlagsSet(ServoCtrlHandle,0x01); //ServoCtrlS1
		 }
		 if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
		 {
			  osThreadFlagsSet(ServoCtrlHandle,0x02); //ServoCtrlS2
		 }
		 if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
		 {
			  osThreadFlagsSet(ServoCtrlHandle,0x04); //ServoCtrlS3
		 }
		 if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
		 {
			  osThreadFlagsSet(ServoCtrlHandle,0x08); //ServoCtrlS4
			 
		 }
	 } 
 }
 
 /**************************************/
 
 

