#include "cmd.h"
//#include "adc.h"
#include "usart.h"
//#include "moto.h"
//#include "bsp_StepMotor.h"
#include "ProtocolUART.h"

osThreadId cmdTaskHandle;
osPoolId uarttxpool;
osPoolId uartrxpool;
osMessageQId  uarttxMsgBox;
osMessageQId  uartrxMsgBox;
osMessageQDef(uartrxMsgBox, 20, SProtocolData);

void CmdTask(void const * argument);

uint16_t uadc[2];

//SProtocolData uartrx[20];
//SProtocolData uarttx[20];
//ringbuff_t uartrxbuff,uarttxbuff;

SProtocolData sProtocolData = { 0 };

void SendCmd(uint8_t CMD,uint32_t Data);
uint8_t getCheckSum(const uint8_t *pData, int len);
uint8_t sendProtocol(const BYTE *pData, int len);
int parseProtocol(const BYTE *pData, UINT len);

int32_t moto_sendcmd; // 向电机任务发送的指令

/* 接收参数 */
uint8_t StepperDis = 0;
uint32_t StepperSpeed=0; // rad/sec
uint32_t StepperAccel=0; // rad/sec^2
uint32_t StepperDecel=0; // rad/sec^2
uint32_t StepperState=0; // 0 关 1 开

extern uint16_t MotoSpeed1; // 转每分钟
extern uint16_t MotoSpeed2; // 转每分钟
uint32_t MotoState=0; // 0 关 1 开

/* 发送参数 */
uint16_t MotoSpeedNow1=0,MotoSpeedNow2=0; // 转每分钟

void cmd_init()
{ 
	//ringbuff_init(&uartrxbuff,uartrx,sizeof(uartrx));
	//ringbuff_init(&uarttxbuff,uarttx,sizeof(uarttx));
	
  osPoolDef(uarttxpool, 20, SProtocolData);
  uarttxpool = osPoolCreate(osPool(uarttxpool));	
	
  osPoolDef(uartrxpool, 20, SProtocolData);
  uartrxpool = osPoolCreate(osPool(uartrxpool));
  
  osMessageQDef(uarttxMsgBox, 20, SProtocolData);
  uarttxMsgBox = osMessageCreate(osMessageQ(uarttxMsgBox), NULL);
	
  
  uartrxMsgBox = osMessageCreate(osMessageQ(uartrxMsgBox), NULL);
	
  osThreadDef(cmdTask, CmdTask, osPriorityNormal, 0, 128);
  cmdTaskHandle = osThreadCreate(osThread(cmdTask), NULL);
}
SProtocolData sProtocolData1;
void CmdTask(void const * argument)
{
	HAL_ADC_Start_DMA(&hadc1,(uint32_t *)&uadc,2);
	
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE); //开启串口空闲中断 ，不定长接收
	HAL_UART_Receive_DMA(&huart1,uart1_rx_buffer,100);
	
	SProtocolData *recd;
	osEvent cmdevent;
	uint32_t time=0;
	for(;;)
	{
		//osDelay(500);
		cmdevent = osMessageGet(uartrxMsgBox,80);
		if(cmdevent.status == osEventMessage) {
			recd = cmdevent.value.p;
			memcpy(&sProtocolData1,recd,sizeof(SProtocolData));
			switch(recd->cmdID)
			{
				case Rec_StepperDis:
					StepperDis = recd->data;
					break;
				case Rec_StepperSpeed:
					StepperSpeed = recd->data;
					break;
				case Rec_StepperAccel:
					StepperAccel = recd->data;
					break;
				case Rec_StepperDecel:
					StepperDecel = recd->data;
					break;
				case Rec_StepperState:
					StepperState = recd->data;
					if(StepperState == 1)
					{
						LimPosi[AXIS_X] = 0;
						LimNega[AXIS_X] = 0;
						STEPMOTOR_DisMoveAbs(AXIS_X,StepperDis*10,StepperAccel,StepperDecel,StepperSpeed);
					}
					else
					{
						srd[AXIS_X].run_state = STOP;
					}
					break;
					/* MCU部分 */
				case Rec_ConTest:
					HAL_GPIO_TogglePin(D2_GPIO_Port,D2_Pin);
					break;
				case Rec_MCUReset:
					HAL_NVIC_SystemReset();
					break;
					/* 电机部分 */
				case Rec_MotoSpeed:
					MotoSpeed1 = recd->data>>16;
					MotoSpeed2 = recd->data&0x0000ffff;
					moto_sendcmd = (uint16_t)MotoSpeed1+(MotoState<<16);
					osSignalSet(MotoTaskHandle,moto_sendcmd);
				
					MotoSpeedNow1 = (uint16_t)Encoderspeed[0]; // 测试
					MotoSpeedNow2 = (uint16_t)Encoderspeed[1]; // 测试
					break;
				case Rec_MotoState:
					MotoState = recd->data;
				   moto_sendcmd = (uint16_t)MotoSpeed1+(MotoState<<16);
					osSignalSet(MotoTaskHandle,moto_sendcmd);
					break;
				case Rec_MotoPI1:
					moto1pid.Kp = (float)(recd->data>>16) / 32768.f;
					moto1pid.Ki = (float)(recd->data&0x0000ffff) / 32768.f;
					break;
				case Rec_MotoD1_Set:
					moto1pid.Kd = (float)(recd->data>>16) / 32768.f;
				   if((recd->data&0x0000ffff) == 1)
					{
						arm_pid_reset_f32(&moto1pid);
						arm_pid_init_f32(&moto1pid,1);
					}
					break;	
				case Rec_MotoPI2:
					moto2pid.Kp = (float)(recd->data>>16) / 32768.f;
					moto2pid.Ki = (float)(recd->data&0x0000ffff) / 32768.f;
					break;
				case Rec_MotoD2_Set:
					moto2pid.Kd = (float)(recd->data>>16) / 32768.f;
				   if((recd->data&0x0000ffff) == 1)
					{
						arm_pid_reset_f32(&moto2pid);
						arm_pid_init_f32(&moto2pid,1);
					}
					break;
				case Rec_Motion:
					if(recd->data == MotionUP)
					{
						HAL_GPIO_TogglePin(D2_GPIO_Port,D2_Pin);
						STEPMOTOR_DisMoveAbs(AXIS_Y,0,50,50,350);
					}
					else if(recd->data == MotionSTOP) srd[AXIS_Y].run_state = STOP;
					else STEPMOTOR_DisMoveAbs(AXIS_Y,90,50,50,220);
					
					break;
				default:
					break;
			}
			osPoolFree(uartrxpool,recd);
		}
		else if(cmdevent.status == osEventTimeout) {
//			send.cmdID = Send_NiujvbothADC;
//			send.data = uadc[0]<< 16 | uadc[1];
//			sendProtocol((const uint8_t *)&send,sizeof(SProtocolData));
			
			SendCmd(Send_NiujvbothADC,uadc[0]<< 16 | uadc[1]);
			
			if(time%5==0) {
				MotoSpeedNow1 = (uint16_t)velocityLow[0]; // 测试
			   MotoSpeedNow2 = (uint16_t)velocityLow[1]; // 测试
				SendCmd(Send_MotoSpeedNow,MotoSpeedNow1<<16|MotoSpeedNow2);	
			}
			if(time%3==0) {
				if(StepperState == 1)
					SendCmd(Send_StepperDisNow,location[AXIS_X]);	// 上传步进电机实时距离
			}			
			time ++ ;
			if(time > 1000) time = 0;
		}
	}
	
	
}

void SendCmd(uint8_t CMD,uint32_t Data) {
	SProtocolData Send;
	Send.cmdID = CMD;
	Send.data = Data;
	sendProtocol((BYTE *)&Send, sizeof(Send));
}

/**
 * 硬件发送函数 
 */
void protocolDataSend(const BYTE *pData, UINT len)
{
	HAL_UART_Transmit(&huart1,(uint8_t *)&pData,len,50);
}



/**
 * 解析每一帧数据 解析协议回调函数
 */
void procParse(const BYTE *pData, UINT len) {
	
//	SProtocolData *Spr;
//	Spr = osPoolAlloc(uartrxpool);
//	memcpy(&sProtocolData, pData + DATA_PACKAGE_MIN_LEN-1, (long) (sizeof(sProtocolData))); // 测试
//	memcpy(Spr, pData + DATA_PACKAGE_MIN_LEN-1, (long) (sizeof(*Spr)));
//	osMessagePut(uartrxMsgBox,(uint32_t)Spr,osWaitForever);
	//ringbuff_write(&uartrxbuff,(const uint8_t *)&sProtocolData,sizeof(SProtocolData));
	// 通知协议数据更新
	//notifyProtocolDataUpdate(sProtocolData);
}


