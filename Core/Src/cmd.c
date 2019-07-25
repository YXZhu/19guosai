#include "cmd.h"
#include "usart.h"
#include "ProtocolUART.h"
#include "servoctrl.h"
#include "bsp_StepMotor.h"
#include "spiffs_user.h"
#include "usbd_cdc_if.h"

osThreadId_t CmdTaskHandle;

void CmdTask(void  * argument);

osMessageQueueId_t  uarttxMsgBox;
osMessageQueueId_t  uartrxMsgBox;



uint16_t uadc[2];

//SProtocolData uartrx[20];
//SProtocolData uarttx[20];
//ringbuff_t uartrxbuff,uarttxbuff;

SProtocolData sProtocolData = { 0 };

void SendCmd(uint8_t CMD,uint32_t Data);
uint8_t getCheckSum(const uint8_t *pData, int len);
uint8_t sendProtocol(const BYTE *pData, int len);
int parseProtocol(const BYTE *pData, UINT len);


/* ���ղ��� */


/* ���Ͳ��� */


void cmd_init()
{ 

//	ringbuff_init(&uartrxbuff,uart1_rx_buffer,100);	
//	rimid_MsgQueue = osMessageQueueNew(MSGQUEUE_OBJECTS, sizeof(MSGQUEUE_OBJ_t), NULL);ngbuff_init(&uarttxbuff,uarttx,sizeof(uarttx));

	uartrxMsgBox = osMessageQueueNew(10, sizeof(SProtocolData), NULL);
	uarttxMsgBox = osMessageQueueNew(10, sizeof(SProtocolData), NULL);

	
  const osThreadAttr_t CmdTask_attributes = {
    .name = "CmdTask",
    .priority = (osPriority_t) osPriorityLow3,
    .stack_size = 512
  };
  CmdTaskHandle = osThreadNew(CmdTask, NULL, &CmdTask_attributes);//��������

//  osThreadDef(cmdTask, CmdTask, osPriorityNormal, 0,128);
//  cmdTaskHandle = osThreadCreate(osThread(cmdTask), NULL);
}

SProtocolData sProtocolData1;
Position_x_y Position;
SProtocolData recd;
float S;
	
uint32_t time=0;
void CmdTask(void  * argument)
{	
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE); //�������ڿ����ж� ������������
	HAL_UART_Receive_DMA(&huart1,uart1_rx_buffer,100);
	
	
	osStatus_t status;

	for(;;)
	{
		//osDelay(500);
		
		status = osMessageQueueGet (uartrxMsgBox, &recd, NULL, 80);  // wait for message
    if (status == osOK) 
		{
			switch(recd.cmdID)
			{
				case Rec_Position:
				{
					//DMA���ݷ���
					
					Position.X = (uint8_t)(recd.data >> 8) + (((uint8_t)recd.data) << 8);  					 //data��2λ					
					Position.Y = (recd.data >> 24) + ((uint8_t)(recd.data >> 16) << 8); 			 //data��2λ
					//λ�ô���
				}
				break;
				case Rec_2:
				{
					//Position.S = recd.data/1000.f;  					 
					 S = ((recd.data>>24) + ((uint8_t)(recd.data >> 16)<<8) + 
					((uint8_t)(recd.data >> 8)<<16) + ((uint8_t)recd.data <<24) )/1000.f;
					//λ�ô���
				}
				break;
				default:
				{
					Position.X = 0;
					Position.Y = 0;
					S = 0;
				}
					break;
			}
			//osPoolFree(uartrxpool,recd);
		}
		else if(status ==  osErrorTimeout) 
		{
			time ++ ;
			if(time > 1000) time = 0;
			if(time%3==0)  //����
			{
				printf("%x",recd.data);
				//CDC_Transmit_FS((uint8_t *)&recd.data,4);
				//����Ϣʱ
			}
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
 * Ӳ�����ͺ��� 
 */
void protocolDataSend(const BYTE *pData, UINT len)
{
	HAL_UART_Transmit(&huart1,(uint8_t *)&pData,len,50);
}



/**
 * ����ÿһ֡���� ����Э��ص�����
 */
void procParse(const BYTE *pData, UINT len) {
	
 	 SProtocolData Spr;
//	 memcpy(&Spr, pData + DATA_PACKAGE_MIN_LEN-1, (long) (sizeof(Spr)));
	 memcpy(&Spr, pData + DATA_PACKAGE_MIN_LEN-1, 5);
	 osMessageQueuePut (uartrxMsgBox, &Spr, 0, NULL);
	 
//	Spr = osPoolAlloc(uartrxpool);
//	memcpy(&sProtocolData, pData + DATA_PACKAGE_MIN_LEN-1, (long) (sizeof(sProtocolData))); // ����
//	memcpy(Spr, pData + DATA_PACKAGE_MIN_LEN-1, (long) (sizeof(*Spr)));
//	osMessagePut(uartrxMsgBox,(uint32_t)Spr,osWaitForever);
	//ringbuff_write(&uartrxbuff,(const uint8_t *)&sProtocolData,sizeof(SProtocolData));
	// ֪ͨЭ�����ݸ���
	//notifyProtocolDataUpdate(sProtocolData);
}


