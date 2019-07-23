/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include "usart.h"
#include "usbd_cdc_if.h"

#include "servoctrl.h"
#include "stepper\bsp_StepMotor.h"
#include "mpu9250.h"

#include "spiffs_user.h"
#include "ringbuff/ringbuff.h"

#include "cmd.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define  FASTSEEK_SPEED   300		//原点回归速度
#define  SLOWSEEK_SPEED   100		//原点回归爬行速度
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
// 速度最大值由驱动器和电机决定，有些最大是1800，有些可以达到4000
uint32_t set_speed_x  = 1674;         // 速度 单位为0.05rad/sec
// 加速度和减速度选取一般根据实际需要，值越大速度变化越快，加减速阶段比较抖动
// 所以加速度和减速度值一般是在实际应用中多尝试出来的结果
uint32_t step_accel_x = 1674;         // 加速度 单位为0.025rad/sec^2
uint32_t step_decel_x = 1674;          // 减速度 单位为0.025rad/sec^2
int32_t stepptt_x;

uint32_t set_speed_y  = 1674;         // 速度 单位为0.05rad/sec
// 加速度和减速度选取一般根据实际需要，值越大速度变化越快，加减速阶段比较抖动
// 所以加速度和减速度值一般是在实际应用中多尝试出来的结果
uint32_t step_accel_y = 1674;         // 加速度 单位为0.025rad/sec^2
uint32_t step_decel_y = 1674;          // 减速度 单位为0.025rad/sec^2
int32_t stepptt_y;
uint8_t ssda;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

osThreadId_t messageTaskHandle;  // 消息处理句柄
/* USER CODE END Variables */
osThreadId_t defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void MessageTask(void *argument);
void KernelInfo (void);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	//sys_spiffs_mount_coreflash();
  /* USER CODE END Init */
	osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .priority = (osPriority_t) osPriorityLow3,
    .stack_size = 384
  };
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  const osThreadAttr_t messageTask_attributes = {
    .name = "messageTask",
    .priority = (osPriority_t) osPriorityLow3,
    .stack_size = 256
  };
  messageTaskHandle = osThreadNew(MessageTask, NULL, &messageTask_attributes);
   cmd_init();
  
   STEPMOTOR_TIMx_Init();
   ServoCtrlFreertosInit();
	MPU9250FreertosInit();
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
    
                 
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */

	osDelay(3000);
	KernelInfo();
	osDelay(100);
	sys_spiffs_mount_coreflash();
	//test_spiffs();
	//sys_spiffs_ls();
	//STEPMOTOR_DisMoveAbs(AXIS_X,60,50,50,600);
	
	STEPMOTOR_AxisMoveRel(AXIS_X,3200,50,50,200);
  for(;;)
  {
	  if(ssda == 1)
	  {
		  ssda = 0;
		  //STEPMOTOR_DisMoveAbs(AXIS_X,stepptt,step_accel,step_decel,set_speed);//X轴移动到100mm的位置
		  STEPMOTOR_AxisMoveRel(AXIS_X,stepptt_x,step_accel_x,step_decel_x,set_speed_x);
			STEPMOTOR_DisMoveRel(AXIS_Y,stepptt_y,step_accel_y,step_decel_y,set_speed_y);
		 // STEPMOTOR_DisMoveRel(AXIS_X,stepptt,step_accel,step_decel,set_speed);
	  }
	  //printf("sad");
	  //HAL_GPIO_TogglePin(C13_GPIO_Port,C13_Pin);
    osDelay(100);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

extern USBD_HandleTypeDef hUsbDeviceFS;

char printf_send[200];
ringbuff_t printf_send_ringbuff;

#define MessageTask_Print 0x01
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART2 and Loop until the end of transmission */
  //HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
  ringbuff_write(&printf_send_ringbuff,&ch,1);
  //osThreadFlagsSet(messageTaskHandle,MessageTask_Print);
  return ch;
}

void MessageTask(void *argument)
{
	uint32_t MessageEvent;
	ringbuff_init(&printf_send_ringbuff,printf_send,sizeof(printf_send));
	uint8_t sendbuft[100];
	uint8_t len;
	size_t ringbufffree;
	for(;;)
	{
		MessageEvent = osThreadFlagsWait(0xff,osFlagsWaitAny,10);
		
		if(MessageEvent<=0xff)
		{
			if(READ_BIT(MessageEvent,MessageTask_Print))
			{				

			}
			
		}
		else if(MessageEvent == osFlagsErrorTimeout)
		{

			ringbufffree = ringbuff_get_free(&printf_send_ringbuff);
			if(ringbufffree < printf_send_ringbuff.size)
			{
				
				len = ringbuff_read(&printf_send_ringbuff,sendbuft,sizeof(sendbuft));
				CDC_Transmit_FS((uint8_t *)&sendbuft,len);
			}
//			if(bufcount != 0)
//			{
//				//memcpy(sendbuft,printf_send,bufcount);
//				usb_result = CDC_Transmit_FS((uint8_t *)&printf_send,bufcount);
//				if(usb_result != USBD_BUSY)
//					bufcount = 0;
//			}

		}	
		
	}

}	

void KernelInfo (void) {
  char infobuf[100];
  osVersion_t osv;
  osStatus_t status;
 
  status = osKernelGetInfo(&osv, infobuf, sizeof(infobuf));
  if(status == osOK) {
    printf("Kernel Information: %s\r\n", infobuf);
    printf("Kernel Version    : %d\r\n", osv.kernel);
    printf("Kernel API Version: %d\r\n", osv.api);
  }
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
