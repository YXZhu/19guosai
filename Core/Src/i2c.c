/**
  ******************************************************************************
  * File Name          : I2C.c
  * Description        : This file provides code for the configuration
  *                      of the I2C instances.
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

/* Includes ------------------------------------------------------------------*/
#include "i2c.h"

/* USER CODE BEGIN 0 */
#include "dwt_stm32_delay.h"
/* USER CODE END 0 */

I2C_HandleTypeDef hi2c3;
DMA_HandleTypeDef hdma_i2c3_rx;
DMA_HandleTypeDef hdma_i2c3_tx;

/* I2C3 init function */
void MX_I2C3_Init(void)
{

  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 400000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(i2cHandle->Instance==I2C3)
  {
  /* USER CODE BEGIN I2C3_MspInit 0 */
		/**I2C3 GPIO Configuration    
		PA8     ------> I2C3_SCL
		PB4     ------> I2C3_SDA 
		*/
	  if(!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_4)|| !HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_8))
	  {		  
		  GPIO_InitStruct.Pin = GPIO_PIN_4;
		  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
		  GPIO_InitStruct.Pull = GPIO_PULLUP;
		  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		  
		  GPIO_InitStruct.Pin = GPIO_PIN_8;
		  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
		  GPIO_InitStruct.Pull = GPIO_PULLUP;
		  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);		  
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
		  while(!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_4))
		  {	
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
			  DWT_Delay_us(10);
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
			  DWT_Delay_us(10);
		  }
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
		  DWT_Delay_us(10);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
		  DWT_Delay_us(10);
		  
		  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8);
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_4);
	  }
  /* USER CODE END I2C3_MspInit 0 */
  
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C3 GPIO Configuration    
    PA8     ------> I2C3_SCL
    PB4     ------> I2C3_SDA 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_I2C3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C3 clock enable */
    __HAL_RCC_I2C3_CLK_ENABLE();
  
    /* I2C3 DMA Init */
    /* I2C3_RX Init */
    hdma_i2c3_rx.Instance = DMA1_Stream1;
    hdma_i2c3_rx.Init.Channel = DMA_CHANNEL_1;
    hdma_i2c3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_i2c3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2c3_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2c3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_i2c3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_i2c3_rx.Init.Mode = DMA_NORMAL;
    hdma_i2c3_rx.Init.Priority = DMA_PRIORITY_MEDIUM;
    hdma_i2c3_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_i2c3_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(i2cHandle,hdmarx,hdma_i2c3_rx);

    /* I2C3_TX Init */
    hdma_i2c3_tx.Instance = DMA1_Stream4;
    hdma_i2c3_tx.Init.Channel = DMA_CHANNEL_3;
    hdma_i2c3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_i2c3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2c3_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2c3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_i2c3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_i2c3_tx.Init.Mode = DMA_NORMAL;
    hdma_i2c3_tx.Init.Priority = DMA_PRIORITY_MEDIUM;
    hdma_i2c3_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_i2c3_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(i2cHandle,hdmatx,hdma_i2c3_tx);

    /* I2C3 interrupt Init */
    HAL_NVIC_SetPriority(I2C3_EV_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(I2C3_EV_IRQn);
    HAL_NVIC_SetPriority(I2C3_ER_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(I2C3_ER_IRQn);
  /* USER CODE BEGIN I2C3_MspInit 1 */

  /* USER CODE END I2C3_MspInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C3)
  {
  /* USER CODE BEGIN I2C3_MspDeInit 0 */

  /* USER CODE END I2C3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C3_CLK_DISABLE();
  
    /**I2C3 GPIO Configuration    
    PA8     ------> I2C3_SCL
    PB4     ------> I2C3_SDA 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_4);

    /* I2C3 DMA DeInit */
    HAL_DMA_DeInit(i2cHandle->hdmarx);
    HAL_DMA_DeInit(i2cHandle->hdmatx);

    /* I2C3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(I2C3_EV_IRQn);
    HAL_NVIC_DisableIRQ(I2C3_ER_IRQn);
  /* USER CODE BEGIN I2C3_MspDeInit 1 */

  /* USER CODE END I2C3_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */

void I2CError(I2C_HandleTypeDef *hi2c)
{
	//HAL_I2C_DeInit(hi2c);
	//HAL_I2C_Init(hi2c);
	HAL_I2C_ErrorCallback(hi2c);
}
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	HAL_I2C_DeInit(hi2c);
	if(hi2c->Instance == I2C3)
	{		
		/**I2C3 GPIO Configuration    
		PA8     ------> I2C3_SCL
		PB4     ------> I2C3_SDA 
		*/
	  if(!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_4)|| !HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_8))
	  {
		  
		  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8);
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_4);
		  
		  GPIO_InitStruct.Pin = GPIO_PIN_4;
		  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
		  GPIO_InitStruct.Pull = GPIO_PULLUP;
		  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		  
		  GPIO_InitStruct.Pin = GPIO_PIN_8;
		  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
		  GPIO_InitStruct.Pull = GPIO_PULLUP;
		  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);		  
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
		  while(!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_4))
		  {	
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
			  DWT_Delay_us(10);
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
			  DWT_Delay_us(10);
		  }
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
		  DWT_Delay_us(10);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
		  DWT_Delay_us(10);
 
	  }
	 
	  MX_I2C3_Init(); 
	  HAL_GPIO_TogglePin(C13_GPIO_Port,C13_Pin);			
  }
}
int i2c1Write(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{

	addr <<=1;  /* 直接对7bit地址左移 必须！！*/
	//if(HAL_I2C_Master_Transmit(&hi2c1,addr,t,len,0xff)!= HAL_OK)
	//osMutexWait(I2C1Mutex,100);
	
//	i2cmem.addr = addr;
//	i2cmem.reg = reg;
//	i2cmem.len = len;
//	memcpy(i2cmem.Buf,data,i2cmem.len);
//	//mem = osPoolAlloc(I2CdataSave);
//	/* 大于2个字节用DMA，DMA可以说是特效药，“屡试不爽”。不过要注意，接收大于或等于2个字节时才能使用DMA，不然不能产生EOT-1事件导致NACK不能正确发送。*/
//	if(HAL_I2C_Mem_Write_DMA(&hi2c1,i2cmem.addr,i2cmem.reg,I2C_MEMADD_SIZE_8BIT,i2cmem.Buf,i2cmem.len) != HAL_OK)
//	{
//		//SEGGER_RTT_printf(0, "I2C Write ERROR\n");
//		HAL_I2C_Master_Abort_IT(&hi2c1,i2cmem.addr);
//		I2CError(&hi2c1);
//		osMutexRelease(I2C1Mutex);
//		return -1;
//	}
//	if(osSemaphoreWait(I2c1TxComplete,100) != osOK) HAL_GPIO_TogglePin(D3_GPIO_Port,D3_Pin);	
	//osPoolFree(I2CdataSave,mem);
	if(HAL_I2C_Mem_Write(&hi2c3,(uint16_t)addr,reg,I2C_MEMADD_SIZE_8BIT,(uint8_t*)data,len,10) != HAL_OK)
	{
		//SEGGER_RTT_printf(0, "I2C Write ERROR\n");
		I2CError(&hi2c3);
		//osMutexRelease(I2C1Mutex);
		return -1;
	}	
	//HAL_Delay(1);
	
		//HAL_I2C_Master_Abort_IT(&hi2c1,addr);
	//osDelay(1);
	//osMutexRelease(I2C1Mutex);
	return 0;
	
}

int i2c1Read(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{

	addr <<=1; /* 直接对7bit地址左移 必须！！*/
	//if(HAL_I2C_Master_Receive(&hi2c1,addr,t,len,0xff)!= HAL_OK)
	//osMutexWait(I2C1Mutex,100);
//	i2cmem.addr = addr;
//	i2cmem.reg = reg;
//	i2cmem.len = len;
//	//memset(bufs,0,len1);
//	if(HAL_I2C_Mem_Read_DMA(&hi2c1,(uint16_t)i2cmem.addr,i2cmem.reg,I2C_MEMADD_SIZE_8BIT,(uint8_t*)i2cmem.Buf,i2cmem.len)!= HAL_OK)
//	{
//		//SEGGER_RTT_printf(0, "I2C Read ERROR\n");
//		HAL_I2C_Master_Abort_IT(&hi2c1,i2cmem.addr);
//		I2CError(&hi2c1);
//		osMutexRelease(I2C1Mutex);
//		return -1;
//	}
//	
//	//HAL_Delay(2);
//	if(osSemaphoreWait(I2c1RxComplete,100) != osOK) HAL_GPIO_TogglePin(D3_GPIO_Port,D3_Pin);		
//	memcpy(buf,i2cmem.Buf,i2cmem.len);

	if(HAL_I2C_Mem_Read(&hi2c3,(uint16_t)addr,reg,I2C_MEMADD_SIZE_8BIT,(uint8_t*)buf,len,10)!= HAL_OK)
	{
		//SEGGER_RTT_printf(0, "I2C Read ERROR\n");
		I2CError(&hi2c3);
		//osMutexRelease(I2C1Mutex);
		return -1;
	}

	//if(osSemaphoreWait(I2c1RxComplete,100) != osOK) HAL_GPIO_TogglePin(D3_GPIO_Port,D3_Pin);
	//osDelay(1);
	//osMutexRelease(I2C1Mutex);
	return 0;
}
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
