/**
  ******************************************************************************
  * File Name          : gpio.c
  * Description        : This file provides code for the configuration
  *                      of all used GPIO pins.
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
#include "gpio.h"
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(C13_GPIO_Port, C13_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DIR_1_Pin|ENA_1_Pin|F_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ENA_2_Pin|DIR_2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = C13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(C13_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PAPin PAPin */
  GPIO_InitStruct.Pin = DIR_1_Pin|ENA_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = F_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(F_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PBPin PBPin */
  GPIO_InitStruct.Pin = ENA_2_Pin|DIR_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

}

/* USER CODE BEGIN 2 */
#include "mpu9250.h"
#include "stepper\bsp_StepMotor.h"

//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//	if(GPIO_Pin == INT_Pin)
//	{
//		gyro_data_ready_cb();	
//	}
//	
//}
/**
  * 函数功能: 外部中断服务函数
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 实现判断是否到达极限和检测原点信号
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	
	if(GPIO_Pin == INT_Pin)
	{
		gyro_data_ready_cb();	
	}
  uint8_t i = 0;
  for(i=0; i<1; i++)
  { 
//    if(GPIO_Pin==Origin_Detect[i].Pin)
//    {
//      __HAL_GPIO_EXTI_CLEAR_IT(Origin_Detect[i].Pin);	
//      if( HAL_GPIO_ReadPin(Origin_Detect[i].Port, \
//                           Origin_Detect[i].Pin)==\
//          Origin_Detect[i].Active_Level  )          
//      {										            //以一个脉冲信号的前沿作为近点信号,后沿作为原点信号      
//        DOG[i] = TRUE;				        //可以是上升沿或者是下降沿
//      }
//      else
//        {
//          HomeCapture[i] = TRUE;     //后沿信号标记捕获到原点
//          if(DOG[i] == TRUE)
//          {
//            srd[i].run_state = STOP;  //正常情况下回到原点
//          }
//        }
//    }
    if(GPIO_Pin == LimPos_Detect[i].Pin)	//正转方向的极限位置检测引脚
    {
        __HAL_GPIO_EXTI_CLEAR_IT( LimPos_Detect[i].Pin );
        if( HAL_GPIO_ReadPin( LimPos_Detect[i].Port,   \
                              LimPos_Detect[i].Pin ) ==\
        LimPos_Detect[i].Active_Level)
        {	
          LimPosi[i]	= TRUE;       	 
          srd[i].run_state = STOP;		//碰到两个极限都要停下来
        }
    }
    if(GPIO_Pin == LimNeg_Detect[i].Pin)	//反转方向的极限位置检测引脚
    {
      __HAL_GPIO_EXTI_CLEAR_IT(LimNeg_Detect[i].Pin);
      if( HAL_GPIO_ReadPin( LimNeg_Detect[i].Port,   \
                            LimNeg_Detect[i].Pin ) ==\
      LimNeg_Detect[i].Active_Level )
      {
        LimNega[i] = TRUE;      	
        srd[i].run_state = STOP;		//碰到左右极限都要停下来
      }
    }
  }
}
/* USER CODE END 2 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
