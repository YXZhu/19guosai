#ifndef __BSP_STEPMOTOR_H__
#define __BSP_STEPMOTOR_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
/* 类型定义 ------------------------------------------------------------------*/
typedef struct {
  __IO uint8_t  run_state ;  	// 电机旋转状态
  __IO int8_t   dir ;        	// 电机旋转方向
  __IO uint32_t decel_start; 	// 启动减速位置
  __IO int32_t  step_delay;  	// 下个脉冲周期（时间间隔），启动时为加速度
  __IO int32_t  decel_val;   	// 减速阶段步数
  __IO int32_t  min_delay;   	// 最小脉冲周期(最大速度，即匀速段速度)
  __IO int32_t  accel_count; 	// 加减速阶段计数值
  __IO int32_t  medle_delay;
}speedRampData;

/* 宏定义 --------------------------------------------------------------------*/

#define StepperDelay osDelay

#define STEPMOTOR_TIMx                        TIM9                //定时器输出脉冲
#define STEPMOTOR_TIM_RCC_CLK_ENABLE()        __HAL_RCC_TIM9_CLK_ENABLE()
#define STEPMOTOR_TIM_RCC_CLK_DISABLE()       __HAL_RCC_TIM9_CLK_DISABLE()

#define STEPMOTOR_TIMx_IRQn                   TIM1_BRK_TIM9_IRQn			  //定时器捕获比较中断
#define STEPMOTOR_TIMx_IRQHandler             TIM1_BRK_TIM9_IRQHandler	//中断服务函数


#define STEPMOTOR_PULSE_GPIO_CLK_ENABLE       __HAL_RCC_GPIOA_CLK_ENABLE
#define STEPMOTOR_TIM_PULSE_PORT              GPIOA               // TIM1脉冲输出引脚 4个通道对应的引脚在GPIOA 

#define 	AXIS_X  0   //各轴标号
#define 	AXIS_Y  1

// X轴电机相关引脚定义
#define STEPMOTOR_TIM_CHANNEL1                TIM_CHANNEL_1			  //定时器通道1//X轴
#define STEPMOTOR_TIM_IT_CC1                  TIM_IT_CC1			    //定时器通道1中断使能位
#define STEPMOTOR_TIM_FLAG_CC1                TIM_FLAG_CC1			  //定时器通道1中断标志位
#define STEPMOTOR_TIM_PULSE_PIN_X             GPIO_PIN_2          //输出脉冲给X轴电机控制器
#define GPIO_SET_AF_TIM_CHx                   GPIO_AF3_TIM9

#define ORIGIN_X_DETECT_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOE_CLK_ENABLE()	//X轴原点检测引脚
#define ORIGIN_X_DETECT_PORT					        GPIOC			          					
#define ORIGIN_X_DETECT_PIN					          GPIO_PIN_11
#define GPIO_SET_AF_NORMAL                    GPIO_AF0_TRACE
#define ORIGIN_X_EXTI_IRQn            	      EXTI0_IRQn                     
#define ORIGIN_X_DOG_MSG						          1								              //检测到近点信号时引脚电平

#define LIMPOS_X_GPIO_CLK_ENABLE()      		  __HAL_RCC_GPIOE_CLK_ENABLE()  // 正转极限位输入引脚
#define LIMPOS_X_PORT                   		  GPIOC
#define LIMPOS_X_PIN                    		  GPIO_PIN_11
#define GPIO_SET_AF_NORMAL                    GPIO_AF0_TRACE
#define LIMPOS_X_EXTI_IRQn						        EXTI1_IRQn
#define LIMPOS_X_LEVEL						            1	                            //正转极限引脚有效电平

#define LIMNEG_X_GPIO_CLK_ENABLE()      		  __HAL_RCC_GPIOE_CLK_ENABLE()  // 反转限位输入引脚
#define LIMNEG_X_PORT                   		  GPIOC       
#define LIMNEG_X_PIN                    		  GPIO_PIN_11  
#define GPIO_SET_AF_NORMAL                    GPIO_AF0_TRACE
#define LIMNEG_X_EXTI_IRQn						        EXTI0_IRQn
#define LIMNEG_X_LEVEL						            1	                            //反转极限引脚有效电平

#define STEPMOTOR_X_DIR_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOE_CLK_ENABLE()  // 电机旋转方向控制，如果悬空不接默认正转
#define STEPMOTOR_X_DIR_PORT                  GPIOA                         // 对应STEPMOTOR的DIR-（控制器使用共阳接法）
#define STEPMOTOR_X_DIR_PIN                   GPIO_PIN_0                    // 而DIR+直接接开发板的+5V(或3.3V)
#define GPIO_SET_AF_NORMAL                    GPIO_AF0_TRACE

#define STEPMOTOR_X_ENA_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOC_CLK_ENABLE()  // 电机使能控制，如果悬空不接默认使能电机
#define STEPMOTOR_X_ENA_PORT                  GPIOA                         // 对应STEPMOTOR的ENA-（控制器使用共阳接法）
#define STEPMOTOR_X_ENA_PIN                   GPIO_PIN_1                    // 而ENA+直接开发板的+5V(或3.3V) 
#define GPIO_SET_AF_NORMAL                    GPIO_AF0_TRACE

// Y轴电机相关引脚定义
#define STEPMOTOR_TIM_CHANNEL2                TIM_CHANNEL_2			            // 定时器通道2//Y轴
#define STEPMOTOR_TIM_IT_CC2                  TIM_IT_CC2			              // 定时器通道2中断使能位
#define STEPMOTOR_TIM_FLAG_CC2                TIM_FLAG_CC2                  // 定时器通道1中断标志位
#define STEPMOTOR_TIM_PULSE_PIN_Y             GPIO_PIN_3                    // 输出脉冲给Y轴电机控制器
#define GPIO_SET_AF_TIM_CHx                   GPIO_AF3_TIM9

#define ORIGIN_Y_DETECT_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOE_CLK_ENABLE()	
#define ORIGIN_Y_DETECT_PORT					        GPIOC								          
#define ORIGIN_Y_DETECT_PIN					          GPIO_PIN_3                    //Y轴原点检测功能输入引脚
#define ORIGIN_Y_EXTI_IRQn            	      EXTI3_IRQn                    
#define ORIGIN_Y_DOG_MSG						          0                             //近点信号引脚电平

#define LIMPOS_Y_GPIO_CLK_ENABLE()      		  __HAL_RCC_GPIOE_CLK_ENABLE()  // 正转极限位输入引脚
#define LIMPOS_Y_PORT                   		  GPIOC       
#define LIMPOS_Y_PIN                    		  GPIO_PIN_4
#define LIMPOS_Y_EXTI_IRQn						        EXTI4_IRQn
#define LIMPOS_Y_LEVEL						            0	                            //正转极限引脚有效电平

#define LIMNEG_Y_GPIO_CLK_ENABLE()      		  __HAL_RCC_GPIOE_CLK_ENABLE()  // 反转限位输入引脚
#define LIMNEG_Y_PORT                   		  GPIOC       
#define LIMNEG_Y_PIN                    		  GPIO_PIN_5  
#define LIMNEG_Y_EXTI_IRQn						        EXTI9_5_IRQn
#define LIMNEG_Y_LEVEL						            0	                            //反方向极限引脚有效电平

#define STEPMOTOR_Y_DIR_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOB_CLK_ENABLE()     // 电机旋转方向控制，如果悬空不接默认正转
#define STEPMOTOR_Y_DIR_PORT                  GPIOB                            // 对应STEPMOTOR的DIR+（控制器使用共阴接法）
#define STEPMOTOR_Y_DIR_PIN                   GPIO_PIN_10                      // 而DIR-直接接开发板的GND

#define STEPMOTOR_Y_ENA_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOB_CLK_ENABLE()     // 电机使能控制，如果悬空不接默认使能电机
#define STEPMOTOR_Y_ENA_PORT                  GPIOB                           // 对应STEPMOTOR的ENA+（控制器使用共阴接法）
#define STEPMOTOR_Y_ENA_PIN                   GPIO_PIN_2                       // 而ENA-直接开发板的GND 


#define STEPMOTOR_DIR_FORWARD(Axis) \
HAL_GPIO_WritePin(Stepmotor[Axis].Dir_Port,Stepmotor[Axis].Dir_Pin,GPIO_PIN_SET)//设置电机方向,参数Axis:当前活动轴
#define STEPMOTOR_DIR_REVERSAL(Axis)\
HAL_GPIO_WritePin(Stepmotor[Axis].Dir_Port,Stepmotor[Axis].Dir_Pin,GPIO_PIN_RESET)

#define STEPMOTOR_OUTPUT_ENABLE(Axis)\
HAL_GPIO_WritePin(Stepmotor[Axis].Ena_Port,Stepmotor[Axis].Ena_Pin,GPIO_PIN_SET)//电机使能控制 参数Axis:当前活动轴
#define STEPMOTOR_OUTPUT_DISABLE(Axis)\
HAL_GPIO_WritePin(Stepmotor[Axis].Ena_Port,Stepmotor[Axis].Ena_Pin,GPIO_PIN_RESET)

// 定义定时器预分频，定时器实际时钟频率为：72MHz/（STEPMOTOR_TIMx_PRESCALER+1）
//#define STEPMOTOR_TIM_PRESCALER               7  // 步进电机驱动器细分设置为：   32  细分
#define STEPMOTOR_TIM_PRESCALER               15  // 步进电机驱动器细分设置为：   16  细分
//#define STEPMOTOR_TIM_PRESCALER               31  // 步进电机驱动器细分设置为：   8  细分
//#define STEPMOTOR_TIM_PRESCALER               63  // 步进电机驱动器细分设置为：   4  细分
//#define STEPMOTOR_TIM_PRESCALER               127  // 步进电机驱动器细分设置为：   2  细分
//#define STEPMOTOR_TIM_PRESCALER               255  // 步进电机驱动器细分设置为：   1  细分

// 定义定时器周期，输出比较模式周期设置为0xFFFF
#define STEPMOTOR_TIM_PERIOD                  0xFFFF

#define FALSE                                 0
#define TRUE                                  1
#define CW                                    1  // 顺时针:正方向
#define CCW                                   -1 // 逆时针:反方向
#define ORIGIN_POSITION	                      0  // 原点坐标
#define STOP                                  0  // 加减速曲线状态：停止
#define ACCEL                                 1  // 加减速曲线状态：加速阶段
#define DECEL                                 2  // 加减速曲线状态：减速阶段
#define RUN                                   3  // 加减速曲线状态：匀速阶段

#define IDLE	   							                0	 // 搜索原点状态:空闲
#define FASTSEEK   							              1  // 搜索原点状态:快速搜索
#define SLOWSEEK 						              	  2  // 搜索原点状态:捕获原点
#define MOVETOZERO 					            		  3  // 搜索原点状态:捕获原点

#define T1_FREQ                               (SystemCoreClock/(STEPMOTOR_TIM_PRESCALER+1)) // 频率ft值
#define FSPR                                  200// 步进电机单圈步数  步距角:1.8° 360/1.8 = 200 正常情况下需要200步转一圈
#define MICRO_STEP                            16 // 步进电机驱动器细分数
#define SPR                                   (FSPR*MICRO_STEP)   // 旋转一圈需要的脉冲数

// 数学常数
#define ALPHA                                 ((float)(2*3.14159/SPR))       // α= 2*pi/spr步距角
#define A_T_x10                               ((float)(10*ALPHA*T1_FREQ))
#define T1_FREQ_148                           ((float)((T1_FREQ*0.676)/10))  // 0.676为误差修正值
#define A_SQ                                  ((float)(2*100000*ALPHA)) 
#define A_x200                                ((float)(200*ALPHA))
#define MAX_NUM_LAP 						              INT32_MAX
#define MAX_NUM_STEP 						              UINT32_MAX

#define UNIT_STEP_MM                          (SPR/UNITS_DISTANCE)//步进1mm需要的步数
#define MAX_STEP_MM                           (MAX_DISTANCE/UNITS_DISTANCE)*UNIT_STEP_MM //
#define UNITS_DISTANCE                        5   //步进电机转一圈,导轨前进5mm
#define MAX_DISTANCE                          1000 //导轨可以移动的最长距离1000mm

//脉冲输出通道
typedef struct {
  uint16_t  Pulse_Pin ; 	// 定时器脉冲输出引脚
  uint32_t  Channel;		  // 定时器通道
  uint32_t  IT_CCx ;  		// 定时器通道中断使能位
  uint32_t  Flag_CCx ;    // 定时器SR中断标记位
}Tim;
typedef struct {
  IRQn_Type IRQn;         // 中断编号
  uint8_t   Active_Level; // 中断引脚电平
  uint16_t  Pin ; 	      // 引脚号
  GPIO_TypeDef *Port;
}Detect_PIN;              // 原点检测引脚

typedef struct{\
  uint16_t  Ena_Pin ;     //电机使能引脚编号
  uint16_t  Dir_Pin ;     //电机方向引脚编号
  GPIO_TypeDef *Dir_Port; //电机方向引脚端口
  GPIO_TypeDef *Ena_Port; //电机使能引脚端口
}StepMotor;

/* 扩展变量 ------------------------------------------------------------------*/

extern TIM_HandleTypeDef htimx_STEPMOTOR;
extern __IO uint8_t  MotionStatus[2];
extern speedRampData srd[2];
extern __IO int16_t  location[2];  // 当前位置  单位:毫米(mm)
extern __IO uint8_t  LimPosi[2];      //正转极限标志位  True:到达极限位置  False:未到达极限位置
extern __IO uint8_t  LimNega[2];      //反转极限标志位
extern const Detect_PIN LimNeg_Detect[2];
extern const Detect_PIN LimPos_Detect[2];
/* 函数声明 ------------------------------------------------------------------*/
void HAL_TIM_OC_Callback(uint8_t Axis);
void STEPMOTOR_TIMx_Init( void );
void STEPMOTOR_AxisMoveRel( uint8_t Axis, int32_t step, uint32_t accel, uint32_t decel, uint32_t speed);
void STEPMOTOR_AxisMoveAbs( uint8_t Axis, int32_t targert_step, uint32_t accel, uint32_t decel, uint32_t speed);
void STEPMOTOR_AxisHome( uint8_t Axis, int32_t step, uint32_t accel, uint32_t decel, uint32_t speed);
void STEPMOTOR_DisMoveRel( uint8_t Axis, int16_t distance, uint32_t accel, uint32_t decel, uint32_t speed);
void STEPMOTOR_DisMoveAbs( uint8_t Axis, uint16_t Target_Dis, uint32_t accel, uint32_t decel, uint32_t speed);

#endif	/* __STEPMOTOR_TIM_H__ */
/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
