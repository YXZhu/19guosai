#ifndef __BSP_STEPMOTOR_H__
#define __BSP_STEPMOTOR_H__

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
/* ���Ͷ��� ------------------------------------------------------------------*/
typedef struct {
  __IO uint8_t  run_state ;  	// �����ת״̬
  __IO int8_t   dir ;        	// �����ת����
  __IO uint32_t decel_start; 	// ��������λ��
  __IO int32_t  step_delay;  	// �¸��������ڣ�ʱ������������ʱΪ���ٶ�
  __IO int32_t  decel_val;   	// ���ٽ׶β���
  __IO int32_t  min_delay;   	// ��С��������(����ٶȣ������ٶ��ٶ�)
  __IO int32_t  accel_count; 	// �Ӽ��ٽ׶μ���ֵ
  __IO int32_t  medle_delay;
}speedRampData;

/* �궨�� --------------------------------------------------------------------*/

#define StepperDelay osDelay

#define STEPMOTOR_TIMx                        TIM9                //��ʱ���������
#define STEPMOTOR_TIM_RCC_CLK_ENABLE()        __HAL_RCC_TIM9_CLK_ENABLE()
#define STEPMOTOR_TIM_RCC_CLK_DISABLE()       __HAL_RCC_TIM9_CLK_DISABLE()

#define STEPMOTOR_TIMx_IRQn                   TIM1_BRK_TIM9_IRQn			  //��ʱ������Ƚ��ж�
#define STEPMOTOR_TIMx_IRQHandler             TIM1_BRK_TIM9_IRQHandler	//�жϷ�����


#define STEPMOTOR_PULSE_GPIO_CLK_ENABLE       __HAL_RCC_GPIOA_CLK_ENABLE
#define STEPMOTOR_TIM_PULSE_PORT              GPIOA               // TIM1����������� 4��ͨ����Ӧ��������GPIOA 

#define 	AXIS_X  0   //������
#define 	AXIS_Y  1

// X����������Ŷ���
#define STEPMOTOR_TIM_CHANNEL1                TIM_CHANNEL_1			  //��ʱ��ͨ��1//X��
#define STEPMOTOR_TIM_IT_CC1                  TIM_IT_CC1			    //��ʱ��ͨ��1�ж�ʹ��λ
#define STEPMOTOR_TIM_FLAG_CC1                TIM_FLAG_CC1			  //��ʱ��ͨ��1�жϱ�־λ
#define STEPMOTOR_TIM_PULSE_PIN_X             GPIO_PIN_2          //��������X����������
#define GPIO_SET_AF_TIM_CHx                   GPIO_AF3_TIM9

#define ORIGIN_X_DETECT_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOE_CLK_ENABLE()	//X��ԭ��������
#define ORIGIN_X_DETECT_PORT					        GPIOC			          					
#define ORIGIN_X_DETECT_PIN					          GPIO_PIN_11
#define GPIO_SET_AF_NORMAL                    GPIO_AF0_TRACE
#define ORIGIN_X_EXTI_IRQn            	      EXTI0_IRQn                     
#define ORIGIN_X_DOG_MSG						          1								              //��⵽�����ź�ʱ���ŵ�ƽ

#define LIMPOS_X_GPIO_CLK_ENABLE()      		  __HAL_RCC_GPIOE_CLK_ENABLE()  // ��ת����λ��������
#define LIMPOS_X_PORT                   		  GPIOC
#define LIMPOS_X_PIN                    		  GPIO_PIN_11
#define GPIO_SET_AF_NORMAL                    GPIO_AF0_TRACE
#define LIMPOS_X_EXTI_IRQn						        EXTI1_IRQn
#define LIMPOS_X_LEVEL						            1	                            //��ת����������Ч��ƽ

#define LIMNEG_X_GPIO_CLK_ENABLE()      		  __HAL_RCC_GPIOE_CLK_ENABLE()  // ��ת��λ��������
#define LIMNEG_X_PORT                   		  GPIOC       
#define LIMNEG_X_PIN                    		  GPIO_PIN_11  
#define GPIO_SET_AF_NORMAL                    GPIO_AF0_TRACE
#define LIMNEG_X_EXTI_IRQn						        EXTI0_IRQn
#define LIMNEG_X_LEVEL						            1	                            //��ת����������Ч��ƽ

#define STEPMOTOR_X_DIR_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOE_CLK_ENABLE()  // �����ת������ƣ�������ղ���Ĭ����ת
#define STEPMOTOR_X_DIR_PORT                  GPIOA                         // ��ӦSTEPMOTOR��DIR-��������ʹ�ù����ӷ���
#define STEPMOTOR_X_DIR_PIN                   GPIO_PIN_0                    // ��DIR+ֱ�ӽӿ������+5V(��3.3V)
#define GPIO_SET_AF_NORMAL                    GPIO_AF0_TRACE

#define STEPMOTOR_X_ENA_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOC_CLK_ENABLE()  // ���ʹ�ܿ��ƣ�������ղ���Ĭ��ʹ�ܵ��
#define STEPMOTOR_X_ENA_PORT                  GPIOA                         // ��ӦSTEPMOTOR��ENA-��������ʹ�ù����ӷ���
#define STEPMOTOR_X_ENA_PIN                   GPIO_PIN_1                    // ��ENA+ֱ�ӿ������+5V(��3.3V) 
#define GPIO_SET_AF_NORMAL                    GPIO_AF0_TRACE

// Y����������Ŷ���
#define STEPMOTOR_TIM_CHANNEL2                TIM_CHANNEL_2			            // ��ʱ��ͨ��2//Y��
#define STEPMOTOR_TIM_IT_CC2                  TIM_IT_CC2			              // ��ʱ��ͨ��2�ж�ʹ��λ
#define STEPMOTOR_TIM_FLAG_CC2                TIM_FLAG_CC2                  // ��ʱ��ͨ��1�жϱ�־λ
#define STEPMOTOR_TIM_PULSE_PIN_Y             GPIO_PIN_3                    // ��������Y����������
#define GPIO_SET_AF_TIM_CHx                   GPIO_AF3_TIM9

#define ORIGIN_Y_DETECT_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOE_CLK_ENABLE()	
#define ORIGIN_Y_DETECT_PORT					        GPIOC								          
#define ORIGIN_Y_DETECT_PIN					          GPIO_PIN_3                    //Y��ԭ���⹦����������
#define ORIGIN_Y_EXTI_IRQn            	      EXTI3_IRQn                    
#define ORIGIN_Y_DOG_MSG						          0                             //�����ź����ŵ�ƽ

#define LIMPOS_Y_GPIO_CLK_ENABLE()      		  __HAL_RCC_GPIOE_CLK_ENABLE()  // ��ת����λ��������
#define LIMPOS_Y_PORT                   		  GPIOC       
#define LIMPOS_Y_PIN                    		  GPIO_PIN_4
#define LIMPOS_Y_EXTI_IRQn						        EXTI4_IRQn
#define LIMPOS_Y_LEVEL						            0	                            //��ת����������Ч��ƽ

#define LIMNEG_Y_GPIO_CLK_ENABLE()      		  __HAL_RCC_GPIOE_CLK_ENABLE()  // ��ת��λ��������
#define LIMNEG_Y_PORT                   		  GPIOC       
#define LIMNEG_Y_PIN                    		  GPIO_PIN_5  
#define LIMNEG_Y_EXTI_IRQn						        EXTI9_5_IRQn
#define LIMNEG_Y_LEVEL						            0	                            //��������������Ч��ƽ

#define STEPMOTOR_Y_DIR_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOB_CLK_ENABLE()     // �����ת������ƣ�������ղ���Ĭ����ת
#define STEPMOTOR_Y_DIR_PORT                  GPIOB                            // ��ӦSTEPMOTOR��DIR+��������ʹ�ù����ӷ���
#define STEPMOTOR_Y_DIR_PIN                   GPIO_PIN_10                      // ��DIR-ֱ�ӽӿ������GND

#define STEPMOTOR_Y_ENA_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOB_CLK_ENABLE()     // ���ʹ�ܿ��ƣ�������ղ���Ĭ��ʹ�ܵ��
#define STEPMOTOR_Y_ENA_PORT                  GPIOB                           // ��ӦSTEPMOTOR��ENA+��������ʹ�ù����ӷ���
#define STEPMOTOR_Y_ENA_PIN                   GPIO_PIN_2                       // ��ENA-ֱ�ӿ������GND 


#define STEPMOTOR_DIR_FORWARD(Axis) \
HAL_GPIO_WritePin(Stepmotor[Axis].Dir_Port,Stepmotor[Axis].Dir_Pin,GPIO_PIN_SET)//���õ������,����Axis:��ǰ���
#define STEPMOTOR_DIR_REVERSAL(Axis)\
HAL_GPIO_WritePin(Stepmotor[Axis].Dir_Port,Stepmotor[Axis].Dir_Pin,GPIO_PIN_RESET)

#define STEPMOTOR_OUTPUT_ENABLE(Axis)\
HAL_GPIO_WritePin(Stepmotor[Axis].Ena_Port,Stepmotor[Axis].Ena_Pin,GPIO_PIN_SET)//���ʹ�ܿ��� ����Axis:��ǰ���
#define STEPMOTOR_OUTPUT_DISABLE(Axis)\
HAL_GPIO_WritePin(Stepmotor[Axis].Ena_Port,Stepmotor[Axis].Ena_Pin,GPIO_PIN_RESET)

// ���嶨ʱ��Ԥ��Ƶ����ʱ��ʵ��ʱ��Ƶ��Ϊ��72MHz/��STEPMOTOR_TIMx_PRESCALER+1��
//#define STEPMOTOR_TIM_PRESCALER               7  // �������������ϸ������Ϊ��   32  ϸ��
#define STEPMOTOR_TIM_PRESCALER               15  // �������������ϸ������Ϊ��   16  ϸ��
//#define STEPMOTOR_TIM_PRESCALER               31  // �������������ϸ������Ϊ��   8  ϸ��
//#define STEPMOTOR_TIM_PRESCALER               63  // �������������ϸ������Ϊ��   4  ϸ��
//#define STEPMOTOR_TIM_PRESCALER               127  // �������������ϸ������Ϊ��   2  ϸ��
//#define STEPMOTOR_TIM_PRESCALER               255  // �������������ϸ������Ϊ��   1  ϸ��

// ���嶨ʱ�����ڣ�����Ƚ�ģʽ��������Ϊ0xFFFF
#define STEPMOTOR_TIM_PERIOD                  0xFFFF

#define FALSE                                 0
#define TRUE                                  1
#define CW                                    1  // ˳ʱ��:������
#define CCW                                   -1 // ��ʱ��:������
#define ORIGIN_POSITION	                      0  // ԭ������
#define STOP                                  0  // �Ӽ�������״̬��ֹͣ
#define ACCEL                                 1  // �Ӽ�������״̬�����ٽ׶�
#define DECEL                                 2  // �Ӽ�������״̬�����ٽ׶�
#define RUN                                   3  // �Ӽ�������״̬�����ٽ׶�

#define IDLE	   							                0	 // ����ԭ��״̬:����
#define FASTSEEK   							              1  // ����ԭ��״̬:��������
#define SLOWSEEK 						              	  2  // ����ԭ��״̬:����ԭ��
#define MOVETOZERO 					            		  3  // ����ԭ��״̬:����ԭ��

#define T1_FREQ                               (SystemCoreClock/(STEPMOTOR_TIM_PRESCALER+1)) // Ƶ��ftֵ
#define FSPR                                  200// ���������Ȧ����  �����:1.8�� 360/1.8 = 200 �����������Ҫ200��תһȦ
#define MICRO_STEP                            16 // �������������ϸ����
#define SPR                                   (FSPR*MICRO_STEP)   // ��תһȦ��Ҫ��������

// ��ѧ����
#define ALPHA                                 ((float)(2*3.14159/SPR))       // ��= 2*pi/spr�����
#define A_T_x10                               ((float)(10*ALPHA*T1_FREQ))
#define T1_FREQ_148                           ((float)((T1_FREQ*0.676)/10))  // 0.676Ϊ�������ֵ
#define A_SQ                                  ((float)(2*100000*ALPHA)) 
#define A_x200                                ((float)(200*ALPHA))
#define MAX_NUM_LAP 						              INT32_MAX
#define MAX_NUM_STEP 						              UINT32_MAX

#define UNIT_STEP_MM                          (SPR/UNITS_DISTANCE)//����1mm��Ҫ�Ĳ���
#define MAX_STEP_MM                           (MAX_DISTANCE/UNITS_DISTANCE)*UNIT_STEP_MM //
#define UNITS_DISTANCE                        5   //�������תһȦ,����ǰ��5mm
#define MAX_DISTANCE                          1000 //��������ƶ��������1000mm

/* ��չ���� ------------------------------------------------------------------*/

extern TIM_HandleTypeDef htimx_STEPMOTOR;
extern __IO uint8_t  MotionStatus[2];
extern speedRampData srd[2];
extern __IO int16_t  location[2];  // ��ǰλ��  ��λ:����(mm)
extern __IO uint8_t  LimPosi[2];      //��ת���ޱ�־λ  True:���Ｋ��λ��  False:δ���Ｋ��λ��
extern __IO uint8_t  LimNega[2];      //��ת���ޱ�־λ
/* �������� ------------------------------------------------------------------*/
void HAL_TIM_OC_Callback(uint8_t Axis);
void STEPMOTOR_TIMx_Init( void );
void STEPMOTOR_AxisMoveRel( uint8_t Axis, int32_t step, uint32_t accel, uint32_t decel, uint32_t speed);
void STEPMOTOR_AxisMoveAbs( uint8_t Axis, int32_t targert_step, uint32_t accel, uint32_t decel, uint32_t speed);
void STEPMOTOR_AxisHome( uint8_t Axis, int32_t step, uint32_t accel, uint32_t decel, uint32_t speed);
void STEPMOTOR_DisMoveRel( uint8_t Axis, int16_t distance, uint32_t accel, uint32_t decel, uint32_t speed);
void STEPMOTOR_DisMoveAbs( uint8_t Axis, uint16_t Target_Dis, uint32_t accel, uint32_t decel, uint32_t speed);

#endif	/* __STEPMOTOR_TIM_H__ */
/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
