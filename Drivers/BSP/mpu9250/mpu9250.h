#ifndef __mpu9250_H
#define __mpu9250_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "main.h"
#include "cmsis_os.h"
#include "i2c.h"
//#include "eeprom.h"


extern uint8_t c;
extern osThreadId mpu9250TaskHandle;

extern float Pitch,Roll,Yaw;
extern float HeadingDegrees;
extern float linear_accelX,linear_accelY,linear_accelZ;
extern float Gravity_X,Gravity_Y,Gravity_Z;
extern float GYRO_Spx,GYRO_Spy,GYRO_Spz;
extern float Compass_X,Compass_Y,Compass_Z;

int8_t mpu9250init(void);

void MPU9250FreertosInit(void);

void gyro_data_ready_cb(void);

int mpu_get_tick_count(unsigned long *count);
#ifdef __cplusplus
}
#endif
#endif

