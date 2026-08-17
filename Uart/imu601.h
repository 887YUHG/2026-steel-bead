#ifndef  IMU601_H
#define  IMU601_H

#include "ti_msp_dl_config.h"
#include  "uart.h"
#include <stdint.h>

typedef struct {
    float yaw;
    float pitch;
    float roll;
} Attitude_t;


//接线
//imu601    --   MSPM03507
//V   		--   5V
//G   		--   GND
//T   		--   去接天猛星的RX3对应的是PB13
//R    		--   去接天猛星的TX3对应的是PB12

#define delay_ms(X)  delay_cycles((CPUCLK_FREQ/1000)*(X));			//系统时钟频率（天猛星默认 80MHz）
void IMU601_Init();

extern Attitude_t current_attitude;											//定义的最后接收的变量  当前姿态

#endif
