#ifndef __TCRT5000_H__
#define __TCRT5000_H__


#include "bsp_tb6612.h"
#include "ti_msp_dl_config.h"
#include "encoder.h"

/*引脚接线
OUT1 -- PA0
OUT2 -- PA1
OUT3 -- PA10
OUT4 -- PA11
OUT5 -- PB2
OUT6 -- PB3
OUT7 -- PA21
OUT8 -- PA23
*/

#define OUT1   ( ( DL_GPIO_readPins( GPIO_TCRT5000_OUT1_PORT, GPIO_TCRT5000_OUT1_PIN ) & GPIO_TCRT5000_OUT1_PIN ) ? 1 : 0 )
#define OUT2   ( ( DL_GPIO_readPins( GPIO_TCRT5000_OUT2_PORT, GPIO_TCRT5000_OUT2_PIN ) & GPIO_TCRT5000_OUT2_PIN ) ? 1 : 0 )
#define OUT3   ( ( DL_GPIO_readPins( GPIO_TCRT5000_OUT3_PORT, GPIO_TCRT5000_OUT3_PIN ) & GPIO_TCRT5000_OUT3_PIN ) ? 1 : 0 )
#define OUT4   ( ( DL_GPIO_readPins( GPIO_TCRT5000_OUT4_PORT, GPIO_TCRT5000_OUT4_PIN ) & GPIO_TCRT5000_OUT4_PIN ) ? 1 : 0 )
#define OUT5   ( ( DL_GPIO_readPins( GPIO_TCRT5000_OUT5_PORT, GPIO_TCRT5000_OUT5_PIN ) & GPIO_TCRT5000_OUT5_PIN ) ? 1 : 0 )
#define OUT6   ( ( DL_GPIO_readPins( GPIO_TCRT5000_OUT6_PORT, GPIO_TCRT5000_OUT6_PIN ) & GPIO_TCRT5000_OUT6_PIN ) ? 1 : 0 )
#define OUT7   ( ( DL_GPIO_readPins( GPIO_TCRT5000_OUT7_PORT, GPIO_TCRT5000_OUT7_PIN ) & GPIO_TCRT5000_OUT7_PIN ) ? 1 : 0 )
#define OUT8   ( ( DL_GPIO_readPins( GPIO_TCRT5000_OUT8_PORT, GPIO_TCRT5000_OUT8_PIN ) & GPIO_TCRT5000_OUT8_PIN ) ? 1 : 0 )

#define S1 OUT1
#define S2 OUT2
#define S3 OUT3
#define S4 OUT4
#define S5 OUT5
#define S6 OUT6
#define S7 OUT7
#define S8 OUT8

uint8_t Read_8Track_Sensor(void);
void Track_8Sensor_Process_high(uint8_t track_buf, float speed_run);
void Track_8Sensor_Process_AB(uint8_t track_buf, float speed_run);          //a点到b点循迹
void Track_8Sensor_Process_AA(uint8_t track_buf, float speed_run);          //一整圈带小球循迹

unsigned char Get_OUT1(void);
unsigned char Get_OUT2(void);
unsigned char Get_OUT3(void);
unsigned char Get_OUT4(void);
unsigned char Get_OUT5(void);
unsigned char Get_OUT6(void);
unsigned char Get_OUT7(void);
unsigned char Get_OUT8(void);

#endif
