#ifndef _BSP_TB6612_H
#define _BSP_TB6612_H

#include "ti_msp_dl_config.h"
#include "stdio.h"

/*  接线
TB6612         MG3507输入
PWMA     --     PA16
PWMB     --     PA17
AIN2     --     PA15
AIN1     --     PA14
BIN2     --     PA13
BIN1     --     PA12
STBY     --     3.3V

TB6612       			右电机
BO1         --     		M-
BO2         --     		M+
E2A (编码器A相)   →    PB17
E2B (编码器B相)   →    PB13  PB18


TB6612         左电机
AO2      --     M-
AO1      --     M+
E1A (编码器A相)   →    PB20
E1B (编码器B相)   →    PB24              
*/

#define AIN1_OUT(X)  ( (X) ? (DL_GPIO_setPins(TB6612_PORT,TB6612_AIN1_PIN)) : (DL_GPIO_clearPins(TB6612_PORT,TB6612_AIN1_PIN)) )
#define AIN2_OUT(X)  ( (X) ? (DL_GPIO_setPins(TB6612_PORT,TB6612_AIN2_PIN)) : (DL_GPIO_clearPins(TB6612_PORT,TB6612_AIN2_PIN)) )

#define BIN1_OUT(X)  ( (X) ? (DL_GPIO_setPins(TB6612_PORT,TB6612_BIN1_PIN)) : (DL_GPIO_clearPins(TB6612_PORT,TB6612_BIN1_PIN)) )
#define BIN2_OUT(X)  ( (X) ? (DL_GPIO_setPins(TB6612_PORT,TB6612_BIN2_PIN)) : (DL_GPIO_clearPins(TB6612_PORT,TB6612_BIN2_PIN)) )


void TB6612_Motor_Stop(void);
void AO_Control(uint8_t dir, uint32_t speed);
void BO_Control(uint8_t dir, uint32_t speed);

void Car_Stop(void);
void Car_Forward(uint32_t speed_1,uint32_t speed_2);
void Car_Backward(uint32_t speed_1,uint32_t speed_2);
void Car_TurnLeft(uint32_t speed);
void Car_TurnRight(uint32_t speed);

#endif  /* _BSP_TB6612_H */