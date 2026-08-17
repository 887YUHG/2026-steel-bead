#ifndef _MOTOR_H
#define _MOTOR_H

#include "ti_msp_dl_config.h"
#include "bsp_tb6612.h"


typedef struct
{
    float Kp;      // 比例系数
    float Ki;      // 积分系数
    float Kd;      // 微分系数

    float target;  // 目标速度 (例如 500.0)
    float actual;  // 实际速度 (从 speed_1 获取)
    
    float err;     // 当前误差 e(k)
    float err_last;// 上一次误差 e(k-1)
    float err_prev;// 上上次误差 e(k-2)
	float integral;       // ★新增：积分累积项（位置式PID用）
    float integral_limit; // ★新增：积分限幅（防积分饱和）

    int output;    // 当前输出计算值 (PWM值)
    int max_out;   // 输出限幅 (例如 999)
	int min_out;          // ★新增：最小输出限幅（角度PID需要负值）
} PID_Struct;



int Velocity_A(int TargetVelocity, int CurrentVelocity);
int Velocity_B(int TargetVelocity, int CurrentVelocity);
//void Set_PWM(int pwmA,int pwmB);
int PID_Inc_Calc(PID_Struct *pid);

#endif