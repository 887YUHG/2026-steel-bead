#include "motor.h"

/**
 * @brief 增量式PID计算（用于电机速度闭环）
 * @param pid PID结构体指针
 * @return 计算后的PWM控制量
 * 
 * 公式: Δu = Kp*(e(k)-e(k-1)) + Ki*e(k) + Kd*(e(k)-2*e(k-1)+e(k-2))
 * 输出范围: min_out ~ max_out (默认0~999)
 */
int PID_Inc_Calc(PID_Struct *pid)
{
    // 计算当前误差 = 目标速度 - 实际速度
    pid->err = pid->target - pid->actual;
    
    // 计算增量: Δu
    // 增量公式: Kp*(e(k)-e(k-1)) + Ki*e(k) + Kd*(e(k)-2*e(k-1)+e(k-2))
    float increment = pid->Kp * (pid->err - pid->err_last) + 
                      pid->Ki * pid->err;
    
    // 更新历史误差，供下一次使用
    pid->err_prev = pid->err_last;     //上上次误差 e(k-2)
    pid->err_last = pid->err;		   //上次误差 e(k-1)
    
    // 累加增量到输出
    pid->output += (int)increment;
    
    // 输出限幅 (防止PWM溢出)
    if(pid->output > pid->max_out) pid->output = pid->max_out;
    //if(pid->output < 0)            pid->output = 0; // 假设只做正转控制
    if(pid->output < pid->min_out) pid->output = pid->min_out;
	
    return pid->output;
}
