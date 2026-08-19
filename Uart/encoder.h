#ifndef _ENCODER_H
#define _ENCODER_H

#include "ti_msp_dl_config.h"
#include "bsp_tb6612.h"
#include "motor.h"

/**
 * @brief   取绝对值宏
 * @param   a  输入数值
 * @return  a的绝对值
 * @note    使用三目运算符实现，仅适用于整型和浮点型
 */
#define ABS(a)      ((a)>0 ? (a):(-(a)))

#define PI 3.1416f
/* ===================== 电机与轮子机械参数 ===================== */
/**
 * @brief   编码器每转脉冲数（PPR - Pulses Per Revolution）
 * @details 计算公式：PPR = 编码器盘片线数 × 减速比
 *
 *          本项目使用MG310减速电机：
 *          - 编码器盘片线数 = 13线（单圈产生13个脉冲）
 *          - 减速比 = 1:20（电机轴转20圈，输出轴转1圈）
 *          - 因此：PPR = 13 × 20 = 260
 *          注意：这里使用的是单相单边沿计数方式，
 *          若改用正交解码（4倍频），该值应为 13 × 20 × 4 = 1040
 *          若减速比改为1:30，则该值应为 13 × 30 = 390
 *          请根据实际硬件配置修改此值！
 */
#define MOTOR_XAINSHU        260

/**
 * @brief   驱动轮直径（单位：mm）
 * @details 用于计算轮子周长：周长 = π × D = 3.1416 × 48.0 ≈ 150.8mm
 *          请根据实际使用的车轮直径修改此值
 */
#define MOTOR_WHEEL_D        48.0f

/* ===================== 速度采样与滤波参数 ===================== */
/**
 * @brief   速度采样频率（单位：Hz）
 * @details 20.0f 表示每秒采样20次，即采样周期 = 1/20 = 50ms
 *          与PID定时器中断周期一致（MOTOR_PID_INST配置为50ms触发一次）
 *          在速度计算公式中的作用：
 *          speed = (脉冲变化量 / 每转脉冲数) × π × 轮径 × 采样频率
 *          乘以采样频率将"每个采样周期的距离"转换为"每秒的距离"（即速度）
 */
#define SPEED_SAMPLE_HZ      20.0f

/**
 * @brief   一阶IIR低通滤波器的平滑系数 alpha
 * @details 取值范围 0.0 ~ 1.0，控制滤波强度：
 *          - alpha 越大 → 响应越快，但滤波效果越弱（噪声多）
 *          - alpha 越小 → 滤波越强（更平滑），但响应延迟越大
 *          0.30f 为中等偏快的响应，在循迹场景中平衡了速度实时性和平滑性
 *          滤波公式：filtered = filtered × (1 - alpha) + raw × alpha
 */
#define SPEED_FILTER_ALPHA   0.30f

/**
 * @brief   速度计算脉冲死区阈值（单位：脉冲数）
 * @details 在50ms采样周期内，如果编码器脉冲变化量低于此值（绝对值 < 8），
 *          则认为车轮静止，速度直接置0。
 *          作用：
 *          1. 消除编码器在静止时的噪声脉冲（电磁干扰、机械振动等）
 *          2. 防止小车关闭电机后因残余脉冲导致速度不为0
 *          对应的最低可检测速度 ≈ (8 / 260) × π × 48.0 × 20 ≈ 92.9 mm/s
 */
#define SPEED_PULSE_DEADZONE 8

/**
 * @brief   零速判定阈值（单位：与速度计算结果同单位，通常为 mm/s）
 * @details 当滤波后速度的绝对值低于此值时，强制归零。
 *          这是第二层零速保护，配合SPEED_PULSE_DEADZONE使用：
 *          - 第一层：原始脉冲死区（SPEED_PULSE_DEADZONE）→ raw_speed = 0
 *          - 第二层：滤波值衰减后，低于此阈值时彻底归零
 *          确保小车完全静止时速度显示和距离积分都准确停止
 */
#define SPEED_ZERO_THRESH    5.0f

void motor_init(uint8_t motor_id);

/* ===================== 外部全局变量声明 ===================== */
extern volatile float speed_1;                          /* 左电机（电机A）实时滤波后速度 */
extern volatile float speed_2;
extern PID_Struct pid_motor_left;                       /* 左电机速度闭环PID控制器结构体实例 */
extern PID_Struct pid_motor_right;
extern volatile float target_speed_left;                /* 左电机目标速度（由循迹函数或main函数设定） */
extern volatile float target_speed_right;
extern volatile float total_distance;                  /* 小车累计行驶总距离（由PID中断中速度积分得到） */

/**
 * @brief   速度斜坡系数，取值范围 0.0 ~ 1.0
 * @details 实际输出速度 = 目标速度 × speed_ramp
 *          启动时从0渐增到1（缓加速），停车时从1渐减到0（缓减速）
 */
extern volatile float speed_ramp;

extern volatile int Get_Encoder_countA, Get_Encoder_countB;

#endif
