#ifndef _ENCODER_H
#define _ENCODER_H

#include "ti_msp_dl_config.h"
#include "bsp_tb6612.h"
#include "motor.h"

#define ABS(a)      ((a)>0 ? (a):(-(a)))

#define PI 3.1416f
/* MG310����������: ֻ��A��������ʱ, ÿת���� = 13�� x ���ٱ�
 * ���ٱ� 1:20 -> 260; ���ٱ� 1:30 -> 390, �밴�����޸� */
#define MOTOR_XAINSHU        260
#define MOTOR_WHEEL_D        48.0f
#define SPEED_SAMPLE_HZ      20.0f
#define SPEED_FILTER_ALPHA   0.30f
/* 50ms�����������ڸ���ֵ��Ϊ��ֹ, ���ƿ��عر�ʱ���������ٶ� */
#define SPEED_PULSE_DEADZONE 8
#define SPEED_ZERO_THRESH    5.0f

void motor_init(uint8_t motor_id);
extern volatile float speed_1;
extern volatile float speed_2;

extern PID_Struct pid_motor_left;                     /* �ٶȻ�PID�ṹ�� */
extern PID_Struct pid_motor_right;

extern volatile float target_speed_left;              /* Ŀ���ٶ� */
extern volatile float target_speed_right;
extern volatile float total_distance;                 /* ��ʻ�ܾ��� */
extern volatile float speed_ramp;                     /* ��ͣ�ٶ�б�� 0~1 */

extern volatile int Get_Encoder_countA, Get_Encoder_countB;

#endif