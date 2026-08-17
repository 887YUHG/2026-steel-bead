#include "encoder.h"
#include "bsp_tb6612.h"
#include "board.h"

volatile int Get_Encoder_countA = 0;
volatile int Get_Encoder_countB = 0;

volatile float target_speed_left  = 0.001f;
volatile float target_speed_right = 0.001f;
volatile float total_distance;
volatile float speed_1 = 0.0f;
volatile float speed_2 = 0.0f;
volatile float speed_ramp = 0.0f;                     /* ???��????? 0~1 */

/* ? 0.5s ??????? 0.4s ?????50ms ????? */
#define SPEED_RAMP_UP_STEP    0.10f
#define SPEED_RAMP_DOWN_STEP  0.12f

PID_Struct pid_motor_left;
PID_Struct pid_motor_right;

extern uint8_t task;
extern uint8_t is_start;
extern volatile uint8_t car_stop_flag;

void motor_init(uint8_t motor_id)
{
    if (motor_id == 1)
    {
        DL_Timer_startCounter(PWM_0_INST);
        DL_Timer_setCaptureCompareValue(PWM_0_INST, 0, GPIO_PWM_0_C1_IDX);
    }
    else if (motor_id == 2)
    {
        DL_Timer_startCounter(PWM_0_INST);
        DL_Timer_setCaptureCompareValue(PWM_0_INST, 0, GPIO_PWM_0_C0_IDX);
    }
}

static float Speed_Update(int diff_count, float *filtered_speed)
{
    float raw_speed;

    if (ABS(diff_count) < SPEED_PULSE_DEADZONE)
    {
        raw_speed = 0.0f;
    }
    else
    {
        raw_speed = (float)diff_count / MOTOR_XAINSHU * PI * MOTOR_WHEEL_D * SPEED_SAMPLE_HZ;
    }

    if (raw_speed == 0.0f)
    {
        *filtered_speed *= 0.5f;
        if (ABS(*filtered_speed) < SPEED_ZERO_THRESH)
        {
            *filtered_speed = 0.0f;
        }
    }
    else
    {
        *filtered_speed = (*filtered_speed) * (1.0f - SPEED_FILTER_ALPHA)
                        + raw_speed * SPEED_FILTER_ALPHA;
    }

    return *filtered_speed;
}

void GROUP1_IRQHandler(void)
{
    switch (DL_GPIO_getPendingInterrupt(GPIOB))
    {
        case ENCODERA_EAA_IIDX:
            Get_Encoder_countA++;
            break;

        case ENCODERB_EBA_IIDX:
            Get_Encoder_countB++;
            break;

        default:
            break;
    }
}

float caluclate_speed_1(void)
{
    static int last_countA = 0;
    static float filtered_speed = 0.0f;
    int current_countA;
    int diff_countA;

    current_countA = Get_Encoder_countA;
    diff_countA = current_countA - last_countA;
    last_countA = current_countA;

    speed_1 = Speed_Update(diff_countA, &filtered_speed);
    return speed_1;
}

float caluclate_speed_2(void)
{
    static int last_countB = 0;
    static float filtered_speed = 0.0f;
    int current_countB;
    int diff_countB;

    current_countB = Get_Encoder_countB;
    diff_countB = current_countB - last_countB;
    last_countB = current_countB;

    speed_2 = Speed_Update(diff_countB, &filtered_speed);
    return speed_2;
}

/* ???????��??????????????????? */
static void Speed_Ramp_Update(void)
{
    if (is_start == 0)
    {
        speed_ramp = 0.0f;
        return;
    }

    /* ����1��ֱ���������� / ֱ��ͣ��������������ͣ */
    if (task == 1)
    {
        speed_ramp = (car_stop_flag == 0) ? 1.0f : 0.0f;
        return;
    }

    if (car_stop_flag == 0)
    {
        if (speed_ramp < 1.0f)
        {
            speed_ramp += SPEED_RAMP_UP_STEP;
            if (speed_ramp > 1.0f)
                speed_ramp = 1.0f;
        }
    }
    else
    {
        if (speed_ramp > 0.0f)
        {
            speed_ramp -= SPEED_RAMP_DOWN_STEP;
            if (speed_ramp < 0.0f)
                speed_ramp = 0.0f;
        }
    }
}

static void Motor_PID_Control(void)
{
    int pwm_left;
    int pwm_right;
    float cmd_speed_left;
    float cmd_speed_right;

    cmd_speed_left = target_speed_left * speed_ramp;
    cmd_speed_right = target_speed_right * speed_ramp;

    if (cmd_speed_left < 2.0f && cmd_speed_right < 2.0f)
    {
        pid_motor_left.output    = 0;
        pid_motor_right.output   = 0;
        pid_motor_left.err_last  = 0;
        pid_motor_right.err_last = 0;
        AO_Control(0, 0);
        BO_Control(0, 0);
        return;
    }

    pid_motor_left.target = cmd_speed_left;
    pid_motor_left.actual = speed_1;
    pwm_left = PID_Inc_Calc(&pid_motor_left);
    AO_Control(0, (uint32_t)pwm_left);

    pid_motor_right.target = cmd_speed_right;
    pid_motor_right.actual = speed_2;
    pwm_right = PID_Inc_Calc(&pid_motor_right);
    BO_Control(0, (uint32_t)pwm_right);
}

/* ?????A0?��?: 50ms???? */
void MOTOR_PID_INST_IRQHandler(void)
{
    switch (DL_Timer_getPendingInterrupt(MOTOR_PID_INST))
    {
        case DL_TIMER_IIDX_LOAD:
            if (is_start == 1 && (task == 1 || task == 2 || task == 3))
            {
                Speed_Ramp_Update();
                caluclate_speed_1();
                caluclate_speed_2();
                if (car_stop_flag == 0)
                {
                    total_distance += ((speed_1 + speed_2) / 2.0f) * 0.05f;
                }
                Motor_PID_Control();
            }
            break;

        default:
            break;
    }
}
