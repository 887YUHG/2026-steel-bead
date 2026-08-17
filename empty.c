#include "ti_msp_dl_config.h"
#include "board.h"
#include "stdio.h"
#include "lcd_init.h"
#include "lcd.h"
#include "pic.h"

#include "encoder.h"
#include "bsp_tb6612.h"
#include "motor.h"
#include "TCRT5000.h"
#include "uart.h"
#include "imu601.h"
#include "vofa.h"

char buf[16];
#define SPEED_RUN_NORMAL   343.0f            // ??????????
#define SPEED_RUN_NORMAL_1   225.0f          // ??????????
#define SPEED_RUN_NORMAL_2   225.0f          // ??????????
#define MAX_TASK_NUM       4             	 // ????????

uint8_t track_buf = 0;                    	 // ????????????
volatile uint8_t car_stop_flag = 0; 		 // ?????????
int counter_task = 0;
uint8_t task = 0;
uint8_t is_start = 0;
int counter_is_start = 0;

/* ???????????????????????????????? */
volatile uint32_t g_system_ms = 0;
uint32_t run_time_start_ms = 0;
uint32_t run_time_ms = 0;
uint8_t run_timer_active = 0;

void SysTick_Handler(void)
{
    g_system_ms++;
}

uint8_t get_key_state(uint32_t key)
{
    uint32_t high_bits = DL_GPIO_readPins(KEY_PIN_21_PORT, key);
    if ((high_bits & key) != 0)
        return 0;
    else
        return 1;
}

uint8_t get_key_state_1(uint32_t key)
{
    uint32_t high_bits = DL_GPIO_readPins(KEY_PIN_24_PORT, key);
    if ((high_bits & key) != 0)
        return 0;
    else
        return 1;
}

uint8_t get_key_state_2(uint32_t key)
{
    uint32_t high_bits = DL_GPIO_readPins(KEY_PIN_26_PORT, key);
    if ((high_bits & key) != 0)
        return 0;
    else
        return 1;
}

/* ????/????????1??? */
static void run_timer_restart(void)
{
    run_time_start_ms = g_system_ms;
    run_time_ms = 0;
    run_timer_active = 1;
}

/* ??????????????????????? */
static void run_timer_update(void)
{
    if (run_timer_active == 0)
        return;

    if (car_stop_flag == 0)
    {
        run_time_ms = g_system_ms - run_time_start_ms;
    }
    else
    {
        run_time_ms = g_system_ms - run_time_start_ms;
        run_timer_active = 0;
    }
}

/* ??????????????????? */
static void car_soft_stop_finish(void)
{
    if (speed_ramp > 0.01f)
        return;

    target_speed_left = 0.0f;
    target_speed_right = 0.0f;
    TB6612_Motor_Stop();
//    DL_GPIO_clearPins(LED_PORT, LED_PIN_22_PIN);
//    delay_ms(100);
//    DL_GPIO_setPins(LED_PORT, LED_PIN_22_PIN);
//    delay_ms(100);
}

int main(void)
{
    SYSCFG_DL_init();
    LCD_Init();
    LCD_Fill(0, 0, LCD_W, LCD_H, WHITE);
    TB6612_Motor_Stop();

    /* 1ms ?????????????/?????????? */
    SysTick_Config(CPUCLK_FREQ / 1000);

    motor_init(1);
    motor_init(2);

    IMU601_Init();

    /* ????? PID ???? */
    pid_motor_left.output = 0;
    pid_motor_right.output = 0;
    pid_motor_left.max_out = 999;
    pid_motor_right.max_out = 999;
    pid_motor_left.min_out = 0;
    pid_motor_right.min_out = 0;

    /* PID ????????? VOFA ??????????? */
    pid_motor_left.Kp = 1.f;
    pid_motor_left.Ki = 0.12f;
    pid_motor_left.Kd = 0.0f;

    pid_motor_right.Kp = 1.f;
    pid_motor_right.Ki = 0.09f;
    pid_motor_right.Kd = 0.0f;

//    target_speed_left = 300.0f;
//    target_speed_right = 310.0f;
	motor_init(1);
    motor_init(2);

	car_stop_flag = 0;
    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);
    NVIC_EnableIRQ(RPINT_INST_INT_IRQN);
    DL_Timer_startCounter(PWM_0_INST);
    DL_Timer_startCounter(MOTOR_PID_INST);
    NVIC_EnableIRQ(MOTOR_PID_INST_INT_IRQN);

    while (1)
    {
		
		if(get_key_state_2(KEY_PIN_26_PIN))
        {
            counter_task++;
        }
        else
        {
            counter_task--;
            if (counter_task < 0)
                counter_task = 0;
        }
        if (counter_task > 6)
        {
            task = (task + 1) % MAX_TASK_NUM;
            counter_task = 0;
            is_start = 0;
            car_stop_flag = 0;
            speed_ramp = 0.0f;
            run_timer_active = 0;
            run_time_ms = 0;
            target_speed_left = 0.0f;
            target_speed_right = 0.0f;               
            TB6612_Motor_Stop();
            NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);
        }

        if (get_key_state_1(KEY_PIN_24_PIN))
        {
            counter_is_start++;
        }
        else
        {
            counter_is_start--;
            if (counter_is_start < 0)
                counter_is_start = 0;
        }

        if (counter_is_start > 5)
        {
            counter_is_start = 0;
			is_start = 1;
            car_stop_flag = 0;
            /* task1?????????????????0???? */
            speed_ramp = (task == 1) ? 1.0f : 0.0f;
            run_timer_restart();
        }
		
		LCD_ShowString(2, 2, (u8 *)"Task", RED, WHITE, 16, 0);
        LCD_ShowIntNum(35, 2, task, 4, RED, WHITE, 16);
        LCD_ShowString(75, 2, (u8 *)"Start", RED, WHITE, 16, 0);
        LCD_ShowIntNum(110, 2, is_start, 4, RED, WHITE, 16);

	    track_buf = Read_8Track_Sensor();
		
	    if (task == 1 && is_start == 1)
        {
			LCD_ShowString(10,25,(u8*)"TRACK:",RED,WHITE,16,0);
			LCD_ShowIntNum(50,25,track_buf,4,RED,WHITE,16);

			run_timer_update();
			LCD_ShowString(10, 45, (u8 *)"Time:", RED, WHITE, 16, 0);
			LCD_ShowFloat(50, 45, run_time_ms / 1000.0f, RED, WHITE, 16);

				if (car_stop_flag == 0)
				{
					Track_8Sensor_Process_high(track_buf, SPEED_RUN_NORMAL);
				}
				if(total_distance >= 6282)
				{
					car_stop_flag = 1;
				}
				if(car_stop_flag)
				{
					/* ����1��ֱ��ͣ�����޻�ͣ */
					target_speed_left = 0.0f;
					target_speed_right = 0.0f;
					speed_ramp = 0.0f;
					TB6612_Motor_Stop();
				}
		}
	   if (task == 2 && is_start == 1)
        {
			LCD_ShowString(10,25,(u8*)"TRACK:",RED,WHITE,16,0);
			LCD_ShowIntNum(50,25,track_buf,4,RED,WHITE,16);
			LCD_ShowString(10, 45, (u8 *)"Dist:", RED, WHITE, 16, 0);
            LCD_ShowIntNum(50, 45, (int)total_distance, 4, RED, WHITE, 16);
				if (car_stop_flag == 0)
				{
					UART_send_string(IMU601_INST,"DOWN");
					Track_8Sensor_Process_AB(track_buf, SPEED_RUN_NORMAL_1);
				}
				if(total_distance >= 1800)
				{
					car_stop_flag = 1;
				}
				if (car_stop_flag)
				{			
					car_soft_stop_finish();
				}
		}
		if (task == 3 && is_start == 1)
        {
			LCD_ShowString(10, 25, (u8 *)"TRACK:", RED, WHITE, 16, 0);
            LCD_ShowIntNum(60, 25, track_buf, 4, RED, WHITE, 16);
				if (car_stop_flag == 0)
				{
					UART_send_string(IMU601_INST,"DOWN");
					Track_8Sensor_Process_AA(track_buf, SPEED_RUN_NORMAL_2);
				}
				if(total_distance >= 6600)
				{
					car_stop_flag = 1;
				}
				if (car_stop_flag)
				{				
					car_soft_stop_finish();
				}
		}
    }
}
