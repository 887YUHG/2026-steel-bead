/**
 * @file    empty.c
 * @brief   基于TI MSPM0的智能小车多任务循迹控制主程序
 * @details  支持多个循迹任务(Task1/2/3)，通过按键切换任务和启停控制，
 *          集成8路红外循迹传感器、IMU601陀螺仪、TB6612电机驱动、LCD显示等功能。
 *          - Task1: 高速直道循迹 + 计时
 *          - Task2: AB型赛道循迹 + 软停车
 *          - Task3: AA型赛道循迹 + 软停车
 */


/* ===================== 头文件包含 ===================== */
#include "ti_msp_dl_config.h"    /* TI MSPM0 设备层配置头文件（由SysConfig工具自动生成） */
#include "board.h"               /* 开发板引脚定义与硬件抽象 */
#include "stdio.h"               /* 标准输入输出库 */
#include "lcd_init.h"            /* LCD显示屏初始化驱动 */
#include "lcd.h"                 /* LCD绘图与文字显示接口 */
#include "pic.h"                 /* 图片/图标数据（LCD显示用） */

#include "encoder.h"             /* 编码器读取与速度计算 */
#include "bsp_tb6612.h"          /* TB6612FNG电机驱动芯片BSP层 */
#include "motor.h"               /* 电机PID速度控制与状态管理 */
#include "TCRT5000.h"            /* TCRT5000红外循迹传感器驱动（8路） */
#include "uart.h"                /* 串口通信接口 */
#include "imu601.h"              /* IMU601六轴陀螺仪/加速度计驱动 */
#include "vofa.h"                /* VOFA+上位机调试协议（波形观察） */

/* ===================== 宏定义 ===================== */
char buf[16];								/** LCD显示缓冲区 */
#define SPEED_RUN_NORMAL   343.0f			/** Task1 高速直道循迹的基础运行速度（编码器脉冲/秒） */
#define SPEED_RUN_NORMAL_1   225.0f			/** Task2 低速赛道循迹的基础运行速度（编码器脉冲/秒） */
#define SPEED_RUN_NORMAL_2   225.0f			/** Task3 低速型赛道循迹的基础运行速度（编码器脉冲/秒） */
#define MAX_TASK_NUM       4				/** 最大任务数量（Task0~3，共4个任务槽位） */

/* ===================== 全局变量 ===================== */
/** 8路循迹传感器的原始读数缓冲（每一位对应一个传感器：1=检测到黑线，0=未检测到） */
uint8_t track_buf = 0;
/** 小车停车标志位：0=运行中，1=已停车。使用volatile因为在中断中也可能被修改 */
volatile uint8_t car_stop_flag = 0;
/** 按键消抖计数器：用于任务切换按键（KEY_PIN_26）的软件消抖 */
int counter_task = 0;
/** 当前选中的任务编号（0~3），通过按键循环切换 */
uint8_t task = 0;
/** 启动标志位：0=未启动，1=已按下启动键开始运行 */
uint8_t is_start = 0;
/** 启动按键消抖计数器：用于启动按键（KEY_PIN_24）的软件消抖 */
int counter_is_start = 0;

/* ============== 系统运行计时相关变量 ============== */
/** 系统运行时间计数器（毫秒），由SysTick中断每1ms递增一次，作为全局时间基准 */
volatile uint32_t g_system_ms = 0;

/** 单次计时起始时刻（毫秒），用于记录每个任务开始运行的时刻 */
uint32_t run_time_start_ms = 0;

/** 当前任务的累计运行时间（毫秒），停车后停止累计 */
uint32_t run_time_ms = 0;

/** 计时器激活标志：1=正在计时，0=计时已停止 */
uint8_t run_timer_active = 0;

/* ================ 系统滴答定时器中断 ================ */
void SysTick_Handler(void)
{
    g_system_ms++;
}
/* ===================== 按键读取函数 ===================== */
/**
 * @brief   读取按键状态（KEY_PIN_21引脚）
 * @param   key  引脚掩码（如KEY_PIN_21_PIN）
 * @return  0=按键未按下（高电平），1=按键已按下（低电平）
 * @note    硬件设计为低电平有效：按键按下时GPIO引脚被拉低
 */
uint8_t get_key_state(uint32_t key)
{
    /* 读取指定GPIO端口的引脚状态 */
    uint32_t high_bits = DL_GPIO_readPins(KEY_PIN_21_PORT, key);
    if ((high_bits & key) != 0)
        return 0;   /* 引脚为高电平 → 按键未按下 */
    else
        return 1;   /* 引脚为低电平 → 按键已按下 */
}

/**
 * @brief   读取按键状态（KEY_PIN_24引脚，启动键）
 * @param   key  引脚掩码（如KEY_PIN_24_PIN）
 * @return  0=按键未按下，1=按键已按下
 * @note    用于启动/重新开始当前任务
 */
uint8_t get_key_state_1(uint32_t key)
{
    uint32_t high_bits = DL_GPIO_readPins(KEY_PIN_24_PORT, key);
    if ((high_bits & key) != 0)
        return 0;
    else
        return 1;
}
/**
 * @brief   读取按键状态（KEY_PIN_26引脚，任务切换键）
 * @param   key  引脚掩码（如KEY_PIN_26_PIN）
 * @return  0=按键未按下，1=按键已按下
 * @note    用于在Task0~3之间循环切换当前任务
 */
uint8_t get_key_state_2(uint32_t key)
{
    uint32_t high_bits = DL_GPIO_readPins(KEY_PIN_26_PORT, key);
    if ((high_bits & key) != 0)
        return 0;
    else
        return 1;
}

/* ================== 运行计时器函数 ================== */
/**
 * @brief   重启运行计时器
 * @details 记录当前系统时间作为起始时刻，激活计时标志。
 *          在每次按下启动键时调用，用于重新开始任务计时。
 */
static void run_timer_restart(void)
{
    run_time_start_ms = g_system_ms;   /* 记录起始时刻 */
    run_time_ms = 0;                    /* 清零累计时间 */
    run_timer_active = 1;               /* 激活计时器 */
}

/**
 * @brief   更新运行计时器的累计时间
 * @details 在主循环中持续调用。当计时器激活且小车未停车时，
 *          实时计算并更新已运行时间。当检测到停车标志时，
 *          做最后一次计算后停止计时器。
 */
static void run_timer_update(void)
{
    if (run_timer_active == 0)
        return;   /* 计时器未激活，直接返回 */

    if (car_stop_flag == 0)
    {
        /* 小车运行中：持续更新累计运行时间 */
        run_time_ms = g_system_ms - run_time_start_ms;
    }
    else
    {
        /* 小车已停车：做最后一次时间计算，然后停止计时器 */
        run_time_ms = g_system_ms - run_time_start_ms;
        run_timer_active = 0;
    }
}

/**
 * @brief   小车软停车完成处理
 * @details 当电机速度斜坡降至接近零时，将左右目标速度设为0，
 *          并调用TB6612电机驱动停止函数，使小车完全停止。
 * @note    仅用于Task2和Task3的软停车流程；Task1使用直接硬停车。
 */
static void car_soft_stop_finish(void)
{
    if (speed_ramp > 0.01f)
        return;   /* 速度斜坡尚未降到足够低，继续等待减速 */

    /* 速度已足够低，执行最终停车 */
    target_speed_left = 0.0f;       /* 左电机目标速度归零 */
    target_speed_right = 0.0f;      /* 右电机目标速度归零 */
    TB6612_Motor_Stop();             /* TB6612芯片停止输出，电机断电 */
}

int main(void)
{
    /* ---------- 硬件初始化 ---------- */
    SYSCFG_DL_init();                /* TI SysConfig生成的系统时钟与外设初始化 */
    LCD_Init();                      /* 初始化LCD显示屏（SPI接口） */
    LCD_Fill(0, 0, LCD_W, LCD_H, WHITE);  /* LCD全屏刷白，清空显示 */
    TB6612_Motor_Stop();             /* 初始状态下电机停止，防止上电时电机失控转动 */

    /* 配置SysTick定时器：1ms中断周期，用于系统计时基准 */
    SysTick_Config(CPUCLK_FREQ / 1000);

    /* 初始化电机1（左轮）和电机2（右轮）的编码器与PID控制结构体 */
    motor_init(1);
    motor_init(2);
	
    /* 初始化IMU601六轴传感器（I2C接口），用于获取陀螺仪和加速度数据 */
    IMU601_Init();

    /* ---------- PID参数配置 ---------- */
    /* 清零PID输出值，防止初始化前有残留输出 */
    pid_motor_left.output = 0;
    pid_motor_right.output = 0;

    /* 设置PID输出限幅范围（0~999），对应PWM占空比范围 */
    pid_motor_left.max_out = 999;
    pid_motor_right.max_out = 999;
    pid_motor_left.min_out = 0;
    pid_motor_right.min_out = 0;

    /* 配置左电机PID参数：比例、积分、微分 */
    pid_motor_left.Kp = 1.f;         /* 比例系数：响应速度的主要调节参数 */
    pid_motor_left.Ki = 0.12f;       /* 积分系数：消除稳态误差 */
    pid_motor_left.Kd = 0.0f;        /* 微分系数：此处未使用微分控制 */

    /* 配置右电机PID参数（Ki略小于左电机，补偿两电机特性差异） */
    pid_motor_right.Kp = 1.f;
    pid_motor_right.Ki = 0.09f;
    pid_motor_right.Kd = 0.0f;
	
//    target_speed_left = 300.0f;
//    target_speed_right = 310.0f;
    /* 重新初始化电机（确保PID参数设置后电机状态正确） */
	motor_init(1);
    motor_init(2);

	/* 清除停车标志，确保初始状态为可运行 */
	car_stop_flag = 0;

    /* ---------- 中断与定时器使能 ---------- */
    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);  /* 使能GPIOB外部中断（用于编码器脉冲计数） */
    NVIC_EnableIRQ(RPINT_INST_INT_IRQN);            /* 使能编码器输入捕获中断 */
    DL_Timer_startCounter(PWM_0_INST);              /* 启动PWM定时器（电机驱动信号输出） */
    DL_Timer_startCounter(MOTOR_PID_INST);          /* 启动电机PID控制定时器（固定周期执行PID计算） */
    NVIC_EnableIRQ(MOTOR_PID_INST_INT_IRQN);        /* 使能PID定时器中断 */

    while (1)
    {
		
		/* ---- 任务切换按键检测（KEY_PIN_26） ---- */
		/* 使用计数器实现软件消抖：持续检测到按下时计数器递增，松开时递减 */
		if(get_key_state_2(KEY_PIN_26_PIN))
        {
            counter_task++;       /* 按键按下 → 计数器+1 */
        }
        else
        {
            counter_task--;       /* 按键松开 → 计数器-1，防止误触 */
            if (counter_task < 0)
                counter_task = 0;  /* 计数器不低于0 */
        }

        /* 计数器超过阈值（6次连续检测），确认为有效按键操作 */
        if (counter_task > 6)
        {
            task = (task + 1) % MAX_TASK_NUM;  /* 循环切换任务编号：0→1→2→3→0 */
            counter_task = 0;                    /* 重置消抖计数器 */
            is_start = 0;                        /* 重置启动标志 */
            car_stop_flag = 0;                   /* 清除停车标志 */
            speed_ramp = 0.0f;                   /* 速度斜坡归零 */
            run_timer_active = 0;                /* 停止计时器 */
            run_time_ms = 0;                     /* 清零运行时间 */
            target_speed_left = 0.0f;            /* 左电机目标速度归零 */
            target_speed_right = 0.0f;           /* 右电机目标速度归零 */               
            TB6612_Motor_Stop();                  /* 立即停止电机 */
            NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);  /* 重新使能编码器中断 */
        }

  /* ---- 启动按键检测（KEY_PIN_24） ---- */
        if (get_key_state_1(KEY_PIN_24_PIN))
        {
            counter_is_start++;   /* 按键按下 → 计数器+1 */
        }
        else
        {
            counter_is_start--;   /* 按键松开 → 计数器-1 */
            if (counter_is_start < 0)
                counter_is_start = 0;
        }

        /* 计数器超过阈值（5次），确认为有效启动操作 */
        if (counter_is_start > 5)
        {
            counter_is_start = 0;  /* 重置消抖计数器 */
			is_start = 1;            /* 标记为已启动 */
            car_stop_flag = 0;     /* 清除停车标志，允许运行 */
            /*
             * Task1为直道高速任务，需要立即全速启动（speed_ramp=1.0）
             * 其他任务从0开始缓加速（speed_ramp=0.0），通过斜坡函数逐渐提速
             */
            speed_ramp = (task == 1) ? 1.0f : 0.0f;
            run_timer_restart();    /* 重启运行计时器 */
        }
		
		/* ---- LCD状态信息显示（固定位置，所有任务通用） ---- */
		LCD_ShowString(2, 2, (u8 *)"Task", RED, WHITE, 16, 0);         /* 显示"Task"标签 */
        LCD_ShowIntNum(35, 2, task, 4, RED, WHITE, 16);                /* 显示当前任务编号 */
        LCD_ShowString(75, 2, (u8 *)"Start", RED, WHITE, 16, 0);      /* 显示"Start"标签 */
        LCD_ShowIntNum(110, 2, is_start, 4, RED, WHITE, 16);           /* 显示启动状态 */

	    /* 读取8路循迹传感器的状态（每一位代表一个传感器的检测值） */
	    track_buf = Read_8Track_Sensor();
		
	   /* ============== Task1: 高速直道循迹（带计时） ============== */
	    if (task == 1 && is_start == 1)
        {
			/* 显示循迹传感器原始值 */
			LCD_ShowString(10,25,(u8*)"TRACK:",RED,WHITE,16,0);
			LCD_ShowIntNum(50,25,track_buf,4,RED,WHITE,16);

			/* 更新并显示运行时间（秒） */
			run_timer_update();
			LCD_ShowString(10, 45, (u8 *)"Time:", RED, WHITE, 16, 0);
			LCD_ShowFloat(50, 45, run_time_ms / 1000.0f, RED, WHITE, 16);

				if (car_stop_flag == 0)
				{
					/* 调用高速循迹处理函数，根据传感器偏差调整左右电机速度 */
					Track_8Sensor_Process_high(track_buf, SPEED_RUN_NORMAL);
				}

				/* 行驶距离判断：6282个编码器脉冲 ≈ 一圈赛道的总长度 */
				if(total_distance >= 6282)
				{
					car_stop_flag = 1;  /* 达到目标距离，设置停车标志 */
				}

				/* 停车处理：Task1采用直接硬停车（立即切断电机电源） */
				if(car_stop_flag)
				{
					target_speed_left = 0.0f;       /* 左电机目标速度归零 */
					target_speed_right = 0.0f;      /* 右电机目标速度归零 */
					speed_ramp = 0.0f;              /* 速度斜坡归零 */
					TB6612_Motor_Stop();             /* 立即停止电机输出 */
				}
		}
	   /* ============== Task2: AB型赛道循迹（带软停车） ============== */
	   if (task == 2 && is_start == 1)
        {
			/* 显示循迹传感器原始值和行驶距离 */
			LCD_ShowString(10,25,(u8*)"TRACK:",RED,WHITE,16,0);
			LCD_ShowIntNum(50,25,track_buf,4,RED,WHITE,16);
			LCD_ShowString(10, 45, (u8 *)"Dist:", RED, WHITE, 16, 0);
            LCD_ShowIntNum(50, 45, (int)total_distance, 4, RED, WHITE, 16);
				if (car_stop_flag == 0)
				{
					/* 向STM32串口发送"DOWN"指令，驱动步进电机去抵消加速度导致的小球运动 */
					UART_send_string(IMU601_INST,"DOWN");
					/* 调用低速型循迹处理函数 */
					Track_8Sensor_Process_AB(track_buf, SPEED_RUN_NORMAL_1);
				}
				/* 行驶距离判断：1800个编码器脉冲后停车 */
				if(total_distance >= 1800)
				{
					car_stop_flag = 1;
				}
				/* 停车处理：Task2采用软停车（通过速度斜坡逐渐减速至停止） */
				if (car_stop_flag)
				{					
					car_soft_stop_finish();  /* 检查速度斜坡是否已降至阈值以下，若满足则完全停车 */
				}
		}
		/* ============== Task3:低速循迹（带软停车） ============== */
		if (task == 3 && is_start == 1)
        {
			/* 显示循迹传感器原始值 */
			LCD_ShowString(10, 25, (u8 *)"TRACK:", RED, WHITE, 16, 0);
            LCD_ShowIntNum(60, 25, track_buf, 4, RED, WHITE, 16);
				if (car_stop_flag == 0)
				{
					/*  向STM32串口发送"DOWN"指令，驱动步进电机去抵消加速度导致的小球运动 */
					UART_send_string(IMU601_INST,"DOWN");
					/* 调用低速循迹处理函数， */
					Track_8Sensor_Process_AA(track_buf, SPEED_RUN_NORMAL_2);
				}
				/* 行驶距离判断：6600个编码器脉冲后停车 */
				if(total_distance >= 6600)
				{
					car_stop_flag = 1;
				}
				/* 停车处理：Task3同样采用软停车策略 */
				if (car_stop_flag)
				{					
					car_soft_stop_finish();
				}
		}
    }
}
