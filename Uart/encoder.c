/**
 * @file    encoder.c
 * @brief   编码器读取、速度计算与电机PID速度控制模块
 * @details  
 */

#include "encoder.h"
#include "bsp_tb6612.h"
#include "board.h"

/* ===================== 编码器脉冲计数 ===================== */
/**
 * @brief   电机A（左轮）编码器累计脉冲数
   @brief   电机B（右轮）编码器累计脉冲数
 * @note    在GPIO外部中断中递增，使用volatile确保编译器不优化对其的读写
 */
volatile int Get_Encoder_countA = 0;
volatile int Get_Encoder_countB = 0;

/* ===================== 速度控制相关变量 ===================== */
volatile float target_speed_left  = 0.001f;                //左电机目标速度（编码器脉冲/秒）
volatile float target_speed_right = 0.001f;                //右电机目标速度（编码器脉冲/秒）
volatile float total_distance;                             //小车累计行驶距离
volatile float speed_1 = 0.0f;                             //电机A（左轮）的实时滤波后速度
volatile float speed_2 = 0.0f;                             //电机B（右轮）的实时滤波后速度

/**
 * @brief   速度斜坡系数，取值范围 0.0 ~ 1.0
 * @details 用于实现电机的缓加速和缓减速：
 *          - 实际输出速度 = 目标速度 × speed_ramp
 *          - 启动时从0.0逐渐增加到1.0（缓加速）
 *          - 停车时从1.0逐渐降低到0.0（缓减速/软停车）
 *          - Task1直道任务直接设为1.0，不做缓加速
 */
volatile float speed_ramp = 0.0f;                    

/* ===================== 速度斜坡步进参数 ===================== */
/**
 * @brief   加速步进值：每次PID周期（50ms）speed_ramp增加的量
 * @note    0.10f 意味着从0到满速约需 1.0/0.10 = 10个周期 = 0.5秒
 */
#define SPEED_RAMP_UP_STEP    0.10f

/**
 * @brief   减速步进值：每次PID周期（50ms）speed_ramp减少的量
 * @note    0.12f 意味着从满速到停止约需 1.0/0.12 ≈ 8.3个周期 ≈ 0.42秒
 *          减速比加速快，使停车响应更迅速
 */
#define SPEED_RAMP_DOWN_STEP  0.12f

/* ===================== PID控制结构体实例 ===================== */
PID_Struct pid_motor_left;                     /* 左电机（电机A）的PID控制器 */
PID_Struct pid_motor_right;                    /*（电机B）的PID控制器 */

/* ===================== 外部全局变量声明 ===================== */
extern uint8_t task;                          /* 当前任务编号（main.c中定义） */
extern uint8_t is_start;                      /* 启动标志（main.c中定义） */
extern volatile uint8_t car_stop_flag;        /* 停车标志（main.c中定义） */

/* ===================== 电机初始化 ===================== */
/**
 * @brief   初始化指定电机的PWM输出
 * @param   motor_id  电机编号：1=左电机（通道C1），2=右电机（通道C0）
 * @details 启动PWM定时器，并将对应通道的比较值设为0（初始占空比为0%，电机不转）
 */
void motor_init(uint8_t motor_id)
{
    if (motor_id == 1)
    {
        /* 左电机：启动PWM定时器，将通道C1的比较值设为0（初始停止状态） */
        DL_Timer_startCounter(PWM_0_INST);
        DL_Timer_setCaptureCompareValue(PWM_0_INST, 0, GPIO_PWM_0_C1_IDX);
    }
    else if (motor_id == 2)
    {
        /* 右电机：启动PWM定时器，将通道C0的比较值设为0（初始停止状态） */
        DL_Timer_startCounter(PWM_0_INST);
        DL_Timer_setCaptureCompareValue(PWM_0_INST, 0, GPIO_PWM_0_C0_IDX);
    }
}

/* ===================== 速度计算与滤波 ===================== */
/**
 * @brief   根据编码器脉冲差值计算滤波后的实时速度
 * @param   diff_count      本周期内编码器脉冲变化量
 * @param   filtered_speed  指向上一次滤波速度的指针（静态变量，跨调用保持）
 * @return  滤波后的速度值（单位：与宏定义相关，通常为 mm/s 或 脉冲/s）
 * @details 处理流程：
 *          1. 死区判断：脉冲变化量过小时认为速度为0，避免噪声干扰
 *          2. 原始速度计算：speed = (脉冲数 / 每转脉冲数) × π × 轮径 × 采样频率
 *          3. 一阶低通滤波：当检测到速度为0时，快速衰减（×0.5），加速收敛到0；
 *             当有速度时，使用一阶IIR滤波（alpha系数加权）平滑速度值，抑制高频噪声
 */
static float Speed_Update(int diff_count, float *filtered_speed)
{
    float raw_speed;

    /* 死区判断：脉冲变化量低于死区阈值时，认为车轮静止 */
    if (ABS(diff_count) < SPEED_PULSE_DEADZONE)
    {
        raw_speed = 0.0f;
    }
    else
    {
        /*
         * 原始速度计算公式：
         * raw_speed = (脉冲变化量 / 每转脉冲数) × π × 轮径（直径） × 采样频率  路程/时间
         * 其中各参数由encoder.h中的宏定义给出
         */
        raw_speed = (float)diff_count / MOTOR_XAINSHU * PI * MOTOR_WHEEL_D * SPEED_SAMPLE_HZ;
    }

    if (raw_speed == 0.0f)
    {
        /* 速度为0时：快速衰减滤波值（×0.5），使速度更快收敛到零，避免拖尾 */
        *filtered_speed *= 0.5f;
        if (ABS(*filtered_speed) < SPEED_ZERO_THRESH)
        {
            *filtered_speed = 0.0f;   /* 低于零速阈值时直接归零 */
        }
    }
    else
    {
        /*
         * 一阶IIR低通滤波：
         * filtered = filtered × (1 - alpha) + raw × alpha
         * alpha值越小，滤波越强（响应越慢但更平滑）
         */
        *filtered_speed = (*filtered_speed) * (1.0f - SPEED_FILTER_ALPHA)
                        + raw_speed * SPEED_FILTER_ALPHA;
    }

    return *filtered_speed;
}

/* ===================== 编码器GPIO中断服务函数 ===================== */
/**
 * @brief   GPIOB组中断服务函数（编码器脉冲计数）
 * @details 当编码器A相或B相产生上升/下降沿时触发此中断，
 *          对应的脉冲计数器加1。通过判断GPIOB上具体哪个引脚产生中断，
 *          分别累加左轮或右轮的编码器计数。
 * @note    此处仅做单相计数（非正交解码），适用于只需检测位移量的场景
 */
void GROUP1_IRQHandler(void)
{
    /* 获取GPIOB端口上待处理的中断标志 */
    switch (DL_GPIO_getPendingInterrupt(GPIOB))
    {
        /* 左轮编码器A相脉冲中断 → 左轮计数+1 */
        case ENCODERA_EAA_IIDX:
            Get_Encoder_countA++;
            break;
        /* 右轮编码器B相脉冲中断 → 右轮计数+1 */
        case ENCODERB_EBA_IIDX:
            Get_Encoder_countB++;
            break;
        default:
            break;
    }
}

/* ===================== 单轮速度计算函数 ===================== */
/**
 * @brief   计算左轮（电机A）的实时速度
 * @return  左轮滤波后速度（speed_1）
 * @details 每次调用时读取编码器A的累计脉冲数，与上一次的值做差，
 *          得到本周期内的脉冲变化量，再通过Speed_Update进行死区处理和滤波。
 *          使用static变量保存上次计数值和滤波值，实现跨调用状态保持。
 */
float caluclate_speed_1(void)
{
    static int last_countA = 0;          /* 上次编码器A的累计脉冲数（静态保持） */
    static float filtered_speed = 0.0f;  /* 上次滤波后的速度值（静态保持） */
    int current_countA;                  /* 本次编码器A的累计脉冲数 */
    int diff_countA;                     /* 本周期脉冲变化量 */

    current_countA = Get_Encoder_countA;          /* 读取当前累计脉冲 */
    diff_countA = current_countA - last_countA;   /* 计算本周期脉冲变化量 */
    last_countA = current_countA;                 /* 保存为下次的"上次值" */

    /* 通过死区判断 + 一阶低通滤波计算速度 */
    speed_1 = Speed_Update(diff_countA, &filtered_speed);
    return speed_1;
}

/**
 * @brief   计算右轮（电机B）的实时速度
 * @return  右轮滤波后速度（speed_2）
 * @details 逻辑与caluclate_speed_1()完全相同，操作的是编码器B的计数
 */
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
/* ===================== 速度斜坡控制 ===================== */
/**
 * @brief   更新速度斜坡系数（实现缓加速/缓减速）
 * @details 控制逻辑：
 *          1. 未启动时：speed_ramp强制归零
 *          2. Task1（直道高速）：直接设为1.0或0.0，不做渐变（需要快速响应）
 *          3. Task2/Task3（弯道循迹）：
 *             - 运行中：speed_ramp按SPEED_RAMP_UP_STEP逐步增加至上限1.0
 *             - 停车时：speed_ramp按SPEED_RAMP_DOWN_STEP逐步减少至下限0.0
 *          此函数每50ms在PID定时器中断中调用一次
 */
static void Speed_Ramp_Update(void)
{
    /* 未启动时强制归零，防止任何速度输出 */
    if (is_start == 0)
    {
        speed_ramp = 0.0f;
        return;
    }

    /* Task1直道任务：直接全速或直接停止，无需斜坡渐变 */
    if (task == 1)
    {
        speed_ramp = (car_stop_flag == 0) ? 1.0f : 0.0f;
        return;
    }

    if (car_stop_flag == 0)
    {
        /* 运行中：逐步增加斜坡系数（缓加速） */
        if (speed_ramp < 1.0f)
        {
            speed_ramp += SPEED_RAMP_UP_STEP;
            if (speed_ramp > 1.0f)
                speed_ramp = 1.0f;   /* 上限钳位 */
        }
    }
    else
    {
        /* 停车中：逐步减小斜坡系数（缓减速/软停车） */
        if (speed_ramp > 0.0f)
        {
            speed_ramp -= SPEED_RAMP_DOWN_STEP;
            if (speed_ramp < 0.0f)
                speed_ramp = 0.0f;   /* 下限钳位 */
        }
    }
}
/* ===================== 电机PID控制 ===================== */
/**
 * @brief   执行双电机的PID速度闭环控制
 * @details 控制流程：
 *          1. 计算实际指令速度 = 目标速度 × speed_ramp（斜坡系数）
 *          2. 低速保护：当两个电机的指令速度均低于2.0时，直接停止输出并清除PID状态，
 *             避免低速时PID积分累积导致的不稳定
 *          3. 分别对左右电机执行增量式PID计算，得到PWM占空比
 *          4. 通过AO_Control/BO_Control将PWM值输出到TB6612电机驱动芯片
 */
static void Motor_PID_Control(void)
{
    int pwm_left;           /* 左电机PID计算输出的PWM值 */
    int pwm_right;          /* 右电机PID计算输出的PWM值 */
    float cmd_speed_left;   /* 左电机实际指令速度（经斜坡调制后） */
    float cmd_speed_right;  /* 右电机实际指令速度（经斜坡调制后） */

    /* 将目标速度乘以斜坡系数，实现缓加速/缓减速效果 */
    cmd_speed_left = target_speed_left * speed_ramp;
    cmd_speed_right = target_speed_right * speed_ramp;

    /*
     * 低速保护：双轮指令速度均低于阈值时，直接停止电机输出
     * 目的：避免低速时PID积分项累积导致电机抖动或不可控行为
     */
    if (cmd_speed_left < 2.0f && cmd_speed_right < 2.0f)
    {
        pid_motor_left.output    = 0;     /* 清零PID输出 */
        pid_motor_right.output   = 0;
        pid_motor_left.err_last  = 0;     /* 清零上次误差，防止积分残留 */
        pid_motor_right.err_last = 0;
        AO_Control(0, 0);                 /* 左电机PWM输出为0 */
        BO_Control(0, 0);                 /* 右电机PWM输出为0 */
        return;
    }

    /* 左电机PID控制 */
    pid_motor_left.target = cmd_speed_left;   /* 设定目标速度 */
    pid_motor_left.actual = speed_1;           /* 反馈实际速度 */
    pwm_left = PID_Inc_Calc(&pid_motor_left);  /* 增量式PID计算，输出PWM值 */
    AO_Control(0, (uint32_t)pwm_left);        /* 设置左电机PWM占空比 */

    /* 右电机PID控制 */
    pid_motor_right.target = cmd_speed_right;  /* 设定目标速度 */
    pid_motor_right.actual = speed_2;          /* 反馈实际速度 */
    pwm_right = PID_Inc_Calc(&pid_motor_right); /* 增量式PID计算，输出PWM值 */
    BO_Control(0, (uint32_t)pwm_right);       /* 设置右电机PWM占空比 */
}

/* ===================== PID定时器中断服务函数 ===================== */
/**
 * @brief   电机PID定时器中断服务函数（每50ms触发一次）
 * @details 这是整个速度控制系统的核心调度入口，每50ms执行一次完整的控制周期：
 *          1. 速度斜坡更新（Speed_Ramp_Update）：实现缓加速/缓减速
 *          2. 左右轮速度计算（caluclate_speed_1/2）：读取编码器、滤波得到实时速度
 *          3. 距离积分（total_distance）：用左右轮平均速度乘以周期时间（0.05s）累加
 *          4. PID控制（Motor_PID_Control）：根据偏差计算PWM并驱动电机
 * @note    仅在启动状态且任务为1/2/3时执行，Task0为空闲任务不运行电机
 */
void MOTOR_PID_INST_IRQHandler(void)
{
    switch (DL_Timer_getPendingInterrupt(MOTOR_PID_INST))
    {
        /* 定时器LOAD事件（计数器重载）触发 → 执行一个完整的PID控制周期 */
        case DL_TIMER_IIDX_LOAD:
            if (is_start == 1 && (task == 1 || task == 2 || task == 3))
            {
                /* 步骤1：更新速度斜坡系数（缓加速/缓减速） */
                Speed_Ramp_Update();
                /* 步骤2：计算左右轮实时速度（编码器读取 + 死区判断 + 低通滤波） */
                caluclate_speed_1();
                caluclate_speed_2();
                /* 步骤3：累加行驶距离（仅在未停车时积分） */
                if (car_stop_flag == 0)
                {
                    /*
                     * 距离 = 速度 × 时间
                     * 使用左右轮平均速度，乘以PID周期（0.05秒 = 50ms）
                     */
                    total_distance += ((speed_1 + speed_2) / 2.0f) * 0.05f;
                }
                /* 步骤4：执行PID闭环控制，输出PWM驱动电机 */
                Motor_PID_Control();
            }
            break;
        default:
            break;
    }
}
