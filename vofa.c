#include "vofa.h"
#include "uart.h"
#include "encoder.h"
#include "motor.h"
#include <stdlib.h>
#include <string.h>

#define VOFA_RX_BUF_SIZE 64

/* JustFloat 帧尾 */
static const uint8_t vofa_tail[4] = {0x00, 0x00, 0x80, 0x7F};

static char vofa_rx_buf[VOFA_RX_BUF_SIZE];
static volatile uint8_t vofa_rx_index = 0;
static volatile uint8_t vofa_cmd_ready = 0;
static char vofa_cmd_buf[VOFA_RX_BUF_SIZE];

/*
 * 函 数 名：VOFA_SendJustFloat
 * 功    能：按 JustFloat 协议发送 4 个 float 通道
 */
void VOFA_SendJustFloat(float ch0, float ch1, float ch2, float ch3)
{
    uint8_t frame_buf[20];
    float channel_data[4];
    uint8_t byte_index;

    channel_data[0] = ch0;
    channel_data[1] = ch1;
    channel_data[2] = ch2;
    channel_data[3] = ch3;

    for (byte_index = 0; byte_index < 16; byte_index++)
    {
        frame_buf[byte_index] = ((uint8_t *)channel_data)[byte_index];
    }

    frame_buf[16] = vofa_tail[0];
    frame_buf[17] = vofa_tail[1];
    frame_buf[18] = vofa_tail[2];
    frame_buf[19] = vofa_tail[3];

    UART_send_buffer(RPINT_INST, frame_buf, 20);
}

/*
 * 函 数 名：VOFA_RxByte
 * 功    能：接收一字节，遇换行则置命令就绪标志
 */
void VOFA_RxByte(uint8_t rx_byte)
{
    if (vofa_cmd_ready != 0U)
    {
        return;
    }

    if ((rx_byte == '\r') || (rx_byte == '\n'))
    {
        if (vofa_rx_index > 0U)
        {
            vofa_rx_buf[vofa_rx_index] = '\0';
            memcpy(vofa_cmd_buf, vofa_rx_buf, (size_t)(vofa_rx_index + 1U));
            vofa_rx_index = 0;
            vofa_cmd_ready = 1;
        }
        return;
    }

    if (vofa_rx_index < (VOFA_RX_BUF_SIZE - 1U))
    {
        vofa_rx_buf[vofa_rx_index] = (char)rx_byte;
        vofa_rx_index++;
    }
    else
    {
        vofa_rx_index = 0;
    }
}

/*
 * 函 数 名：VOFA_ApplyParam
 * 功    能：根据命令前缀写入对应 PID/目标速度
 */
static void VOFA_ApplyParam(const char *cmd_key, float param_value)
{
    if (strcmp(cmd_key, "LKp") == 0)
    {
        pid_motor_left.Kp = param_value;
    }
    else if (strcmp(cmd_key, "LKi") == 0)
    {
        pid_motor_left.Ki = param_value;
    }
    else if (strcmp(cmd_key, "LKd") == 0)
    {
        pid_motor_left.Kd = param_value;
    }
    else if (strcmp(cmd_key, "RKp") == 0)
    {
        pid_motor_right.Kp = param_value;
    }
    else if (strcmp(cmd_key, "RKi") == 0)
    {
        pid_motor_right.Ki = param_value;
    }
    else if (strcmp(cmd_key, "RKd") == 0)
    {
        pid_motor_right.Kd = param_value;
    }
    else if (strcmp(cmd_key, "LSpd") == 0)
    {
        target_speed_left = param_value;
    }
    else if (strcmp(cmd_key, "RSpd") == 0)
    {
        target_speed_right = param_value;
    }
    else if (strcmp(cmd_key, "ALLKp") == 0)
    {
        pid_motor_left.Kp = param_value;
        pid_motor_right.Kp = param_value;
    }
    else if (strcmp(cmd_key, "ALLKi") == 0)
    {
        pid_motor_left.Ki = param_value;
        pid_motor_right.Ki = param_value;
    }
    else if (strcmp(cmd_key, "ALLKd") == 0)
    {
        pid_motor_left.Kd = param_value;
        pid_motor_right.Kd = param_value;
    }
    else if (strcmp(cmd_key, "ALLSpd") == 0)
    {
        target_speed_left = param_value;
        target_speed_right = param_value;
    }
}

/*
 * 函 数 名：VOFA_ProcessCmd
 * 功    能：解析形如 LKp:1.70 的文本命令
 */
void VOFA_ProcessCmd(void)
{
    char *colon_pos;
    char cmd_key[16];
    float param_value;
    uint8_t key_len;

    if (vofa_cmd_ready == 0U)
    {
        return;
    }

    colon_pos = strchr(vofa_cmd_buf, ':');
    if (colon_pos != NULL)
    {
        key_len = (uint8_t)(colon_pos - vofa_cmd_buf);
        if ((key_len > 0U) && (key_len < sizeof(cmd_key)))
        {
            memcpy(cmd_key, vofa_cmd_buf, key_len);
            cmd_key[key_len] = '\0';
            param_value = (float)atof(colon_pos + 1);
            VOFA_ApplyParam(cmd_key, param_value);
        }
    }

    vofa_cmd_ready = 0;
}
