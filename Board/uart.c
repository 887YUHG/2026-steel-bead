#include "uart.h"
#include "vofa.h"

/*
 *  发送字符函数
 *  无需配置中断接收
 */
void UART_send_char(UART_Regs *uart, const uint8_t chr)
{
    DL_UART_transmitDataBlocking(uart, chr);
}

/*
 *  发送字符串函数
 *  无需配置中断接收
 */
void UART_send_string(UART_Regs *uart, const char *str)
{
    while (*str)
    {
        UART_send_char(uart, (uint8_t)*str);
        str++;
    }
}

/*
 *  发送数组函数
 */
void UART_send_buffer(UART_Regs *uart, const uint8_t *buf, const uint8_t l)
{
    uint8_t byte_index;

    for (byte_index = 0; byte_index < l; byte_index++)
    {
        UART_send_char(uart, buf[byte_index]);
    }
}

/* 无线串口 UART0 接收中断：交给 VOFA 解析调参命令 */
void UART0_IRQHandler(void)
{
    switch (DL_UART_getPendingInterrupt(RPINT_INST))
    {
        case DL_UART_IIDX_RX:
        {
            uint8_t rec = DL_UART_receiveData(RPINT_INST);
            VOFA_RxByte(rec);
            break;
        }
        default:
            break;
    }
}
