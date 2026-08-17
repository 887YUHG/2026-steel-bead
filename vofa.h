#ifndef VOFA_H
#define VOFA_H

#include "ti_msp_dl_config.h"

/* JustFloat 上报 4 通道：左目标、左实际、右目标、右实际 */
void VOFA_SendJustFloat(float ch0, float ch1, float ch2, float ch3);

/* 串口中断每收到 1 字节调用一次 */
void VOFA_RxByte(uint8_t rx_byte);

/* 主循环中调用，处理已完整接收的命令 */
void VOFA_ProcessCmd(void);

#endif /* VOFA_H */
