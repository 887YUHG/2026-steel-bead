#include  "imu601.h"

uint8_t IMU_RX_buffer[12] = {0};       									//接收串口的数据
uint8_t IMU_RX_index = 0;
uint8_t IMU_last_byte = 0;
uint8_t is_receiving = 0;
Attitude_t current_attitude;											//定义的最后接收的变量  当前姿态

/*
 *  1，重启陀螺仪
 *  2，校准陀螺仪
 */
void IMU601_Init()
{
	const uint8_t IMU_reset[] = {0xAA, 0x55, 0x60, 0x12, 0x00, 0x72};		//需要发送一串字节（就是一个数组）定义
	UART_send_buffer(IMU601_INST,IMU_reset,sizeof(IMU_reset));				//发送一个软复位的指令
	delay_ms(400);
	
	const uint8_t IMU_cali[] = {0xAA, 0x55, 0x60, 0x14, 0x04, 0x00, 0x00, 0xB4, 0x43, 0x6F};	//需要发送一串字节（就是一个数组）定义
	UART_send_buffer(IMU601_INST,IMU_cali,sizeof(IMU_cali));				//校准
	
	NVIC_EnableIRQ(IMU601_INST_INT_IRQN);  //打开串口中断

}

void parse_attitude_only(const uint8_t *payload, Attitude_t *out_attitude)    //解析函数
{
    uint16_t yaw_raw   = (payload[1] << 8) | payload[0];
    int16_t  pitch_raw = (int16_t)((payload[3] << 8) | payload[2]);
    int16_t  roll_raw  = (int16_t)((payload[5] << 8) | payload[4]);
    out_attitude->yaw   = yaw_raw / 100.0f;
    out_attitude->pitch = pitch_raw / 100.0f;
    out_attitude->roll  = roll_raw / 100.0f;
}

void parse_imu601_data()   											//校验和，从第2个数据开始加到最后一个，如果正确的话就等于36（8位字节自带取余）
{
	uint8_t checksum = 0;
	for(int i = 2;i<11;i++)
	{
		checksum += IMU_RX_buffer[i];	
	}
	if(checksum == IMU_RX_buffer[11])     							 //如果等于0x36就代表接收成功，开始解析值
	{
		parse_attitude_only(&IMU_RX_buffer[5],&current_attitude);
	}

}

void IMU601_INST_IRQHandler()   									//接收中断服务函数
{	
	 switch( DL_UART_getPendingInterrupt(IMU601_INST) )				// 如果产生了串口中断
    {
        case DL_UART_IIDX_RX:										// 如果是接收中断
			{

				IMU_RX_buffer[IMU_RX_index] = DL_UART_receiveData(IMU601_INST);	// 读取 UART 接收寄存器里刚收到的 1 字节数据，存入变量 `rec
					if(IMU_RX_buffer[IMU_RX_index] == 0x55 &&  IMU_last_byte == 0xAA)
					{
						IMU_RX_index = 2;
						IMU_RX_buffer[0] = 0xAA;
						IMU_RX_buffer[1] = 0x55;
						is_receiving = 1;
					}
					else
					{
						IMU_RX_index++;
					}	
				IMU_last_byte = IMU_RX_buffer[IMU_RX_index-1];			
				if(IMU_RX_index >= 12)
				 {
					IMU_RX_index = 0;
					  is_receiving = 0;
					 parse_imu601_data();     							//到此数据解析完毕，存到了Attitude_t current_attitude，
				 }
				break;
			}                      
        default:														//其他的串口中断
            break;
    }

}

