#include "TCRT5000.h"

#define delay_ms(X)		delay_cycles( (CPUCLK_FREQ/1000) * (X) )

extern volatile uint8_t car_stop_flag;
extern uint8_t task;
/******************************************************************
 * �� �� �� �ƣ�Get_OUT1
 * �� �� ˵ ������ȡ������ʶ��״̬
 * �� �� �� �Σ���
 * �� �� �� �أ�1=ʶ��Ϊ��ɫ   0=ʶ��Ĳ��Ǻ�ɫ
 * ��       �ߣ�LCKFB
 * ��       ע������ͨ��ģ���ϵĿɵ��������ʶ���ɫ����ֵ
******************************************************************/
unsigned char Get_OUT1(void)
{
    if( OUT1 == 1 )
    {
        return 1;
    }
    else
    {
        return 0;
    }
}


unsigned char Get_OUT2(void)
{
    if( OUT2 == 1 )
    {
        return 1;
    }
    else
    {
        return 0;
    }
}


unsigned char Get_OUT3(void)
{
    if( OUT3 == 1 )
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

unsigned char Get_OUT4(void)
{
    if( OUT4 == 1 )
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

unsigned char Get_OUT5(void)
{
    if( OUT5 == 1 )
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

unsigned char Get_OUT6(void)
{
    if( OUT6 == 1 )
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

unsigned char Get_OUT7(void)
{
    if( OUT7 == 1 )
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

unsigned char Get_OUT8(void)
{
    if( OUT8 == 1 )
    {
        return 1;
    }
    else
    {
        return 0;
    }
}


//// ��ȡ8·������=0����ֽ=1����һ�ζ�ȡ�˸�״̬��bit7=S1 ... bit0=S8
uint8_t Read_8Track_Sensor(void)
{
    uint8_t val = 0;
    val |= (S1) << 7;
    val |= (S2) << 6;
    val |= (S3) << 5;
    val |= (S4) << 4;
    val |= (S5) << 3;
    val |= (S6) << 2;
    val |= (S7) << 1;
    val |= (S8) << 0;
    return val;
}

void Track_8Sensor_Process_high(uint8_t track_buf, float speed_run)
{
    switch(track_buf)
    {
        //==================================================================
        // 一、全白 / 无线检测 → 直行
        //==================================================================
        case 0:     // 全白 (0000 0000)
            target_speed_left = speed_run+2;
            target_speed_right = speed_run+10;           
            break;

		case 24:     // S4+S5 双中检测 (0001 1000) - 理想状态
			target_speed_left = speed_run-11;
            target_speed_right = speed_run+5;
            break;
        //==================================================================
        // 二、正中线附近 → 微调直行
        //==================================================================
        
		case 16:	 // 仅S4检测到 (0001 0000) - 稍偏右
		case 48:	 // S3+S4检测到 (0011 0000)
	    case 8:      // 仅S5检测到 (0000 1000) - 稍偏左
            target_speed_left = speed_run-15;
            target_speed_right = speed_run+10;
            break;
                       
        //==================================================================
        // 三、轻微偏移 → 小幅度修正
        //==================================================================
        
        // --- 向左偏（线在左边，需要向左转）---
        case 32:    // S3检测到 (0010 0000)
		case 96:    // S2+S3检测到 (0110 0000)
            target_speed_left = speed_run - 50;
            target_speed_right = speed_run + 35;
            break;
                       
        // --- 向右偏（线在右边，需要向右转）---
		
		case 12:    // S5+S6检测到 (0000 1100)
        target_speed_left = speed_run + 45;
        target_speed_right = speed_run - 60;
            break;
				
        case 4:     // S6检测到 (0000 0100)
		case 6:     // S6+S7检测到 (0000 0110)
            target_speed_left = speed_run + 80;
            target_speed_right = speed_run - 90;
            break;
            
        //==================================================================
        // 四、中度偏移 → 中等幅度修正
        //==================================================================
        
        // --- 明显向左偏 ---
        case 64:    // S2检测到 (0100 0000)
     
            target_speed_left = speed_run - 100;
            target_speed_right = speed_run + 100;
            break;
                       
        // --- 明显向右偏 ---
        case 2:     // S7检测到 (0000 0010)			
            target_speed_left = speed_run + 105;
            target_speed_right = speed_run - 110;
            break;
           
        //==================================================================
        // 五、大幅度偏移 → 急转修正
        //==================================================================
        
        // --- 严重向左偏 ---
        case 128:   // 仅S1检测到 (1000 0000) - 最左边！
		case 192:   // S1+S2检测到 (1100 0000)
            target_speed_left = speed_run - 190;
            target_speed_right = speed_run + 175;
            break;            		
	
        // --- 严重向右偏 ---
        case 1:     // 仅S8检测到 (0000 0001) - 最右边！
		case 3:		// 仅S7 S8,检测到 (0000 0011) - 最右边！
            target_speed_left = speed_run + 195;
            target_speed_right = speed_run - 195;
		delay_ms(20);
            break;      
//         case 255:   // 全黑 (1111 1111) - 十字路口/终点/T字路口
//		 case 124:
//		 case 112:
//		 case 224:			 
//		 case 31: 
//		 case 30:
//		 case 60: 
//		 case 240: 
//		 case 248: 
//		 case 252:
//		 case 120:			 
//		 case 62: 
//		 case 63:
//		 case 56:
//            /* task1直接停车清零目标；其它任务保留目标供缓停 */
//            if (task == 1)
//            {
//                target_speed_left = 0;
//                target_speed_right = 0;
//            }
//			car_stop_flag = 1;
//            break;		
/*		
        case 7:     // S6+S7+S8检测到 (0000 0111)      
            target_speed_left = speed_run + 70;    // 加速
            target_speed_right = speed_run - 85;   // 大幅减速/反转
            break;
*/
        default:
            break;
    }
}


/**
 * @brief 八路循迹传感器处理函数（十进制版本）
 * @param track_buf 八路传感器读值 (0~255)
 * @param speed_run 基础运行速度 (mm/s)

 *         从A点到B点循迹，带小球平衡的循迹差速比

 * 传感器排列: S1(左) S2 S3 S4 S5 S6 S7 S8(右)
 * 对应bit:    bit7  6   5   4   3   2   1   bit0
 */
void Track_8Sensor_Process_AB(uint8_t track_buf, float speed_run)
{
    switch(track_buf)
    {
        //==================================================================
        // 一、全白 / 无线检测 → 直行
        //==================================================================
        case 0:     // 全白 (0000 0000)
            target_speed_left = speed_run+3;
            target_speed_right = speed_run+10;           
            break;

        //==================================================================
        // 二、正中线附近 → 微调直行
        //==================================================================
        case 24:     // S4+S5 双中检测 (0001 1000) - 理想状态
		case 16:	 // 仅S4检测到 (0001 0000) - 稍偏右
		case 48:	 // S3+S4检测到 (0011 0000)
	    case 8:      // 仅S5检测到 (0000 1000) - 稍偏左
            target_speed_left = speed_run-5;
            target_speed_right = speed_run+5;
            break;
                       
        //==================================================================
        // 三、轻微偏移 → 小幅度修正
        //==================================================================
        
        // --- 向左偏（线在左边，需要向左转）---
        case 32:    // S3检测到 (0010 0000)
		case 96:    // S2+S3检测到 (0110 0000)
            target_speed_left = speed_run - 55;
            target_speed_right = speed_run + 30;
            break;
                       
        // --- 向右偏（线在右边，需要向右转）---
		
		case 12:    // S5+S6检测到 (0000 1100)
        target_speed_left = speed_run + 35;
        target_speed_right = speed_run - 55;
            break;
				
        case 4:     // S6检测到 (0000 0100)
		case 6:     // S6+S7检测到 (0000 0110)
            target_speed_left = speed_run + 70;
            target_speed_right = speed_run - 95;
            break;
            
        //==================================================================
        // 四、中度偏移 → 中等幅度修正
        //==================================================================
        
        // --- 明显向左偏 ---
        case 64:    // S2检测到 (0100 0000)
     
            target_speed_left = speed_run - 110;
            target_speed_right = speed_run + 100;
            break;
                       
        // --- 明显向右偏 ---
        case 2:     // S7检测到 (0000 0010)			
            target_speed_left = speed_run + 85;
            target_speed_right = speed_run - 120;
            break;
           
        //==================================================================
        // 五、大幅度偏移 → 急转修正
        //==================================================================
        
        // --- 严重向左偏 ---
        case 128:   // 仅S1检测到 (1000 0000) - 最左边！
		case 192:   // S1+S2检测到 (1100 0000)
            target_speed_left = speed_run - 160;
            target_speed_right = speed_run + 160;
            break;            		
		
        // --- 严重向右偏 ---
        case 1:     // 仅S8检测到 (0000 0001) - 最右边！
		case 3:		// 仅S7 S8,检测到 (0000 0011) - 最右边！
            target_speed_left = speed_run + 170;
            target_speed_right = speed_run - 175;
		delay_ms(20);
            break;      
         case 255:   // 全黑 (1111 1111) - 十字路口/终点/T字路口
		 case 124:
		 case 112:
		 case 224:			 
		 case 31: 
		 case 30:
		 case 60: 
		 case 240: 
		 case 248: 
		 case 252:
		 case 120:			 
		 case 62: 
		 case 63:
		 case 56:
            /* task1直接停车清零目标；其它任务保留目标供缓停 */
            if (task == 1)
            {
                target_speed_left = 0;
                target_speed_right = 0;
            }
			car_stop_flag = 1;
            break;		
/*		
        case 7:     // S6+S7+S8检测到 (0000 0111)
        case 15:    // 右半边全检测到 (0000 1111)
            target_speed_left = speed_run + 70;    // 加速
            target_speed_right = speed_run - 85;   // 大幅减速/反转
            break;
*/
        default:
            break;
    }
}

/**
 * @brief 八路循迹传感器处理函数（十进制版本）
 * @param track_buf 八路传感器读值 (0~255)
 * @param speed_run 基础运行速度 (mm/s)

 *         第5和6问整圈循迹用于平衡球的循迹差速比

 * 传感器排列: S1(左) S2 S3 S4 S5 S6 S7 S8(右)
 * 对应bit:    bit7  6   5   4   3   2   1   bit0
 */
void Track_8Sensor_Process_AA(uint8_t track_buf, float speed_run)
{
    switch(track_buf)
    {
        //==================================================================
        // 一、全白 / 无线检测 → 直行
        //==================================================================
        case 0:     // 全白 (0000 0000)
            target_speed_left = speed_run+2;
            target_speed_right = speed_run+10;           
            break;

        //==================================================================
        // 二、正中线附近 → 微调直行
        //==================================================================
        case 24:     // S4+S5 双中检测 (0001 1000) - 理想状态
		case 16:	 // 仅S4检测到 (0001 0000) - 稍偏右
		case 48:	 // S3+S4检测到 (0011 0000)
	    case 8:      // 仅S5检测到 (0000 1000) - 稍偏左
            target_speed_left = speed_run-5;
            target_speed_right = speed_run+5;
            break;
                       
        //==================================================================
        // 三、轻微偏移 → 小幅度修正
        //==================================================================
        
        // --- 向左偏（线在左边，需要向左转）---
        case 32:    // S3检测到 (0010 0000)
		case 96:    // S2+S3检测到 (0110 0000)
            target_speed_left = speed_run - 55;
            target_speed_right = speed_run + 30;
            break;
                       
        // --- 向右偏（线在右边，需要向右转）---
		
		case 12:    // S5+S6检测到 (0000 1100)
        target_speed_left = speed_run + 35;
        target_speed_right = speed_run - 55;
            break;
				
        case 4:     // S6检测到 (0000 0100)
		case 6:     // S6+S7检测到 (0000 0110)
            target_speed_left = speed_run + 70;
            target_speed_right = speed_run - 95;
            break;
            
        //==================================================================
        // 四、中度偏移 → 中等幅度修正
        //==================================================================
        
        // --- 明显向左偏 ---
        case 64:    // S2检测到 (0100 0000)
     
            target_speed_left = speed_run - 110;
            target_speed_right = speed_run + 100;
            break;
                       
        // --- 明显向右偏 ---
        case 2:     // S7检测到 (0000 0010)			
            target_speed_left = speed_run + 85;
            target_speed_right = speed_run - 120;
            break;
           
        //==================================================================
        // 五、大幅度偏移 → 急转修正
        //==================================================================
        
        // --- 严重向左偏 ---
        case 128:   // 仅S1检测到 (1000 0000) - 最左边！
		case 192:   // S1+S2检测到 (1100 0000)
            target_speed_left = speed_run - 160;
            target_speed_right = speed_run + 160;
            break;            		
/*		
        case 224:   // S1+S2+S3检测到 (1110 0000)			
        case 240:   // 左半边全检测到 (1111 0000)
            target_speed_left = speed_run - 85;   // 大幅减速/反转
            target_speed_right = speed_run + 70;   // 加速
            break;           
        case 112:   // S2+S3+S4检测到 (0111 0000)
        case 120:   // S2+S3+S4+S5检测到 (0111 1000)
            target_speed_left = speed_run - 60;
            target_speed_right = speed_run + 50;
            break;
*/		
        // --- 严重向右偏 ---
        case 1:     // 仅S8检测到 (0000 0001) - 最右边！
		case 3:		// 仅S7 S8,检测到 (0000 0011) - 最右边！
            target_speed_left = speed_run + 170;
            target_speed_right = speed_run - 175;
		delay_ms(20);
            break;    
		
//         case 255:   // 全黑 (1111 1111) - 十字路口/终点/T字路口
//		 case 124:
//		 case 112:
//		 case 224:			 
//		 case 31: 
//		 case 30:
//		 case 60: 
//		 case 240: 
//		 case 248: 
//		 case 252:
//		 case 120:			 
//		 case 62: 
//		 case 63:
//		 case 56:
//            /* task1直接停车清零目标；其它任务保留目标供缓停 */
//            if (task == 1)
//            {
//                target_speed_left = 0;
//                target_speed_right = 0;
//            }
//			car_stop_flag = 1;
//            break;		

        default:
            break;
    }
}

/*
//// 读取5路（黑线=0，白纸=1）    这个是封装好的，可以一次读取五个状态
//uint8_t Read_5Track_Sensor(void)
//{
//    uint8_t val = 0;
//    val |= (S1) << 4;
//    val |= (S2) << 3;
//    val |= (S3) << 2;
//    val |= (S4) << 1;
//    val |= (S5) << 0;
//    return val;
//}

//void Track_8Sensor_Process(uint8_t track_buf, float speed_run)
//{
//	switch(track_buf)
//         {
//				// 居中直行（最稳）
//				case 0:	
//					// Car_Forward(CAR_SPEED+7,CAR_SPEED);
//				target_speed_left = speed_run;
//                target_speed_right = speed_run;
//				//DL_GPIO_setPins(GPIO_PORT, GPIO_IN1_PIN);	
//				  break;
//				case 4:
//				//	case 31:
//				   target_speed_left = speed_run - 40;
//                   target_speed_right = speed_run - 40;
//				  // Car_Forward(CAR_SPEED+7,CAR_SPEED);
//				  // delay_ms(10);
//					break;
//				// 需要轻微向左转向，来矫正姿态（S4）
//				case 12:
//				  // Car_Forward(CAR_SPEED, OFFSET_MID);   // 200    170
//					target_speed_left = speed_run - 23;
//					target_speed_right = speed_run +20;
//				 //  delay_ms(50);
//					break;		
//				// 需要中度向左转向，来矫正姿态（out2）
//				case 24:
//				case 8:
//				  // Car_Forward(OFFSET_MID - OFFSET_SMALL, OFFSET_MID);	
//					target_speed_left = speed_run - 65;
//					target_speed_right = speed_run +20;
//					//delay_ms(100);
//					break;		
//				// 需要大幅度向左转向，来矫正姿态（S5） BIG_TURN       
//				case 16:
//					//Car_Forward(CAR2_SPEED - BIG_TURN, CAR2_SPEED ); 
//					target_speed_left = speed_run - 85;
//					target_speed_right = speed_run + 60;
//					//delay_ms(160);	
//					break;
//				
//				 // 需要轻微向右转向，来矫正姿态（S2）
//				case 6:
//					 //Car_Forward( OFFSET_MID+10, CAR_SPEED); 
//					target_speed_left = speed_run +20;
//					target_speed_right = speed_run - 55;
//				
//				  // delay_ms(50);		
//					break;		
//				// 需要中度向右转向，来矫正姿态（S2+S3）
//				case 3:
//				case 2:
//					//Car_Forward(OFFSET_MID+4 , OFFSET_MID - OFFSET_SMALL);  
//					target_speed_left =speed_run +20;
//					target_speed_right = speed_run - 65;
//					//delay_ms(70);	
//					break;     
//				// 需要大幅度向右转向，来矫正姿态（S1+S2）BIG_TURN
//				case 1:    
//					//Car_Forward(CAR2_SPEED-26, CAR2_SPEED - BIG_TURN-5);
//					target_speed_left = speed_run + 70;
//					target_speed_right = speed_run - 85;				
//					//delay_ms(160);
//					break;
//				
//				// 全黑=终点/十字停车
//				case 31:
//				case 7:
//				case 14:	
//				case 28:	
//					//TB6612_Motor_Stop();
//					target_speed_left = 0;
//					target_speed_right = 0;
//					car_stop_flag = 1;
//					break;
//				default:
//					//Car_Forward(CAR_SPEED, CAR_SPEED);
//					break;
//            }



//}

*/