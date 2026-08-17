/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)


#define CPUCLK_FREQ                                                     32000000



/* Defines for PWM_0 */
#define PWM_0_INST                                                         TIMA1
#define PWM_0_INST_IRQHandler                                   TIMA1_IRQHandler
#define PWM_0_INST_INT_IRQN                                     (TIMA1_INT_IRQn)
#define PWM_0_INST_CLK_FREQ                                              8000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_0_C0_PORT                                                 GPIOA
#define GPIO_PWM_0_C0_PIN                                         DL_GPIO_PIN_17
#define GPIO_PWM_0_C0_IOMUX                                      (IOMUX_PINCM39)
#define GPIO_PWM_0_C0_IOMUX_FUNC                     IOMUX_PINCM39_PF_TIMA1_CCP0
#define GPIO_PWM_0_C0_IDX                                    DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_0_C1_PORT                                                 GPIOA
#define GPIO_PWM_0_C1_PIN                                         DL_GPIO_PIN_16
#define GPIO_PWM_0_C1_IOMUX                                      (IOMUX_PINCM38)
#define GPIO_PWM_0_C1_IOMUX_FUNC                     IOMUX_PINCM38_PF_TIMA1_CCP1
#define GPIO_PWM_0_C1_IDX                                    DL_TIMER_CC_1_INDEX



/* Defines for MOTOR_PID */
#define MOTOR_PID_INST                                                   (TIMA0)
#define MOTOR_PID_INST_IRQHandler                               TIMA0_IRQHandler
#define MOTOR_PID_INST_INT_IRQN                                 (TIMA0_INT_IRQn)
#define MOTOR_PID_INST_LOAD_VALUE                                       (49999U)



/* Defines for IMU601 */
#define IMU601_INST                                                        UART3
#define IMU601_INST_FREQUENCY                                           32000000
#define IMU601_INST_IRQHandler                                  UART3_IRQHandler
#define IMU601_INST_INT_IRQN                                      UART3_INT_IRQn
#define GPIO_IMU601_RX_PORT                                                GPIOB
#define GPIO_IMU601_TX_PORT                                                GPIOB
#define GPIO_IMU601_RX_PIN                                        DL_GPIO_PIN_13
#define GPIO_IMU601_TX_PIN                                        DL_GPIO_PIN_12
#define GPIO_IMU601_IOMUX_RX                                     (IOMUX_PINCM30)
#define GPIO_IMU601_IOMUX_TX                                     (IOMUX_PINCM29)
#define GPIO_IMU601_IOMUX_RX_FUNC                      IOMUX_PINCM30_PF_UART3_RX
#define GPIO_IMU601_IOMUX_TX_FUNC                      IOMUX_PINCM29_PF_UART3_TX
#define IMU601_BAUD_RATE                                                (115200)
#define IMU601_IBRD_32_MHZ_115200_BAUD                                      (17)
#define IMU601_FBRD_32_MHZ_115200_BAUD                                      (23)
/* Defines for RPINT */
#define RPINT_INST                                                         UART0
#define RPINT_INST_FREQUENCY                                            32000000
#define RPINT_INST_IRQHandler                                   UART0_IRQHandler
#define RPINT_INST_INT_IRQN                                       UART0_INT_IRQn
#define GPIO_RPINT_RX_PORT                                                 GPIOA
#define GPIO_RPINT_TX_PORT                                                 GPIOA
#define GPIO_RPINT_RX_PIN                                         DL_GPIO_PIN_31
#define GPIO_RPINT_TX_PIN                                         DL_GPIO_PIN_28
#define GPIO_RPINT_IOMUX_RX                                       (IOMUX_PINCM6)
#define GPIO_RPINT_IOMUX_TX                                       (IOMUX_PINCM3)
#define GPIO_RPINT_IOMUX_RX_FUNC                        IOMUX_PINCM6_PF_UART0_RX
#define GPIO_RPINT_IOMUX_TX_FUNC                        IOMUX_PINCM3_PF_UART0_TX
#define RPINT_BAUD_RATE                                                 (115200)
#define RPINT_IBRD_32_MHZ_115200_BAUD                                       (17)
#define RPINT_FBRD_32_MHZ_115200_BAUD                                       (23)




/* Defines for SPI_LCD */
#define SPI_LCD_INST                                                       SPI1
#define SPI_LCD_INST_IRQHandler                                 SPI1_IRQHandler
#define SPI_LCD_INST_INT_IRQN                                     SPI1_INT_IRQn
#define GPIO_SPI_LCD_PICO_PORT                                            GPIOB
#define GPIO_SPI_LCD_PICO_PIN                                     DL_GPIO_PIN_8
#define GPIO_SPI_LCD_IOMUX_PICO                                 (IOMUX_PINCM25)
#define GPIO_SPI_LCD_IOMUX_PICO_FUNC                 IOMUX_PINCM25_PF_SPI1_PICO
#define GPIO_SPI_LCD_POCI_PORT                                            GPIOB
#define GPIO_SPI_LCD_POCI_PIN                                     DL_GPIO_PIN_7
#define GPIO_SPI_LCD_IOMUX_POCI                                 (IOMUX_PINCM24)
#define GPIO_SPI_LCD_IOMUX_POCI_FUNC                 IOMUX_PINCM24_PF_SPI1_POCI
/* GPIO configuration for SPI_LCD */
#define GPIO_SPI_LCD_SCLK_PORT                                            GPIOB
#define GPIO_SPI_LCD_SCLK_PIN                                     DL_GPIO_PIN_9
#define GPIO_SPI_LCD_IOMUX_SCLK                                 (IOMUX_PINCM26)
#define GPIO_SPI_LCD_IOMUX_SCLK_FUNC                 IOMUX_PINCM26_PF_SPI1_SCLK



/* Port definition for Pin Group LED */
#define LED_PORT                                                         (GPIOB)

/* Defines for PIN_22: GPIOB.22 with pinCMx 50 on package pin 21 */
#define LED_PIN_22_PIN                                          (DL_GPIO_PIN_22)
#define LED_PIN_22_IOMUX                                         (IOMUX_PINCM50)
/* Port definition for Pin Group LCD */
#define LCD_PORT                                                         (GPIOB)

/* Defines for RES: GPIOB.10 with pinCMx 27 on package pin 62 */
#define LCD_RES_PIN                                             (DL_GPIO_PIN_10)
#define LCD_RES_IOMUX                                            (IOMUX_PINCM27)
/* Defines for DC: GPIOB.11 with pinCMx 28 on package pin 63 */
#define LCD_DC_PIN                                              (DL_GPIO_PIN_11)
#define LCD_DC_IOMUX                                             (IOMUX_PINCM28)
/* Defines for CS: GPIOB.14 with pinCMx 31 on package pin 2 */
#define LCD_CS_PIN                                              (DL_GPIO_PIN_14)
#define LCD_CS_IOMUX                                             (IOMUX_PINCM31)
/* Defines for BLK: GPIOB.26 with pinCMx 57 on package pin 28 */
#define LCD_BLK_PIN                                             (DL_GPIO_PIN_26)
#define LCD_BLK_IOMUX                                            (IOMUX_PINCM57)
/* Port definition for Pin Group ENCODERA */
#define ENCODERA_PORT                                                    (GPIOB)

/* Defines for EAA: GPIOB.20 with pinCMx 48 on package pin 19 */
// groups represented: ["ENCODERB","ENCODERA"]
// pins affected: ["EBA","EAA"]
#define GPIO_MULTIPLE_GPIOB_INT_IRQN                            (GPIOB_INT_IRQn)
#define GPIO_MULTIPLE_GPIOB_INT_IIDX            (DL_INTERRUPT_GROUP1_IIDX_GPIOB)
#define ENCODERA_EAA_IIDX                                   (DL_GPIO_IIDX_DIO20)
#define ENCODERA_EAA_PIN                                        (DL_GPIO_PIN_20)
#define ENCODERA_EAA_IOMUX                                       (IOMUX_PINCM48)
/* Defines for EAB: GPIOB.24 with pinCMx 52 on package pin 23 */
#define ENCODERA_EAB_PIN                                        (DL_GPIO_PIN_24)
#define ENCODERA_EAB_IOMUX                                       (IOMUX_PINCM52)
/* Port definition for Pin Group ENCODERB */
#define ENCODERB_PORT                                                    (GPIOB)

/* Defines for EBA: GPIOB.17 with pinCMx 43 on package pin 14 */
#define ENCODERB_EBA_IIDX                                   (DL_GPIO_IIDX_DIO17)
#define ENCODERB_EBA_PIN                                        (DL_GPIO_PIN_17)
#define ENCODERB_EBA_IOMUX                                       (IOMUX_PINCM43)
/* Defines for EBB: GPIOB.18 with pinCMx 44 on package pin 15 */
#define ENCODERB_EBB_PIN                                        (DL_GPIO_PIN_18)
#define ENCODERB_EBB_IOMUX                                       (IOMUX_PINCM44)
/* Port definition for Pin Group TB6612 */
#define TB6612_PORT                                                      (GPIOA)

/* Defines for AIN1: GPIOA.14 with pinCMx 36 on package pin 7 */
#define TB6612_AIN1_PIN                                         (DL_GPIO_PIN_14)
#define TB6612_AIN1_IOMUX                                        (IOMUX_PINCM36)
/* Defines for AIN2: GPIOA.15 with pinCMx 37 on package pin 8 */
#define TB6612_AIN2_PIN                                         (DL_GPIO_PIN_15)
#define TB6612_AIN2_IOMUX                                        (IOMUX_PINCM37)
/* Defines for BIN1: GPIOA.12 with pinCMx 34 on package pin 5 */
#define TB6612_BIN1_PIN                                         (DL_GPIO_PIN_12)
#define TB6612_BIN1_IOMUX                                        (IOMUX_PINCM34)
/* Defines for BIN2: GPIOA.13 with pinCMx 35 on package pin 6 */
#define TB6612_BIN2_PIN                                         (DL_GPIO_PIN_13)
#define TB6612_BIN2_IOMUX                                        (IOMUX_PINCM35)
/* Defines for OUT1: GPIOA.0 with pinCMx 1 on package pin 33 */
#define GPIO_TCRT5000_OUT1_PORT                                          (GPIOA)
#define GPIO_TCRT5000_OUT1_PIN                                   (DL_GPIO_PIN_0)
#define GPIO_TCRT5000_OUT1_IOMUX                                  (IOMUX_PINCM1)
/* Defines for OUT2: GPIOA.1 with pinCMx 2 on package pin 34 */
#define GPIO_TCRT5000_OUT2_PORT                                          (GPIOA)
#define GPIO_TCRT5000_OUT2_PIN                                   (DL_GPIO_PIN_1)
#define GPIO_TCRT5000_OUT2_IOMUX                                  (IOMUX_PINCM2)
/* Defines for OUT3: GPIOA.10 with pinCMx 21 on package pin 56 */
#define GPIO_TCRT5000_OUT3_PORT                                          (GPIOA)
#define GPIO_TCRT5000_OUT3_PIN                                  (DL_GPIO_PIN_10)
#define GPIO_TCRT5000_OUT3_IOMUX                                 (IOMUX_PINCM21)
/* Defines for OUT4: GPIOA.11 with pinCMx 22 on package pin 57 */
#define GPIO_TCRT5000_OUT4_PORT                                          (GPIOA)
#define GPIO_TCRT5000_OUT4_PIN                                  (DL_GPIO_PIN_11)
#define GPIO_TCRT5000_OUT4_IOMUX                                 (IOMUX_PINCM22)
/* Defines for OUT5: GPIOB.2 with pinCMx 15 on package pin 50 */
#define GPIO_TCRT5000_OUT5_PORT                                          (GPIOB)
#define GPIO_TCRT5000_OUT5_PIN                                   (DL_GPIO_PIN_2)
#define GPIO_TCRT5000_OUT5_IOMUX                                 (IOMUX_PINCM15)
/* Defines for OUT6: GPIOB.3 with pinCMx 16 on package pin 51 */
#define GPIO_TCRT5000_OUT6_PORT                                          (GPIOB)
#define GPIO_TCRT5000_OUT6_PIN                                   (DL_GPIO_PIN_3)
#define GPIO_TCRT5000_OUT6_IOMUX                                 (IOMUX_PINCM16)
/* Defines for OUT7: GPIOB.15 with pinCMx 32 on package pin 3 */
#define GPIO_TCRT5000_OUT7_PORT                                          (GPIOB)
#define GPIO_TCRT5000_OUT7_PIN                                  (DL_GPIO_PIN_15)
#define GPIO_TCRT5000_OUT7_IOMUX                                 (IOMUX_PINCM32)
/* Defines for OUT8: GPIOB.16 with pinCMx 33 on package pin 4 */
#define GPIO_TCRT5000_OUT8_PORT                                          (GPIOB)
#define GPIO_TCRT5000_OUT8_PIN                                  (DL_GPIO_PIN_16)
#define GPIO_TCRT5000_OUT8_IOMUX                                 (IOMUX_PINCM33)
/* Defines for PIN_21: GPIOB.21 with pinCMx 49 on package pin 20 */
#define KEY_PIN_21_PORT                                                  (GPIOB)
#define KEY_PIN_21_PIN                                          (DL_GPIO_PIN_21)
#define KEY_PIN_21_IOMUX                                         (IOMUX_PINCM49)
/* Defines for PIN_26: GPIOA.26 with pinCMx 59 on package pin 30 */
#define KEY_PIN_26_PORT                                                  (GPIOA)
#define KEY_PIN_26_PIN                                          (DL_GPIO_PIN_26)
#define KEY_PIN_26_IOMUX                                         (IOMUX_PINCM59)
/* Defines for PIN_24: GPIOA.24 with pinCMx 54 on package pin 25 */
#define KEY_PIN_24_PORT                                                  (GPIOA)
#define KEY_PIN_24_PIN                                          (DL_GPIO_PIN_24)
#define KEY_PIN_24_IOMUX                                         (IOMUX_PINCM54)



/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_PWM_0_init(void);
void SYSCFG_DL_MOTOR_PID_init(void);
void SYSCFG_DL_IMU601_init(void);
void SYSCFG_DL_RPINT_init(void);
void SYSCFG_DL_SPI_LCD_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
