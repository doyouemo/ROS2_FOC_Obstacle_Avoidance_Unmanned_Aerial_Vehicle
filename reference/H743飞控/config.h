/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#define FC_TARGET_MCU     STM32H743

#define BOARD_NAME        UrmoukyH743
#define MANUFACTURER_ID   UMKY

#define TARGET_BOARD_IDENTIFIER "UMKYH7"
#define USBD_PRODUCT_STRING     "Urmouky H743"

#define CLOUD_BUILD     // 加这个就没有那堆莫名其妙的Feature包含，看个人习惯，嫌麻烦可以注释掉

#define USE_ACC
#define USE_ACC_SPI_MPU6500
#define USE_GYRO
#define USE_GYRO_SPI_MPU6500
#define USE_ACC_SPI_MPU6000
#define USE_GYRO_SPI_MPU6000
#define USE_ACC_SPI_ICM42688P
#define USE_GYRO_SPI_ICM42688P
#define USE_ACC_SPI_ICM42605
#define USE_GYRO_SPI_ICM42605
#define USE_BARO
#define USE_BARO_BMP280
#define USE_BARO_DPS310
#define USE_MAX7456
#define USE_SDCARD
#define USE_GPS                     // 启用GPS
#define USE_MAG        
#define USE_MAG_QMC5883             // 启用磁力计
#define USE_SERIALRX
#define USE_SERIALRX_CRSF
#define USE_SERIALRX_IBUS           // 接收机协议
#define USE_TELEMETRY
#define USE_TELEMETRY_CRSF
#define USE_TELEMETRY_IBUS        
#define USE_TELEMETRY_MAVLINK       // 遥测协议
#define USE_DSHOT                   // 电调协议
#define USE_PWM_OUTPUT              // 我也不知道有什么用，加上吧
#define USE_ACRO_TRAINER            // 训练模式？不知道有什么用
#define USE_OSD_SD                  // 模拟OSD
#define USE_OSD_HD                  // 高清OSD
#define USE_PINIO                   // 这是个很复杂的机制，我也看不懂
#define USE_SERVOS                  // 舵机
#define USE_LED_STRIP               // 灯带控制
#define USE_VTX                     // 模拟图传
#define USE_ALTITUDE_HOLD           // 高度保持
#define USE_POSITION_HOLD           // 位置保持


#define BEEPER_PIN           PA15
#define MOTOR1_PIN           PB0
#define MOTOR2_PIN           PB1
#define MOTOR3_PIN           PA0
#define MOTOR4_PIN           PA1
#define MOTOR5_PIN           PA2
#define MOTOR6_PIN           PA3
#define MOTOR7_PIN           PD12
#define MOTOR8_PIN           PD13
#define SERVO1_PIN           PD14
#define SERVO2_PIN           PD15
#define PPM_PIN              PC7
#define LED_STRIP_PIN        PA8
#define UART1_TX_PIN         PA9
#define UART2_TX_PIN         PD5
#define UART3_TX_PIN         PD8
#define UART4_TX_PIN         PB9
#define UART6_TX_PIN         PC6
#define UART7_TX_PIN         PE8
#define UART8_TX_PIN         PE1
#define UART1_RX_PIN         PA10
#define UART2_RX_PIN         PD6
#define UART3_RX_PIN         PD9
#define UART4_RX_PIN         PB8
#define UART6_RX_PIN         PC7
#define UART7_RX_PIN         PE7
#define UART8_RX_PIN         PE0
#define I2C1_SCL_PIN         PB6
#define I2C1_SDA_PIN         PB7
#define I2C2_SCL_PIN         PB10
#define I2C2_SDA_PIN         PB11
#define LED0_PIN             PE3
#define LED1_PIN             PE4
#define SPI1_SCK_PIN         PA5
#define SPI2_SCK_PIN         PB13
#define SPI4_SCK_PIN         PE12
#define SPI1_SDI_PIN         PA6
#define SPI2_SDI_PIN         PB14
#define SPI4_SDI_PIN         PE13
#define SPI1_SDO_PIN         PD7
#define SPI2_SDO_PIN         PB15
#define SPI4_SDO_PIN         PE14
#define ADC_VBAT_PIN         PC0
#define ADC_RSSI_PIN         PC5
#define ADC_CURR_PIN         PC1
#define ADC_EXTERNAL1_PIN    PC4
#define SDIO_CK_PIN          PC12
#define SDIO_CMD_PIN         PD2
#define SDIO_D0_PIN          PC8
#define SDIO_D1_PIN          PC9
#define SDIO_D2_PIN          PC10
#define SDIO_D3_PIN          PC11
#define PINIO1_PIN           PD10
#define PINIO2_PIN           PD11
#define MAX7456_SPI_CS_PIN   PB12
#define GYRO_1_EXTI_PIN      PE5
#define GYRO_1_CS_PIN        PC15
#define GYRO_2_EXTI_PIN      PE15
#define GYRO_2_CS_PIN        PE11

#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, PB0 , 3,  2) \
    TIMER_PIN_MAP( 1, PB1 , 3,  3) \
    TIMER_PIN_MAP( 2, PA0 , 5,  0) \
    TIMER_PIN_MAP( 3, PA1 , 5,  1) \
    TIMER_PIN_MAP( 4, PA2 , 5,  2) \
    TIMER_PIN_MAP( 5, PA3 , 5,  3) \
    TIMER_PIN_MAP( 6, PD12, 4,  0) \
    TIMER_PIN_MAP( 7, PD13, 4,  1) \
    TIMER_PIN_MAP( 8, PD14, 4,  2) \
    TIMER_PIN_MAP( 9, PD15, 4,  3) \
    TIMER_PIN_MAP(10, PE5 , 15, 0) \
    TIMER_PIN_MAP(11, PE6 , 15, 1) \
    TIMER_PIN_MAP(12, PA8 , 1,  0) \
    TIMER_PIN_MAP(13, PA15, 2,  0) \
    TIMER_PIN_MAP(14, PC7 , 8,  1) \
    TIMER_PIN_MAP(15, PC6 , 8,  0) \
    TIMER_PIN_MAP(16, PB8 , 16, 0) \
    TIMER_PIN_MAP(17, PB9 , 17, 0) 

#define ADC1_DMA_OPT        8
#define ADC3_DMA_OPT        9
#define TIMUP1_DMA_OPT      0
#define TIMUP2_DMA_OPT      0
#define TIMUP3_DMA_OPT      2
#define TIMUP4_DMA_OPT      1
#define TIMUP5_DMA_OPT      0
#define TIMUP8_DMA_OPT      0

#define BARO_I2C_INSTANCE I2CDEV_2
#define MAG_I2C_INSTANCE I2CDEV_1

#define DEFAULT_RX_FEATURE FEATURE_RX_SERIAL
#define DEFAULT_FEATURES (FEATURE_TELEMETRY | FEATURE_OSD)
#define SERIALRX_UART SERIAL_PORT_UART4
#define SERIALRX_PROVIDER SERIALRX_CRSF

#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SCALE 250

#define BEEPER_INVERTED

#define SDCARD_DETECT_PIN NONE
#define SDIO_DEVICE SDIODEV_1
#define SDIO_USE_4BIT 1
#define DEFAULT_BLACKBOX_DEVICE BLACKBOX_DEVICE_SDCARD
#define FLASH_SPI_INSTANCE SPI4

#define MAX7456_SPI_INSTANCE SPI2

#define PINIO1_BOX 40
#define PINIO2_BOX 41
#define GYRO_1_SPI_INSTANCE SPI1
#define GYRO_1_ALIGN CW270_DEG_FLIP
#define GYRO_2_SPI_INSTANCE SPI4
#define GYRO_2_ALIGN CW180_DEG
//#define GYRO_1_ALIGN_PITCH 1800
//#define GYRO_2_ALIGN_PITCH 1800   // 这两个不能和GYRO_x_ALIGN一起定义

//#define ENSURE_MPU_DATA_READY_IS_LOW   // 感觉没什么用