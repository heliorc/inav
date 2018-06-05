/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#define TARGET_BOARD_IDENTIFIER "HESP"
#define USBD_PRODUCT_STRING     "HELIOSPRING"

// Use target-specific hardware descriptors (don't use common_hardware.c)

#define LED0                    PB7

#define BEEPER                  PC15
#define BEEPER_INVERTED

#define USE_I2C
#define USE_I2C_DEVICE_2
#define I2C_DEVICE_2_SHARES_UART3

#define USE_MAG
#define MAG_I2C_BUS             BUS_I2C2
#define MAG_HMC5883_ALIGN       CW90_DEG
#define USE_MAG_HMC5883
#define USE_MAG_MAG3110
#define USE_MAG_QMC5883
#define USE_MAG_IST8310

#define USE_BARO
#define BARO_I2C_BUS             BUS_I2C2
#define USE_BARO_BMP085
#define USE_BARO_BMP280
#define USE_BARO_MS5611

#define USE_PITOT_ADC
#define USE_PITOT_MS4525
#define PITOT_I2C_BUS            BUS_I2C2

#define USE_GYRO
#define USE_ACC

#define DEFAULT_ATTITUDE_UPDATE_INTERVAL 1000 
#define DEFAULT_ACC_SAMPLE_INTERVAL      1000

#define USE_FAST_SPI_DRIVER
#define USE_GYRO_IMUF9001
#define USE_QUAT_IMUF9001
#define USE_ACC_IMUF9001
#define IMUF9001_CS_PIN         PB1
#define IMUF9001_RST_PIN        PA4 
#define IMUF9001_SPI_INSTANCE   BUS_SPI1
#define USE_EXTI
#define MPU_INT_EXTI            PB0
#define USE_MPU_DATA_READY_SIGNAL

#define USE_DSHOT_DMAR
#define ENABLE_DSHOT_DMAR       true

#define M25P16_CS_PIN           PC14
#define M25P16_SPI_INSTANCE     BUS_SPI3
#define USE_FLASHFS
#define USE_FLASH_M25P16

#define USE_VCP

#define VBUS_SENSING_PIN        PC5

#define USE_OSD
#define USE_MAX7456
#define MAX7456_SPI_BUS         BUS_SPI3
#define MAX7456_CS_PIN          PA15
#define MAX7456_SPI_CLK         (SPI_CLOCK_STANDARD) // 10MHz
#define MAX7456_RESTORE_CLK     (SPI_CLOCK_FAST)
#define M25P16_SPI_BUS          BUS_SPI3

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9

#define USE_UART2
#define UART2_RX_PIN            PA3
#define UART2_TX_PIN            PA2

#define USE_UART3
#define UART3_RX_PIN            PB11
#define UART3_TX_PIN            PB10

#define USE_UART4
#define UART4_RX_PIN            PC11
#define UART4_TX_PIN            NONE

#define USE_UART5
#define UART5_RX_PIN            PD2
#define UART5_TX_PIN            PC12

#define SERIAL_PORT_COUNT       6 //VCP, USART1, USART2, USART3, UART4, USART5

#define USE_SPI

#define USE_SPI_DEVICE_1
#define SPI1_NSS_PIN            PB1
#define SPI1_SCK_PIN            PB3
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define USE_DMA_SPI_DEVICE

#define DMA_SPI_NSS_PIN_SRC        GPIO_PinSource1
#define DMA_SPI_NSS_PIN            GPIO_Pin_1
#define DMA_SPI_NSS_PORT           GPIOB
#define DMA_SPI_NSS_AF             GPIO_AF_SPI1
#define DMA_SPI_SCK_PIN_SRC        GPIO_PinSource3
#define DMA_SPI_SCK_PIN            GPIO_Pin_3
#define DMA_SPI_SCK_PORT           GPIOB
#define DMA_SPI_SCK_AF             GPIO_AF_SPI1
#define DMA_SPI_MISO_PIN_SRC       GPIO_PinSource6
#define DMA_SPI_MISO_PIN           GPIO_Pin_6
#define DMA_SPI_MISO_PORT          GPIOA
#define DMA_SPI_MISO_AF            GPIO_AF_SPI1
#define DMA_SPI_MOSI_PIN_SRC       GPIO_PinSource7
#define DMA_SPI_MOSI_PIN           GPIO_Pin_7
#define DMA_SPI_MOSI_PORT          GPIOA
#define DMA_SPI_MOSI_AF            GPIO_AF_SPI1
#define DMA_SPI_RST_MSK            RCC_APB2RSTR_SPI1RST
#define DMA_SPI_PER                RCC->APB2RSTR

#define DMA_SPI_SPI                SPI1
#define DMA_SPI_BAUDRATE           SPI_BaudRatePrescaler_4
#define DMA_SPI_CPOL               SPI_CPOL_Low
#define DMA_SPI_CPHA               SPI_CPHA_1Edge

#define DMA_SPI_DMA                DMA2
#define DMA_SPI_TX_DMA_STREAM      DMA2_Stream3
#define DMA_SPI_RX_DMA_STREAM      DMA2_Stream2
#define DMA_SPI_TX_DMA_CHANNEL     DMA_Channel_3
#define DMA_SPI_RX_DMA_CHANNEL     DMA_Channel_3
#define DMA_SPI_TX_DMA_HANDLER     DMA2_Stream3_IRQHandler
#define DMA_SPI_RX_DMA_HANDLER     DMA2_Stream2_IRQHandler
#define DMA_SPI_TX_DMA_IRQn        DMA2_Stream3_IRQn
#define DMA_SPI_RX_DMA_IRQn        DMA2_Stream2_IRQn

#define DMA_SPI_DMA_RX_PRE_PRI     0x0E
#define DMA_SPI_DMA_RX_SUB_PRI     0x0E

#define DMA_SPI_TX_DMA_FLAG_ALL      DMA_FLAG_FEIF3 | DMA_FLAG_DMEIF3 | DMA_FLAG_TEIF3 | DMA_FLAG_HTIF3 | DMA_FLAG_TCIF3
#define DMA_SPI_TX_DMA_FLAG_GL       DMA_FLAG_TCIF3
#define DMA_SPI_TX_DMA_FLAG_TC       DMA_FLAG_TCIF3
#define DMA_SPI_RX_DMA_FLAG_ALL      DMA_FLAG_FEIF2 | DMA_FLAG_DMEIF2 | DMA_FLAG_TEIF2 | DMA_FLAG_HTIF2 | DMA_FLAG_TCIF2
#define DMA_SPI_RX_DMA_FLAG_TC       DMA_FLAG_TCIF2
#define DMA_SPI_RX_DMA_FLAG_GL       DMA_FLAG_TCIF2


//#define USE_SPI_DEVICE_2
//#define SPI2_NSS_PIN            PB12
//#define SPI2_SCK_PIN            PB13
//#define SPI2_MISO_PIN           PC2
//#define SPI2_MOSI_PIN           PC3

#define USE_SPI_DEVICE_3
#define SPI3_NSS_PIN            PA15
#define SPI3_SCK_PIN            PC10
#define SPI3_MISO_PIN           PB4
#define SPI3_MOSI_PIN           PB5

#define USE_TARGET_CONFIG
#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define SERIALRX_UART           SERIAL_PORT_USART2
#define DEFAULT_FEATURES        (FEATURE_TELEMETRY | FEATURE_OSD | FEATURE_AIRMODE)

#define USE_GPS
#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_PIN  NONE  // (HARDARE=0,PPM)
#define USE_SERIAL_4WAY_BLHELI_INTERFACE

// Number of available PWM outputs
#define MAX_PWM_OUTPUT_PORTS    3

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         0xffff

#define USABLE_TIMER_CHANNEL_COUNT 3
//#define USED_TIMERS             ( TIM_N(1) | TIM_N(3) | TIM_N(8) | TIM_N(4) | TIM_N(12) )
#define USED_TIMERS             ( TIM_N(3) | TIM_N(8) )


#define SENSORS_SET (SENSOR_ACC|SENSOR_MAG|SENSOR_BARO)

#define USE_ADC
#define ADC_CHANNEL_1_PIN               PA1
#define ADC_CHANNEL_2_PIN               PA0
#define ADC_CHANNEL_3_PIN               PA5
#define CURRENT_METER_ADC_CHANNEL       ADC_CHN_1
#define VBAT_ADC_CHANNEL                ADC_CHN_2
#define RSSI_ADC_CHANNEL                ADC_CHN_3

#define IMUF_DEFAULT_PITCH_Q 3500
#define IMUF_DEFAULT_PITCH_W 6
#define IMUF_DEFAULT_ROLL_Q 3500
#define IMUF_DEFAULT_ROLL_W 6
#define IMUF_DEFAULT_YAW_Q 2500
#define IMUF_DEFAULT_YAW_W 6