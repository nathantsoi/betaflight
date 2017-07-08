/*
 * This is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#if defined(OMNIBUSF4NOX)
  #define TARGET_BOARD_IDENTIFIER "OBNX"
  #define USBD_PRODUCT_STRING "OmnibusF4 Nox"
#else
  #define TARGET_BOARD_IDENTIFIER "OBEV"
  #define USBD_PRODUCT_STRING "OmnibusF4 Evo"
#endif

#define LED0_PIN                PA4

#define BEEPER                  PC13

#define INVERTER_PIN_UART2      PC14

#define USE_SPI
#define USE_SPI_DEVICE_1
#define SPI1_NSS_PIN            PA15
#define SPI1_SCK_PIN            PB3
#define SPI1_MISO_PIN           PB4
#define SPI1_MOSI_PIN           PB5

#define USE_SPI_DEVICE_2
#define SPI2_NSS_PIN            PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define MPU6500_CS_PIN          SPI2_NSS_PIN
#define MPU6500_SPI_INSTANCE    SPI2

#define ACC
#define USE_ACC_MPU6500
#define USE_ACC_SPI_MPU6500
#define ACC_MPU6500_ALIGN       ALIGN_DEFAULT

#define GYRO
#define USE_GYRO_MPU6500
#define USE_GYRO_SPI_MPU6500
#define GYRO_MPU6500_ALIGN      ALIGN_DEFAULT

#define MPU6000_CS_PIN          SPI2_NSS_PIN
#define MPU6000_SPI_INSTANCE    SPI2

#define USE_ACC_MPU6000
#define USE_ACC_SPI_MPU6000
#define ACC_MPU6000_ALIGN       ALIGN_DEFAULT

#define USE_GYRO_MPU6000
#define USE_GYRO_SPI_MPU6000
#define GYRO_MPU6000_ALIGN      ALIGN_DEFAULT

#define USE_EXTI
#define MPU_INT_EXTI            PA8
#define USE_MPU_DATA_READY_SIGNAL

#define BARO
#define USE_BARO_BMP280
#define USE_BARO_SPI_BMP280
#define BMP280_SPI_INSTANCE     SPI2
#define BMP280_CS_PIN           PA9

#define OSD
#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI2
#define MAX7456_SPI_CS_PIN      PA10
#define MAX7456_SPI_CLK         (SPI_CLOCK_STANDARD*2)
#define MAX7456_RESTORE_CLK     (SPI_CLOCK_FAST)

#define USE_VCP

#define USE_UART1
#define UART1_RX_PIN            PB7
#define UART1_TX_PIN            PB6

#define USE_UART2
#define UART2_RX_PIN            PA3
#define UART2_TX_PIN            PA2

#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2

#define SERIAL_PORT_COUNT       5

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_HARDWARE 0 // PWM 1

#define USE_ADC
#if defined(OMNIBUSF4NOX)
  #define CURRENT_METER_ADC_PIN   PA6
  #define VBAT_ADC_PIN            PA5
#else
  #define VBAT_ADC_PIN            PA1
#endif

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff

#define USABLE_TIMER_CHANNEL_COUNT 8
#define USED_TIMERS             ( TIM_N(2) | TIM_N(3) | TIM_N(4) )
