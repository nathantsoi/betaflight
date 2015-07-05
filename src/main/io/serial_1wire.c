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
 *
 * Ported from https://github.com/4712/BLHeliSuite/blob/master/Interfaces/Arduino1Wire/Source/Arduino1Wire_C/Arduino1Wire.c
 *  by Nathan Tsoi <nathan@vertile.com>
 */

#include <stdbool.h>

#include "platform.h"

#ifdef USE_SERIAL_1WIRE

#include "drivers/gpio.h"
#include "drivers/system.h"
#include "drivers/light_led.h"
#include "io/serial_1wire.h"

// Figure out esc clocks and pins, extrapolated from timer.c
// Periphs could be pulled progmatically... but I'll leave that for another exercise
#if defined(STM32F3DISCOVERY) && !(defined(CHEBUZZF3))
const escHardware_t escHardware[ESC_COUNT] = {
    { RCC_AHBPeriph_GPIOD, GPIOD, GPIO_Pin_12 },
    { RCC_AHBPeriph_GPIOD, GPIOD, GPIO_Pin_13 },
    { RCC_AHBPeriph_GPIOD, GPIOD, GPIO_Pin_14 },
    { RCC_AHBPeriph_GPIOD, GPIOD, GPIO_Pin_15 },
    { RCC_AHBPeriph_GPIOA, GPIOA, GPIO_Pin_1 },
    { RCC_AHBPeriph_GPIOA, GPIOA, GPIO_Pin_2 }
};
#elif defined(CJMCU) || defined(EUSTM32F103RC) || defined(NAZE) || defined(OLIMEXINO) || defined(PORT103R)
const escHardware_t escHardware[ESC_COUNT] = {
    { RCC_APB2Periph_GPIOA, GPIOA, GPIO_Pin_8 },
    { RCC_APB2Periph_GPIOA, GPIOA, GPIO_Pin_11 },
    { RCC_APB2Periph_GPIOB, GPIOB, GPIO_Pin_6 },
    { RCC_APB2Periph_GPIOB, GPIOB, GPIO_Pin_7 },
    { RCC_APB2Periph_GPIOB, GPIOB, GPIO_Pin_8 },
    { RCC_APB2Periph_GPIOB, GPIOB, GPIO_Pin_9 }
};
#elif CC3D
const escHardware_t escHardware[ESC_COUNT] = {
    { RCC_APB2Periph_GPIOB, GPIOB, GPIO_Pin_9 },
    { RCC_APB2Periph_GPIOB, GPIOB, GPIO_Pin_8 },
    { RCC_APB2Periph_GPIOB, GPIOB, GPIO_Pin_7 },
    { RCC_APB2Periph_GPIOA, GPIOA, GPIO_Pin_8 },
    { RCC_APB2Periph_GPIOB, GPIOB, GPIO_Pin_4 },
    { RCC_APB2Periph_GPIOA, GPIOA, GPIO_Pin_2 }
};
#endif

static void gpio_enable_clock(uint32_t Periph_GPIOx);
static void gpio_set_mode(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t mode);

// Setup some debugging LEDs
// Leds on STM32F3DISCOVERY, note: inverted output
// Top Left LD4, PE8 (blue)-- from programmer (RX)
#define LED_PRGMR_RX      GPIOE, GPIO_Pin_8
// Top Right LD5, PE10 (orange) -- to programmer (TX)
#define LED_PRGMR_TX      GPIOE, GPIO_Pin_10
// Bottom Left (orange) LD8, PE14 -- from esc (RX)
#define LED_ESC_RX        GPIOE, GPIO_Pin_14
// Bottom Right (blue) LD9, PE12 -- to esc (TX)
#define LED_ESC_TX        GPIOE, GPIO_Pin_12
// Left (green) LD6, PE15 -- esc input (rx) mode
#define LED_ESC_MODE_RX   GPIOE, GPIO_Pin_15
// Right (green) LD7, PE11 -- esc output (tx) mode
#define LED_ESC_MODE_TX   GPIOE, GPIO_Pin_11

#if defined(STM32F3DISCOVERY)
static void ledSetState(GPIO_TypeDef *GPIOx, uint16_t pin, BitAction on)
{
 GPIO_WriteBit(GPIOx, pin, on);
}


static void ledInitDebug(void)
{
  GPIO_DeInit(GPIOE);
  gpio_enable_clock(RCC_AHBPeriph_GPIOE);
  gpio_set_mode(GPIOE, GPIO_Pin_8|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_14|GPIO_Pin_15, Mode_Out_PP);
  // Inverted LEDs
  ledSetState(GPIOE, GPIO_Pin_8|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_14|GPIO_Pin_15, Bit_RESET);
  return;
}
#endif

// set output if output = 1, otherwise input
static void gpio_enable_clock(uint32_t Periph_GPIOx) {
  // Enable the clock
#if defined(STM32F303xC)
  RCC_AHBPeriphClockCmd(Periph_GPIOx, ENABLE);
#else
  RCC_APB2PeriphClockCmd(Periph_GPIOx, ENABLE);
#endif
}

static void gpio_set_mode(GPIO_TypeDef* gpio, uint16_t pin, GPIO_Mode mode) {
  gpio_config_t cfg;
  cfg.pin = pin;
  cfg.mode = mode;
  cfg.speed = Speed_2MHz;
  gpioInit(gpio, &cfg);
}

#define disable_hardware_uart  __disable_irq()
#define enable_hardware_uart  __enable_irq()
#define ESC_HI(escIndex) GPIO_ReadInputDataBit(escHardware[escIndex].gpio, escHardware[escIndex].pin)
#define RX_HI GPIO_ReadInputDataBit(S1W_RX_GPIO, S1W_RX_PIN)
#define ESC_INPUT(escIndex)  gpio_set_mode(escHardware[escIndex].gpio, escHardware[escIndex].pin, Mode_IPU)
#define ESC_OUTPUT(escIndex)  gpio_set_mode(escHardware[escIndex].gpio, escHardware[escIndex].pin, Mode_Out_PP)
#define ESC_SET_HI(escIndex) GPIO_WriteBit(escHardware[escIndex].gpio, escHardware[escIndex].pin, Bit_SET)
#define ESC_SET_LO(escIndex)  GPIO_WriteBit(escHardware[escIndex].gpio, escHardware[escIndex].pin, Bit_RESET)
#define TX_SET_HIGH GPIO_WriteBit(S1W_TX_GPIO, S1W_TX_PIN, Bit_SET)
#define TX_SET_LO GPIO_WriteBit(S1W_TX_GPIO, S1W_TX_PIN, Bit_RESET)

#if defined(STM32F3DISCOVERY)
#define RX_LED_OFF ledSetState(LED_PRGMR_RX, Bit_RESET)
#define RX_LED_ON ledSetState(LED_PRGMR_RX, Bit_SET)
#define TX_LED_OFF ledSetState(LED_PRGMR_TX, Bit_RESET)
#define TX_LED_ON ledSetState(LED_PRGMR_TX, Bit_SET)
#else
#define RX_LED_OFF LED0_OFF
#define RX_LED_ON LED0_ON
#define TX_LED_OFF LED1_OFF
#define TX_LED_ON LED1_ON
#endif

 
// This method translates 2 wires (a tx and rx line) to 1 wire, by letting the
// RX line control when data should be read or written from the single line
void usb1WirePassthrough(int8_t escIndex)
{
  //***init port and config
  // Take control of the LEDs
  // Reset ESC GPIO
  GPIO_DeInit(escHardware[escIndex].gpio);
  //Escs
  gpio_enable_clock(escHardware[escIndex].periph);

  #if defined(STM32F3DISCOVERY)
  ledInitDebug();
  #endif
  
  //Disable Hardware UART
  
  //Disable all interrupts
  disable_hardware_uart;

  //Programmer RX
  gpio_set_mode(S1W_RX_GPIO, S1W_RX_PIN, Mode_IPU);

  // Programmer TX
  gpio_set_mode(S1W_TX_GPIO, S1W_TX_PIN, Mode_Out_PP);

  // Escs pins start in input mode, pullup is always on
  ESC_INPUT(escIndex);
  // reset all the pins, 1wire goes into input mode, pullup on
  GPIO_ResetBits(S1W_RX_GPIO, S1W_RX_PIN);
  GPIO_ResetBits(escHardware[escIndex].gpio, escHardware[escIndex].pin);
  // turn on the ESC now

  // Wait for programmer to go from 1 -> 0 indicating incoming data
  while(RX_HI);
  while(1) 
  {
    // A new iteration on this loop starts when we have data from the programmer (read_programmer goes low)
    // Setup escIndex pin to send data, pullup is the default
    ESC_OUTPUT(escIndex);
    // Write the first bit
    ESC_SET_LO(escIndex);
    // Echo on the programmer tx line
    TX_SET_LO;
    //set LEDs
    RX_LED_OFF;
    TX_LED_ON;
    // Wait for programmer to go 0 -> 1
    uint32_t ct=3000;
    while(!RX_HI)
    {
       ct--;
       if (ct==0)
       {
          // Reset ESC GPIO
          GPIO_DeInit(escHardware[escIndex].gpio);
          // Programmer RX
          //gpio_set_mode(S1W_RX_GPIO, S1W_RX_PIN, Mode_IPU);
          // Programmer TX
          gpio_set_mode(S1W_TX_GPIO, S1W_TX_PIN, Mode_AF_PP);
          //Enable Hardware UART
          enable_hardware_uart; 
          return;
       } 
    }
    // Programmer is high, end of bit
    // Echo to the esc
    ESC_SET_HI(escIndex);
    // Listen to the escIndex, input mode, pullup resistor is on
    ESC_INPUT(escIndex);
    TX_LED_OFF;
    // Listen to the escIndex while there is no data from the programmer
    while (RX_HI) 
    {
      if (ESC_HI(escIndex)) 
      {
        TX_SET_HIGH;
        RX_LED_OFF;
      }
      else 
      {
        TX_SET_LO;
        RX_LED_ON;
      }
    }
  }
}

#endif
