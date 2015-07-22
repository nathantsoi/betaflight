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
    { RCC_AHBPeriph_GPIOD, GPIOD, GPIO_Pin_12, 12 },
    { RCC_AHBPeriph_GPIOD, GPIOD, GPIO_Pin_13, 13 },
    { RCC_AHBPeriph_GPIOD, GPIOD, GPIO_Pin_14, 14 },
    { RCC_AHBPeriph_GPIOD, GPIOD, GPIO_Pin_15, 15 },
    { RCC_AHBPeriph_GPIOA, GPIOA, GPIO_Pin_1, 1 },
    { RCC_AHBPeriph_GPIOA, GPIOA, GPIO_Pin_2, 2 }
};
#elif defined(CJMCU) || defined(EUSTM32F103RC) || defined(NAZE) || defined(OLIMEXINO) || defined(PORT103R)
const escHardware_t escHardware[ESC_COUNT] = {
    { RCC_APB2Periph_GPIOA, GPIOA, GPIO_Pin_8, 8 },
    { RCC_APB2Periph_GPIOA, GPIOA, GPIO_Pin_11, 11 },
    { RCC_APB2Periph_GPIOB, GPIOB, GPIO_Pin_6, 6 },
    { RCC_APB2Periph_GPIOB, GPIOB, GPIO_Pin_7, 7 },
    { RCC_APB2Periph_GPIOB, GPIOB, GPIO_Pin_8, 8 },
    { RCC_APB2Periph_GPIOB, GPIOB, GPIO_Pin_9, 9 }
};
#elif CC3D
const escHardware_t escHardware[ESC_COUNT] = {
    { RCC_APB2Periph_GPIOB, GPIOB, GPIO_Pin_9, 9 },
    { RCC_APB2Periph_GPIOB, GPIOB, GPIO_Pin_8, 9 },
    { RCC_APB2Periph_GPIOB, GPIOB, GPIO_Pin_7, 7 },
    { RCC_APB2Periph_GPIOA, GPIOA, GPIO_Pin_8, 8 },
    { RCC_APB2Periph_GPIOB, GPIOB, GPIO_Pin_4, 4 },
    { RCC_APB2Periph_GPIOA, GPIOA, GPIO_Pin_2, 2 }
};
#endif


static void gpio_enable_clock(uint32_t Periph_GPIOx);
static void gpio_set_mode(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t mode);

#if defined(STM32F3DISCOVERY)
// Top Left LD4, PE8 (blue)-- from programmer (RX)
#define LED_PRGMR_RX      GPIO_Pin_8
// Top Right LD5, PE10 (orange) -- to programmer (TX)
#define LED_PRGMR_TX      GPIO_Pin_10
// gpioinit
static void ledInitDebug(void)
{
  uint32_t pinmask = LED_PRGMR_RX|LED_PRGMR_TX;
  GPIO_DeInit(GPIOE);
  gpio_enable_clock(RCC_AHBPeriph_GPIOE);
  gpio_set_mode(GPIOE, pinmask, Mode_Out_PP);
  GPIOE->BRR = pinmask;
}
#endif

// set output if output = 1, otherwise input
static void gpio_enable_clock(uint32_t Periph_GPIOx) {
  // Enable the clock
#ifdef STM32F303xC
  RCC_AHBPeriphClockCmd(Periph_GPIOx, ENABLE);
#else
  RCC_APB2PeriphClockCmd(Periph_GPIOx, ENABLE);
#endif
}

#ifdef STM32F10X
static volatile uint32_t original_cr_mask, in_cr_mask, out_cr_mask;
static __IO uint32_t *cr;
static void gpioPrepVars(uint16_t escIndex)
{
  GPIO_TypeDef *gpio = escHardware[escIndex].gpio;
  uint32_t pinpos = escHardware[escIndex].pinpos;
  // mask out extra bits from pinmode, leaving just CNF+MODE
  uint32_t inmode = Mode_IPU & 0x0F;
  uint32_t outmode = (Mode_Out_PP & 0x0F) | Speed_10MHz;
  // reference CRL or CRH, depending whether pin number is 0..7 or 8..15
  cr = &gpio->CRL + (pinpos / 8);
  // offset to CNF and MODE portions of CRx register
  uint32_t shift = (pinpos % 8) * 4;
  // Read out current CRx value
  original_cr_mask = in_cr_mask = out_cr_mask = *cr;
  // Mask out 4 bits
  in_cr_mask &= ~(0xF << shift);
  out_cr_mask &= ~(0xF << shift);
  // save current pinmode
  in_cr_mask |= inmode << shift;
  out_cr_mask |= outmode << shift;
}

static void gpioSetOne(uint16_t escIndex, GPIO_Mode mode) {
  // reference CRL or CRH, depending whether pin number is 0..7 or 8..15
  if (mode == Mode_IPU) {
    *cr = in_cr_mask;
    escHardware[escIndex].gpio->ODR |= (1U << escHardware[escIndex].pinpos);
  }
  else {
    *cr = out_cr_mask;
  }
}
#endif

static void gpio_set_mode(GPIO_TypeDef* gpio, uint16_t pin, GPIO_Mode mode) {
  gpio_config_t cfg;
  cfg.pin = pin;
  cfg.mode = mode;
  cfg.speed = Speed_10MHz;
  gpioInit(gpio, &cfg);
}

#define disable_hardware_uart  __disable_irq()
#define enable_hardware_uart   __enable_irq()
#define ESC_HI(escIndex)       ((escHardware[escIndex].gpio->IDR & escHardware[escIndex].pin) != (uint32_t)Bit_RESET)
#define RX_HI                  ((S1W_RX_GPIO->IDR & S1W_RX_PIN) != (uint32_t)Bit_RESET)
#define ESC_SET_HI(escIndex)   escHardware[escIndex].gpio->BSRR = escHardware[escIndex].pin
#define ESC_SET_LO(escIndex)   escHardware[escIndex].gpio->BRR = escHardware[escIndex].pin
#define TX_SET_HIGH            S1W_TX_GPIO->BSRR = S1W_TX_PIN
#define TX_SET_LO              S1W_TX_GPIO->BRR = S1W_TX_PIN

#ifdef STM32F303xC
#define ESC_INPUT(escIndex)    escHardware[escIndex].gpio->MODER &= ~(GPIO_MODER_MODER0 << (escHardware[escIndex].pinpos * 2))
#define ESC_OUTPUT(escIndex)   escHardware[escIndex].gpio->MODER |= GPIO_Mode_OUT << (escHardware[escIndex].pinpos * 2)
#endif

#ifdef STM32F10X
#define ESC_INPUT(escIndex)    gpioSetOne(escIndex, Mode_IPU)
#define ESC_OUTPUT(escIndex)   gpioSetOne(escIndex, Mode_Out_PP)
#endif

#if defined(STM32F3DISCOVERY)
#define RX_LED_OFF GPIOE->BRR = LED_PRGMR_RX
#define RX_LED_ON  GPIOE->BSRR = LED_PRGMR_RX
#define TX_LED_OFF GPIOE->BRR = LED_PRGMR_TX
#define TX_LED_ON  GPIOE->BSRR = LED_PRGMR_TX
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
  // Reset ESC GPIO -- https://github.com/nathantsoi/stm32-serial-1wire-passthrough/issues/2#issuecomment-119400845
  //GPIO_DeInit(escHardware[escIndex].gpio);

  // maybe we dont need thits?
  gpio_enable_clock(escHardware[escIndex].periph);

#ifdef STM32F3DISCOVERY
  ledInitDebug();
#endif

  //Disable all interrupts
  disable_hardware_uart;

  // reset all the pins
  GPIO_ResetBits(S1W_RX_GPIO, S1W_RX_PIN);
  GPIO_ResetBits(S1W_TX_GPIO, S1W_TX_PIN);
  GPIO_ResetBits(escHardware[escIndex].gpio, escHardware[escIndex].pin);
  // configure gpio
  gpio_set_mode(S1W_RX_GPIO, S1W_RX_PIN, Mode_IPU);
  gpio_set_mode(S1W_TX_GPIO, S1W_TX_PIN, Mode_Out_PP);
  gpio_set_mode(escHardware[escIndex].gpio, escHardware[escIndex].pin, Mode_IPU);
  // hey user, turn on your ESC now

#ifdef STM32F10X
  // reset our gpio register pointers and bitmask values
  gpioPrepVars(escIndex);
#endif

  // Wait for programmer to go from 1 -> 0 indicating incoming data
  while(RX_HI);
  while(1) {
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
    while(!RX_HI) {
      ct--;
      if (ct==0) {
        // Reset ESC GPIO -- https://github.com/nathantsoi/stm32-serial-1wire-passthrough/issues/2#issuecomment-119400845
        //GPIO_DeInit(escHardware[escIndex].gpio);
        // Programmer RX
        //gpio_set_mode(S1W_RX_GPIO, S1W_RX_PIN, Mode_IPU);
        // Programmer TX
        gpio_set_mode(S1W_TX_GPIO, S1W_TX_PIN, Mode_AF_PP);
#ifdef STM32F10X
        *cr = original_cr_mask;
#endif
        // Enable Hardware UART
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
    while (RX_HI) {
      if (ESC_HI(escIndex)) {
        TX_SET_HIGH;
        RX_LED_OFF;
      }
      else {
        TX_SET_LO;
        RX_LED_ON;
      }
    }
  }
}

#endif
