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

// for a baud of 19200 we get 52 microseconds/bit
// not sure if we need this at all
#define BIT_DELAY 52
#define BIT_DELAY_HALF 26

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

static void ledSetState(GPIO_TypeDef *GPIOx, uint16_t pin, BitAction on)
{
#if defined(STM32F3DISCOVERY)
  GPIO_WriteBit(GPIOx, pin, on);
#else
  if (on)
    LED0_ON;
  else
    LED0_OFF;
#endif
}

static void ledInitDebug(void)
{
#if defined(STM32F3DISCOVERY)
  GPIO_DeInit(GPIOE);
  gpio_enable_clock(RCC_AHBPeriph_GPIOE);
  gpio_set_mode(GPIOE, GPIO_Pin_8|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_14|GPIO_Pin_15, Mode_Out_PP);
  // Inverted LEDs
  ledSetState(GPIOE, GPIO_Pin_8|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_14|GPIO_Pin_15, Bit_RESET);
#endif
  return;
}


static void disable_hardware_uart() {
  // Disable all interrupts
  __disable_irq();
}

static void deinit_esc_gpio(int8_t escIndex) {
  int i = 0;
  if (escIndex == -1) {
    for (i = 0; i < ESC_COUNT; i++) {
      GPIO_DeInit(escHardware[i].gpio);
    }
  }
  else {
    GPIO_DeInit(escHardware[escIndex].gpio);
  }
}

static void deinit_gpio(int8_t escIndex) {
  GPIO_DeInit(S1W_TX_GPIO);
  GPIO_DeInit(S1W_RX_GPIO);
  deinit_esc_gpio(escIndex);
}

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

static void gpio_enable_clock_escs(int8_t escIndex) {
  int i = 0;
  if (escIndex == -1) {
    for (i = 0; i < ESC_COUNT; i++) {
      gpio_enable_clock(escHardware[i].periph);
    }
  }
  else {
    gpio_enable_clock(escHardware[escIndex].periph);
  }
}

// set output if output = 1, otherwise input
static void gpio_set_mode_escs(int8_t escIndex, uint8_t mode) {
  int i = 0;
  ledSetState(LED_ESC_MODE_TX, mode != Mode_IPU);
  ledSetState(LED_ESC_MODE_RX, mode == Mode_IPU);
  if (escIndex == -1) {
    escIndex = 0;
    // only configure all escs if going into Mode_Out_PP
    if (mode == Mode_Out_PP) {
      for (i = 0; i < ESC_COUNT; i++) {
        gpio_set_mode(escHardware[i].gpio, escHardware[i].pin, mode);
      }
      return;
    }
  }
  gpio_set_mode(escHardware[escIndex].gpio, escHardware[escIndex].pin, mode);
}

// Configure all GPIOs
static void init_all_gpio(int8_t escIndex) {
  // Programmer RX
  gpio_enable_clock(S1W_RX_PERIPH);
  gpio_set_mode(S1W_RX_GPIO, S1W_RX_PIN, Mode_IPU);
  // Programmer TX
  gpio_enable_clock(S1W_TX_PERIPH);
  gpio_set_mode(S1W_TX_GPIO, S1W_TX_PIN, Mode_Out_PP);
  // Escs
  gpio_enable_clock_escs(escIndex);
  // Escs pins start in input mode, pullup is always on
  gpio_set_mode_escs(escIndex, Mode_IPU);
}

// Reset relevant bits on GPIOs to their initial states
static void reset_all_gpio(int8_t escIndex) {
  int i = 0;
  GPIO_ResetBits(S1W_RX_GPIO, S1W_RX_PIN);
  if (escIndex == -1) {
    for (i = 0; i < ESC_COUNT; i++) {
      GPIO_ResetBits(escHardware[i].gpio, escHardware[i].pin);
    }
  } else {
    GPIO_ResetBits(escHardware[escIndex].gpio, escHardware[escIndex].pin);
  }
}

static uint8_t rxHi() {
  uint16_t val = 0;
  val = (GPIO_ReadInputDataBit(S1W_RX_GPIO, S1W_RX_PIN));
  ledSetState(LED_PRGMR_RX, val == Bit_RESET);
  return val;
}

static void txSet(BitAction val) {
  GPIO_WriteBit(S1W_TX_GPIO, S1W_TX_PIN, val);
  // low == data
  ledSetState(LED_PRGMR_TX, val == Bit_RESET);
}

static uint8_t escHi(int8_t escIndex) {
  uint16_t val = 0;
  // we can only read / passthrough 1 escs, lets pass the first one in "all" mode
  if (escIndex == -1) { escIndex = 1; }
  // inverted schmitt trigger on the input vs. the non-inverted schmitt trigger on the avr
  // invert the input to get the true edge direction
  // bit reset (0) indicates a true rising edge
  val = (GPIO_ReadInputDataBit(escHardware[escIndex].gpio, escHardware[escIndex].pin));
  ledSetState(LED_ESC_RX, val == Bit_RESET);
  return val;
}

void escSet(int8_t escIndex, BitAction val) {
  int i = 0;
  // Write all esc pins
  if (escIndex == -1) {
    for (i = 0; i < ESC_COUNT; i++) {
      GPIO_WriteBit(escHardware[i].gpio, escHardware[i].pin, val);
    }
  }
  // Write the specified pin
  else {
    GPIO_WriteBit(escHardware[escIndex].gpio, escHardware[i].pin, val);
  }
  // low == data
  ledSetState(LED_ESC_TX, val == Bit_RESET);
}

// This method translates 2 wires (a tx and rx line) to 1 wire, by letting the
// RX line control when data should be read or written from the single line
void usb1WirePassthrough(int8_t escIndex)
{
  // Reset all GPIO
  deinit_gpio(escIndex);
  // Take control of the LEDs
  ledInitDebug();
  //delay(1000);
  disable_hardware_uart();
  init_all_gpio(escIndex);
  // reset all the pins, 1wire goes into input mode, pullup on
  reset_all_gpio(escIndex);
  // turn on the ESC now

  // set the programmer high
  txSet(Bit_SET);

  // Wait for programmer to go from 1 -> 0 indicating incoming data
  while(rxHi());
  while(1) {
    // A new iteration on this loop starts when we have data from the programmer (read_programmer goes low)
    // Setup escIndex pin to send data, pullup is the default
    gpio_set_mode_escs(escIndex, Mode_Out_PP);
    // Write the first bit
    escSet(escIndex, Bit_RESET);
    // Echo on the programmer tx line
    txSet(Bit_RESET);

    // Wait for programmer to go 0 -> 1
    while(!rxHi());

    // Programmer is high, end of bit
    // Echo to the esc
    escSet(escIndex, Bit_SET);
    // Listen to the escIndex, input mode, pullup resistor is on
    gpio_set_mode_escs(escIndex, Mode_IPU);

    // Listen to the escIndex while there is no data from the programmer
    while (rxHi()) {
      if (escHi(escIndex)) {
        txSet(Bit_SET);
      }
      else {
        txSet(Bit_RESET);
      }
    }
  }
}

#endif
