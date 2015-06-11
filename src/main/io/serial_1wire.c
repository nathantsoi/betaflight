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

// Copy-pasted from simonk
// All transmissions have a leader of 23 1-bits followed by 1 0-bit.
// Bit encoding starts at the least significant bit and is 8 bits wide.
// 1-bits are encoded as 64.0us high, 72.8us low (135.8us total).
// 0-bits are encoded as 27.8us high, 34.5us low, 34.4us high, 37.9 low
// (134.6us total)
// End of encoding adds 34.0us high, then return to input mode.
// The last 0-bit low time is 32.6us instead of 37.9us, for some reason.

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "drivers/system.h"
#include "drivers/gpio.h"
#include "drivers/light_led.h"
#include "drivers/serial.h"
#include "io/serial.h"
#include "io/serial_1wire.h"

#ifdef USE_SERIAL_1WIRE

#define BIT_DELAY_HALF 34
#define BIT_DELAY 68

#if defined(CJMCU) || defined(EUSTM32F103RC) || defined(NAZE) || defined(OLIMEXINO) || defined(PORT103R)
const serial1WireHardware_t serial1WireOut[SERIAL_1WIRE_MOTOR_COUNT] = {
    { GPIOA, Pin_8 },       // PWM9 - OUT1
    { GPIOA, Pin_11 },      // PWM10 - OUT2
    { GPIOB, Pin_6, },      // PWM11 - OUT3
    { GPIOB, Pin_7, },      // PWM12 - OUT4
    { GPIOB, Pin_8, },      // PWM13 - OUT5
    { GPIOB, Pin_9, }       // PWM14 - OUT6
};
#endif

#ifdef CC3D
const serial1WireHardware_t serial1WireOut[SERIAL_1WIRE_MOTOR_COUNT] = {
    { GPIOB, Pin_9 },    // S1_OUT
    { GPIOB, Pin_8 },    // S2_OUT
    { GPIOB, Pin_7 },    // S3_OUT
    { GPIOA, Pin_8 },    // S4_OUT
    { GPIOB, Pin_4 },    // S5_OUT - GPIO_PartialRemap_TIM3 - LED Strip
    { GPIOA, Pin_2 }     // S6_OUT
};
#endif

volatile uint8_t serialBuffer[255];

void gpio_config_out(const serial1WireHardware_t *serial1WireHardwarePtr)
{
    gpio_config_t cfg;

    cfg.mode = Mode_Out_PP;
    cfg.pin = serial1WireHardwarePtr->pin;
    cfg.speed = Speed_2MHz;

    gpioInit(serial1WireHardwarePtr->gpio, &cfg);
}

void gpio_config_in(const serial1WireHardware_t *serial1WireHardwarePtr)
{
    gpio_config_t cfg;

    cfg.mode = Mode_IPU;
    cfg.pin = serial1WireHardwarePtr->pin;
    cfg.speed = Speed_2MHz;

    gpioInit(serial1WireHardwarePtr->gpio, &cfg);
}

void sendDigital1(const serial1WireHardware_t *serial1WireHardwarePtr)
{
    digitalHi(serial1WireHardwarePtr->gpio, serial1WireHardwarePtr->pin);
    delayMicroseconds(BIT_DELAY);
    digitalLo(serial1WireHardwarePtr->gpio, serial1WireHardwarePtr->pin);
    delayMicroseconds(BIT_DELAY);
}

void sendDigital0(const serial1WireHardware_t *serial1WireHardwarePtr)
{
    digitalHi(serial1WireHardwarePtr->gpio, serial1WireHardwarePtr->pin);
    delayMicroseconds(BIT_DELAY_HALF);
    digitalLo(serial1WireHardwarePtr->gpio, serial1WireHardwarePtr->pin);
    delayMicroseconds(BIT_DELAY_HALF);
    digitalHi(serial1WireHardwarePtr->gpio, serial1WireHardwarePtr->pin);
    delayMicroseconds(BIT_DELAY_HALF);
    digitalLo(serial1WireHardwarePtr->gpio, serial1WireHardwarePtr->pin);
    delayMicroseconds(BIT_DELAY_HALF);
}

void sendByte(uint8_t byte, const serial1WireHardware_t *serial1WireHardwarePtr)
{
    for(uint8_t i = 0; i < 8; i++)
    {
        if(byte & (1 << i))
        {
            sendDigital1(serial1WireHardwarePtr);
        } else {
            sendDigital0(serial1WireHardwarePtr);
        }
    }
}

void sendBuf(uint8_t txlen, const serial1WireHardware_t *serial1WireHardwarePtr)
{
    gpio_config_out(serial1WireHardwarePtr);

    // send intro message
    for(uint8_t i = 0; i < 23; i++)
    {
        sendDigital1(serial1WireHardwarePtr);
    }
    sendDigital0(serial1WireHardwarePtr);

    for(uint8_t i = 0; i < txlen; i++)
    {
        sendByte(serialBuffer[i], serial1WireHardwarePtr);
    }

    // send trailing message
    digitalHi(serial1WireHardwarePtr->gpio, serial1WireHardwarePtr->pin);
    delayMicroseconds(BIT_DELAY_HALF);

    gpio_config_in(serial1WireHardwarePtr);
}

int8_t readBit(uint32_t bitPeriod, const serial1WireHardware_t *serial1WireHardwarePtr)
{
    uint32_t startTime = micros();
    while(digitalIn(serial1WireHardwarePtr->gpio, serial1WireHardwarePtr->pin)) // wait to go low
        if (micros() > startTime + 250)
            return -1;
    while(!digitalIn(serial1WireHardwarePtr->gpio, serial1WireHardwarePtr->pin)) // wait to go high
        if (micros() > startTime + 250)
            return -1;
    uint32_t endTime = micros();

    if((endTime - startTime) < (bitPeriod / 1.5)) // short pulses
    {
        while(digitalIn(serial1WireHardwarePtr->gpio, serial1WireHardwarePtr->pin)) // wait for second half of bit
            if (micros() > startTime + 250)
                return -1;
        while(!digitalIn(serial1WireHardwarePtr->gpio, serial1WireHardwarePtr->pin))
            if (micros() > startTime + 250)
                return -1;
        return 0;
    }
    return 1;
}

void serial1Wire(serialPort_t *serialPortIn, uint8_t motorIndex)
{
    LED0_OFF;
#ifdef LED1
    LED1_OFF;
#endif
    const serial1WireHardware_t *serial1WireHardwarePtr = &serial1WireOut[motorIndex];

    gpio_config_in(serial1WireHardwarePtr);
    uint16_t lastPin = 0;

    while(1)
    {
        if (serialTotalBytesWaiting(serialPortIn))
        {
            LED1_ON;
            uint16_t rxlen = 0;
            while(serialTotalBytesWaiting(serialPortIn))
            {
                serialBuffer[rxlen++] = serialRead(serialPortIn);
                delay(2);

                if (rxlen == 255)
                {
                    while(serialTotalBytesWaiting(serialPortIn))
                        serialRead(serialPortIn);
                    break;
                }
            }
#ifdef LED1
            LED1_OFF;
#endif
            sendBuf(rxlen, serial1WireHardwarePtr);
            lastPin = 1;
        } else {
            // read from pin
            uint16_t curPin = digitalIn(serial1WireHardwarePtr->gpio, serial1WireHardwarePtr->pin);

            if ((lastPin == 0) && (curPin != 0)) // pin went high from low
            {
                LED0_ON;
                // get sync time from header
                volatile uint32_t startTime, endTime, bitPeriod;

                startTime = micros();

                // get starting time at next low-high transition
                while(digitalIn(serial1WireHardwarePtr->gpio, serial1WireHardwarePtr->pin)) // wait to go low
                    if (micros() > startTime + 250)
                                break;
                while(!digitalIn(serial1WireHardwarePtr->gpio, serial1WireHardwarePtr->pin)) // wait to go high
                    if (micros() > startTime + 250)
                                break;
                startTime = micros();

                // get ending time at next low-high transition
                while(digitalIn(serial1WireHardwarePtr->gpio, serial1WireHardwarePtr->pin)) // wait to go low
                    if (micros() > startTime + 250)
                                break;
                while(!digitalIn(serial1WireHardwarePtr->gpio, serial1WireHardwarePtr->pin)) // wait to go high
                    if (micros() > startTime + 250)
                                break;
                endTime = micros();

                bitPeriod = endTime - startTime; // doesn't include overflow case

                uint8_t introCount = 0;
                while(readBit(bitPeriod, serial1WireHardwarePtr) == 1) // exit on last intro bit, which is 0
                {
                    introCount++;
                }
                if (introCount > 10) // decent threshold
                {
                    uint8_t rxlen = 0;
                    int8_t tmp;
                    uint8_t timeout = 0;
                    while(timeout == 0)
                    {
                        for (int8_t i = 0; i < 8; i++)
                        {
                            if (i == 0)
                                serialBuffer[rxlen] = 0; // reset byte for bitwise operations
                            tmp = readBit(bitPeriod, serial1WireHardwarePtr);
                            if (tmp == -1) // timeout reached
                            {
                                timeout = 1;
                                break;
                            } else {
                                serialBuffer[rxlen] |=  (tmp << i); // LSB first
                                if (i == 7)
                                    rxlen++;
                            }
                        }
                    }

                    for (uint8_t i = 0; i < rxlen; i++)
                    {
                        serialWrite(serialPortIn, serialBuffer[i]);
                    }
                }
                LED0_OFF;
            }
            lastPin = curPin;
        }
    }
}

#endif
