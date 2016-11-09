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

#include <stdbool.h>
#include <stdint.h>

#include <platform.h>
#include "drivers/io.h"

#include "drivers/timer.h"
#include "drivers/dma.h"

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM3,  IO_TAG(PA4),  TIM_Channel_2, TIM_USE_MOTOR, 1, GPIO_AF_2, NULL, 0 }, // PWM1 - PA4
    { TIM17, IO_TAG(PA7),  TIM_Channel_1, TIM_USE_MOTOR, 1, GPIO_AF_1, NULL, 0 }, // PWM2 - PA7
    { TIM1,  IO_TAG(PA8),  TIM_Channel_1, TIM_USE_MOTOR, 1, GPIO_AF_6, NULL, 0 }, // PWM3 - PA8
    { TIM3,  IO_TAG(PB0),  TIM_Channel_3, TIM_USE_MOTOR, 1, GPIO_AF_2, NULL, 0 }, // PWM4 - PB0
    { TIM3,  IO_TAG(PB1),  TIM_Channel_4, TIM_USE_MOTOR, 1, GPIO_AF_2, NULL, 0 }, // PWM5 - PB1
    { TIM2,  IO_TAG(PA1),  TIM_Channel_2, TIM_USE_PPM,   0, GPIO_AF_1, NULL, 0 }, // PWM6 - PPM
    { TIM16, IO_TAG(PB8),  TIM_Channel_1, TIM_USE_LED,   0, GPIO_AF_1, DMA1_Channel3, DMA1_CH3_HANDLER}, // PWM6 - PPM
};
