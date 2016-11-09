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

#include <stdint.h>

#include <platform.h>
#include "drivers/io.h"

#include "drivers/timer.h"

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {

    { TIM4,  IO_TAG(PB6),  TIM_Channel_1, TIM_USE_MOTOR, 1, GPIO_AF_2 }, // PWM1 - PB6
    { TIM4,  IO_TAG(PB7),  TIM_Channel_2, TIM_USE_MOTOR, 1, GPIO_AF_2 }, // PWM2 - PB6
    { TIM4,  IO_TAG(PB8),  TIM_Channel_3, TIM_USE_MOTOR, 1, GPIO_AF_2 }, // PWM3 - PB8
    { TIM4,  IO_TAG(PB9),  TIM_Channel_4, TIM_USE_MOTOR, 1, GPIO_AF_2 }, // PWM4 - PB9
    { TIM3,  IO_TAG(PB0),  TIM_Channel_3, TIM_USE_MOTOR, 1, GPIO_AF_2 }, // PWM5 - PB0  - *TIM3_CH3
    { TIM3,  IO_TAG(PB1),  TIM_Channel_4, TIM_USE_MOTOR, 1, GPIO_AF_2 }, // PWM6 - PB1  - *TIM3_CH4
    { TIM2,  IO_TAG(PB11), TIM_Channel_4, TIM_USE_PPM,   0, GPIO_AF_1 }, // RC_CH3 - PB11 - *TIM2_CH4, USART3_RX (AF7)y
};


