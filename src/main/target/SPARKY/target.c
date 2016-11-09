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
#include "drivers/dma.h"

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    // 6 3-pin headers
    { TIM15, IO_TAG(PB15), TIM_Channel_2, TIM_USE_MOTOR, 1, GPIO_AF_1, NULL, 0 }, // PWM1  - PB15 - TIM1_CH3N, TIM15_CH1N, *TIM15_CH2
    { TIM15, IO_TAG(PB14), TIM_Channel_1, TIM_USE_MOTOR, 1, GPIO_AF_1, NULL, 0 }, // PWM2  - PB14 - TIM1_CH2N, *TIM15_CH1
    { TIM1,  IO_TAG(PA8),  TIM_Channel_1, TIM_USE_MOTOR, 1, GPIO_AF_6, NULL, 0 }, // PWM3  - PA8  - *TIM1_CH1, TIM4_ETR
    { TIM3,  IO_TAG(PB0),  TIM_Channel_3, TIM_USE_MOTOR, 0, GPIO_AF_2, NULL, 0 }, // PWM4  - PB0  - *TIM3_CH3, TIM1_CH2N, TIM8_CH2N
    { TIM16, IO_TAG(PA6),  TIM_Channel_1, TIM_USE_MOTOR | TIM_USE_LED, 0, GPIO_AF_1, DMA1_Channel3, DMA1_CH3_HANDLER }, // PWM5  - PA6  - *TIM3_CH1, TIM8_BKIN, TIM1_BKIN, TIM16_CH1
    { TIM2,  IO_TAG(PA2),  TIM_Channel_3, TIM_USE_MOTOR, 0, GPIO_AF_1, NULL, 0 }, // PWM6  - PA2  - *TIM2_CH3, !TIM15_CH1
    // PWM7 - PMW10
    { TIM3,  IO_TAG(PB1),  TIM_Channel_4, TIM_USE_PWM,   0, GPIO_AF_2, NULL, 0 }, // PWM7  - PB1  - *TIM3_CH4, TIM1_CH3N, TIM8_CH3N
    { TIM17, IO_TAG(PA7),  TIM_Channel_1, TIM_USE_PWM,   1, GPIO_AF_1, NULL, 0 }, // PWM8  - PA7  - !TIM3_CH2, *TIM17_CH1, TIM1_CH1N, TIM8_CH1
    { TIM3,  IO_TAG(PA4),  TIM_Channel_2, TIM_USE_PWM,   0, GPIO_AF_2, NULL, 0 }, // PWM9  - PA4  - *TIM3_CH2
    { TIM2,  IO_TAG(PA1),  TIM_Channel_2, TIM_USE_PWM,   0, GPIO_AF_1, NULL, 0 }, // PWM10 - PA1  - *TIM2_CH2, TIM15_CH1N
    // PPM PORT - Also USART2 RX (AF5)
    { TIM2,  IO_TAG(PA3),  TIM_Channel_4, TIM_USE_PPM,   0, GPIO_AF_1, NULL, 0 } // PPM   - PA3  - TIM2_CH4, TIM15_CH2 - PWM13
};

