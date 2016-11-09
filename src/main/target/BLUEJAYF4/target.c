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
    { TIM8, IO_TAG(PC7), TIM_Channel_2, TIM_USE_PPM,                 0, GPIO_AF_TIM8, NULL,         0,             0  }, // PPM IN
    { TIM5, IO_TAG(PA0), TIM_Channel_1, TIM_USE_MOTOR,               1, GPIO_AF_TIM5, DMA1_Stream2, DMA_Channel_6, DMA1_ST2_HANDLER }, // S1_OUT
    { TIM5, IO_TAG(PA1), TIM_Channel_2, TIM_USE_MOTOR,               1, GPIO_AF_TIM5, DMA1_Stream4, DMA_Channel_6, DMA1_ST4_HANDLER }, // S2_OUT
    { TIM2, IO_TAG(PA2), TIM_Channel_3, TIM_USE_MOTOR,               1, GPIO_AF_TIM2, DMA1_Stream1, DMA_Channel_3, DMA1_ST1_HANDLER }, // S3_OUT
    { TIM2, IO_TAG(PA3), TIM_Channel_4, TIM_USE_MOTOR,               1, GPIO_AF_TIM2, DMA1_Stream6, DMA_Channel_3, DMA1_ST6_HANDLER }, // S4_OUT
    { TIM3, IO_TAG(PB1), TIM_Channel_4, TIM_USE_MOTOR | TIM_USE_LED, 1, GPIO_AF_TIM3, DMA1_Stream2, DMA_Channel_5, DMA1_ST2_HANDLER }, // S5_OUT
    { TIM3, IO_TAG(PB0), TIM_Channel_3, TIM_USE_MOTOR,               1, GPIO_AF_TIM3, DMA1_Stream7, DMA_Channel_5, DMA1_ST7_HANDLER }, // S6_OUT
};

