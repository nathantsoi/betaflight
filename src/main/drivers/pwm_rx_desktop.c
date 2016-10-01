#include <stdbool.h>
#include <stdint.h>

#include <platform.h>

#include "build/build_config.h"
#include "build/debug.h"

#include "common/utils.h"

#include "system.h"

#include "nvic.h"
#include "io.h"
#include "timer.h"

#include "pwm_output.h"
#include "pwm_mapping.h"

#include "pwm_rx.h"

// TODO: desktop
uint16_t pwmRead(uint8_t channel)
{
    return 0;
}
