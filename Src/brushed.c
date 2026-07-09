/*
 * brushed.c - extracted from main.c (behavior-neutral)
 */

#include "brushed.h"

#ifdef BRUSHED_MODE

#include "motor_runtime.h"
#include "main.h"
#include "common.h"
#include "control_loop.h"
#include "phaseouts.h"
#include "peripherals.h"
#include "functions.h"
#include "eeprom.h"
#include "signal.h"
#include "targets.h"

void runBrushedLoop(void)
{
    uint16_t brushed_duty_cycle = 0;

    if (brushed_direction_set == 0 && adjusted_input > 48) {
        if (forward) {
            allOff();
            delayMicros(10);
            twoChannelForward();
        } else {
            allOff();
            delayMicros(10);
            twoChannelReverse();
        }
        brushed_direction_set = 1;
    }

    brushed_duty_cycle = map(adjusted_input, 48, 2047, 0,
        (TIMER1_MAX_ARR - (TIMER1_MAX_ARR / 20)));

    if (degrees_celsius > eepromBuffer.limits.temperature) {
        duty_cycle_maximum = map(degrees_celsius, eepromBuffer.limits.temperature,
            eepromBuffer.limits.temperature + 20, TIMER1_MAX_ARR / 2, 1);
    } else {
        duty_cycle_maximum = TIMER1_MAX_ARR - 50;
    }
    if (brushed_duty_cycle > duty_cycle_maximum) {
        brushed_duty_cycle = duty_cycle_maximum;
    }

    if (use_current_limit) {
        use_current_limit_adjust -= (int16_t)(doPidCalculations(&currentPid, actual_current,
                                                  CURRENT_LIMIT * 100)
            / 10000);
        if (use_current_limit_adjust < minimum_duty_cycle) {
            use_current_limit_adjust = minimum_duty_cycle;
        }

        if (brushed_duty_cycle > use_current_limit_adjust) {
            brushed_duty_cycle = use_current_limit_adjust;
        }
    }
    if ((brushed_duty_cycle > 0) && armed) {
        SET_DUTY_CYCLE_ALL(brushed_duty_cycle);
    } else {
        SET_DUTY_CYCLE_ALL(0);
        brushed_direction_set = 0;
    }
}

#endif /* BRUSHED_MODE */
