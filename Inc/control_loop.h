/*
 * control_loop.h - throttle mapping, duty slew, 20 kHz control tick
 */
#ifndef CONTROL_LOOP_H_
#define CONTROL_LOOP_H_

#include <stdint.h>
#include "common.h"

int32_t doPidCalculations(struct fastPID *pidnow, int actual, int target);
uint16_t getSmoothedCurrent(void);
void setInput(void);
void tenKhzRoutine(void);
void processDshot(void);

#endif /* CONTROL_LOOP_H_ */
