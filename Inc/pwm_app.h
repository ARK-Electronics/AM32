/*
 * pwm_app.h - application PWM helpers (dead-time programming)
 */
#ifndef PWM_APP_H_
#define PWM_APP_H_

#include <stdint.h>

/* Program bridge dead-time (MCU-specific timer register). */
void setPwmDeadTime(uint8_t dead_time);

#endif /* PWM_APP_H_ */
