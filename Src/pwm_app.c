/*
 * pwm_app.c - PWM dead-time programming (vendor register write)
 *
 * Extracted from settings.c so loadEEpromSettings stays MCU-agnostic.
 */

#include "pwm_app.h"
#include "targets.h"

#ifdef STMICRO
#include "main.h"
#endif
#ifdef ARTERY
#include "main.h"
#endif
#ifdef GIGADEVICES
#include "main.h"
#endif
#ifdef NXP
#include "main.h"
#endif
#ifdef WCH
#include "main.h"
#endif

void setPwmDeadTime(uint8_t dead_time)
{
#ifdef STMICRO
    TIM1->BDTR |= dead_time;
#endif
#ifdef ARTERY
    TMR1->brk |= dead_time;
#endif
#ifdef GIGADEVICES
    TIMER_CCHP(TIMER0) |= dead_time;
#endif
#ifdef NXP
    for (int submodule = 0; submodule <= 2; submodule++) {
        FLEXPWM0->SM[submodule].DTCNT0 = PWM_DTCNT0_DTCNT0(dead_time);
        FLEXPWM0->SM[submodule].DTCNT1 = PWM_DTCNT1_DTCNT1(dead_time);
    }
#endif
#ifdef WCH
    TIM1->BDTR |= dead_time;
#endif
    (void)dead_time;
}
