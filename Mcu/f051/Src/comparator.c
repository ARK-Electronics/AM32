/*
 * comparator.c
 *
 *  Created on: Sep. 26, 2020
 *      Author: Alka
 */

#include "comparator.h"
#include "common.h"
#include "targets.h"

COMP_TypeDef* active_COMP = COMP1;

RAM_FUNC void maskPhaseInterrupts()
{
  EXTI->IMR &= ~(1 << 21);
  EXTI->PR = EXTI_LINE;
}

RAM_FUNC void enableCompInterrupts()
{
  EXTI->PR = EXTI_LINE; // discard any edge latched while masked (comStep/PWM
                        // ring during the wait window) so the first edge after
                        // unmask is genuine - critical for the demag release
                        // edge, whose ISR branch has no interval/confirm gate
  EXTI->IMR |= (1 << 21);
}

RAM_FUNC void changeCompInput()
{

if((average_interval < 400)){
COMP->CSR = COMP->CSR & ~(1<<2);
}else{
COMP->CSR  = COMP->CSR | 1<<2;
}
  if (auto_blanking) { // look for the demag release edge first, reversed polarity
    EXTI->RTSR = rising << 21;
    EXTI->FTSR = !rising << 21;
  } else {
    EXTI->RTSR = !rising << 21;
    EXTI->FTSR = rising << 21;
  }
}
