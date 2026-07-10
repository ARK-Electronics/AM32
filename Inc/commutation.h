/*
 * commutation.h - six-step commutation and BEMF polling helpers
 */
#ifndef COMMUTATION_H_
#define COMMUTATION_H_

#include <stdint.h>

void getBemfState(void);
void commutate(void);
void advanceincrement(void);
void zcfoundroutine(void);

#endif /* COMMUTATION_H_ */
