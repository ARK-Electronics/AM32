/*
  IO.c - SITL stubs. DShot/servo signal input is not supported in SITL,
  input comes from DroneCAN
 */

#include "IO.h"

#include "targets.h"

uint32_t dma_buffer[64];
volatile char out_put;
char ic_timer_prescaler;
uint8_t buffer_padding;

void changeToOutput(void) { }
void changeToInput(void) { }
void receiveDshotDma(void) { }
void sendDshotDma(void) { }

uint8_t getInputPinState(void)
{
    return 1; // idle high, no signal
}

void setInputPolarityRising(void) { }
void setInputPullDown(void) { }
void setInputPullUp(void) { }
void setInputPullNone(void) { }
void enableHalfTransferInt(void) { }
