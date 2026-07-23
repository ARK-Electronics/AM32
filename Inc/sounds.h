/*
 * sounds.h
 *
 *  Created on: May 13, 2020
 *      Author: Alka
 */

#ifndef SOUNDS_H_
#define SOUNDS_H_

#include "main.h"

void playStartupTune(void);
void playInputTune(void);
void playBrushedStartupTune(void);
void playInputTune2(void);
void playBeaconTune3(void);
void playDuskingTune(void);
void playDefaultTone(void);
void playChangedTone(void);
void playSignalLostTone(void);

/* Call just before NVIC_SystemReset on RC signal timeout so the next boot
 * plays playSignalLostTone instead of the full ARK startup signature. */
void bootSoundMarkSignalLost(void);

void setVolume(uint8_t volume);

extern void delayMillis(uint32_t millis);

#endif /* SOUNDS_H_ */
