#ifndef _UTILS_H_
#define _UTILS_H_
#include <Arduino.h>
#include <avr/pgmspace.h>
#include "Common.h"

// Bit vector from bit position
#define BV(bit) (1 << bit)

u32  rand32_r(u32 *seed, u8 update);
u32  rand32();

#ifdef __DEBUG_PRINTF__
void printf(char *fmt, ... );
void printf(const __FlashStringHelper *fmt, ... );
#else
#define printf(...)
#endif
#endif
