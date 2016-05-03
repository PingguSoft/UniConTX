/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is derived from deviationTx project for Arduino.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 see <http://www.gnu.org/licenses/>
*/

#ifndef _UTILS_H_
#define _UTILS_H_
#include <Arduino.h>
#include <avr/pgmspace.h>
#include "Common.h"

// Bit vector from bit position
#define BV(bit) (1 << (bit))

u32  rand32_r(u32 *seed, u8 update);
u32  rand32();

#define __DEBUG_PRINTF__    1

#ifdef __DEBUG_PRINTF__
void printf2(char *fmt, ... );
void printf2(const __FlashStringHelper *fmt, ... );
#else
#define printf2(...)
#endif
#endif

void dump(char *name, u8 *data, u16 cnt);