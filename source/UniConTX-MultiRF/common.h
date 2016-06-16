/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 see <http://www.gnu.org/licenses/>
*/


#ifndef _COMMON_H_
#define _COMMON_H_

#include <stdint.h>
#include <string.h>
#include <inttypes.h>

typedef int8_t s8;
typedef int16_t s16;
typedef int32_t s32;
typedef uint8_t u8;
//typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;

#ifndef NULL
#define NULL    0
#endif

#ifndef TRUE
#define TRUE    1
#endif

#ifndef FALSE
#define FALSE   0
#endif

#define PPM_MIN_VALUE   1000
#define PPM_MAX_VALUE   2000

#define CHAN_MIN_VALUE -500
#define CHAN_MID_VALUE  0
#define CHAN_MAX_VALUE  500

#define __DEBUG__       1
#define __STD_SERIAL__  0

#endif
