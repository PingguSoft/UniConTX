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

#include <Arduino.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <SPI.h>

#include "common.h"
#include "utils.h"
#include "RCRcvrPPM.h"

#define PIN_PPM     2
#define MULTIPLIER (F_CPU / 8000000)
#define CH_CNT      8

// TREA1234
static s16 sRC[CH_CNT];

void calcPPM();

s16 RCRcvrPPM::getRC(u8 ch)
{
    if (ch >= getChCnt())
        return -500;

    return sRC[ch];
}

s16 *RCRcvrPPM::getRCs(void)
{
    return sRC;
}

u8 RCRcvrPPM::getChCnt(void)
{
    return CH_CNT;
}

void RCRcvrPPM::init(void)
{
    memset(sRC, 0, sizeof(sRC));
    sRC[0] = -500;  // throttle min

    pinMode(PIN_PPM, INPUT);
    attachInterrupt(PIN_PPM - 2, calcPPM, RISING);

#if 0
    TCCR1A = 0;
    TCCR1B = 0;
    TCCR1B |= (1 << CS11);  //set timer1 to increment every 0,5 us
#endif
}

void RCRcvrPPM::close(void)
{
    detachInterrupt(PIN_PPM - 2);
}

void calcPPM()
{
    static u8   ch;
    static u32  lastTS;

    u32 ts = micros();
    u32 diff = ts - lastTS;

    if (diff > 2500) {
        ch = 0;
    } else {
        if (ch < CH_CNT - 1) {
            u16 val = constrain(diff, 1000, 2000);
            sRC[ch++] = map(val, 1000, 2000, -500, 500);
        }
    }
    lastTS = ts;
}

