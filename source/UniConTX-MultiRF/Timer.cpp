/*
 *      This program is free software; you can redistribute it and/or modify
 *      it under the terms of the GNU General Public License as published by
 *      the Free Software Foundation; either version 2 of the License, or
 *      (at your option) any later version.
 *
 *      This program is distributed in the hope that it will be useful,
 *      but WITHOUT ANY WARRANTY; without even the implied warranty of
 *      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *      GNU General Public License for more details.
 *
 *      You should have received a copy of the GNU General Public License
 *      along with this program; if not, write to the Free Software
 *      Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 *      MA 02110-1301, USA.
 */

/*  * * * * * * * * * * * * * * * * * * * * * * * * * * *
 Code by Simon Monk
 http://www.simonmonk.org
* * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// For Arduino 1.0 and earlier
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "Timer.h"
#include "Utils.h"

Timer::Timer(void)
{
    mPeriod = 0;
}

int8_t Timer::after(unsigned long period)
{
    if (mPeriod != period) {
        mLastEventTime = micros();
        mPeriod = period;
    }
    return 1;
}

void Timer::stop(int8_t id)
{
    mPeriod = 0;
}

void Timer::update(unsigned long now)
{
    unsigned long diff;

    if (mPeriod == 0)
        return;

#if 0
    if (now < mLastEventTime) {
        diff = 0xffffffffL - lastEventTime;
        diff += (now + 1);
    } else {
        diff = now - mLastEventTime;
    }
#endif

    diff = now - mLastEventTime;

    if (diff >= mPeriod) {
        mLastEventTime = now;
        handleTimer(1);
    }

    if (diff > (mPeriod + mPeriod * 0.5)) {
        pf("TOO LATE !!!\n");
    }
}
