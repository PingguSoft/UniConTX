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

#include "Event.h"

Event::Event(void)
{
	eventType = EVENT_NONE;
}

uint8_t Event::update(void)
{
    unsigned long now = micros();
    return update(now);
}

uint8_t Event::update(unsigned long now)
{
  unsigned long diff;
  uint8_t ret = 0;

  if (now < lastEventTime)
  {
      diff = 0xffffffffL - lastEventTime;
      diff += (now + 1);
  } else {
      diff = now - lastEventTime;
  }

    if (diff >= period)
    {
        ret = eventType;
        switch (eventType)
        {
            case EVENT_EVERY:
                if (callback)
                    (*callback)();
                break;

#if 0
            case EVENT_OSCILLATE:
                pinState = ! pinState;
                digitalWrite(pin, pinState);
                break;
#endif

        }
        lastEventTime = now;
        count++;
    }
    if (repeatCount > -1 && count >= repeatCount)
    {
        eventType = EVENT_NONE;
    }

    return ret;
}
