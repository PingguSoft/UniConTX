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

#ifndef _RCRCVR_H_
#define _RCRCVR_H_
#include <Arduino.h>
#include <avr/pgmspace.h>
#include "Common.h"
#include "RFProtocol.h"

#define CH_CNT      8

class RCRcvr
{

public:
    RCRcvr()    {  }
    ~RCRcvr()   { end();   }

    s16 getRC(u8 ch)
    {
        if (ch < getChCnt())
            return sRC[ch];
        return CHAN_MIN_VALUE;
    }

    s16 *getRCs(void)
    {
        return (s16*)sRC;
    }

    void setRC(u8 ch, s16 val)
    {
        if (ch < getChCnt())
            sRC[ch] = val;
    }

    void end(void)
    {
        close();
    }

    virtual void init(void) = 0;
    virtual void close(void) = 0;
    virtual u8   getChCnt(void) = 0;
    virtual u32  loop(void) { return 0; }

    // TREA1234
    // static for ISR routine
    static s16 sRC[CH_CNT];
};

#endif
