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

#ifndef _RCVR_PWM_H_
#define _RCVR_PWM_H_
#include <Arduino.h>
#include <avr/pgmspace.h>
#include "Common.h"
#include "RCRcvr.h"

class RCRcvrPWM : public RCRcvr
{

public:
    RCRcvrPWM():RCRcvr() { }
    ~RCRcvrPWM()  { close(); }

    
    virtual void init(void);
    virtual void close(void);
    virtual s16  getRC(u8 ch);
    virtual s16 *getRCs(void);
    virtual u8   getChCnt(void);

private:

};

#endif
