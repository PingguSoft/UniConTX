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


#ifndef _SERIAL_PROTOCOL_H_
#define _SERIAL_PROTOCOL_H_

#include "Common.h"
#include <stdarg.h>

class SerialProtocol
{

public:
    SerialProtocol();
    ~SerialProtocol();

    void begin(u32 baud, u8 config=SERIAL_8N1);

    static void clearTX(void);
    static void clearRX(void);
    static u8   available(void);
    static u8   read(void);
    static u8   read(u8 *buf);

    static void printf(char *fmt, ... );
    static void printf(const __FlashStringHelper *fmt, ... );
    static void dumpHex(char *name, u8 *data, u16 cnt);

private:

};

#endif
