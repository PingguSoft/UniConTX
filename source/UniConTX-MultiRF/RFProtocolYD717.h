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

#ifndef _PROTOCOL_YD717_
#define _PROTOCOL_YD717_

#include "DeviceNRF24L01.h"
#include "RFProtocol.h"


class RFProtocolYD717 : public RFProtocol
{
#define PAYLOADSIZE          8  // receive data pipes set to this size, but unused
#define MAX_PACKET_SIZE      9  // YD717 packets have 8-byte payload, Syma X4 is 9
#define ADDR_BUF_SIZE        5

public:
    RFProtocolYD717(u32 id):RFProtocol(id) { }
    ~RFProtocolYD717() { close(); }

// for protocol
    virtual int  init(void);
    virtual int  close(void);
    virtual int  reset(void);
    virtual int  getInfo(s8 id, u8 *data);
    virtual u16  callState(u32 now, u32 expected);

private:
    u8   getCheckSum(u8 *data);
    u8   checkStatus(void);
    u8   getControl(u8 id);
    void getControls(u8* throttle, u8* rudder, u8* elevator, u8* aileron,
                     u8* flags, u8* rudder_trim, u8* elevator_trim, u8* aileron_trim);
    void sendPacket(u8 bind);
    void initRxTxAddr(void);
    void init1(void);
    void init2(void);
    void init3(void);
    void setRFChannel(u8 address);
    void updateTelemetry(void);

    void testUp(void);
    void testDown(void);

// variables
    DeviceNRF24L01  mDev;

    u32  mPacketCtr;
    u16  mBindCtr;
    u8   mPacketBuf[MAX_PACKET_SIZE];
    u8   mRxTxAddrBuf[ADDR_BUF_SIZE];
    u8   mState;

protected:

};

#endif
