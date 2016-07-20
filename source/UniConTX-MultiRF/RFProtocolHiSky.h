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

#ifndef _PROTOCOL_HISKY_
#define _PROTOCOL_HISKY_

#include "DeviceNRF24L01.h"
#include "RFProtocol.h"

class RFProtocolHiSky : public RFProtocol
{

#define MAX_PACKET_SIZE     10
#define ADDR_BUF_SIZE        5
#define MAX_RF_CHANNELS     20

public:
    RFProtocolHiSky(u32 id):RFProtocol(id) { }
    ~RFProtocolHiSky() { close(); }

// for protocol
    virtual int  init(void);
    virtual int  close(void);
    virtual int  reset(void);
    virtual int  getInfo(s8 id, u8 *data);
    virtual u16  callState(u32 now, u32 expected);

private:
    void buildRFChannels(u32 seed);
    void buildBindingPacket(void);
    void buildDataPacket(void);
    void initRxTxAddr(void);
    void init1(void);
    void setTxID(u32 id);
    u16  getChannel(u8 id);


// variables
    DeviceNRF24L01  mDev;
    u32  mPacketCtr;
    u16  mBindCtr;
    u8   mRFChanBufs[MAX_RF_CHANNELS];
    u8   mPacketBuf[MAX_PACKET_SIZE];
    u8   mBindingBufs[4][MAX_PACKET_SIZE];
    u8   mRxTxAddrBuf[ADDR_BUF_SIZE];

    u8   mCurRFChan;
    u8   mBindingIdx;
    u8   mCtr1ms;
    u8   mState;
protected:

};

#endif
