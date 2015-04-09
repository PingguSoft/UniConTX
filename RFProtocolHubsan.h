#ifndef _PROTOCOL_HUBSAN_H
#define _PROTOCOL_HUBSAN_H

#include "DeviceA7105.h"
#include "RFProtocol.h"
#include "Timer.h"

class RFProtocolHubsan : public RFProtocol
{
#define MAX_PACKET_SIZE         16
#define INITIAL_WAIT_uS         10000

public:
    RFProtocolHubsan(u32 id):RFProtocol(id) { }
    ~RFProtocolHubsan() { close(); }

// for protocol
    virtual int  init(void);
    virtual int  close(void);
    virtual int  reset(void);
    virtual int  getInfo(s8 id, u8 *data);
    virtual u16  callState(void);

private:
    int  init1(void);
    void updateCRC(void);
    void buildBindPacket(u8 state);
    void buildPacket(void);
    s16  getChannel(u8 ch, s32 scale, s32 center, s32 range);
    u8   checkIntegrity(void);

// variables
    DeviceA7105  mDev;
    u32  mSessionID;
    u32  mTXID;
    u16  mBindCtr;
    u8   mPacketBuf[MAX_PACKET_SIZE];
    u8   mCurRFChan;

    u8   mPacketCtr;    
    u8   mState;
    
protected:

};

#endif
