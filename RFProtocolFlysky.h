#ifndef _PROTOCOL_FLYSKY_H
#define _PROTOCOL_FLYSKY_H

#include "DeviceA7105.h"
#include "RFProtocol.h"
#include "Timer.h"

class RFProtocolFlysky : public RFProtocol
{
#define MAX_PACKET_SIZE         21

public:
    RFProtocolFlysky(u32 id):RFProtocol(id) { }
    ~RFProtocolFlysky() { close(); }

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
    void buildPacket(u8 init);

// variables
    DeviceA7105  mDev;
    u32  mTXID;
    u32  mPacketCtr;
    u16  mBindCtr;
    u8   mPacketBuf[MAX_PACKET_SIZE];
    u8   mCurRFChanRow;
    u8   mCurRFChanCol;
    u8   mRFChanOffset;
    u8   mCurRFChan;
    
protected:

};

#endif
