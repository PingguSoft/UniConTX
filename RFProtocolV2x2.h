#ifndef _PROTOCOL_V2x2_
#define _PROTOCOL_V2x2_

#include "DeviceNRF24L01.h"
#include "RFProtocol.h"

class RFProtocolV2x2 : public RFProtocol
{

#define MAX_PACKET_SIZE     16
#define ADDR_BUF_SIZE        3
#define MAX_RF_CHANNELS     17

public:
    RFProtocolV2x2(u32 id):RFProtocol(id) { }
    ~RFProtocolV2x2() { close(); }

// for protocol
    virtual int  init(void);
    virtual int  close(void);
    virtual int  reset(void);
    virtual int  getInfo(s8 id, u8 *data);
    virtual u16  callState(void);

private:
    u8   getCheckSum(u8 *data);
    u8   checkStatus(void);
    u8   getChannel(u8 id);
    void getControls(u8* throttle, u8* rudder, u8* elevator, u8* aileron, u8* flags, u16 *led_blink);
    void sendPacket(u8 bind);
    void initRxTxAddr(void);
    void init1(void);
    void init2(void);
    void setTxID(u32 id);

// variables
    DeviceNRF24L01  mDev;
    u32  mPacketCtr;
    u16  mBindCtr;
    u16  mLedBlinkCtr;
    u8   mRFChanBufs[MAX_RF_CHANNELS];
    u8   mPacketBuf[MAX_PACKET_SIZE];
    u8   mRxTxAddrBuf[ADDR_BUF_SIZE];
    u8   mCurRFChan;
    u8   mAuxFlag;
    u8   mState;
    u8   mPacketSent;
protected:

};

#endif
