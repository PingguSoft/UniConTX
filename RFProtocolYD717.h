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
    virtual u16  callState(void);

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
