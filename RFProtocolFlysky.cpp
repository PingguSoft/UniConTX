#include <SPI.h>
#include "RFProtocolFlysky.h"
#include "utils.h"

#define MAX_BIND_COUNT          2500
#define INITIAL_WAIT_uS         2400

#define PROTO_OPT_WLTOYS        0x01

static const PROGMEM u8 TBL_INIT_REGS[] = {
     -1,  0x42, 0x00, 0x14, 0x00,  -1 ,  -1 , 0x00, 0x00, 0x00, 0x00, 0x01, 0x21, 0x05, 0x00, 0x50,
    0x9e, 0x4b, 0x00, 0x02, 0x16, 0x2b, 0x12, 0x00, 0x62, 0x80, 0x80, 0x00, 0x0a, 0x32, 0xc3, 0x0f,
    0x13, 0xc3, 0x00,  -1,  0x00, 0x00, 0x3b, 0x00, 0x17, 0x47, 0x80, 0x03, 0x01, 0x45, 0x18, 0x00,
    0x01, 0x0f,  -1,
};
static const PROGMEM u8 TBL_TX_CHANS[16][16] = {
  {0x0a, 0x5a, 0x14, 0x64, 0x1e, 0x6e, 0x28, 0x78, 0x32, 0x82, 0x3c, 0x8c, 0x46, 0x96, 0x50, 0xa0},
  {0xa0, 0x50, 0x96, 0x46, 0x8c, 0x3c, 0x82, 0x32, 0x78, 0x28, 0x6e, 0x1e, 0x64, 0x14, 0x5a, 0x0a},
  {0x0a, 0x5a, 0x50, 0xa0, 0x14, 0x64, 0x46, 0x96, 0x1e, 0x6e, 0x3c, 0x8c, 0x28, 0x78, 0x32, 0x82},
  {0x82, 0x32, 0x78, 0x28, 0x8c, 0x3c, 0x6e, 0x1e, 0x96, 0x46, 0x64, 0x14, 0xa0, 0x50, 0x5a, 0x0a},
  {0x28, 0x78, 0x0a, 0x5a, 0x50, 0xa0, 0x14, 0x64, 0x1e, 0x6e, 0x3c, 0x8c, 0x32, 0x82, 0x46, 0x96},
  {0x96, 0x46, 0x82, 0x32, 0x8c, 0x3c, 0x6e, 0x1e, 0x64, 0x14, 0xa0, 0x50, 0x5a, 0x0a, 0x78, 0x28},
  {0x50, 0xa0, 0x28, 0x78, 0x0a, 0x5a, 0x1e, 0x6e, 0x3c, 0x8c, 0x32, 0x82, 0x46, 0x96, 0x14, 0x64},
  {0x64, 0x14, 0x96, 0x46, 0x82, 0x32, 0x8c, 0x3c, 0x6e, 0x1e, 0x5a, 0x0a, 0x78, 0x28, 0xa0, 0x50},
  {0x50, 0xa0, 0x46, 0x96, 0x3c, 0x8c, 0x28, 0x78, 0x0a, 0x5a, 0x32, 0x82, 0x1e, 0x6e, 0x14, 0x64},
  {0x64, 0x14, 0x6e, 0x1e, 0x82, 0x32, 0x5a, 0x0a, 0x78, 0x28, 0x8c, 0x3c, 0x96, 0x46, 0xa0, 0x50},
  {0x46, 0x96, 0x3c, 0x8c, 0x50, 0xa0, 0x28, 0x78, 0x0a, 0x5a, 0x1e, 0x6e, 0x32, 0x82, 0x14, 0x64},
  {0x64, 0x14, 0x82, 0x32, 0x6e, 0x1e, 0x5a, 0x0a, 0x78, 0x28, 0xa0, 0x50, 0x8c, 0x3c, 0x96, 0x46},
  {0x46, 0x96, 0x0a, 0x5a, 0x3c, 0x8c, 0x14, 0x64, 0x50, 0xa0, 0x28, 0x78, 0x1e, 0x6e, 0x32, 0x82},
  {0x82, 0x32, 0x6e, 0x1e, 0x78, 0x28, 0xa0, 0x50, 0x64, 0x14, 0x8c, 0x3c, 0x5a, 0x0a, 0x96, 0x46},
  {0x46, 0x96, 0x0a, 0x5a, 0x50, 0xa0, 0x3c, 0x8c, 0x28, 0x78, 0x1e, 0x6e, 0x32, 0x82, 0x14, 0x64},
  {0x64, 0x14, 0x82, 0x32, 0x6e, 0x1e, 0x78, 0x28, 0x8c, 0x3c, 0xa0, 0x50, 0x5a, 0x0a, 0x96, 0x46},
};

int RFProtocolFlysky::init1(void)
{
    u8 if_calibration1;
    u8 vco_calibration0;
    u8 vco_calibration1;
    //u8 vco_current;

    mDev.writeID(0x5475c52a);
    u8 reg;
    for (u8 i = 0; i < 0x33; i++) {
        reg = pgm_read_byte(TBL_INIT_REGS + i);
        if((s8)reg != -1)
            mDev.writeReg(i, reg);
    }
    mDev.strobe(A7105_STANDBY);

    //IF Filter Bank Calibration
    mDev.writeReg(0x02, 1);
    //vco_current =
    mDev.readReg(0x02);

    u32 ms = millis();
    while(millis()  - ms < 500) {
        if(! mDev.readReg(0x02))
            break;
    }
    if (millis() - ms >= 500)
        return 0;
    
    if_calibration1 = mDev.readReg(A7105_22_IF_CALIB_I);
    mDev.readReg(A7105_24_VCO_CURCAL);
    if(if_calibration1 & A7105_MASK_FBCF) {
        //Calibration failed...what do we do?
        return 0;
    }

    //VCO Current Calibration
    mDev.writeReg(0x24, 0x13); //Recomended calibration from A7105 Datasheet

    //VCO Bank Calibration
    mDev.writeReg(0x26, 0x3b); //Recomended limits from A7105 Datasheet

    //VCO Bank Calibrate channel 0?
    //Set Channel
    mDev.writeReg(A7105_0F_CHANNEL, 0);
    
    //VCO Calibration
    mDev.writeReg(0x02, 2);
    ms = millis();
    while(millis()  - ms < 500) {
        if(! mDev.readReg(0x02))
            break;
    }
    if (millis() - ms >= 500)
        return 0;

    vco_calibration0 = mDev.readReg(A7105_25_VCO_SBCAL_I);
    if (vco_calibration0 & A7105_MASK_VBCF) {
        //Calibration failed...what do we do?
        return 0;
    }

    //Calibrate channel 0xa0?
    //Set Channel
    mDev.writeReg(A7105_0F_CHANNEL, 0xa0);
    //VCO Calibration
    mDev.writeReg(A7105_02_CALC, 2);
    ms = millis();
    while(millis()  - ms < 500) {
        if(! mDev.readReg(A7105_02_CALC))
            break;
    }
    if (millis() - ms >= 500)
        return 0;

    vco_calibration1 = mDev.readReg(A7105_25_VCO_SBCAL_I);
    if (vco_calibration1 & A7105_MASK_VBCF) {
        //Calibration failed...what do we do?
    }

    //Reset VCO Band calibration
    mDev.writeReg(0x25, 0x08);
    mDev.setTxRxMode(TX_EN);
    mDev.setRFPower(getRFPower());
    mDev.strobe(A7105_STANDBY);

    return 1;
}

void RFProtocolFlysky::buildPacket(u8 init)
{
    //-100% =~ 0x03e8
    //+100% =~ 0x07ca
    //Calculate:
    //Center = 0x5d9
    //1 %    = 5
    memset(mPacketBuf, 0, sizeof(mPacketBuf));
    mPacketBuf[0] = init ? 0xaa : 0x55;
    mPacketBuf[1] = (mTXID >>  0) & 0xff;
    mPacketBuf[2] = (mTXID >>  8) & 0xff;
    mPacketBuf[3] = (mTXID >> 16) & 0xff;
    mPacketBuf[4] = (mTXID >> 24) & 0xff;
    for (u8 i = 0; i < 8; i++) {
        s32 value = (s32)getControlByOrder(i) * 0x1f1 / CHAN_MAX_VALUE + 0x5d9;
        if (value < 0)
            value = 0;
        mPacketBuf[5 + i * 2] = value & 0xff;
        mPacketBuf[6 + i * 2] = (value >> 8) & 0xff;
    }
    if ((getProtocolOpt() & PROTO_OPT_WLTOYS) == PROTO_OPT_WLTOYS) {
        if(getControlByOrder(4) > 0)
            mPacketBuf[12] |= 0x20;
        if(getControlByOrder(5) > 0)
            mPacketBuf[10] |= 0x40;
        if(getControlByOrder(6) > 0)
            mPacketBuf[10] |= 0x80;
        if(getControlByOrder(7) > 0)
            mPacketBuf[12] |= 0x10;
    }
}

u16 RFProtocolFlysky::callState(void)
{
    if (mBindCtr) {
        buildPacket(1);
        mCurRFChan = 1;
        mDev.writeData(mPacketBuf, MAX_PACKET_SIZE, mCurRFChan);
        mBindCtr--;
#ifdef BOGUS
        if (!mBindCtr)
            PROTOCOL_SetBindState(0);
#endif /* BOGUS */
    } else {
        buildPacket(0);
        mCurRFChan = pgm_read_byte(TBL_TX_CHANS + mCurRFChanRow * 16 + mCurRFChanCol) - mRFChanOffset;
        mDev.writeData(mPacketBuf, MAX_PACKET_SIZE, mCurRFChan);
        mCurRFChanCol = (mCurRFChanCol + 1) % 16;
        if (!mCurRFChanCol)                                 //Keep transmit power updated
            mDev.setRFPower(getRFPower());
        mPacketCtr++;
    }

    return 1460;
}

int RFProtocolFlysky::init(void)
{
    while (1) {
        mDev.reset();
        if (init1())
            break;
    }

    mTXID = getControllerID();
    mCurRFChanRow = mTXID % 16;
    mRFChanOffset = (mTXID & 0xff) / 16;
    mBindCtr      = MAX_BIND_COUNT;
    mPacketCtr    = 0;

    startState(INITIAL_WAIT_uS);

    return 0;
}

int RFProtocolFlysky::close(void)
{
    //printf(F("%08ld : %s\n"), millis(), __PRETTY_FUNCTION__);
    
    RFProtocol::close();
    mDev.initialize();
    return (mDev.reset() ? 1L : -1L);
}

int RFProtocolFlysky::reset(void)
{
    return close();
}

int RFProtocolFlysky::getInfo(s8 id, u8 *data)
{
    u8 size;

    size = RFProtocol::getInfo(id, data);
    if (size == 0) {
        switch (id) {
            case INFO_STATE:
                *data = (mBindCtr ? 1 : 0);
                size = 1;
                break;

            case INFO_CHANNEL:
                *data = mCurRFChan;
                size = 1;
                break;

            case INFO_PACKET_CTR:
                size = sizeof(mPacketCtr);
                *((u32*)data) = mPacketCtr;
                break;
        }
    }
    return size;
}

