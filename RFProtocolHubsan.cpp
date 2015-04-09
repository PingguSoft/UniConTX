#include <SPI.h>
#include "RFProtocolHubsan.h"
#include "utils.h"

#define PROTO_OPT_TELEMETRY     0x80
#define PROTO_OPT_VTX(f)        (5645 + ((f) & 0x7f) * 5)

#define WAIT_WRITE              0x80

enum {
    FLAG_VIDEO= 0x01,   // record video
    FLAG_FLIP = 0x08,   // enable flips
    FLAG_LED  = 0x04    // enable LEDs
};

enum {
    BIND_1,
    BIND_2,
    BIND_3,
    BIND_4,
    BIND_5,
    BIND_6,
    BIND_7,
    BIND_8,
    DATA_1,
    DATA_2,
    DATA_3,
    DATA_4,
    DATA_5,
};

static const PROGMEM u8 TBL_INIT_REGS[] = {
     A7105_01_MODE_CONTROL,     0x63,
     A7105_03_FIFOI,            0x0f,
     A7105_0D_CLOCK,            0x05,
     A7105_0E_DATA_RATE,        0x04,
     A7105_15_TX_II,            0x2b,
     A7105_18_RX,               0x62,
     A7105_19_RX_GAIN_I,        0x80,
     A7105_1C_RX_GAIN_IV,       0x0A,
     A7105_1F_CODE_I,           0x07,
     A7105_20_CODE_II,          0x17,
     A7105_29_RX_DEM_TEST_I,    0x47
};

int RFProtocolHubsan::init1(void)
{
    u8 if_calibration1;
    u8 vco_calibration0;
    u8 vco_calibration1;
    //u8 vco_current;

    mDev.writeID(0x55201041);

    u8 reg, val;
    for (u8 i = 0; i < sizeof(TBL_INIT_REGS) / 2; i++) {
        reg = pgm_read_byte(TBL_INIT_REGS + i * 2);
        val = pgm_read_byte(TBL_INIT_REGS + i * 2 + 1);
        mDev.writeReg(reg, val);
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
    //mDev.writeReg(0x24, 0x13); //Recomended calibration from A7105 Datasheet

    //VCO Bank Calibration
    //mDev.writeReg(0x26, 0x3b); //Recomended limits from A7105 Datasheet

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
    //mDev.writeReg(0x25, 0x08);
    mDev.setTxRxMode(TX_EN);
    mDev.setRFPower(getRFPower());
    mDev.strobe(A7105_STANDBY);

    return 1;
}

void RFProtocolHubsan::updateCRC(void)
{
    int sum = 0;
    for(int i = 0; i < MAX_PACKET_SIZE - 1; i++)
        sum += mPacketBuf[i];
    mPacketBuf[MAX_PACKET_SIZE - 1] = (256 - (sum % 256)) & 0xff;
}

void RFProtocolHubsan::buildBindPacket(u8 state)
{
    mPacketBuf[0] = state;
    mPacketBuf[1] = mCurRFChan;
    mPacketBuf[2] = (mSessionID >> 24) & 0xff;
    mPacketBuf[3] = (mSessionID >> 16) & 0xff;
    mPacketBuf[4] = (mSessionID >>  8) & 0xff;
    mPacketBuf[5] = (mSessionID >>  0) & 0xff;
    mPacketBuf[6] = 0x08;
    mPacketBuf[7] = 0xe4; //???
    mPacketBuf[8] = 0xea;
    mPacketBuf[9] = 0x9e;
    mPacketBuf[10] = 0x50;
    mPacketBuf[11] = (mTXID >> 24) & 0xff;
    mPacketBuf[12] = (mTXID >> 16) & 0xff;
    mPacketBuf[13] = (mTXID >>  8) & 0xff;
    mPacketBuf[14] = (mTXID >>  0) & 0xff;
    updateCRC();
}

s16 RFProtocolHubsan::getChannel(u8 ch, s32 scale, s32 center, s32 range)
{
    s32 value = (s32)RFProtocol::getControl(ch) * scale / CHAN_MAX_VALUE + center;

    if (value < center - range)
        value = center - range;
    if (value >= center + range)
        value = center + range -1;

    return value;
}

void RFProtocolHubsan::buildPacket(void)
{
    static s16 vtx_freq = 0;
    
    memset(mPacketBuf, 0, MAX_PACKET_SIZE);

    // set vTX frequency (H107D)
    if(vtx_freq != PROTO_OPT_VTX(getProtocolOpt()) || mPacketCtr == 100) {
        vtx_freq = PROTO_OPT_VTX(getProtocolOpt());
        mPacketBuf[0] = 0x40;
        mPacketBuf[1] = (vtx_freq >> 8) & 0xff;
        mPacketBuf[2] = vtx_freq & 0xff;
        mPacketBuf[3] = 0x82;
        mPacketCtr++;      
    } else {                                                            //20 00 00 00 80 00 7d 00 84 02 64 db 04 26 79 7b
        mPacketBuf[0] = 0x20;
        mPacketBuf[2] = getChannel(CH_THROTTLE, 0x80, 0x80, 0x80);      //Throttle
    }
    mPacketBuf[4] = 0xff - getChannel(CH_RUDDER, 0x80, 0x80, 0x80);     //Rudder is reversed
    mPacketBuf[6] = 0xff - getChannel(CH_ELEVATOR, 0x80, 0x80, 0x80);   //Elevator is reversed
    mPacketBuf[8] = getChannel(CH_AILERON, 0x80, 0x80, 0x80);           //Aileron 

    if(mPacketCtr < 100) {
        mPacketBuf[9] = 0x02 | FLAG_LED | FLAG_FLIP;                    // sends default value for the 100 first packets
        mPacketCtr++;
    } else {
        mPacketBuf[9] = 0x02;

        if(getControl(CH_AUX1) >= 0)
            mPacketBuf[9] |= FLAG_LED;

        if(getControl(CH_AUX2) >= 0)
            mPacketBuf[9] |= FLAG_FLIP;

        if(getControl(CH_AUX3) > 0)                                     // off by default
            mPacketBuf[9] |= FLAG_VIDEO;
    }
    mPacketBuf[10] = 0x64;
    mPacketBuf[11] = (mTXID >> 24) & 0xff;
    mPacketBuf[12] = (mTXID >> 16) & 0xff;
    mPacketBuf[13] = (mTXID >>  8) & 0xff;
    mPacketBuf[14] = (mTXID >>  0) & 0xff;
    updateCRC();
}

u8 RFProtocolHubsan::checkIntegrity(void) 
{
    int sum = 0;
    for(int i = 0; i < MAX_PACKET_SIZE - 1; i++)
        sum += mPacketBuf[i];
    return mPacketBuf[MAX_PACKET_SIZE - 1] == ((256 - (sum % 256)) & 0xff);
}

u16 RFProtocolHubsan::callState(void)
{
    static u16 delay = 0;
    static u8  txState = 0;
    static u8  rfMode=0;
    
    u8 i;
    
    switch (mState) {
        case BIND_1:
        case BIND_3:
        case BIND_5:
        case BIND_7:
            buildBindPacket(mState == BIND_7 ? 9 : (mState == BIND_5 ? 1 : mState + 1 - BIND_1));
            mDev.strobe(A7105_STANDBY);
            mDev.writeData(mPacketBuf, 16, mCurRFChan);
            mState |= WAIT_WRITE;
            return 3000;
            
        case BIND_1 | WAIT_WRITE:
        case BIND_3 | WAIT_WRITE:
        case BIND_5 | WAIT_WRITE:
        case BIND_7 | WAIT_WRITE:
            //wait for completion
            for (i = 0; i < 20; i++) {
               if ((mDev.readReg(A7105_00_MODE) & 0x01))
                   break;
            }
            //if (i == 20)
            //    printf("Failed to complete write\n");
            mDev.setTxRxMode(RX_EN);
            mDev.strobe(A7105_RX);
            mState &= ~WAIT_WRITE;
            mState++;
            return 4500;                            //7.5msec elapsed since last write
            
        case BIND_2:
        case BIND_4:
        case BIND_6:
            mDev.setTxRxMode(TX_EN);
            if(mDev.readReg(A7105_00_MODE) & 0x01) {
                mState = BIND_1;
                return 4500;                        //No signal, restart binding procedure.  12msec elapsed since last write
            }
            
            mDev.readData(mPacketBuf, MAX_PACKET_SIZE);
            mState++;
            if (mState == BIND_5) {
                u32 id = ((u32)mPacketBuf[2] << 24) | ((u32)mPacketBuf[3] << 16) | ((u32)mPacketBuf[4] << 8) | mPacketBuf[5];
                mDev.writeID(id);
            }
            
            return 500;                             //8msec elapsed time since last write;

        case BIND_8:
            mDev.setTxRxMode(TX_EN);
            if (mDev.readReg(A7105_00_MODE) & 0x01) {
                mState = BIND_7;
                return 15000;                       //22.5msec elapsed since last write
            }
            mDev.readData(mPacketBuf, MAX_PACKET_SIZE);
            if (mPacketBuf[1] == 9) {
                mState = DATA_1;
                mDev.writeReg(A7105_1F_CODE_I, 0x0F);
                return 28000;                       //35.5msec elapsed since last write
            } else {
                mState = BIND_7;
                return 15000;                       //22.5 msec elapsed since last write
            }
            break;

        case DATA_1:
        case DATA_2:
        case DATA_3:
        case DATA_4:
        case DATA_5:
            if (txState == 0) {                                         // send packet
                rfMode = A7105_TX;
                if (mState == DATA_1) {
                    mDev.setRFPower(getRFPower());                      //Keep transmit power in sync
                }
                buildPacket();
                mDev.strobe(A7105_STANDBY);
                mDev.writeData( mPacketBuf, 16, mState == DATA_5 ? mCurRFChan + 0x23 : mCurRFChan);
                if (mState == DATA_5)
                    mState = DATA_1;
                else
                    mState++;
                delay = 3000;
            } else {
                if (getProtocolOpt() & PROTO_OPT_TELEMETRY) {
                    if (rfMode == A7105_TX) {                           // switch to rx mode 3ms after mPacketBuf sent
                        for (i = 0; i < 10; i++) {
                            if (!(mDev.readReg(A7105_00_MODE) & 0x01)) {// wait for tx completion
                                mDev.setTxRxMode(RX_EN);
                                mDev.strobe(A7105_RX); 
                                rfMode = A7105_RX;
                                break;
                            }
                        }
                    }
                    if (rfMode == A7105_RX) {                           // check for telemetry frame
                        for (i = 0; i < 10; i++) {
                            if (!(mDev.readReg(A7105_00_MODE) & 0x01)) {// data received
                                mDev.readData(mPacketBuf, 16);
                                //updateTelemetry();
                                mDev.strobe(A7105_RX);
                                break;
                            }
                        }
                    }
                }
                delay = 1000;
            }
            
            if (++txState == 8) { // 3ms + 7 * 1ms = 10ms
                mDev.setTxRxMode(TX_EN);
                txState = 0;
            }
            return delay;
    }

    return delay;
}

static const PROGMEM u8 ALLOWED_CH[] = {0x14, 0x1e, 0x28, 0x32, 0x3c, 0x46, 0x50, 0x5a, 0x64, 0x6e, 0x78, 0x82};

int RFProtocolHubsan::init(void)
{
    mTXID = 0xdb042679; // getControllerID();

    while(1) {
        mDev.reset();
        if (init1())
            break;
    }

    mSessionID = getControllerID();
    mSessionID = rand32_r(&mSessionID, 0);
    mCurRFChan = pgm_read_byte(ALLOWED_CH + (mSessionID % sizeof(ALLOWED_CH)));
    mPacketCtr = 0;
    mState     = BIND_1;

    startState(INITIAL_WAIT_uS);

    return 0;
}

int RFProtocolHubsan::close(void)
{
    //printf(F("%08ld : %s\n"), millis(), __PRETTY_FUNCTION__);
    
    RFProtocol::close();
    mDev.initialize();
    return (mDev.reset() ? 1L : -1L);
}

int RFProtocolHubsan::reset(void)
{
    return close();
}

int RFProtocolHubsan::getInfo(s8 id, u8 *data)
{
    u8 size;

    size = RFProtocol::getInfo(id, data);
    if (size == 0) {
        switch (id) {
            case INFO_STATE:
                *data = mState;
                size = 1;
                break;

            case INFO_CHANNEL:
                *data = mCurRFChan;
                size = 1;
                break;

            case INFO_PACKET_CTR:
                size  = sizeof(mPacketCtr);
                *data = mPacketCtr;
                break;
        }
    }
    return size;
}

