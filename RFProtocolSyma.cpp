#include <SPI.h>
#include "RFProtocolSyma.h"
#include "utils.h"

#define PAYLOADSIZE         10  // receive data pipes set to this size, but unused
#define MAX_BIND_COUNT      345

#define PACKET_PERIOD_uS    4000
#define INITIAL_WAIT_uS     500
#define FIRST_PACKET_uS     12000

#define FLAG_FLIP           0x01
#define FLAG_VIDEO          0x02
#define FLAG_PICTURE        0x04

#define PROTO_OPT_X5C_X2    0x01

enum {
    SYMAX_INIT1 = 0,
    SYMAX_BIND2,
    SYMAX_BIND3,
    SYMAX_DATA  = 0x10
};


u8 RFProtocolSyma::getCheckSum(u8 *data)
{
    u8 sum = data[0];

    for (int i=1; i < mPacketSize-1; i++)
        if (getProtocolOpt() == PROTO_OPT_X5C_X2)
            sum += data[i];
        else
            sum ^= data[i];

    return sum + ((getProtocolOpt() == PROTO_OPT_X5C_X2) ? 0 : 0x55);
}

u8 RFProtocolSyma::checkStatus()
{
    u8 stat = mDev.readReg(NRF24L01_07_STATUS);

    return stat;
}


#define BABS(X) (((X) < 0) ? -(u8)(X) : (X))
u8 RFProtocolSyma::getChannel(u8 id)
{
    s32 ch = RFProtocol::getControl(id);
    if (ch < CHAN_MIN_VALUE) {
        ch = CHAN_MIN_VALUE;
    } else if (ch > CHAN_MAX_VALUE) {
        ch = CHAN_MAX_VALUE;
    }

    u8 ret = (u8) ((ch < 0 ? 0x80 : 0) | BABS(ch * 127 / CHAN_MAX_VALUE));
    return ret;
}

void RFProtocolSyma::getControls(u8* throttle, u8* rudder, u8* elevator, u8* aileron, u8* flags)
{
    // Protocol is registered AETRF, that is
    // Aileron is channel 1, Elevator - 2, Throttle - 3, Rudder - 4, Flip control - 5

    *aileron  = getChannel(CH_AILERON);
    *elevator = getChannel(CH_ELEVATOR);
    *throttle = getChannel(CH_THROTTLE);
    *throttle = (*throttle & 0x80) ? (0xff - *throttle) : (0x80 + *throttle);
    *rudder   = getChannel(CH_RUDDER);

    // Channel 5
    if (RFProtocol::getControl(CH_AUX1) <= 0)
        *flags &= ~FLAG_FLIP;
    else
        *flags |= FLAG_FLIP;

    // Channel 6
    if (RFProtocol::getControl(CH_AUX2) <= 0)
        *flags &= ~FLAG_PICTURE;
    else
        *flags |= FLAG_PICTURE;

    // Channel 7
    if (RFProtocol::getControl(CH_AUX3) <= 0)
        *flags &= ~FLAG_VIDEO;
    else
        *flags |= FLAG_VIDEO;
}

#define X5C_CHAN2TRIM(X) ((((X) & 0x80 ? 0xff - (X) : 0x80 + (X)) >> 2) + 0x20)
void RFProtocolSyma::buildPacketX5C(u8 bind)
{
    u8 flag;
    
    if (bind) {
        memset(mPacketBuf, 0, mPacketSize);
        mPacketBuf[7] = 0xae;
        mPacketBuf[8] = 0xa9;
        mPacketBuf[14] = 0xc0;
        mPacketBuf[15] = 0x17;
    } else {
        getControls(&mPacketBuf[0], &mPacketBuf[1], &mPacketBuf[2], &mPacketBuf[3], &flag);
        mPacketBuf[2] ^= 0x80;  // reversed from default
        mPacketBuf[4] = X5C_CHAN2TRIM(getChannel(CH_RUDDER) ^ 0x80);     // drive trims for extra control range
        mPacketBuf[5] = X5C_CHAN2TRIM(getChannel(CH_ELEVATOR));
        mPacketBuf[6] = X5C_CHAN2TRIM(getChannel(CH_AILERON) ^ 0x80);
        mPacketBuf[7] = 0xae;
        mPacketBuf[8] = 0xa9;
        mPacketBuf[9] = 0x00;
        mPacketBuf[10] = 0x00;
        mPacketBuf[11] = 0x00;
        mPacketBuf[12] = 0x00;
        mPacketBuf[13] = 0x00;
        mPacketBuf[14] = ((flag & FLAG_VIDEO)   ? 0x10 : 0x00)
                       | ((flag & FLAG_PICTURE) ? 0x08 : 0x00)
                       | ((flag & FLAG_FLIP)    ? 0x01 : 0x00)
                       | 0x04;  // always high rates (bit 3 is rate control)
        mPacketBuf[15] = getCheckSum(mPacketBuf);
    }
}

void RFProtocolSyma::buildPacket(u8 bind)
{
    u8 flag;
    
    if (bind) {
        mPacketBuf[0] = mRxTxAddrBuf[4];
        mPacketBuf[1] = mRxTxAddrBuf[3];
        mPacketBuf[2] = mRxTxAddrBuf[2];
        mPacketBuf[3] = mRxTxAddrBuf[1];
        mPacketBuf[4] = mRxTxAddrBuf[0];
        mPacketBuf[5] = 0xaa;
        mPacketBuf[6] = 0xaa;
        mPacketBuf[7] = 0xaa;
        mPacketBuf[8] = 0x00;
    } else {
        getControls(&mPacketBuf[0], &mPacketBuf[2], &mPacketBuf[1], &mPacketBuf[3], &flag);
        mPacketBuf[4] = ((flag & FLAG_VIDEO)   ? 0x80 : 0x00)
                      | ((flag & FLAG_PICTURE) ? 0x40 : 0x00);
        // use trims to extend controls
        mPacketBuf[5] = (getChannel(CH_ELEVATOR) >> 2) | 0xc0; // always high rates (bit 7 is rate control)
        mPacketBuf[6] = (getChannel(CH_RUDDER) >> 2)   | ((flag & FLAG_FLIP) ? 0x40 : 0x00);
        mPacketBuf[7] = getChannel(CH_AILERON) >> 2;
        mPacketBuf[8] = 0x00;
    }
    mPacketBuf[9] = getCheckSum(mPacketBuf);

}

void RFProtocolSyma::sendPacket(u8 bind)
{
    if (getProtocolOpt() == PROTO_OPT_X5C_X2)
      buildPacketX5C(bind);
    else
      buildPacket(bind);

    // clear mPacketBuf status bits and TX FIFO
    mDev.writeReg(NRF24L01_07_STATUS, 0x70);
    mDev.writeReg(NRF24L01_00_CONFIG, 0x2e);
    mDev.writeReg(NRF24L01_05_RF_CH, mRFChanBufs[mCurRFChan]);
    mDev.flushTx();
    mDev.writePayload(mPacketBuf, mPacketSize);


    if (mPacketCtr++ % 2) {   // use each channel twice
        mCurRFChan = (mCurRFChan + 1) % mRFChanCnt;
    }

//    printf(F("SEND PACKET bind:%d :%d\n"), bind, mPacketCtr);

//    radio.ce(HIGH);
//    delayMicroseconds(15);
    // It saves power to turn off radio after the transmission,
    // so as long as we have pins to do so, it is wise to turn
    // it back.
//    radio.ce(LOW);


    // Check and adjust transmission power. We do this after
    // transmission to not bother with timeout after power
    // settings change -  we have plenty of time until next
    // packet
    if (isRFPowerUpdated()) {
        mDev.setRFPower(getRFPower());
        clearRFPowerUpdated();
    }
}


void RFProtocolSyma::initRxTxAddr(void)
{
    u32 lfsr = getControllerID();

    // Pump zero bytes for LFSR to diverge more
    for (u8 i = 0; i < sizeof(lfsr); ++i)
      rand32_r(&lfsr, 0);

    mRxTxAddrBuf[4] = 0xa2;
    for (u8 i = 0; i < sizeof(mRxTxAddrBuf)-1; ++i) {
        mRxTxAddrBuf[i] = lfsr & 0xff;
        rand32_r(&lfsr, i);
    }
}

static const PROGMEM u8 BIND_RX_TX_ADDR[] = {0xab,0xac,0xad,0xae,0xaf};
static const PROGMEM u8 RX_TX_ADDR_X5C[]  = {0x6d,0x6a,0x73,0x73,0x73};    // X5C uses same address for bind and data
static const PROGMEM u8 TBL_INIT_REGS[] = {
    BV(NRF24L01_00_EN_CRC) | BV(NRF24L01_00_CRCO),  // 00 : 2-bytes CRC, radio off
    0x00,                                           // 01 : No Auto Acknoledgement
    0x3F,                                           // 02 : Enable all data pipes (even though not used?)
    0x03,                                           // 03 : 5-byte RX/TX address
    0xee,                                           // 04 : 3.75mS retransmit t/o, 14 tries (retries w/o AA?)
    0x08,                                           // 05 : RF channel : 8
    0xff,                                           // 06 : skip
    0x70,                                           // 07 : Clear data ready, data sent, and retransmit
    0xff,                                           // 08 : skip
    0xff,                                           // 09 : skip
    0xff,                                           // 0A : skip
    0xff,                                           // 0B : skip
    0xC3,                                           // 0C : LSB byte of pipe 2 receive address
    0xC4,                                           // 0D :
    0xC5,                                           // 0E :
    0xC6,                                           // 0F :
    0xff,                                           // 10 : skip
    PAYLOADSIZE,                                    // 11 : bytes of data payload for pipe 1
    PAYLOADSIZE,                                    // 12 :
    PAYLOADSIZE,                                    // 13 :
    PAYLOADSIZE,                                    // 14 :
    PAYLOADSIZE,                                    // 15 :
    PAYLOADSIZE,                                    // 16 :
    0x00                                            // 17 : Just in case, no real bits to write here
};

void RFProtocolSyma::init1(void)
{
    u8 val;
    u8 bitrate;
    
    mDev.initialize();
    mDev.setTxRxMode(TX_EN);
    mDev.readReg(NRF24L01_07_STATUS);

    if (getProtocolOpt() == PROTO_OPT_X5C_X2) {
        bitrate = NRF24L01_BR_1M;
        mPacketSize = 16;
    } else {
        bitrate = NRF24L01_BR_250K;
        mPacketSize = 10;
    }

    for (u8 i = 0; i < sizeof(TBL_INIT_REGS) ; i++) {
        if (i == NRF24L01_06_RF_SETUP) {
            mDev.setBitrate(bitrate);
            mDev.setRFPower(getRFPower());
        } else {
            val = pgm_read_byte(TBL_INIT_REGS + i);
            if (val != 0xff)
                mDev.writeReg(i, val);
        }
    }
    mDev.writeRegMulti_P(NRF24L01_10_TX_ADDR,
                                (getProtocolOpt() == PROTO_OPT_X5C_X2) ? RX_TX_ADDR_X5C : BIND_RX_TX_ADDR,
                                5);
    mDev.readReg(NRF24L01_07_STATUS);

    mDev.flushTx();
    mDev.readReg(NRF24L01_07_STATUS);
    mDev.writeReg(NRF24L01_07_STATUS, 0x0e);
    mDev.readReg(NRF24L01_00_CONFIG);
    mDev.writeReg(NRF24L01_00_CONFIG, 0x0c);
    mDev.writeReg(NRF24L01_00_CONFIG, 0x0e);  // power on
}


// write a strange first mPacketBuf to RF channel 8 ...
static const PROGMEM u8 FIRST_PACKET[]   = { 0xf9, 0x96, 0x82, 0x1b, 0x20, 0x08, 0x08, 0xf2,
                                             0x7d, 0xef, 0xff, 0x00, 0x00, 0x00, 0x00 };
static const PROGMEM u8 CHANS_BIND[]     = { 0x4b, 0x30, 0x40, 0x2e };
static const PROGMEM u8 CHANS_BIND_X5C[] = { 0x27, 0x1b, 0x39, 0x28, 0x24, 0x22, 0x2e, 0x36,
                                             0x19, 0x21, 0x29, 0x14, 0x1e, 0x12, 0x2d, 0x18};

void RFProtocolSyma::init2(void)
{
    mDev.flushTx();
    mDev.writeReg(NRF24L01_05_RF_CH, 0x08);
    mDev.writePayload_P(FIRST_PACKET, sizeof(FIRST_PACKET));

    if (getProtocolOpt() == PROTO_OPT_X5C_X2) {
      mRFChanCnt = sizeof(CHANS_BIND_X5C);
      memcpy_P(mRFChanBufs, CHANS_BIND_X5C, mRFChanCnt);
    } else {
      initRxTxAddr();   // make info available for bind packets
      mRFChanCnt = sizeof(CHANS_BIND);
      memcpy_P(mRFChanBufs, CHANS_BIND, mRFChanCnt);
    }

    mCurRFChan   = 0;
    mPacketCtr = 0;
}

static const PROGMEM u8 CHANS_DATA_X5C[] = {0x1d, 0x2f, 0x26, 0x3d, 0x15, 0x2b, 0x25, 0x24,
                                            0x27, 0x2c, 0x1c, 0x3e, 0x39, 0x2d, 0x22};
void RFProtocolSyma::init3(void)
{
    if (getProtocolOpt() == PROTO_OPT_X5C_X2) {
      mRFChanCnt = sizeof(CHANS_DATA_X5C);
      memcpy_P(mRFChanBufs, CHANS_DATA_X5C, mRFChanCnt);
    } else {
      setRFChannel(mRxTxAddrBuf[0]);
      mDev.writeRegMulti(NRF24L01_10_TX_ADDR, mRxTxAddrBuf, 5);
    }
    mCurRFChan   = 0;
    mPacketCtr = 0;
}

static const PROGMEM u8 START_CHANS_1[] = {0x0a, 0x1a, 0x2a, 0x3a};
static const PROGMEM u8 START_CHANS_2[] = {0x2a, 0x0a, 0x42, 0x22};
static const PROGMEM u8 START_CHANS_3[] = {0x1a, 0x3a, 0x12, 0x32};

// channels determined by last byte of tx address
void RFProtocolSyma::setRFChannel(u8 address)
{
    u8  laddress = address & 0x1f;
    u8  i;
    u32 *pchans = (u32 *)mRFChanBufs;   // avoid compiler warning

    mRFChanCnt = 4;
    if (laddress < 0x10) {
        if (laddress == 6)
            laddress = 7;
        for(i=0; i < mRFChanCnt; i++) {
            mRFChanBufs[i] = pgm_read_byte(START_CHANS_1 + i) + laddress;
        }
    } else if (laddress < 0x18) {
        for(i=0; i < mRFChanCnt; i++) {
            mRFChanBufs[i] = pgm_read_byte(START_CHANS_2 + i) + (laddress & 0x07);
        }
        if (laddress == 0x16) {
            mRFChanBufs[0] += 1;
            mRFChanBufs[1] += 1;
        }
    } else if (laddress < 0x1e) {
        for(i=0; i < mRFChanCnt; i++) {
            mRFChanBufs[i] = pgm_read_byte(START_CHANS_3 + i) + (laddress & 0x07);
        }
    } else if (laddress == 0x1e) {
        *pchans = 0x38184121;
    } else {
        *pchans = 0x39194121;
    }
}

u16 RFProtocolSyma::callState(void)
{
    switch (mState) {
    case SYMAX_INIT1:
        init2();
        mState = SYMAX_BIND2;
        return FIRST_PACKET_uS;
        break;

    case SYMAX_BIND2:
        mBindCtr = MAX_BIND_COUNT;
        mState   = SYMAX_BIND3;
        sendPacket(1);
        break;

    case SYMAX_BIND3:
        if (mBindCtr == 0) {
            init3();
            mState = SYMAX_DATA;
        } else {
            sendPacket(1);
            mBindCtr--;
        }
        break;

    case SYMAX_DATA:
        sendPacket(0);
        break;
    }
    return PACKET_PERIOD_uS;
}

int RFProtocolSyma::init(void)
{
    mPacketCtr = 0;

    init1();
    mState = SYMAX_INIT1;

    startState(INITIAL_WAIT_uS);

    return 0;
}

int RFProtocolSyma::close(void)
{
    RFProtocol::close();
    //printf(F("%08ld : %s\n"), millis(), __PRETTY_FUNCTION__);
    mDev.initialize();
    return (mDev.reset() ? 1L : -1L);
}

int RFProtocolSyma::reset(void)
{
    return close();
}

int RFProtocolSyma::getInfo(s8 id, u8 *data)
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
                *data = mRFChanBufs[mCurRFChan];
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

