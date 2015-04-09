#include <SPI.h>
#include "RFProtocolV2x2.h"
#include "utils.h"

#define MAX_BIND_COUNT    1000
#define PACKET_PERIOD_uS  4000
#define PACKET_CHKTIME_uS  100
#define INITIAL_WAIT_uS  50000

// Every second
#define BLINK_COUNT         250
// ~ every 0.25 sec
#define BLINK_COUNT_MIN     64
// ~ every 2 sec
#define BLINK_COUNT_MAX     512

enum {
    V2x2_FLAG_CAMERA = 0x01, // also automatic Missile Launcher and Hoist in one direction
    V2x2_FLAG_VIDEO  = 0x02, // also Sprayer, Bubbler, Missile Launcher(1), and Hoist in the other dir.
    V2x2_FLAG_FLIP   = 0x04,
    V2x2_FLAG_UNK9   = 0x08,
    V2x2_FLAG_LED    = 0x10,
    V2x2_FLAG_UNK10  = 0x20,
    V2x2_FLAG_BIND   = 0xC0
};

enum {
    V202_INIT2 = 0,
    V202_INIT2_NO_BIND,
    V202_BIND1,
    V202_BIND2,
    V202_DATA  = 0x10
};

#define PROTO_OPT_SKIP_BIND 0x01

enum {
    USEBLINK_NO  = 0,
    USEBLINK_YES = 1,
};

// Packet ack status values
enum {
    PKT_PENDING = 0,
    PKT_ACKED,
    PKT_TIMEOUT
};

u8 RFProtocolV2x2::getCheckSum(u8 *data)
{
    u8 sum = 0;
    for (u8 i = 0; i < MAX_PACKET_SIZE - 1;  ++i) 
        sum += data[i];
    return sum;
}

u8 RFProtocolV2x2::checkStatus()
{
    u8 stat = mDev.readReg(NRF24L01_07_STATUS);

    switch (stat & (BV(NRF24L01_07_TX_DS) | BV(NRF24L01_07_MAX_RT))) {
    case BV(NRF24L01_07_TX_DS):
        return PKT_ACKED;
    case BV(NRF24L01_07_MAX_RT):
        return PKT_TIMEOUT;
    }
    return PKT_PENDING;
}

u8 RFProtocolV2x2::getChannel(u8 id)
{
    s32 ch = RFProtocol::getControl(id);
    if (ch < CHAN_MIN_VALUE) {
        ch = CHAN_MIN_VALUE;
    } else if (ch > CHAN_MAX_VALUE) {
        ch = CHAN_MAX_VALUE;
    }

    u8 ret =  (u8) (((ch * 0xFF / CHAN_MAX_VALUE) + 0x100) >> 1);
    return ret;
}

void RFProtocolV2x2::getControls(u8* throttle, u8* rudder, u8* elevator, u8* aileron, u8* flags, u16 *led_blink)
{
    // Protocol is registered AETRG, that is
    // Aileron is channel 0, Elevator - 1, Throttle - 2, Rudder - 3
    // Sometimes due to imperfect calibration or mixer settings
    // throttle can be less than CHAN_MIN_VALUE or larger than
    // CHAN_MAX_VALUE. As we have no space here, we hard-limit
    // channels values by min..max range
    u8 a;

    *throttle = getChannel(CH_THROTTLE);    

    a         = getChannel(CH_RUDDER);
    *rudder   = a < 0x80 ? 0x7f - a : a;
    
    a         = getChannel(CH_ELEVATOR);
    *elevator = a < 0x80 ? 0x7f - a : a;

    a        = getChannel(CH_AILERON);
    *aileron = a < 0x80 ? 0x7f - a : a;


    // Channel 5
    // 512 - slow blinking (period 4ms*2*512 ~ 4sec), 64 - fast blinking (4ms*2*64 ~ 0.5sec)
    u16 nNewLedBlinkCtr;
    s32 ch = RFProtocol::getControl(CH_AUX1);
    if (ch == CHAN_MIN_VALUE) {
        nNewLedBlinkCtr = BLINK_COUNT_MAX + 1;
    } else if (ch == CHAN_MAX_VALUE) {
        nNewLedBlinkCtr = BLINK_COUNT_MIN - 1;
    } else {
        nNewLedBlinkCtr = (BLINK_COUNT_MAX+BLINK_COUNT_MIN)/2 -
            ((s32) RFProtocol::getControl(CH_AUX1) * (BLINK_COUNT_MAX-BLINK_COUNT_MIN) / (2*CHAN_MAX_VALUE));
    }
    if (*led_blink != nNewLedBlinkCtr) {
        if (mBindCtr > nNewLedBlinkCtr) 
            mBindCtr = nNewLedBlinkCtr;
        *led_blink = nNewLedBlinkCtr;
    }

    // Channel 6
    if (RFProtocol::getControl(CH_AUX2) <= 0)
        *flags &= ~V2x2_FLAG_FLIP;
    else
        *flags |= V2x2_FLAG_FLIP;

    // Channel 7
    if (RFProtocol::getControl(CH_AUX3) <= 0)
        *flags &= ~V2x2_FLAG_CAMERA;
    else
        *flags |= V2x2_FLAG_CAMERA;

    // Channel 8
    if (RFProtocol::getControl(CH_AUX4) <= 0)
        *flags &= ~V2x2_FLAG_VIDEO;
    else
        *flags |= V2x2_FLAG_VIDEO;
}

void RFProtocolV2x2::sendPacket(u8 bind)
{
    if (bind) {
        mAuxFlag      = V2x2_FLAG_BIND;
        mPacketBuf[0] = 0;
        mPacketBuf[1] = 0;
        mPacketBuf[2] = 0;
        mPacketBuf[3] = 0;
        mPacketBuf[4] = 0;
        mPacketBuf[5] = 0;
        mPacketBuf[6] = 0;
    } else {
        getControls(&mPacketBuf[0], &mPacketBuf[1], &mPacketBuf[2], &mPacketBuf[3], &mAuxFlag, &mLedBlinkCtr);
        // Trims, middle is 0x40
        mPacketBuf[4] = 0x40;   // yaw
        mPacketBuf[5] = 0x40;   // pitch
        mPacketBuf[6] = 0x40;   // roll
    }
        // TX id
    mPacketBuf[7] = mRxTxAddrBuf[0];
    mPacketBuf[8] = mRxTxAddrBuf[1];
    mPacketBuf[9] = mRxTxAddrBuf[2];
    // empty
    mPacketBuf[10] = 0x00;
    mPacketBuf[11] = 0x00;
    mPacketBuf[12] = 0x00;
    mPacketBuf[13] = 0x00;
    //
    mPacketBuf[14] = mAuxFlag;
    mPacketBuf[15] = getCheckSum(mPacketBuf);

    mPacketSent = 0;
    // Each packet is repeated twice on the same
    // channel, hence >> 1
    // We're not strictly repeating them, rather we
    // send new packet on the same frequency, so the
    // receiver gets the freshest command. As receiver
    // hops to a new frequency as soon as valid packet
    // received it does not matter that the packet is
    // not the same one repeated twice - nobody checks this
    u8 rf_ch = mRFChanBufs[mCurRFChan >> 1];
    mCurRFChan = (mCurRFChan + 1) & 0x1F;
    mDev.writeReg(NRF24L01_05_RF_CH, rf_ch);
    mDev.flushTx();
    mDev.writePayload(mPacketBuf, sizeof(mPacketBuf));
    ++mPacketCtr;
    mPacketSent = 1;

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

void RFProtocolV2x2::initRxTxAddr(void)
{
    u32 lfsr = getControllerID();

    // Pump zero bytes for LFSR to diverge more
    for (u8 i = 0; i < sizeof(lfsr); ++i)
      rand32_r(&lfsr, 0);

    setTxID(lfsr);
}

static const PROGMEM u8 RX_TX_ADDR[] = {0x66, 0x88, 0x68, 0x68, 0x68};
static const PROGMEM u8 RX_P1_ADDR[] = {0x88, 0x66, 0x86, 0x86, 0x86};
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
    MAX_PACKET_SIZE,                                // 11 : bytes of data payload for pipe 1
    MAX_PACKET_SIZE,                                // 12 :
    MAX_PACKET_SIZE,                                // 13 :
    MAX_PACKET_SIZE,                                // 14 :
    MAX_PACKET_SIZE,                                // 15 :
    MAX_PACKET_SIZE,                                // 16 :
    0x00                                            // 17 : Just in case, no real bits to write here
};

void RFProtocolV2x2::init1(void)
{
    u8 val;
    
    mDev.initialize();
    for (u8 i = 0; i < sizeof(TBL_INIT_REGS) ; i++) {
        if (i == NRF24L01_06_RF_SETUP) {
            mDev.setBitrate(NRF24L01_BR_1M);
            mDev.setRFPower(getRFPower());
        } else {
            val = pgm_read_byte(TBL_INIT_REGS + i);
            if (val != 0xff)
                mDev.writeReg(i, val);
        }
    }

    mDev.writeRegMulti_P(NRF24L01_0A_RX_ADDR_P0, RX_TX_ADDR, 5);
    mDev.writeRegMulti_P(NRF24L01_0B_RX_ADDR_P1, RX_P1_ADDR, 5);
    mDev.writeRegMulti_P(NRF24L01_10_TX_ADDR, RX_TX_ADDR, 5);
}

void RFProtocolV2x2::init2(void)
{
    mDev.flushTx();
    mDev.setTxRxMode(TX_EN);
    u8 config = BV(NRF24L01_00_EN_CRC) | BV(NRF24L01_00_CRCO) | BV(NRF24L01_00_PWR_UP);
    mDev.writeReg(NRF24L01_00_CONFIG, config);

    mCurRFChan    = 0;
    mPacketSent = 0;
}

// This is frequency hopping table for V202 protocol
// The table is the first 4 rows of 32 frequency hopping
// patterns, all other rows are derived from the first 4.
// For some reason the protocol avoids channels, dividing
// by 16 and replaces them by subtracting 3 from the channel
// number in this case.
// The pattern is defined by 5 least significant bits of
// sum of 3 bytes comprising TX id
static const PROGMEM u8 FREQ_HOPPING[][16] = {
 { 0x27, 0x1B, 0x39, 0x28, 0x24, 0x22, 0x2E, 0x36,
   0x19, 0x21, 0x29, 0x14, 0x1E, 0x12, 0x2D, 0x18 }, //  00
 { 0x2E, 0x33, 0x25, 0x38, 0x19, 0x12, 0x18, 0x16,
   0x2A, 0x1C, 0x1F, 0x37, 0x2F, 0x23, 0x34, 0x10 }, //  01
 { 0x11, 0x1A, 0x35, 0x24, 0x28, 0x18, 0x25, 0x2A,
   0x32, 0x2C, 0x14, 0x27, 0x36, 0x34, 0x1C, 0x17 }, //  02
 { 0x22, 0x27, 0x17, 0x39, 0x34, 0x28, 0x2B, 0x1D,
   0x18, 0x2A, 0x21, 0x38, 0x10, 0x26, 0x20, 0x1F }  //  03
};

void RFProtocolV2x2::setTxID(u32 id)
{
    u8 sum;
    mRxTxAddrBuf[0] = (id >> 16) & 0xFF;
    mRxTxAddrBuf[1] = (id >> 8) & 0xFF;
    mRxTxAddrBuf[2] = (id >> 0) & 0xFF;
    sum = mRxTxAddrBuf[0] + mRxTxAddrBuf[1] + mRxTxAddrBuf[2];

    
    const u8 *fh_row = FREQ_HOPPING[sum & 0x03];        // Base row is defined by lowest 2 bits
    u8 increment = (sum & 0x1e) >> 2;                   // Higher 3 bits define increment to corresponding row
    
    for (u8 i = 0; i < 16; ++i) {
        u8 val = pgm_read_byte(fh_row[i] + increment);
        mRFChanBufs[i] = (val & 0x0f) ? val : val - 3;  // Strange avoidance of channels divisible by 16
    }
}

u16 RFProtocolV2x2::callState(void)
{
    switch (mState) {
    case V202_INIT2:
        init2();
        mState = V202_BIND2;
        return 1;

    case V202_INIT2_NO_BIND:
        init2();
        mState   = V202_DATA;
        return 1;

    case V202_BIND1:
        sendPacket(1);
        if (getChannel(CH_THROTTLE) >= 240) 
            mState = V202_BIND2;
        break;

    case V202_BIND2:
        if (mPacketSent && checkStatus() != PKT_ACKED) {
            return PACKET_CHKTIME_uS;
        }
        sendPacket(1);
        if (--mBindCtr == 0) {
            mState = V202_DATA;
            mBindCtr = mLedBlinkCtr;
            mAuxFlag = 0;
        }
        break;

    case V202_DATA:
        if (mLedBlinkCtr > BLINK_COUNT_MAX) {
            mAuxFlag |= V2x2_FLAG_LED;
        } else if (mLedBlinkCtr < BLINK_COUNT_MIN) {
            mAuxFlag &= ~V2x2_FLAG_LED;
        } else if (--mBindCtr == 0) {
            mBindCtr = mLedBlinkCtr;
            mAuxFlag ^= V2x2_FLAG_LED;
        }
        if (mPacketSent && checkStatus() != PKT_ACKED) {
            return PACKET_CHKTIME_uS;
        }
        sendPacket(0);
        break;
    }
    return PACKET_PERIOD_uS;
}

int RFProtocolV2x2::init(void)
{
    mPacketCtr = 0;
    mAuxFlag   = 0;
    mLedBlinkCtr = BLINK_COUNT_MAX;

    init1();
    if (getProtocolOpt() == PROTO_OPT_SKIP_BIND) {
        mState   = V202_INIT2_NO_BIND;
        mBindCtr = BLINK_COUNT;
    } else {
        mState   = V202_INIT2;
        mBindCtr = MAX_BIND_COUNT;
    }
    initRxTxAddr();
    startState(INITIAL_WAIT_uS);

    return 0;
}

int RFProtocolV2x2::close(void)
{
    RFProtocol::close();
    mDev.initialize();
    return (mDev.reset() ? 1L : -1L);
}

int RFProtocolV2x2::reset(void)
{
    return close();
}

int RFProtocolV2x2::getInfo(s8 id, u8 *data)
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

