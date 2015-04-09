#include <SPI.h>
#include "RFProtocolHiSky.h"
#include "utils.h"

#define MAX_BIND_COUNT     800

#define PACKET_PERIOD_uS     1000
#define INITIAL_WAIT_uS      1000
enum {
    HISKY_INIT = 0,
    HISKY_DATA = 0x10
};


void RFProtocolHiSky::buildRFChannels(u32 seed)
{
    int idx = 0;
    u32 rnd = seed;
    while (idx < MAX_RF_CHANNELS) {
        int i;
        int count_2_26 = 0, count_27_50 = 0, count_51_74 = 0;
        rnd = rnd * 0x0019660D + 0x3C6EF35F; // Randomization

        // Use least-significant byte. 73 is prime, so channels 76..77 are unused
        u8 next_ch = ((rnd >> 8) % 73) + 2;
        // Keep the distance 2 between the channels - either odd or even
        if (((next_ch ^ seed) & 0x01 )== 0)
            continue;
        // Check that it's not duplicate and spread uniformly
        for (i = 0; i < idx; i++) {
            if(mRFChanBufs[i] == next_ch)
                break;
            if(mRFChanBufs[i] <= 26)
                count_2_26++;
            else if (mRFChanBufs[i] <= 50)
                count_27_50++;
            else
                count_51_74++;
        }
        if (i != idx)
            continue;
        if ((next_ch <= 26 && count_2_26 < 8)
          ||(next_ch >= 27 && next_ch <= 50 && count_27_50 < 8)
          ||(next_ch >= 51 && count_51_74 < 8))
        {
            mRFChanBufs[idx++] = next_ch;
        }
    }
}

void RFProtocolHiSky::buildBindingPacket(void)
{
    u8 i;
    unsigned int  sum;
    u8 sum_l,sum_h;

    mCtr1ms  = 0;
    mCurRFChan = 0;

    sum = 0;
    for (i= 0; i < 5; i++)
        sum += mRxTxAddrBuf[i];
    sum_l = (u8)sum;
    sum >>= 8;
    sum_h = (u8)sum;
    mBindingBufs[0][0] = 0xff;
    mBindingBufs[0][1] = 0xaa;
    mBindingBufs[0][2] = 0x55;
    for (i = 3; i < 8; i++)
        mBindingBufs[0][i] = mRxTxAddrBuf[i-3];

    for(i = 1; i < 4; i++) {
        mBindingBufs[i][0] = sum_l;
        mBindingBufs[i][1] = sum_h;
        mBindingBufs[i][2] = i-1;
    }
    
    for(i = 0; i < 7; i++)
        mBindingBufs[1][i+3] = mRFChanBufs[i];
    for(i = 0; i < 7; i++)
        mBindingBufs[2][i+3] = mRFChanBufs[i+7];
    for(i = 0; i < 6; i++)
        mBindingBufs[3][i+3] = mRFChanBufs[i+14];

    mBindingIdx = 0;
}

u16 RFProtocolHiSky::getChannel(u8 id)
{
    s32 ch = RFProtocol::getControl(id);
    if (ch < CHAN_MIN_VALUE) {
        ch = CHAN_MIN_VALUE;
    } else if (ch > CHAN_MAX_VALUE) {
        ch = CHAN_MAX_VALUE;
    }

    u16 ret = (s32)RFProtocol::getControl(id) * 450 / CHAN_MAX_VALUE + 500; // max/min servo range is +-125%
    if (id == CH_THROTTLE)
        ret = 1000 - ret;

    if (ret < 0)
        ret = 0;
    else if (ret > 1000)
        ret = 1000;
    
    return ret;
}

// HiSky channel sequence: AILE  ELEV  THRO  RUDD  GEAR  PITH, channel data value is from 0 to 1000
void RFProtocolHiSky::buildDataPacket(void)
{
    u8  i;
    u16 ch;

    mPacketBuf[8] = 0;
    mPacketBuf[9] = 0;
    
    ch = getChannel(CH_AILERON);
    mPacketBuf[0]  = (u8)ch;
    mPacketBuf[8] |= (u8)((ch >> 8) & 0x0003);
    
    ch = getChannel(CH_ELEVATOR);
    mPacketBuf[1]  = (u8)ch;
    mPacketBuf[8] |= (u8)((ch >> 6) & 0x000c);
    
    ch = getChannel(CH_THROTTLE);
    mPacketBuf[2]  = (u8)ch;
    mPacketBuf[8] |= (u8)((ch >> 4) & 0x0030);
    
    ch = getChannel(CH_RUDDER);
    mPacketBuf[3] = (u8)ch;
    mPacketBuf[8] |= (u8)((ch >> 2) & 0x00c0);
    
    for (i = 7; i >= 4; i--) {
        ch = getChannel(i);
        mPacketBuf[i]  = (u8)ch;
        mPacketBuf[9] |= (u8)((ch >> 2) & 0x0003);
        mPacketBuf[9] <<= 2;
    }
    mPacketCtr++;
}

void RFProtocolHiSky::initRxTxAddr(void)
{
    u32 lfsr = getControllerID();

    // Pump zero bytes for LFSR to diverge more
    for (u8 i = 0; i < ADDR_BUF_SIZE; ++i)
      rand32_r(&lfsr, 0);

    for (u8 i = 0; i < ADDR_BUF_SIZE; ++i) {
        mRxTxAddrBuf[i] = lfsr & 0xff;
        rand32_r(&lfsr, i);
    }

    // Use LFSR to seed frequency hopping sequence after another
    // divergence round
    for (u8 i = 0; i < sizeof(lfsr); ++i) 
        rand32_r(&lfsr, 0);
    buildRFChannels(lfsr);    

}

void RFProtocolHiSky::init1(void)
{
    mDev.initialize();
    mDev.writeReg(NRF24L01_02_EN_RXADDR, 0x01);                         // Enable p0 rx
    mDev.writeReg(NRF24L01_01_EN_AA, 0x00);                             // No Auto Acknoledgement
    mDev.writeRegMulti(NRF24L01_10_TX_ADDR, mRxTxAddrBuf, ADDR_BUF_SIZE);
    mDev.writeRegMulti(NRF24L01_0A_RX_ADDR_P0, mRxTxAddrBuf, ADDR_BUF_SIZE);
    mDev.writeReg(NRF24L01_11_RX_PW_P0, MAX_PACKET_SIZE);               // payload size = 10
    mDev.writeReg(NRF24L01_05_RF_CH, 81);                               // binding packet must be set in channel 81

    // 2-bytes CRC, radio off
    mDev.setTxRxMode(TX_EN);
    mDev.writeReg(NRF24L01_00_CONFIG,
            BV(NRF24L01_00_EN_CRC) | BV(NRF24L01_00_CRCO) | BV(NRF24L01_00_PWR_UP));
    mDev.writeReg(NRF24L01_03_SETUP_AW, 0x03);                          // 5-byte RX/TX address (byte -2)
    mDev.setBitrate(NRF24L01_BR_1M);                                    // 1Mbps
    mDev.setRFPower(getRFPower());
    mDev.writeReg(NRF24L01_07_STATUS, 0x70);                            // Clear data ready, data sent, and retransmit

}


static const PROGMEM u8 BINDING_ADDR[5] = { 0x12,0x23,0x23,0x45,0x78 };        // fixed binding ids for all planes

u16 RFProtocolHiSky::callState(void)
{
    mCtr1ms++;

    switch (mCtr1ms) {
    case 1:
        mDev.flushTx();
        break;
        
    case 2:
        if (mBindCtr > 0) {
            mDev.writeRegMulti_P(NRF24L01_10_TX_ADDR, BINDING_ADDR, 5);
            mDev.writeReg(NRF24L01_05_RF_CH, 81);
        }
        break;
        
    case 3:
        if (mBindCtr > 0) {
            mBindCtr--;
            if (!mBindCtr) {    // binding finished, change tx add
                mState = HISKY_DATA;
            }
            mDev.writePayload(mBindingBufs[mBindingIdx], MAX_PACKET_SIZE);
            mBindingIdx++;
            if (mBindingIdx >= 4)
                mBindingIdx = 0;
        }
        break;

    case 4:        
        if (mBindCtr > 0)
            mDev.flushTx();
        break;
        
    case 5:
        mDev.setRFPower(getRFPower());
        break;
        
    case 6:
        mDev.writeRegMulti(NRF24L01_10_TX_ADDR, mRxTxAddrBuf, 5);
        mDev.writeReg(NRF24L01_05_RF_CH, mRFChanBufs[mCurRFChan]);
        mCurRFChan++;
        if (mCurRFChan >= MAX_RF_CHANNELS)
            mCurRFChan = 0;
        break;

    case 7:
        buildDataPacket();
        break;

    case 8: // none
        break;
        
    default:
        mCtr1ms = 0;
        mDev.writePayload(mPacketBuf, MAX_PACKET_SIZE);
        break;
    }

    return PACKET_PERIOD_uS;    // send 1 binding packet and 1 data packet per 9ms
}

int RFProtocolHiSky::init(void)
{
    mState     = HISKY_INIT;
    mPacketCtr = 0;

    initRxTxAddr();
    buildBindingPacket();
    init1();

    if (getProtocolOpt()) {
        mBindCtr = 0;
    } else {
        mBindCtr = MAX_BIND_COUNT;
    }

    startState(INITIAL_WAIT_uS);

    return 0;
}

int RFProtocolHiSky::close(void)
{
    RFProtocol::close();
    mDev.initialize();
    return (mDev.reset() ? 1L : -1L);
}

int RFProtocolHiSky::reset(void)
{
    return close();
}

int RFProtocolHiSky::getInfo(s8 id, u8 *data)
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

