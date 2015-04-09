#include <SPI.h>
#include "RFProtocolCFlie.h"
#include "utils.h"


// Packet ack status values
enum {
    PKT_PENDING = 0,
    PKT_ACKED,
    PKT_TIMEOUT
};

enum {
    CFLIE_INIT_SEARCH = 0,
    CFLIE_INIT_DATA,
    CFLIE_SEARCH,
    CFLIE_DATA = 0x10
};

u8 RFProtocolCFlie::checkStatus(void)
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

void RFProtocolCFlie::setRateAndCh(u8 rate, u8 channel)
{
    mDev.writeReg(NRF24L01_05_RF_CH, channel);  // Defined by model id
    mDev.setBitrate(rate);                      // Defined by model id
}

void RFProtocolCFlie::sendSearchPacket(void)
{
    u8 buf = 0xff;

    // clear packet status bits and TX FIFO
    mDev.writeReg(NRF24L01_07_STATUS, (BV(NRF24L01_07_TX_DS) | BV(NRF24L01_07_MAX_RT)));
    mDev.flushTx();

    if (mCurRFChan++ > 125) {
        mCurRFChan = 0;
        switch(mDataRate) {
        case NRF24L01_BR_250K:
            mDataRate = NRF24L01_BR_1M;
            break;
            
        case NRF24L01_BR_1M:
            mDataRate = NRF24L01_BR_2M;
            break;
            
        case NRF24L01_BR_2M:
            mDataRate = NRF24L01_BR_250K;
            break;
        }
    }
    setRateAndCh(mDataRate, mCurRFChan);
    mDev.writePayload(&buf, sizeof(buf));
    ++mPacketCtr;
}

// Frac 16.16
#define FRAC_MANTISSA 16
#define FRAC_SCALE ((s32)1 << FRAC_MANTISSA)

// Convert fractional 16.16 to float32
void RFProtocolCFlie::frac2float(s32 n, float* res)
{
    if (n == 0) {
        *res = 0.0;
        return;
    }
    u32 m = n < 0 ? -n : n;
    int i;
    for (i = (31-FRAC_MANTISSA); (m & 0x80000000) == 0; i--, m <<= 1) ;
    m <<= 1; // Clear implicit leftmost 1
    m >>= 9;
    u32 e = 127 + i;
    if (n < 0) m |= 0x80000000;
    m |= e << 23;
    *((u32 *) res) = m;
}

void RFProtocolCFlie::sendCmdPacket(void)
{
    float x_roll, x_pitch, yaw;
  
    // Channels in AETR order

    // Roll, aka aileron, float +- 50.0 in degrees
    // float roll  = -(float) Channels[0]*50.0/10000;
    s32 f_roll = -(s32)RFProtocol::getControl(CH_AILERON) * FRAC_SCALE / (10000 / 50);

    // Pitch, aka elevator, float +- 50.0 degrees
    //float pitch = -(float) Channels[1]*50.0/10000;
    s32 f_pitch = -(s32)RFProtocol::getControl(CH_ELEVATOR) * FRAC_SCALE / (10000 / 50);

    // Thrust, aka throttle 0..65535, working range 5535..65535
    // No space for overshoot here, hard limit Channel3 by -10000..10000
    s32 ch = (s32)RFProtocol::getControl(CH_THROTTLE);
    if (ch < CHAN_MIN_VALUE) {
        ch = CHAN_MIN_VALUE;
    } else if (ch > CHAN_MAX_VALUE) {
        ch = CHAN_MAX_VALUE;
    }
    uint16_t thrust  = ch*3L + 35535L;

    // Yaw, aka rudder, float +- 400.0 deg/s
    // float yaw   = -(float) Channels[3]*400.0/10000;
    s32 f_yaw = -(s32)RFProtocol::getControl(CH_RUDDER) * FRAC_SCALE / (10000 / 400);
    frac2float(f_yaw, &yaw);
  
    // Convert + to X. 181 / 256 = 0.70703125 ~= sqrt(2) / 2
    s32 f_x_roll = (f_roll + f_pitch) * 181 / 256;
    frac2float(f_x_roll, &x_roll);
    s32 f_x_pitch = (f_pitch - f_roll) * 181 / 256;
    frac2float(f_x_pitch, &x_pitch);

    int bufptr = 0;
    mPacketBuf[bufptr++] = 0x30; // Commander packet to channel 0
    memcpy(&mPacketBuf[bufptr], (char*) &x_roll, 4);    bufptr += 4;
    memcpy(&mPacketBuf[bufptr], (char*) &x_pitch, 4);   bufptr += 4;
    memcpy(&mPacketBuf[bufptr], (char*) &yaw, 4);       bufptr += 4;
    memcpy(&mPacketBuf[bufptr], (char*) &thrust, 2);    bufptr += 2;

    // clear packet status bits and TX FIFO
    mDev.writeReg(NRF24L01_07_STATUS, (BV(NRF24L01_07_TX_DS) | BV(NRF24L01_07_MAX_RT)));
    mDev.flushTx();
    mDev.writePayload(mPacketBuf, sizeof(mPacketBuf));
    ++mPacketCtr;

//    radio.ce(HIGH);
//    delayMicroseconds(15);
    // It saves power to turn off radio after the transmission,
    // so as long as we have pins to do so, it is wise to turn
    // it back.
//    radio.ce(LOW);

    // Check and adjust transmission power. We do this after
    // transmission to not bother with timeout after power
    // settings change -  we have plenty of time until next
    // packet.
    if (isRFPowerUpdated()) {
        mDev.setRFPower(getRFPower());
        clearRFPowerUpdated();
    }
}

void RFProtocolCFlie::initRxTxAddr(void)
{
    // CFlie uses fixed address
    mRxTxAddrBuf[0] = 0xE7;
    mRxTxAddrBuf[1] = 0xE7;
    mRxTxAddrBuf[2] = 0xE7;
    mRxTxAddrBuf[3] = 0xE7;
    mRxTxAddrBuf[4] = 0xE7;

    mDataRate = NRF24L01_BR_250K;
    mCurRFChan  = 0;
    mState    = CFLIE_INIT_SEARCH;
}

void RFProtocolCFlie::init1(void)
{
    mDev.initialize();

    // CRC, radio on
    mDev.setTxRxMode(TX_EN);
    mDev.writeReg(NRF24L01_00_CONFIG, BV(NRF24L01_00_EN_CRC) | BV(NRF24L01_00_CRCO) | BV(NRF24L01_00_PWR_UP)); 
//    mDev.writeReg(NRF24L01_01_EN_AA, 0x00);               // No Auto Acknowledgement
    mDev.writeReg(NRF24L01_01_EN_AA, 0x01);                 // Auto Acknowledgement for data pipe 0
    mDev.writeReg(NRF24L01_02_EN_RXADDR, 0x01);             // Enable data pipe 0
    mDev.writeReg(NRF24L01_03_SETUP_AW, ADDR_BUF_SIZE - 2);     // 5-byte RX/TX address
    mDev.writeReg(NRF24L01_04_SETUP_RETR, 0x13);            // 3 retransmits, 500us delay

    mDev.writeReg(NRF24L01_05_RF_CH, mCurRFChan);           // Defined by model id
    mDev.setBitrate(mDataRate);
    
    mDev.setRFPower(getRFPower());
    mDev.writeReg(NRF24L01_07_STATUS, 0x70);                // Clear data ready, data sent, and retransmit

/*
    mDev.writeReg(NRF24L01_0C_RX_ADDR_P2, 0xC3); // LSB byte of pipe 2 receive address
    mDev.writeReg(NRF24L01_0D_RX_ADDR_P3, 0xC4);
    mDev.writeReg(NRF24L01_0E_RX_ADDR_P4, 0xC5);
    mDev.writeReg(NRF24L01_0F_RX_ADDR_P5, 0xC6);
    mDev.writeReg(NRF24L01_11_RX_PW_P0, PAYLOADSIZE);   // bytes of data payload for pipe 1
    mDev.writeReg(NRF24L01_12_RX_PW_P1, PAYLOADSIZE);
    mDev.writeReg(NRF24L01_13_RX_PW_P2, PAYLOADSIZE);
    mDev.writeReg(NRF24L01_14_RX_PW_P3, PAYLOADSIZE);
    mDev.writeReg(NRF24L01_15_RX_PW_P4, PAYLOADSIZE);
    mDev.writeReg(NRF24L01_16_RX_PW_P5, PAYLOADSIZE);
*/

    mDev.writeReg(NRF24L01_17_FIFO_STATUS, 0x00);  // Just in case, no real bits to write here

    // this sequence necessary for module from stock tx
    mDev.readReg(NRF24L01_1D_FEATURE);
    mDev.activate(0x73);                          // Activate feature register
    mDev.readReg(NRF24L01_1D_FEATURE);

    mDev.writeReg(NRF24L01_1C_DYNPD, 0x01);       // Enable Dynamic Payload Length on pipe 0
    mDev.writeReg(NRF24L01_1D_FEATURE, 0x06);     // Enable Dynamic Payload Length, enable Payload with ACK

    mDev.writeRegMulti(NRF24L01_0A_RX_ADDR_P0, mRxTxAddrBuf, ADDR_BUF_SIZE);
    mDev.writeRegMulti(NRF24L01_10_TX_ADDR, mRxTxAddrBuf, ADDR_BUF_SIZE);

    printf(F("init1 : %ld\n"), millis());
}


u16 RFProtocolCFlie::callState(void)
{
    switch (mState) {
    case CFLIE_INIT_SEARCH:
        sendSearchPacket();
        mState = CFLIE_SEARCH;
        break;
        
    case CFLIE_INIT_DATA:
        sendCmdPacket();
        mState = CFLIE_DATA;
        break;
        
    case CFLIE_SEARCH:
        switch (checkStatus()) {
        case PKT_PENDING:
            return PACKET_CHK_uS;               // packet send not yet complete

        case PKT_ACKED:
            mState = CFLIE_DATA;
            break;

        case PKT_TIMEOUT:
            sendSearchPacket();
            mBindCtr = MAX_BIND_COUNT;
        }
        break;

    case CFLIE_DATA:
        if (checkStatus() == PKT_PENDING)
            return PACKET_CHK_uS;               // packet send not yet complete
        sendCmdPacket();
        break;
    }

    return PACKET_PERIOD_uS;
}

int RFProtocolCFlie::init(void)
{
    mPacketCtr = 0;

    initRxTxAddr();
    init1();
    startState(INITIAL_WAIT_uS);

    return 0;
}

int RFProtocolCFlie::close(void)
{
    RFProtocol::close();
    mDev.initialize();
    return (mDev.reset() ? 1L : -1L);
}

int RFProtocolCFlie::reset(void)
{
    return close();
}

int RFProtocolCFlie::getInfo(s8 id, u8 *data)
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
                size = sizeof(mPacketCtr);
                *((u32*)data) = mPacketCtr;
                break;
        }
    }
    return size;
}

