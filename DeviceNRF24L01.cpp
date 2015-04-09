#include <SPI.h>
#include "DeviceNRF24L01.h"

/* Instruction Mnemonics */
#define R_REGISTER    0x00
#define W_REGISTER    0x20
#define REGISTER_MASK 0x1F
#define ACTIVATE      0x50
#define R_RX_PL_WID   0x60
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xA0
#define W_ACK_PAYLOAD 0xA8
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2
#define REUSE_TX_PL   0xE3
#define NOP           0xFF

void DeviceNRF24L01::initialize()
{
    pinMode(PIN_IRQ, INPUT);
    pinMode(PIN_CSN, OUTPUT);
    pinMode(PIN_CE, OUTPUT);

    CS_HI();
    CE_HI();

    SPI.begin();
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
    SPI.setClockDivider(SPI_CLOCK_DIV2);

    mRFsetup = 0x0F;
}

#define PROTOSPI_xfer   SPI.transfer

u8 DeviceNRF24L01::writeReg(u8 reg, u8 data)
{
    CS_LO();
    u8 res = PROTOSPI_xfer(W_REGISTER | (REGISTER_MASK & reg));
    PROTOSPI_xfer(data);
    CS_HI();
    return res;
}

u8 DeviceNRF24L01::writeRegMulti(u8 reg, const u8 *data, u8 length)
{
    CS_LO();
    u8 res = PROTOSPI_xfer(W_REGISTER | ( REGISTER_MASK & reg));
    for (u8 i = 0; i < length; i++) {
        PROTOSPI_xfer(*data++);
    }
    CS_HI();
    return res;
}

u8 DeviceNRF24L01::writeRegMulti_P(u8 reg, const u8 *data, u8 length)
{
    CS_LO();
    u8 res = PROTOSPI_xfer(W_REGISTER | ( REGISTER_MASK & reg));
    for (u8 i = 0; i < length; i++) {
        PROTOSPI_xfer(pgm_read_byte(data++));
    }
    CS_HI();
    return res;
}

u8 DeviceNRF24L01::writePayload(u8 *data, u8 length)
{
    CS_LO();
    u8 res = PROTOSPI_xfer(W_TX_PAYLOAD);
    for (u8 i = 0; i < length; i++) {
        PROTOSPI_xfer(*data++);
    }
    CS_HI();
    return res;
}

u8 DeviceNRF24L01::writePayload_P(const u8 *data, u8 length)
{
    CS_LO();
    u8 res = PROTOSPI_xfer(W_TX_PAYLOAD);
    for (u8 i = 0; i < length; i++) {
        PROTOSPI_xfer(pgm_read_byte(data++));
    }
    CS_HI();
    return res;
}

u8 DeviceNRF24L01::readReg(u8 reg)
{
    CS_LO();
    PROTOSPI_xfer(R_REGISTER | (REGISTER_MASK & reg));
    u8 data = PROTOSPI_xfer(0xFF);
    CS_HI();
    return data;
}

u8 DeviceNRF24L01::readRegMulti(u8 reg, u8 data[], u8 length)
{
    CS_LO();
    u8 res = PROTOSPI_xfer(R_REGISTER | (REGISTER_MASK & reg));
    for(u8 i = 0; i < length; i++) {
        data[i] = PROTOSPI_xfer(0xFF);
    }
    CS_HI();
    return res;
}

u8 DeviceNRF24L01::readPayload(u8 *data, u8 length)
{
    CS_LO();
    u8 res = PROTOSPI_xfer(R_RX_PAYLOAD);
    for(u8 i = 0; i < length; i++) {
        data[i] = PROTOSPI_xfer(0xFF);
    }
    CS_HI();
    return res;
}

u8 DeviceNRF24L01::strobe(u8 state)
{
    CS_LO();
    u8 res = PROTOSPI_xfer(state);
    CS_HI();
    return res;
}

u8 DeviceNRF24L01::flushTx()
{
    return strobe(FLUSH_TX);
}

u8 DeviceNRF24L01::flushRx()
{
    return strobe(FLUSH_RX);
}

u8 DeviceNRF24L01::activate(u8 code)
{
    CS_LO();
    u8 res = PROTOSPI_xfer(ACTIVATE);
    PROTOSPI_xfer(code);
    CS_HI();
    return res;
}

u8 DeviceNRF24L01::setBitrate(u8 bitrate)
{
    // Note that bitrate 250kbps (and bit RF_DR_LOW) is valid only
    // for nRF24L01+. There is no way to programmatically tell it from
    // older version, nRF24L01, but the older is practically phased out
    // by Nordic, so we assume that we deal with with modern version.

    // Bit 0 goes to RF_DR_HIGH, bit 1 - to RF_DR_LOW
    mRFsetup = (mRFsetup & 0xD7) | ((bitrate & 0x02) << 4) | ((bitrate & 0x01) << 3);
    return writeReg(NRF24L01_06_RF_SETUP, mRFsetup);
}

// Power setting is 0..3 for nRF24L01
// Claimed power amp for nRF24L01 from eBay is 20dBm.
//      Raw            w 20dBm PA
// 0 : -18dBm  (16uW)   2dBm (1.6mW)
// 1 : -12dBm  (60uW)   8dBm   (6mW)
// 2 :  -6dBm (250uW)  14dBm  (25mW)
// 3 :   0dBm   (1mW)  20dBm (100mW)
// So it maps to Deviation as follows
/*
TXPOWER_100uW  = -10dBm
TXPOWER_300uW  = -5dBm
TXPOWER_1mW    = 0dBm
TXPOWER_3mW    = 5dBm
TXPOWER_10mW   = 10dBm
TXPOWER_30mW   = 15dBm
TXPOWER_100mW  = 20dBm
TXPOWER_150mW  = 22dBm
*/
u8 DeviceNRF24L01::setRFPower(u8 power)
{
    u8 nrf_power = 0;

    switch(power) {
        case TXPOWER_100uW:
        case TXPOWER_300uW:
        case TXPOWER_1mW:   
            nrf_power = 0; 
            break;

        case TXPOWER_3mW:
        case TXPOWER_10mW:  
            nrf_power = 1; 
            break;

        case TXPOWER_30mW:  
            nrf_power = 2; 
            break;

        case TXPOWER_100mW:
        case TXPOWER_150mW: 
            nrf_power = 3; 
            break;
    };
    
    // Power is in range 0..3 for nRF24L01
    mRFsetup = (mRFsetup & 0xF9) | ((nrf_power & 0x03) << 1);
    return writeReg(NRF24L01_06_RF_SETUP, mRFsetup);
}

void DeviceNRF24L01::setTxRxMode(enum TXRX_State mode)
{
    if(mode == TX_EN) {
        CE_LO();
        writeReg(NRF24L01_07_STATUS, (1 << NRF24L01_07_RX_DR)    //reset the flag(s)
                                            | (1 << NRF24L01_07_TX_DS)
                                            | (1 << NRF24L01_07_MAX_RT));
        writeReg(NRF24L01_00_CONFIG, (1 << NRF24L01_00_EN_CRC)   // switch to TX mode
                                            | (1 << NRF24L01_00_CRCO)
                                            | (1 << NRF24L01_00_PWR_UP));
        delayMicroseconds(150);
        CE_HI();
    } else if (mode == RX_EN) {
        CE_LO();
        writeReg(NRF24L01_07_STATUS, 0x70);        // reset the flag(s)
        writeReg(NRF24L01_00_CONFIG, 0x0F);        // switch to RX mode
        writeReg(NRF24L01_07_STATUS, (1 << NRF24L01_07_RX_DR)    //reset the flag(s)
                                            | (1 << NRF24L01_07_TX_DS)
                                            | (1 << NRF24L01_07_MAX_RT));
        writeReg(NRF24L01_00_CONFIG, (1 << NRF24L01_00_EN_CRC)   // switch to RX mode
                                            | (1 << NRF24L01_00_CRCO)
                                            | (1 << NRF24L01_00_PWR_UP)
                                            | (1 << NRF24L01_00_PRIM_RX));
        delayMicroseconds(150);
        CE_HI();
    } else {
        writeReg(NRF24L01_00_CONFIG, (1 << NRF24L01_00_EN_CRC)); //PowerDown
        CE_LO();
    }
}

int DeviceNRF24L01::reset()
{
    flushTx();
    flushRx();
    u8 status1 = strobe(NOP);
    u8 status2 = readReg(0x07);
    setTxRxMode(TXRX_OFF);
    return (status1 == status2 && (status1 & 0x0f) == 0x0e);
}

