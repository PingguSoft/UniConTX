#ifndef _DEVICE_CYRF6936_H_
#define _DEVICE_CYRF6936_H_

#include "Common.h"
#include <Arduino.h>
#include <avr/pgmspace.h>

enum {
    CYRF_00_CHANNEL        = 0x00,
    CYRF_01_TX_LENGTH      = 0x01,
    CYRF_02_TX_CTRL        = 0x02,
    CYRF_03_TX_CFG         = 0x03,
    CYRF_04_TX_IRQ_STATUS  = 0x04,
    CYRF_05_RX_CTRL        = 0x05,
    CYRF_06_RX_CFG         = 0x06,
    CYRF_07_RX_IRQ_STATUS  = 0x07,
    CYRF_08_RX_STATUS      = 0x08,
    CYRF_09_RX_COUNT       = 0x09,
    CYRF_0A_RX_LENGTH      = 0x0A,
    CYRF_0B_PWR_CTRL       = 0x0B,
    CYRF_0C_XTAL_CTRL      = 0x0C,
    CYRF_0D_IO_CFG         = 0x0D,
    CYRF_0E_GPIO_CTRL      = 0x0E,
    CYRF_0F_XACT_CFG       = 0x0F,
    CYRF_10_FRAMING_CFG    = 0x10,
    CYRF_11_DATA32_THOLD   = 0x11,
    CYRF_12_DATA64_THOLD   = 0x12,
    CYRF_13_RSSI           = 0x13,
    CYRF_14_EOP_CTRL       = 0x14,
    CYRF_15_CRC_SEED_LSB   = 0x15,
    CYRF_16_CRC_SEED_MSB   = 0x16,
    CYRF_17_TX_CRC_LSB     = 0x17,
    CYRF_18_TX_CRC_MSB     = 0x18,
    CYRF_19_RX_CRC_LSB     = 0x19,
    CYRF_20_RX_CRC_MSB     = 0x1A,
    CYRF_1B_TX_OFFSET_LSB  = 0x1B,
    CYRF_1C_TX_OFFSET_MSB  = 0x1C,
    CYRF_1D_MODE_OVERRIDE  = 0x1D,
    CYRF_1E_RX_OVERRIDE    = 0x1E,
    CYRF_1F_TX_OVERRIDE    = 0x1F,
    /*Register Files */
    CYRF_20_TX_BUFFER      = 0x20,
    CYRF_21_RX_BUFFER      = 0x21,
    CYRF_22_SOP_CODE       = 0x22,
    CYRF_23_DATA_CODE      = 0x23,
    CYRF_24_PREAMBLE       = 0x24,
    CYRF_25_MFG_ID         = 0x25,
    /*****************/
    CYRF_26_XTAL_CFG       = 0x26,
    CYRF_27_CLK_OVERRIDE   = 0x27,
    CYRF_28_CLK_EN         = 0x28,
    CYRF_29_RX_ABORT       = 0x29,
    CYRF_32_AUTO_CAL_TIME  = 0x32,
    CYRF_35_AUTOCAL_OFFSET = 0x35,
    CYRF_39_ANALOG_CTRL    = 0x39,
};

enum CYRF_PWR {
    CYRF_PWR_100MW,
    CYRF_PWR_10MW,
    CYRF_PWR_DEFAULT,
};

class DeviceCYRF6936
{
    #define PIN_RXEN      6
    #define PIN_TXEN      7
    #define PIN_CSN       9
    #define PIN_IRQ       2

    #define CS_HI() digitalWrite(PIN_CSN, HIGH);
    #define CS_LO() digitalWrite(PIN_CSN, LOW);
    #define TX_HI() digitalWrite(PIN_TXEN, HIGH);
    #define TX_LO() digitalWrite(PIN_TXEN, LOW);
    #define RX_HI() digitalWrite(PIN_RXEN, HIGH);
    #define RX_LO() digitalWrite(PIN_RXEN, LOW);
    
public:
    void initialize();
    int  reset();
    u8   writeReg(u8 reg, u8 data);
    u8   writeRegMulti(u8 reg, const u8 *data, u8 length);
    u8   writeRegMulti_P(u8 reg, const u8 *data, u8 length);
    u8   readReg(u8 reg);
    u8   readRegMulti(u8 reg, u8 *data, u8 length);
    u8   setRFPower(u8 power);
    void setTxRxMode(enum TXRX_State);
    void readMfgID(u8 *data);
    void setRFChannel(u8 ch);
    void setCRCSeed(u16 crc);
    void setSOPCode(const u8 *sopcodes);
    void setSOPCode_P(const u8 *sopcodes);
    void setDataCode(const u8 *datacodes, u8 len);
    void writePreamble(u32 preamble);
    void startReceive();
    u8   writePayload(const u8 *data, u8 length);
    u8   writePayload_P(const u8 *data, u8 length);
    u8   readPayload(u8 *data, u8 length);
    u8   readRSSI(u32 dodummyread);
    void findBestChannels(u8 *channels, u8 len, u8 minspace, u8 min, u8 max);
    
// To enable radio transmit after WritePayload you need to turn the radio
//void PulseCE();

private:
    u8   strobe(u8 state);

// variables

};

#endif
