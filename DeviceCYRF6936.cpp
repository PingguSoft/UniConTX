#include <SPI.h>
#include "DeviceCYRF6936.h"

void DeviceCYRF6936::initialize()
{
    pinMode(PIN_IRQ, INPUT);
    pinMode(PIN_CSN, OUTPUT);
    pinMode(TX_EN, OUTPUT);
    pinMode(RX_EN, OUTPUT);

    CS_HI();
    TX_LO();
    RX_LO();

    SPI.begin();
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
    SPI.setClockDivider(SPI_CLOCK_DIV2);
}

#define PROTOSPI_xfer   SPI.transfer

u8 DeviceCYRF6936::writeReg(u8 reg, u8 data)
{
    CS_LO();
    u8 res = PROTOSPI_xfer(0x80 | reg);
    PROTOSPI_xfer(data);
    CS_HI();
    return res;
}

u8 DeviceCYRF6936::writeRegMulti(u8 reg, const u8 *data, u8 length)
{
    CS_LO();
    u8 res = PROTOSPI_xfer(0x80 | reg);
    for (u8 i = 0; i < length; i++) {
        PROTOSPI_xfer(*data++);
    }
    CS_HI();
    return res;
}

u8 DeviceCYRF6936::writeRegMulti_P(u8 reg, const u8 *data, u8 length)
{
    CS_LO();
    u8 res = PROTOSPI_xfer(0x80 | reg);
    for (u8 i = 0; i < length; i++) {
        PROTOSPI_xfer(pgm_read_byte(data++));
    }
    CS_HI();
    return res;
}

u8 DeviceCYRF6936::readReg(u8 reg)
{
    CS_LO();
    PROTOSPI_xfer(reg);
    u8 data = PROTOSPI_xfer(0xFF);
    CS_HI();
    return data;
}

u8 DeviceCYRF6936::readRegMulti(u8 reg, u8 *data, u8 length)
{
    CS_LO();
    u8 res = PROTOSPI_xfer(reg);
    for(u8 i = 0; i < length; i++) {
        *data++ = PROTOSPI_xfer(0xFF);
    }
    CS_HI();
    return res;
}

u8 DeviceCYRF6936::strobe(u8 state)
{
    CS_LO();
    u8 res = PROTOSPI_xfer(state);
    CS_HI();
    return res;
}

u8 DeviceCYRF6936::setRFPower(u8 power)
{
    u8 val = readReg(0x03) & 0xF8;
    writeReg(0x03, val | (power & 0x07));

    return val;
}

void DeviceCYRF6936::setTxRxMode(enum TXRX_State mode)
{
    if(mode == TX_EN) {
        writeReg(0x0E,0x20);
    } else {
        writeReg(0x0E,0x80);
    }
}

int DeviceCYRF6936::reset()
{
    writeReg(CYRF_1D_MODE_OVERRIDE, 0x01);
    delay(200);
    /* Reset the CYRF chip */

    writeReg(CYRF_0C_XTAL_CTRL, 0xC0); //Enable XOUT as GPIO
    writeReg(CYRF_0D_IO_CFG, 0x04); //Enable PACTL as GPIO
    setTxRxMode(TXRX_OFF);
    
    //Verify the CYRD chip is responding
    return (readReg(CYRF_10_FRAMING_CFG) == 0xa5);
}

void DeviceCYRF6936::readMfgID(u8 *data)
{
    /* Fuses power on */
    writeReg(0x25, 0xFF);

    readRegMulti(0x25, data, 6);

    /* Fuses power off */
    writeReg(0x25, 0x00); 
}

void DeviceCYRF6936::setRFChannel(u8 ch)
{
    writeReg(0x00, ch);
}

void DeviceCYRF6936::setCRCSeed(u16 crc)
{
    writeReg(0x15,crc & 0xff);
    writeReg(0x16,crc >> 8);
}

/*
 * these are the recommended sop codes from Cypress
 * See "WirelessUSB LP/LPstar and PRoC LP/LPstar Technical Reference Manual"
 */
void DeviceCYRF6936::setSOPCode(const u8 *sopcodes)
{
    //NOTE: This can also be implemented as:
    //for(i = 0; i < 8; i++) WriteRegister)0x23, sopcodes[i];
    writeRegMulti(0x22, sopcodes, 8);
}

void DeviceCYRF6936::setSOPCode_P(const u8 *sopcodes)
{
    //NOTE: This can also be implemented as:
    //for(i = 0; i < 8; i++) WriteRegister)0x23, sopcodes[i];
    writeRegMulti_P(0x22, sopcodes, 8);
}

void DeviceCYRF6936::setDataCode(const u8 *datacodes, u8 len)
{
    //NOTE: This can also be implemented as:
    //for(i = 0; i < len; i++) WriteRegister)0x23, datacodes[i];
    writeRegMulti(0x23, datacodes, len);
}

void DeviceCYRF6936::writePreamble(u32 preamble)
{
    CS_LO();
    PROTOSPI_xfer(0x80 | 0x24);
    PROTOSPI_xfer(preamble & 0xff);
    PROTOSPI_xfer((preamble >> 8) & 0xff);
    PROTOSPI_xfer((preamble >> 16) & 0xff);
    CS_HI();
}

void DeviceCYRF6936::startReceive()
{
    writeReg(0x05,0x87);
}

u8 DeviceCYRF6936::writePayload(const u8 *data, u8 length)
{
    u8 res = writeReg(CYRF_01_TX_LENGTH, length);
    writeReg(0x02, 0x40);
    writeRegMulti(0x20, data, length);
    writeReg(0x02, 0xBF);
    
    return res;
}

u8 DeviceCYRF6936::writePayload_P(const u8 *data, u8 length)
{
    u8 res = writeReg(CYRF_01_TX_LENGTH, length);
    writeReg(0x02, 0x40);
    writeRegMulti_P(0x20, data, length);
    writeReg(0x02, 0xBF);
    
    return res;
}

u8 DeviceCYRF6936::readPayload(u8 *data, u8 length)
{
    return readRegMulti(0x21, data, length);
}

u8 DeviceCYRF6936::readRSSI(u32 dodummyread)
{
    u8 result;
    
    if(dodummyread) {
        readReg(0x13);
    }
    
    result = readReg(0x13);
    if(result & 0x80) {
        result = readReg(0x13);
    }
    return (result & 0x0F);
}

//NOTE: This routine will reset the CRC Seed
void DeviceCYRF6936::findBestChannels(u8 *channels, u8 len, u8 minspace, u8 min, u8 max)
{
    #define NUM_FREQ 80
    #define FREQ_OFFSET 4
    u8 rssi[NUM_FREQ];

    if (min < FREQ_OFFSET)
        min = FREQ_OFFSET;
    if (max > NUM_FREQ)
        max = NUM_FREQ;

    int i;
    int j;
    memset(channels, 0, sizeof(u8) * len);
    
    setCRCSeed(0x0000);
    setTxRxMode(RX_EN);
    //Wait for pre-amp to switch from send to receive
    delay(1000);
    for(i = 0; i < NUM_FREQ; i++) {
        setRFChannel(i);
        readReg(0x13);
        startReceive();
        delay(10);
        rssi[i] = readReg(0x13);
    }

    for (i = 0; i < len; i++) {
        channels[i] = min;
        for (j = min; j < max; j++) {
            if (rssi[j] < rssi[channels[i]]) {
                channels[i] = j;
            }
            
        }
        for (j = channels[i] - minspace; j < channels[i] + minspace; j++) {
            //Ensure we don't reuse any channels within minspace of the selected channel again
            if (j < 0 || j >= NUM_FREQ)
                continue;
            rssi[j] = 0xff;
        }
    }
    setTxRxMode(TX_EN);
}
