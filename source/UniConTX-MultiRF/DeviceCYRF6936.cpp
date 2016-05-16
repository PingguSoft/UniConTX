/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is derived from deviationTx project for Arduino.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 see <http://www.gnu.org/licenses/>
*/


#include <SPI.h>
#include "DeviceCYRF6936.h"
#include "Utils.h"

#define CYRF_CS_HI()    digitalWrite(PIN_CYRF_CSN, HIGH);
#define CYRF_CS_LO()    digitalWrite(PIN_CYRF_CSN, LOW);
#define CYRF_RST_HI()   digitalWrite(PIN_CYRF_RESET, HIGH);
#define CYRF_RST_LO()   digitalWrite(PIN_CYRF_RESET, LOW);

DeviceCYRF6936::DeviceCYRF6936()
{
    initialize();
}

void DeviceCYRF6936::initialize()
{
    setRFSwitch(TX_CYRF6936);

    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
    //SPI.setClockDivider(SPI_CLOCK_DIV2);

    CYRF_RST_HI();
    delay(100);
    CYRF_RST_LO();
    delay(100);
}

u8 DeviceCYRF6936::writeReg(u8 reg, u8 data)
{
    CYRF_CS_LO();
    u8 res = SPI.transfer(0x80 | reg);
    SPI.transfer(data);
    CYRF_CS_HI();
    return res;
}

u8 DeviceCYRF6936::writeRegMulti(u8 reg, const u8 *data, u8 length)
{
    CYRF_CS_LO();
    u8 res = SPI.transfer(0x80 | reg);
    for (u8 i = 0; i < length; i++) {
        SPI.transfer(data[i]);
    }
    CYRF_CS_HI();
    return res;
}

u8 DeviceCYRF6936::writeRegMulti_P(u8 reg, const u8 *data, u8 length)
{
    CYRF_CS_LO();
    u8 res = SPI.transfer(0x80 | reg);
    for (u8 i = 0; i < length; i++) {
        SPI.transfer(pgm_read_byte(data + i));
    }
    CYRF_CS_HI();
    return res;
}

u8 DeviceCYRF6936::readReg(u8 reg)
{
    CYRF_CS_LO();
    SPI.transfer(reg);
    u8 data = SPI.transfer(0);
    CYRF_CS_HI();
    return data;
}

u8 DeviceCYRF6936::readRegMulti(u8 reg, u8 *data, u8 length)
{
    CYRF_CS_LO();
    u8 res = SPI.transfer(reg);
    for(u8 i = 0; i < length; i++) {
        *data++ = SPI.transfer(0);
    }
    CYRF_CS_HI();
    return res;
}

u8 DeviceCYRF6936::strobe(u8 state)
{
    CYRF_CS_LO();
    u8 res = SPI.transfer(state);
    CYRF_CS_HI();
    return res;
}

u8 DeviceCYRF6936::setRFPower(u8 power)
{
    u8 val = readReg(CYRF_03_TX_CFG) & 0xF8;
    writeReg(CYRF_03_TX_CFG, val | (power & 0x07));

    return val;
}

void DeviceCYRF6936::setRFModeImpl(enum RF_MODE mode)
{
    u8 mod;
    u8 gpio;

   if(mode == RF_TX) {
        mod  = 0x28;        // force SYNTH MODE (8) = TX
        gpio = 0x80;        // XOUT(7)=1, PACTL(5)=0       XOUT is a switch for TX
    } else if (mode == RF_RX) {
        mod  = 0x2C;        // force SYNTH MODE (C) = RX
        gpio = 0x20;        // XOUT(7)=0, PACTL(5)=1       PACTL is a switch for RX
    } else {
        mod  = 0x24;        // force IDLE
        gpio = 0x00;
    }
    writeReg(CYRF_0E_GPIO_CTRL, gpio);
    writeReg(CYRF_0F_XACT_CFG,  mod);
}

int DeviceCYRF6936::reset()
{
    writeReg(CYRF_1D_MODE_OVERRIDE, 0x01);
    delay(200);

/*
    CYRF_RST_HI();
    delay(100);
    CYRF_RST_LO();
    delay(100);
*/
    /* Reset the CYRF chip */
    writeReg(CYRF_0C_XTAL_CTRL, 0xC0);  // Enable XOUT as GPIO
    writeReg(CYRF_0D_IO_CFG, 0x04);     // Enable PACTL as GPIO
    setRFMode(RF_IDLE);

    //Verify the CYRD chip is responding
    LOG(F("ID:%x\n"), readReg(CYRF_10_FRAMING_CFG));
    return (readReg(CYRF_10_FRAMING_CFG) == 0xa5);
}

void DeviceCYRF6936::readMfgID(u8 *data)
{
    /* Fuses power on */
    writeReg(CYRF_25_MFG_ID, 0xFF);

    readRegMulti(CYRF_25_MFG_ID, data, 6);

    /* Fuses power off */
    writeReg(CYRF_25_MFG_ID, 0x00);
    DUMP("MFG", data, 6);
}

void DeviceCYRF6936::setRFChannel(u8 ch)
{
    writeReg(CYRF_00_CHANNEL, ch);
}

void DeviceCYRF6936::setCRCSeed(u16 crc)
{
    writeReg(CYRF_15_CRC_SEED_LSB, crc & 0xff);
    writeReg(CYRF_16_CRC_SEED_MSB, crc >> 8);
}

/*
 * these are the recommended sop codes from Cypress
 * See "WirelessUSB LP/LPstar and PRoC LP/LPstar Technical Reference Manual"
 */
void DeviceCYRF6936::setSOPCode(u8 *sopcodes)
{
    //NOTE: This can also be implemented as:
    //for(i = 0; i < 8; i++) WriteRegister)0x23, sopcodes[i];
    writeRegMulti(CYRF_22_SOP_CODE, sopcodes, 8);
}

void DeviceCYRF6936::setSOPCode_P(const u8 *sopcodes)
{
    //NOTE: This can also be implemented as:
    //for(i = 0; i < 8; i++) WriteRegister)0x23, sopcodes[i];
    writeRegMulti_P(CYRF_22_SOP_CODE, sopcodes, 8);
}

void DeviceCYRF6936::setDataCode(u8 *datacodes, u8 len)
{
    //NOTE: This can also be implemented as:
    //for(i = 0; i < len; i++) WriteRegister)0x23, datacodes[i];
    writeRegMulti(CYRF_23_DATA_CODE, datacodes, len);
}

void DeviceCYRF6936::writePreamble(u32 preamble)
{
    CYRF_CS_LO();
    SPI.transfer(0x80 | 0x24);
    SPI.transfer(preamble & 0xff);
    SPI.transfer((preamble >> 8) & 0xff);
    SPI.transfer((preamble >> 16) & 0xff);
    CYRF_CS_HI();
}

void DeviceCYRF6936::startReceive()
{
    writeReg(CYRF_05_RX_CTRL, 0x87);
}

u8 DeviceCYRF6936::writePayload(u8 *data, u8 length)
{
    u8 res = writeReg(CYRF_01_TX_LENGTH, length);
    writeReg(CYRF_02_TX_CTRL, 0x40);    // TX CLR
    writeRegMulti(CYRF_20_TX_BUFFER, data, length);
    writeReg(CYRF_02_TX_CTRL, 0xBF);    // TX GO

    return res;
}

u8 DeviceCYRF6936::writePayload_P(const u8 *data, u8 length)
{
    u8 res = writeReg(CYRF_01_TX_LENGTH, length);
    writeReg(CYRF_02_TX_CTRL, 0x40);    // TX CLR
    writeRegMulti_P(CYRF_20_TX_BUFFER, data, length);
    writeReg(CYRF_02_TX_CTRL, 0xBF);    // TX GO

    return res;
}

u8 DeviceCYRF6936::readPayload(u8 *data, u8 length)
{
    return readRegMulti(CYRF_21_RX_BUFFER, data, length);
}

u8 DeviceCYRF6936::readRSSI(u32 dodummyread)
{
    u8 result;

    if(dodummyread) {
        readReg(CYRF_13_RSSI);
    }

    result = readReg(CYRF_13_RSSI);
    if(result & 0x80) {
        result = readReg(CYRF_13_RSSI);
    }
    return (result & 0x1F);
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
    setRFMode(RF_RX);
    //Wait for pre-amp to switch from send to receive
    delay(1000);
    for(i = 0; i < NUM_FREQ; i++) {
        setRFChannel(i);
        readReg(CYRF_13_RSSI);
        startReceive();
        delay(10);
        rssi[i] = readReg(CYRF_13_RSSI);
        //LOG(F("CH:%d, RSSI:%d\n"), i, rssi[i]);
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
    setRFMode(RF_TX);
}
