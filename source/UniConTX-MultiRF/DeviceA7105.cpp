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
#include "DeviceA7105.h"

#define A7105_CS_HI()   digitalWrite(PIN_CSN_7105, HIGH);
#define A7105_CS_LO()   digitalWrite(PIN_CSN_7105, LOW);

DeviceA7105::DeviceA7105()
{
    initialize();
}

void DeviceA7105::initialize()
{
    setRFSwitch(TX_A7105);
    
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
    SPI.setClockDivider(SPI_CLOCK_DIV2);
}

u8 DeviceA7105::writeReg(u8 reg, u8 data)
{
    A7105_CS_LO();
    u8 res = SPI.transfer(reg);
    SPI.transfer(data);
    A7105_CS_HI();

    return res;
}

u8 DeviceA7105::writeData(const u8 *data, u8 length, u8 channel)
{
    u8 i;

    A7105_CS_LO();
    SPI.transfer(A7105_RST_WRPTR);
    u8 res = SPI.transfer(0x05);
    for (i = 0; i < length; i++)
        SPI.transfer(*data++);
    A7105_CS_HI();

    writeReg(0x0F, channel);

    A7105_CS_LO();
    SPI.transfer(A7105_TX);
    A7105_CS_HI();

    return res;
}

u8 DeviceA7105::writeData_P(const u8 *data, u8 length,  u8 channel)
{
    int i;
    A7105_CS_LO();
    SPI.transfer(A7105_RST_WRPTR);
    u8 res = SPI.transfer(0x05);
    for (i = 0; i < length; i++)
        SPI.transfer(pgm_read_byte(data++));
    A7105_CS_HI();

    writeReg(0x0F, channel);

    A7105_CS_LO();
    SPI.transfer(A7105_TX);
    A7105_CS_HI();

    return res;
}

u8 DeviceA7105::readReg(u8 reg)
{
    A7105_CS_LO();
    SPI.transfer(0x40 | reg);
    u8 data = SPI.transfer(0xFF);
    A7105_CS_HI();
    return data;
}

u8 DeviceA7105::readData(u8 *data, u8 length)
{
    u8 res = strobe(A7105_RST_RDPTR);
    for(u8 i = 0; i < length; i++)
        *data = readReg(0x05);
    return res;
}

u8 DeviceA7105::strobe(u8 state)
{
    A7105_CS_LO();
    u8 res = SPI.transfer(state);
    A7105_CS_HI();
    return res;
}

u8 DeviceA7105::setRFPower(u8 power)
{
    /*
    Power amp is ~+16dBm so:
    TXPOWER_100uW  = -23dBm == PAC=0 TBG=0
    TXPOWER_300uW  = -20dBm == PAC=0 TBG=1
    TXPOWER_1mW    = -16dBm == PAC=0 TBG=2
    TXPOWER_3mW    = -11dBm == PAC=0 TBG=4
    TXPOWER_10mW   = -6dBm  == PAC=1 TBG=5
    TXPOWER_30mW   = 0dBm   == PAC=2 TBG=7
    TXPOWER_100mW  = 1dBm   == PAC=3 TBG=7
    TXPOWER_150mW  = 1dBm   == PAC=3 TBG=7
    */
    u8 pac, tbg;
    switch(power) {
        case 0: pac = 0; tbg = 0; break;
        case 1: pac = 0; tbg = 1; break;
        case 2: pac = 0; tbg = 2; break;
        case 3: pac = 0; tbg = 4; break;
        case 4: pac = 1; tbg = 5; break;
        case 5: pac = 2; tbg = 7; break;
        case 6: pac = 3; tbg = 7; break;
        case 7: pac = 3; tbg = 7; break;
        default: pac = 0; tbg = 0; break;
    };
    return writeReg(0x28, (pac << 3) | tbg);
}

void DeviceA7105::setRFModeImpl(enum RF_MODE mode)
{
    if(mode == RF_TX) {
//        writeReg(A7105_0B_GPIO1_PIN1, 0x33);
//        writeReg(A7105_0C_GPIO2_PIN_II, 0x31);
    } else if (mode == RF_RX) {
//        writeReg(A7105_0B_GPIO1_PIN1, 0x31);
//        writeReg(A7105_0C_GPIO2_PIN_II, 0x33);
    } else {
        //The TX_A7105 seems to some with a cross-wired power-amp (A7700)
        //On the XL7105-D03, RF_TX -> RXSW and RF_RX -> TXSW
        //This means that sleep mode is wired as RF_RX = 1 and RF_TX = 1
        //If there are other amps in use, we'll need to fix this
//        writeReg(A7105_0B_GPIO1_PIN1, 0x33);
//        writeReg(A7105_0C_GPIO2_PIN_II, 0x33);
    }
}

int DeviceA7105::reset()
{
    writeReg(0x00, 0x00);
    delayMicroseconds(1000);

    // enable 4 wires SPI, GIO1 is connected to MISO
    writeReg(A7105_0B_GPIO1_PIN1, 0x19);

    setRFMode(RF_IDLE);
    int res = (readReg(0x10) == 0x9E);
    strobe(A7105_STANDBY);
    return res;
}

void DeviceA7105::writeID(u32 id)
{
    A7105_CS_LO();
    SPI.transfer(0x06);
    SPI.transfer((id >> 24) & 0xFF);
    SPI.transfer((id >> 16) & 0xFF);
    SPI.transfer((id >> 8) & 0xFF);
    SPI.transfer((id >> 0) & 0xFF);
    A7105_CS_HI();
}