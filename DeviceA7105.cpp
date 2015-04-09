#include <SPI.h>
#include "DeviceA7105.h"


void DeviceA7105::initialize()
{
    pinMode(PIN_RXEN, OUTPUT);
    pinMode(PIN_TXEN, OUTPUT);
    pinMode(PIN_CSN,  OUTPUT);

    CS_HI();
    TX_LO();
    RX_LO();
    
    SPI.begin();
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
    SPI.setClockDivider(SPI_CLOCK_DIV2);
}

#define PROTOSPI_xfer   SPI.transfer

u8 DeviceA7105::writeReg(u8 reg, u8 data)
{
    CS_LO();
    u8 res = PROTOSPI_xfer(reg);
    PROTOSPI_xfer(data);
    CS_HI();

    return res;
}

u8 DeviceA7105::writeData(const u8 *data, u8 length, u8 channel)
{
    u8 i;
    
    CS_LO();
    PROTOSPI_xfer(A7105_RST_WRPTR);
    u8 res = PROTOSPI_xfer(0x05);
    for (i = 0; i < length; i++)
        PROTOSPI_xfer(*data++);
    CS_HI();

    writeReg(0x0F, channel);

    CS_LO();
    PROTOSPI_xfer(A7105_TX);
    CS_HI();

    return res;
}

u8 DeviceA7105::writeData_P(const u8 *data, u8 length,  u8 channel)
{
    int i;
    CS_LO();
    PROTOSPI_xfer(A7105_RST_WRPTR);
    u8 res = PROTOSPI_xfer(0x05);
    for (i = 0; i < length; i++)
        PROTOSPI_xfer(pgm_read_byte(data++));
    CS_HI();

    writeReg(0x0F, channel);

    CS_LO();
    PROTOSPI_xfer(A7105_TX);
    CS_HI();

    return res;
}

u8 DeviceA7105::readReg(u8 reg)
{
    CS_LO();
    PROTOSPI_xfer(0x40 | reg);
    u8 data = PROTOSPI_xfer(0xFF);
    CS_HI();
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
    CS_LO();
    u8 res = PROTOSPI_xfer(state);
    CS_HI();
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

void DeviceA7105::setTxRxMode(enum TXRX_State mode)
{
    if(mode == TX_EN) {
//        writeReg(A7105_0B_GPIO1_PIN1, 0x33);
//        writeReg(A7105_0C_GPIO2_PIN_II, 0x31);
        RX_LO();
        TX_HI();
    } else if (mode == RX_EN) {
//        writeReg(A7105_0B_GPIO1_PIN1, 0x31);
//        writeReg(A7105_0C_GPIO2_PIN_II, 0x33);
        TX_LO();
        RX_HI();
    } else {
        //The A7105 seems to some with a cross-wired power-amp (A7700)
        //On the XL7105-D03, TX_EN -> RXSW and RX_EN -> TXSW
        //This means that sleep mode is wired as RX_EN = 1 and TX_EN = 1
        //If there are other amps in use, we'll need to fix this
//        writeReg(A7105_0B_GPIO1_PIN1, 0x33);
//        writeReg(A7105_0C_GPIO2_PIN_II, 0x33);
        TX_LO();
        RX_LO();
    }
}

int DeviceA7105::reset()
{
    writeReg(0x00, 0x00);
    delayMicroseconds(1000);

    // enable 4 wires SPI, GIO1 is connected to MISO
    writeReg(A7105_0B_GPIO1_PIN1, 0x19);

    setTxRxMode(TXRX_OFF);
    int res = (readReg(0x10) == 0x9E);
    strobe(A7105_STANDBY);
    return res;
}

void DeviceA7105::writeID(u32 id)
{
    CS_LO();
    PROTOSPI_xfer(0x06);
    PROTOSPI_xfer((id >> 24) & 0xFF);
    PROTOSPI_xfer((id >> 16) & 0xFF);
    PROTOSPI_xfer((id >> 8) & 0xFF);
    PROTOSPI_xfer((id >> 0) & 0xFF);
    CS_HI();
}