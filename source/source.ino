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

#include <Arduino.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <SPI.h>
#include <EEPROM.h>

#include "common.h"
#include "utils.h"
#include "RFProtocolSyma.h"
#include "RFProtocolYD717.h"
#include "RFProtocolV2x2.h"
#include "RFProtocolHiSky.h"
#include "RFProtocolCFlie.h"
#include "RFProtocolDevo.h"
#include "RFProtocolHubsan.h"
#include "RFProtocolFlysky.h"
#include "SerialProtocol.h"
#include "RCRcvrPWM.h"

#define FW_VERSION  0x0120

static SerialProtocol  mSerial;
static u8 mBaudAckLen;
static u8 mBaudChkCtr;
static u8 mBaudAckStr[12];
static RFProtocol *mRFProto = NULL;
static RCRcvr *mRcvr = NULL;

struct Config {
    u32 dwSignature;
    u32 dwProtoID;
    u32 dwConID;
    u8  ucPower;
};

u8 initProtocol(u32 id)
{
    u8  ret = 0;

    // receiver
    if (mRcvr) {
        mRcvr->close();
        delete mRcvr;
        mRcvr = NULL;
    }
    switch(RFProtocol::getRcvr(id)) {
        case RFProtocol::RCVR_PWM:
            mRcvr = new RCRcvrPWM();
            break;
    }
    if (mRcvr)
        mRcvr->init();

    // protocol
    if (mRFProto) {
        delete mRFProto;
        mRFProto = NULL;
    }
    switch (RFProtocol::getModule(id)) {
        case RFProtocol::TX_NRF24L01: {
            ret = 1;
            switch (RFProtocol::getProtocol(id)) {
                case RFProtocol::PROTO_NRF24L01_SYMAX:
                    mRFProto = new RFProtocolSyma(id);
                    break;

                case RFProtocol::PROTO_NRF24L01_YD717:
                    mRFProto = new RFProtocolYD717(id);
                    break;

                case RFProtocol::PROTO_NRF24L01_V2x2:
                    mRFProto = new RFProtocolV2x2(id);
                    break;

                case RFProtocol::PROTO_NRF24L01_HISKY:
                    mRFProto = new RFProtocolHiSky(id);
                    break;

                case RFProtocol::PROTO_NRF24L01_CFLIE:
                    mRFProto = new RFProtocolCFlie(id);
                    break;
                default:
                    ret = 0;
                    break;
            }
        }
        break;

#if 0
        case RFProtocol::TX_CYRF6936:
            mRFProto = new RFProtocolDevo(id);
            ret = 1;
        break;

        case RFProtocol::TX_A7105: {
            ret = 1;
            switch (RFProtocol::getProtocol(id)) {
                case RFProtocol::PROTO_A7105_FLYSKY:
                    mRFProto = new RFProtocolFlysky(id);
                    break;

                case RFProtocol::PROTO_A7105_HUBSAN:
                    mRFProto = new RFProtocolHubsan(id);
                    break;

                default:
                    ret = 0;
                    break;
            }
        }
        break;
#endif
    }

    return ret;
}

u32 serialCallback(u8 cmd, u8 *data, u8 size)
{
    u32 id;
    u16 ram;
    u8  ret = 0;
    u8  buf[5];
    u8  sz = 0;

    switch (cmd) {
        case SerialProtocol::CMD_GET_VERSION:
            ram = FW_VERSION;
            mSerial.sendResponse(true, cmd, (u8*)&ram, sizeof(ram));
            break;

        case SerialProtocol::CMD_SET_RFPROTOCOL:
            id = *(u32*)data;

            ret = initProtocol(id);
            //mSerial.sendResponse(true, cmd, (u8*)&ret, sizeof(ret));
            mSerial.sendResponse(true, cmd, (u8*)mBaudAckStr, mBaudAckLen);
            break;

        case SerialProtocol::CMD_START_RF:
            id = *(u32*)data;
            sz = *(data + 4);
            if (mRFProto) {
                mRFProto->setControllerID(id);
                mRFProto->setRFPower(sz);
                mRFProto->init();
                ret = 1;
            }
            struct Config conf;

            conf.dwSignature = 0xCAFEBABE;
            conf.dwProtoID   = mRFProto->getProtoID();
            conf.dwConID     = id;
            conf.ucPower     = sz;
            EEPROM.put(0, conf);

            mSerial.sendResponse(true, cmd, (u8*)&ret, sizeof(ret));
            break;

        case SerialProtocol::CMD_STOP_RF:
            if (mRFProto) {
                mRFProto->close();
                delete mRFProto;
                mRFProto = NULL;
                ret = 1;
            }
            mSerial.sendResponse(true, cmd, (u8*)&ret, sizeof(ret));
            break;

        case SerialProtocol::CMD_SET_RF_POWER:
            if (mRFProto) {
                mRFProto->setRFPower(*data);
                ret = 1;
            }
            mSerial.sendResponse(true, cmd, (u8*)&ret, sizeof(ret));
            break;

        case SerialProtocol::CMD_INJECT_CONTROLS:
            if (mRFProto && mRFProto->getRcvr() == RFProtocol::RCVR_MSP) {
                mRFProto->injectControls((s16*)data, size >> 1);
                ret = 1;
            }
            mSerial.sendResponse(true, cmd, (u8*)&ret, sizeof(ret));
            break;

        case SerialProtocol::CMD_GET_INFO:
            buf[0] = *data;
            if (mRFProto) {
                sz = mRFProto->getInfo(buf[0], &buf[1]);
            }
            mSerial.sendResponse(true, cmd, buf, sz + 1);
            break;

        case SerialProtocol::CMD_GET_FREE_RAM:
            ram = freeRam();
            mSerial.sendResponse(true, cmd, (u8*)&ram, sizeof(ram));
            break;

        case SerialProtocol::CMD_CHANGE_BAUD:
            mSerial.clearRX();
            mSerial.clearTX();
            for (u8 i = 0; i < 3; i++) {
                mSerial.sendString_P(PSTR("AT"));
                delay(1050);
            }
            strncpy_P((char*)mBaudAckStr, PSTR("AT+BAUD"), 7);
            mBaudAckStr[7] = *data;
            mBaudAckStr[8] = 0;
            mSerial.sendString((char*)mBaudAckStr);
            delay(2000);
            mBaudAckLen = mSerial.getString(mBaudAckStr);
            while(1);
            break;
    }
    return ret;
}

void setup()
{
    mSerial.begin(9600);
    mSerial.setCallback(serialCallback);
    mBaudChkCtr = 0;

    struct Config conf;
    EEPROM.get(0, conf);

    if (conf.dwSignature == 0xCAFEBABE) {
        initProtocol(conf.dwProtoID);
        if (mRFProto) {
            mRFProto->setControllerID(conf.dwConID);
            mRFProto->setRFPower(conf.ucPower);
            mRFProto->init();
        }
    }
}

void loop()
{
    if (mBaudChkCtr == 0) {
        mSerial.sendString_P(PSTR("AT+BAUD7"));
        delay(2000);
        mBaudAckLen = mSerial.getString(mBaudAckStr);

        mSerial.begin(57600);
        delay(500);

        mSerial.sendString_P(PSTR("AT+NAMEUniConTX"));
        delay(1050);
        mBaudAckLen = mSerial.getString(mBaudAckStr);
        mBaudChkCtr++;
    } else {
#if 1
        mSerial.handleRX();
        if (mRFProto) {
            if (mRcvr)
                mRFProto->injectControls(mRcvr->getRCs(), mRcvr->getMaxCh());
            mRFProto->loop();
        }
#else
        if (mRcvr) {
            mSerial.sendString("%4d %4d %4d\n", mRcvr->getRC(4), mRcvr->getRC(5), mRcvr->getRC(6));
            delay(100);
        }
#endif
    }
}

int freeRam() {
    extern int __heap_start, *__brkval;
    int v;
    return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}
