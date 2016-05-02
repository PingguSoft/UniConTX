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
#include "RCRcvrPWM.h"

#define FW_VERSION  0x0120

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

static void initReceiver(u32 id)
{
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
}

static u8 initProtocol(u32 id)
{
    u8  ret = 0;

    initReceiver(id);

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

#if 1
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

void setup()
{
    Serial.begin(100000);

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
#if 1
    if (mRFProto) {
        if (mRcvr) {
            mRFProto->injectControls(mRcvr->getRCs(), mRcvr->getChCnt());
        }
        mRFProto->loop();
    }
#else
    if (mRcvr) {
        mSerial.sendString("%4d %4d %4d %d\n", mRcvr->getRC(4), mRcvr->getRC(5), mRcvr->getRC(6), mRcvr->getChCnt());
        delay(100);
    }
#endif
}

int freeRam() {
    extern int __heap_start, *__brkval;
    int v;
    return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}
