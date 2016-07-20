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
#include "RFProtocolDSM.h"
#include "RFProtocolHubsan.h"
#include "RFProtocolFlysky.h"
#include "RCRcvrPPM.h"
#include "RCRcvrERSkySerial.h"
#include "SerialProtocol.h"

#define FW_VERSION  0x0120

static u8 mBaudAckLen;
static u8 mBaudChkCtr;
static u8 mBaudAckStr[12];
static RFProtocol *mRFProto = NULL;
static RCRcvr     *mRcvr = NULL;
static SerialProtocol mSerial;

struct Config {
    u32 dwSignature;
    u32 dwProtoID;
    u32 dwConID;
    u8  ucPower;
};

static u8 initProtocol(u32 id)
{
    u8  ret = 0;

    // protocol
    if (mRFProto) {
        delete mRFProto;
        mRFProto = NULL;
    }
    switch (RFProtocol::getModule(id)) {
        case TX_NRF24L01: {
            ret = 1;
            switch (RFProtocol::getProtocol(id)) {
                case RFProtocol::PROTO_NRF24L01_SYMAX:
                    mRFProto = new RFProtocolSyma(id);
                    break;

                case RFProtocol::PROTO_NRF24L01_YD717:
                    mRFProto = new RFProtocolYD717(id);
                    break;
/*
                case RFProtocol::PROTO_NRF24L01_V2x2:
                    mRFProto = new RFProtocolV2x2(id);
                    break;

                case RFProtocol::PROTO_NRF24L01_HISKY:
                    mRFProto = new RFProtocolHiSky(id);
                    break;

                case RFProtocol::PROTO_NRF24L01_CFLIE:
                    mRFProto = new RFProtocolCFlie(id);
                    break;
*/
                default:
                    ret = 0;
                    break;
            }
        }
        break;

        case TX_CYRF6936:
            ret = 1;
            switch (RFProtocol::getProtocol(id)) {
                case RFProtocol::PROTO_CYRF6936_DEVO:
                    mRFProto = new RFProtocolDevo(id);
                    break;

                case RFProtocol::PROTO_CYRF6936_DSMX:
                    mRFProto = new RFProtocolDSM(id);
                    break;

                default:
                    ret = 0;
                    break;
            }
            break;

/*
        case TX_A7105: {
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
*/
    }
    return ret;
}

void setup()
{
#if __STD_SERIAL__
    Serial.begin(57600);
#else
    mSerial.begin(100000, SERIAL_8E2);
#endif

    LOG(F("Start!!\n"));

    mRcvr = new RCRcvrPPM();
//    mRcvr = new RCRcvrERSkySerial();
    mRcvr->init();

#if 1
    struct Config conf;
    EEPROM.get(0, conf);

    conf.dwSignature = 0xCAFEBABE;
    conf.dwProtoID   = RFProtocol::buildID(TX_CYRF6936, RFProtocol::PROTO_CYRF6936_DSMX, 0);
//    conf.dwProtoID   = RFProtocol::buildID(TX_CYRF6936, RFProtocol::PROTO_CYRF6936_DEVO, 0);
//    conf.dwProtoID   = RFProtocol::buildID(TX_NRF24L01, RFProtocol::PROTO_NRF24L01_SYMAX, 0);
    conf.dwConID     = 0x12345678;
    conf.ucPower     = TXPOWER_150mW;

    if (conf.dwSignature == 0xCAFEBABE) {
        initProtocol(conf.dwProtoID);
        if (mRFProto) {
            mRFProto->setControllerID(conf.dwConID);
            mRFProto->setRFPower(conf.ucPower);
//            mRFProto->init();
        }
    }
#endif
}

s16 thr  = CHAN_MIN_VALUE;
s16 ele  = CHAN_MID_VALUE;
s16 ail  = CHAN_MAX_VALUE / 2;
s16 rud  = CHAN_MIN_VALUE / 2;
s16 step_thr = 2;
s16 step_ele = 2;
s16 step_ail = 2;
s16 step_rud = 2;
u8  sim = 0;
u32 ts = 0;
void loop()
{
    ts = micros();

    if (mRcvr) {
#if 0
        u32 proto = mRcvr->loop();

        if (proto) {
            LOG(F("PROTO TJ :%x\n"), proto);
            initProtocol(proto);
            if (mRFProto) {
                mRFProto->setControllerID(0x12345678);
                mRFProto->setRFPower(TXPOWER_150mW);
                mRFProto->init();
            }
        }
#else
        if (Serial.available()) {
            u8 ch = Serial.read();

            switch (ch) {
                case 'b':
                    sim = 0;
                    mRFProto->init();
                    break;

                case 's':
                    sim = 1;
                    break;
            }
        }

        if (sim) {
            static u32 lastTS;
            if (ts - lastTS > 20000) {
                if (thr <  CHAN_MIN_VALUE || thr > CHAN_MAX_VALUE)
                    step_thr = -step_thr;

                if (ele <  CHAN_MIN_VALUE || ele > CHAN_MAX_VALUE)
                    step_ele = -step_ele;

                if (ail <  CHAN_MIN_VALUE || ail > CHAN_MAX_VALUE)
                    step_ail = -step_ail;

                if (rud <  CHAN_MIN_VALUE || rud > CHAN_MAX_VALUE)
                    step_rud = -step_rud;

                thr += step_thr;
                ele += step_ele;
                ail += step_ail;
                rud += step_rud;

                mRcvr->setRC(RFProtocol::CH_THROTTLE, thr);
                mRcvr->setRC(RFProtocol::CH_AILERON, ail);
                mRcvr->setRC(RFProtocol::CH_RUDDER, rud);
                mRcvr->setRC(RFProtocol::CH_ELEVATOR, ele);
//                LOG("T:%4d R:%4d E:%4d A:%4d %4d %4d %4d %4d\n", mRcvr->getRC(0), mRcvr->getRC(1), mRcvr->getRC(2), mRcvr->getRC(3), mRcvr->getRC(4),
//                    mRcvr->getRC(5), mRcvr->getRC(6), mRcvr->getRC(7), mRcvr->getRC(8));
                lastTS = ts;
            }
        }
#endif
    }

    if (mRFProto) {
        mRFProto->injectControls(mRcvr->getRCs(), mRcvr->getChCnt());
        mRFProto->loop(ts);
    }
}

int freeRam() {
    extern int __heap_start, *__brkval;
    int v;
    return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}
