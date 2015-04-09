#include <Arduino.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <SPI.h>

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

#define FW_VERSION  0x0100

static SerialProtocol  mSerial;
static u8 mBaudAckLen;
static u8 mBaudChkCtr;
static u8 mBaudAckStr[12];
static RFProtocol *mRFProto = NULL;

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
            if (mRFProto) {
                delete mRFProto;
                mRFProto = NULL;
            }
            
            id = *(u32*)data;
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

#if 0
                        case RFProtocol::PROTO_NRF24L01_CFLIE:
                            mRFProto = new RFProtocolCFlie(id);
                            break;
#endif
                        default:
                            ret = 0;
                            break;
                    }
                }
                break;

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
                    
            }
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
            if (mRFProto) {
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
        mSerial.handleRX();
        if (mRFProto)
            mRFProto->loop();
    }
}

int freeRam() {
    extern int __heap_start, *__brkval;
    int v;
    return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}
