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

#ifndef _PROTOCOL_H_
#define _PROTOCOL_H_

#include "Common.h"
#include "utils.h"
#include "Timer.h"
#include "DeviceCommon.h"

class RFProtocol : public Timer
{
public:
    enum {
        RCVR_MSP,
        RCVR_PWM,
        RCVR_PPM
    };

    enum {
        TX_NRF24L01,
        TX_A7105,
        TX_CYRF6936,
    };

    enum {
        PROTO_NRF24L01_V2x2,
        PROTO_NRF24L01_HISKY,
        PROTO_NRF24L01_YD717,
        PROTO_NRF24L01_SYMAX,
        PROTO_NRF24L01_CFLIE,
    };

    enum {
        PROTO_A7105_FLYSKY,
        PROTO_A7105_HUBSAN,
    };

    enum {
        PROTO_CYRF6936_DEVO,
    };

    enum {
        CH_THROTTLE = 0,
        CH_RUDDER,
        CH_ELEVATOR,
        CH_AILERON,
        CH_AUX1,
        CH_AUX2,
        CH_AUX3,
        CH_AUX4,
        CH_AUX5,
        CH_AUX6,
        CH_AUX7,
        CH_AUX8,
        MAX_CHANNEL
    };

    enum {
        TRIM_RUDDER,
        TRIM_ELEVATOR,
        TRIM_AILERON,
        MAX_TRIM
    };

    enum {
        INFO_STATE,
        INFO_CHANNEL,
        INFO_PACKET_CTR,
        INFO_ID,
        INFO_RF_POWER,
    };

    // utility functions
    static u32   buildID(u8 module, u8 proto, u8 option)  { return ((u32)module << 16 | (u32)proto << 8 | option); }
    static u8    getRcvr(u32 id)        { return (id >> 20) & 0x0f; }
    static u8    getModule(u32 id)      { return (id >> 16) & 0x0f; }
    static u8    getProtocol(u32 id)    { return (id >> 8) & 0xff;  }
    static u8    getProtocolOpt(u32 id) { return id & 0xff;         }


    RFProtocol(u32 id);
    virtual ~RFProtocol();

    u32  getProtoID(void)           { return mProtoID; }
    u8   getRcvr(void)              { return (mProtoID >> 20) & 0x0f; }
    u8   getModule(void)            { return (mProtoID >> 16) & 0x0f; }
    u8   getProtocol(void)          { return (mProtoID >> 8) & 0xff;  }
    u8   getProtocolOpt(void)       { return mProtoID & 0xff; }
    void setControllerID(u32 id)    { mConID = id;     }
    u32  getControllerID()          { return mConID;   }

    void injectControl(u8 ch, s16 val);
    void injectControls(s16 *data, int size);
    s16  getControl(u8 ch);         // TREA order
    s16  getControlByOrder(u8 ch);  // AETR order : deviation order

    // power
    u8   getRFPower(void);
    bool isRFPowerUpdated(void);
    void clearRFPowerUpdated(void);


    void startState(unsigned long period);


    // for timer
    virtual void handleTimer(s8 id);

    // for protocol
    virtual void loop(void);
    virtual int  init(void);
    virtual int  close(void);
    virtual int  reset(void);
    virtual int  setRFPower(u8 power);
    virtual int  getInfo(s8 id, u8 *data);
    virtual u16  callState(void) = 0;

private:
    void initVars();

    u32  mProtoID;
    u32  mConID;
    s16  mBufControls[MAX_CHANNEL];
    s8   mTmrState;
    u8   mTXPower;
};

#endif
