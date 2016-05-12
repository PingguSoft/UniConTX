#ifndef _TELEMETRY_H_
#define _TELEMETRY_H_

#include "Common.h"
#include "Utils.h"

class Telemetry
{
public:
    struct gps {
        s32 latitude;
        s32 longitude;
        s32 altitude;
        u32 velocity;
        u32 time;
        u16 heading;
        u8  satcount;
    };

    Telemetry()  {}
    ~Telemetry() {}

    inline void setVolt(u8 idx, u8 val)    { mVolt[idx] = val; }
    inline u8   getVolt(u8 idx)            { return mVolt[idx];}

    inline void setTemp(u8 idx, u8 val)    { mTemp[idx] = val; }
    inline u8   getTemp(u8 idx)            { return mTemp[idx];}

    inline void setRPM(u8 idx, u16 val)    { mRPM[idx] = val;  }
    inline u16   getRPM(u8 idx)            { return mRPM[idx]; }

    inline void setRSSI(u8 val)            { mRSSI = val;      }
    inline u8   getRSSI(void)              { return mRSSI;     }

    inline void  setGPS(struct gps *g)     { mGPS = *g;        }
    inline struct gps getGPS(void)         { return mGPS;      }

    void showInfo(void)                    {
        LOG(F("VOLT - V1:%d, V2:%d, V3:%d\n"), mVolt[0], mVolt[1], mVolt[2]);
        LOG(F("TEMP - T1:%d, T2:%d, T3:%d, T4:%d\n"), mTemp[0], mTemp[1], mTemp[2], mTemp[3]);
        LOG(F("RPM  - R1:%d, R2:%d, R3:%d\n"), mRPM[0], mRPM[1], mRPM[2]);
    }

private:
    u8  mVolt[3];
    u8  mTemp[4];
    u16 mRPM[3];
    u8  mRSSI;
    struct gps mGPS;
};
#endif
