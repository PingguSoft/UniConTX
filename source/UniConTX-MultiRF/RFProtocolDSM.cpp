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
#include "RFProtocolDSM.h"
#include "utils.h"

#define RANDOM_CHANNELS 0
#define BIND_CHANNEL    0x0d        //This can be any odd channel
#define NUM_WAIT_LOOPS  (100 / 5)   //each loop is ~5us.  Do not wait more than 100us

#define __PRINT_FUNC__  //LOG(F("%08ld : %s\n"), millis(), __PRETTY_FUNCTION__);

enum {
    PROTOOPTS_DSMX = 1,
    PROTOOPTS_TELEMETRY = 2,
    LAST_PROTO_OPT,
};

#define TELEM_ON        1
#define TELEM_OFF       0

//During binding we will send BIND_COUNT/2 packets
//One packet each 10msec

#define BIND_COUNT      600
enum {
    DSM2_BIND = 0,
    DSM2_CHANSEL     = BIND_COUNT + 0,
    DSM2_CH1_WRITE_A = BIND_COUNT + 1,
    DSM2_CH1_CHECK_A = BIND_COUNT + 2,
    DSM2_CH2_WRITE_A = BIND_COUNT + 3,
    DSM2_CH2_CHECK_A = BIND_COUNT + 4,
    DSM2_CH2_READ_A  = BIND_COUNT + 5,
    DSM2_CH1_WRITE_B = BIND_COUNT + 6,
    DSM2_CH1_CHECK_B = BIND_COUNT + 7,
    DSM2_CH2_WRITE_B = BIND_COUNT + 8,
    DSM2_CH2_CHECK_B = BIND_COUNT + 9,
    DSM2_CH2_READ_B  = BIND_COUNT + 10,
};

static const PROGMEM u8 TBL_PNCODES[5][9][8] = {
    /* Note these are in order transmitted (LSB 1st) */
    { /* Row 0 */
      /* Col 0 */ {0x03, 0xBC, 0x6E, 0x8A, 0xEF, 0xBD, 0xFE, 0xF8},
      /* Col 1 */ {0x88, 0x17, 0x13, 0x3B, 0x2D, 0xBF, 0x06, 0xD6},
      /* Col 2 */ {0xF1, 0x94, 0x30, 0x21, 0xA1, 0x1C, 0x88, 0xA9},
      /* Col 3 */ {0xD0, 0xD2, 0x8E, 0xBC, 0x82, 0x2F, 0xE3, 0xB4},
      /* Col 4 */ {0x8C, 0xFA, 0x47, 0x9B, 0x83, 0xA5, 0x66, 0xD0},
      /* Col 5 */ {0x07, 0xBD, 0x9F, 0x26, 0xC8, 0x31, 0x0F, 0xB8},
      /* Col 6 */ {0xEF, 0x03, 0x95, 0x89, 0xB4, 0x71, 0x61, 0x9D},
      /* Col 7 */ {0x40, 0xBA, 0x97, 0xD5, 0x86, 0x4F, 0xCC, 0xD1},
      /* Col 8 */ {0xD7, 0xA1, 0x54, 0xB1, 0x5E, 0x89, 0xAE, 0x86}
    },
    { /* Row 1 */
      /* Col 0 */ {0x83, 0xF7, 0xA8, 0x2D, 0x7A, 0x44, 0x64, 0xD3},
      /* Col 1 */ {0x3F, 0x2C, 0x4E, 0xAA, 0x71, 0x48, 0x7A, 0xC9},
      /* Col 2 */ {0x17, 0xFF, 0x9E, 0x21, 0x36, 0x90, 0xC7, 0x82},
      /* Col 3 */ {0xBC, 0x5D, 0x9A, 0x5B, 0xEE, 0x7F, 0x42, 0xEB},
      /* Col 4 */ {0x24, 0xF5, 0xDD, 0xF8, 0x7A, 0x77, 0x74, 0xE7},
      /* Col 5 */ {0x3D, 0x70, 0x7C, 0x94, 0xDC, 0x84, 0xAD, 0x95},
      /* Col 6 */ {0x1E, 0x6A, 0xF0, 0x37, 0x52, 0x7B, 0x11, 0xD4},
      /* Col 7 */ {0x62, 0xF5, 0x2B, 0xAA, 0xFC, 0x33, 0xBF, 0xAF},
      /* Col 8 */ {0x40, 0x56, 0x32, 0xD9, 0x0F, 0xD9, 0x5D, 0x97}
    },
    { /* Row 2 */
      /* Col 0 */ {0x40, 0x56, 0x32, 0xD9, 0x0F, 0xD9, 0x5D, 0x97},
      /* Col 1 */ {0x8E, 0x4A, 0xD0, 0xA9, 0xA7, 0xFF, 0x20, 0xCA},
      /* Col 2 */ {0x4C, 0x97, 0x9D, 0xBF, 0xB8, 0x3D, 0xB5, 0xBE},
      /* Col 3 */ {0x0C, 0x5D, 0x24, 0x30, 0x9F, 0xCA, 0x6D, 0xBD},
      /* Col 4 */ {0x50, 0x14, 0x33, 0xDE, 0xF1, 0x78, 0x95, 0xAD},
      /* Col 5 */ {0x0C, 0x3C, 0xFA, 0xF9, 0xF0, 0xF2, 0x10, 0xC9},
      /* Col 6 */ {0xF4, 0xDA, 0x06, 0xDB, 0xBF, 0x4E, 0x6F, 0xB3},
      /* Col 7 */ {0x9E, 0x08, 0xD1, 0xAE, 0x59, 0x5E, 0xE8, 0xF0},
      /* Col 8 */ {0xC0, 0x90, 0x8F, 0xBB, 0x7C, 0x8E, 0x2B, 0x8E}
    },
    { /* Row 3 */
      /* Col 0 */ {0xC0, 0x90, 0x8F, 0xBB, 0x7C, 0x8E, 0x2B, 0x8E},
      /* Col 1 */ {0x80, 0x69, 0x26, 0x80, 0x08, 0xF8, 0x49, 0xE7},
      /* Col 2 */ {0x7D, 0x2D, 0x49, 0x54, 0xD0, 0x80, 0x40, 0xC1},
      /* Col 3 */ {0xB6, 0xF2, 0xE6, 0x1B, 0x80, 0x5A, 0x36, 0xB4},
      /* Col 4 */ {0x42, 0xAE, 0x9C, 0x1C, 0xDA, 0x67, 0x05, 0xF6},
      /* Col 5 */ {0x9B, 0x75, 0xF7, 0xE0, 0x14, 0x8D, 0xB5, 0x80},
      /* Col 6 */ {0xBF, 0x54, 0x98, 0xB9, 0xB7, 0x30, 0x5A, 0x88},
      /* Col 7 */ {0x35, 0xD1, 0xFC, 0x97, 0x23, 0xD4, 0xC9, 0x88},
      /* Col 8 */ {0x88, 0xE1, 0xD6, 0x31, 0x26, 0x5F, 0xBD, 0x40}
    },
    { /* Row 4 */
      /* Col 0 */ {0xE1, 0xD6, 0x31, 0x26, 0x5F, 0xBD, 0x40, 0x93},
      /* Col 1 */ {0xDC, 0x68, 0x08, 0x99, 0x97, 0xAE, 0xAF, 0x8C},
      /* Col 2 */ {0xC3, 0x0E, 0x01, 0x16, 0x0E, 0x32, 0x06, 0xBA},
      /* Col 3 */ {0xE0, 0x83, 0x01, 0xFA, 0xAB, 0x3E, 0x8F, 0xAC},
      /* Col 4 */ {0x5C, 0xD5, 0x9C, 0xB8, 0x46, 0x9C, 0x7D, 0x84},
      /* Col 5 */ {0xF1, 0xC6, 0xFE, 0x5C, 0x9D, 0xA5, 0x4F, 0xB7},
      /* Col 6 */ {0x58, 0xB5, 0xB3, 0xDD, 0x0E, 0x28, 0xF1, 0xB0},
      /* Col 7 */ {0x5F, 0x30, 0x3B, 0x56, 0x96, 0x45, 0xF4, 0xA1},
      /* Col 8 */ {0x03, 0xBC, 0x6E, 0x8A, 0xEF, 0xBD, 0xFE, 0xF8}
    },
};

static const PROGMEM u8 TBL_PN_BIND[] = { 0xc6,0x94,0x22,0xfe,0x48,0xe6,0x57,0x4e };

// MAP order : 0123 - TAER
static const PROGMEM u8 ch_map4[] = {0, 1, 2, 3, 0xff, 0xff, 0xff};     //Guess
static const PROGMEM u8 ch_map5[] = {0, 1, 2, 3, 4,    0xff, 0xff};     //Guess
static const PROGMEM u8 ch_map6[] = {1, 5, 2, 3, 0,    4,    0xff};     //HP6DSM
static const PROGMEM u8 ch_map7[] = {1, 5, 2, 4, 3,    6,    0};        //DX6i
static const PROGMEM u8 ch_map8[] = {1, 5, 2, 3, 6,    0xff, 0xff, 4, 0, 7,    0xff, 0xff, 0xff, 0xff}; //DX8
static const PROGMEM u8 ch_map9[] = {3, 2, 1, 5, 0,    4,    6,    7, 8, 0xff, 0xff, 0xff, 0xff, 0xff}; //DM9
static const PROGMEM u8 ch_map10[] = {3, 2, 1, 5, 0,    4,    6,    7, 8, 9, 0xff, 0xff, 0xff, 0xff};
static const PROGMEM u8 ch_map11[] = {3, 2, 1, 5, 0,    4,    6,    7, 8, 9, 10, 0xff, 0xff, 0xff};
static const PROGMEM u8 ch_map12[] = {3, 2, 1, 5, 0,    4,    6,    7, 8, 9, 10, 11, 0xff, 0xff};
static const u8 *ch_map[] = {ch_map4, ch_map5, ch_map6, ch_map7, ch_map8, ch_map9, ch_map10, ch_map11, ch_map12};
static const PROGMEM u8 ch_cvt[] = { RFProtocol::CH_THROTTLE, RFProtocol::CH_AILERON, RFProtocol::CH_ELEVATOR, RFProtocol::CH_RUDDER };

void RFProtocolDSM::build_bind_packet(void)
{
    u8 i;
    u16 sum = 384 - 0x10;

    __PRINT_FUNC__;
    mPacketBuf[0] = 0xff ^ mMfgIDBuf[0];
    mPacketBuf[1] = 0xff ^ mMfgIDBuf[1];
    mPacketBuf[2] = 0xff ^ mMfgIDBuf[2];
    mPacketBuf[3] = 0xff ^ mMfgIDBuf[3];
    mPacketBuf[4] = mPacketBuf[0];
    mPacketBuf[5] = mPacketBuf[1];
    mPacketBuf[6] = mPacketBuf[2];
    mPacketBuf[7] = mPacketBuf[3];
    for(i = 0; i < 8; i++)
        sum += mPacketBuf[i];
    mPacketBuf[8] = sum >> 8;
    mPacketBuf[9] = sum & 0xff;
    mPacketBuf[10] = 0x01; //???
    mPacketBuf[11] = mChanCnt;

    if (getProtocolOpt() & PROTOOPTS_DSMX) {
        mPacketBuf[12] = (mChanCnt < 8) ? 0xa2 : 0xb2;
//        mPacketBuf[12] = mChanCnt < 8 && Model.proto_opts[PROTOOPTS_TELEMETRY] == TELEM_OFF ? 0xa2 : 0xb2;
    } else {
        mPacketBuf[12] = mChanCnt < 8 ? 0x01 : 0x02;
    }

    mPacketBuf[13] = 0x00; //???
    for(i = 8; i < 14; i++)
        sum += mPacketBuf[i];
    mPacketBuf[14] = sum >> 8;
    mPacketBuf[15] = sum & 0xff;

//    DUMP("BIND", mPacketBuf, 16);
}


void RFProtocolDSM::build_data_packet(u8 upper)
{
    u8 i;
    u8 bits;
    const u8 *chmap = ch_map[mChanCnt - 4];

    if (mIsBinding && isStickMoved(0)) {
        //Don't turn off dialog until sticks are moved
        //PROTOCOL_SetBindState(0);  //Turn off Bind dialog
        mIsBinding = 0;
//        LOG(F("BINDING FINISH\n"));
    }

    if (getProtocolOpt() & PROTOOPTS_DSMX) {
        mPacketBuf[0] = mMfgIDBuf[2];
        mPacketBuf[1] = mMfgIDBuf[3];
        bits = 11;
    } else {
        mPacketBuf[0] = (0xff ^ mMfgIDBuf[2]);
        mPacketBuf[1] = (0xff ^ mMfgIDBuf[3]);
        bits = 10;
    }

    u16 max = 1 << bits;

    for (i = 0; i < 7; i++) {
       u8  idx = pgm_read_byte(chmap + (upper * 7 + i));
       u8  ch;
       s16 value;

       if (idx == 0xff) {
           value = 0xffff;
       } else {
            if (mIsBinding) {
                value = max >> 1;
                if (idx == 0)
                    value = 1;
            } else {
                if (idx < 4)
                    ch = pgm_read_byte(ch_cvt + idx);
                else
                    ch = idx;

                value = map(getControl(ch) , CHAN_MIN_VALUE, CHAN_MAX_VALUE, 0, max - 1);
            }
            value |= ((upper && i == 0) ? 0x8000 : 0) | (idx << bits);
       }
       mPacketBuf[i*2+2] = (value >> 8) & 0xff;
       mPacketBuf[i*2+3] = (value >> 0) & 0xff;
    }
}

u8 RFProtocolDSM::get_pn_row(u8 channel)
{
    return (getProtocolOpt() & PROTOOPTS_DSMX)
           ? (channel - 2) % 5
           : channel % 5;
}

static const PROGMEM u8 TBL_INIT_VALS[][2] = {
    {CYRF_02_TX_CTRL,       0x02},
    {CYRF_05_RX_CTRL,       0x00},
    {CYRF_28_CLK_EN,        0x02},
    {CYRF_32_AUTO_CAL_TIME, 0x3c},
    {CYRF_35_AUTOCAL_OFFSET,0x14},
    {CYRF_06_RX_CFG,        0x4A},
    {CYRF_1B_TX_OFFSET_LSB, 0x55},
    {CYRF_1C_TX_OFFSET_MSB, 0x05},
    {CYRF_0F_XACT_CFG,      0x24},
    {CYRF_03_TX_CFG,        0x38 | 7},
    {CYRF_12_DATA64_THOLD,  0x0a},
    {CYRF_0F_XACT_CFG,      0x04},
    {CYRF_39_ANALOG_CTRL,   0x01},
    {CYRF_0F_XACT_CFG,      0x24},      //Force IDLE
    {CYRF_29_RX_ABORT,      0x00},      //Clear RX abort
    {CYRF_12_DATA64_THOLD,  0x0a},      //set pn correlation threshold
    {CYRF_10_FRAMING_CFG,   0x4a},      //set sop len and threshold
    {CYRF_29_RX_ABORT,      0x0f},      //Clear RX abort?
    {CYRF_03_TX_CFG,        0x38 | 7},  //Set 64chip, SDR mode, max-power
    {CYRF_10_FRAMING_CFG,   0x4a},      //set sop len and threshold
    {CYRF_1F_TX_OVERRIDE,   0x04},      //disable tx CRC
    {CYRF_1E_RX_OVERRIDE,   0x14},      //disable rx CRC
    {CYRF_14_EOP_CTRL,      0x02},      //set EOP sync == 2
    {CYRF_01_TX_LENGTH,     0x10},      //16byte packet
};

void RFProtocolDSM::cyrf_config(void)
{
    __PRINT_FUNC__;

    u8 reg, val;

    for (u8 i = 0; i < sizeof(TBL_INIT_VALS) / 2; i++) {
        reg = pgm_read_byte(&TBL_INIT_VALS[i][0]);
        val = pgm_read_byte(&TBL_INIT_VALS[i][1]);
        mDev.writeReg(reg, val);
    }
    mDev.writePreamble(0x333304);
    mDev.setRFChannel(0x61);
}

void RFProtocolDSM::initialize_bind_state(void)
{
    u8 data_code[32];

    __PRINT_FUNC__;
    mDev.setRFChannel(BIND_CHANNEL); //This seems to be random?
    u8 pn_row = get_pn_row(BIND_CHANNEL);

    //LOG(F("Ch: %d Row: %d SOP: %d Data: %d\n"), BIND_CHANNEL, pn_row, mSOPCol, mDataCol);
    mDev.setCRCSeed(mCRC);
    mDev.setSOPCode_P(TBL_PNCODES[pn_row][mSOPCol], 8);
    memcpy_P(data_code, TBL_PNCODES[pn_row][mSOPCol], 8);
    //DUMP("sop_code", data_code, 8);

    memcpy_P(data_code, TBL_PNCODES[pn_row][mDataCol], 16);
    memcpy_P(data_code + 16, TBL_PNCODES[0][8], 8);
    memcpy_P(data_code + 24, TBL_PN_BIND, 8);
    mDev.setDataCode(data_code, 32);
    //DUMP("data_code", data_code, 32);
    build_bind_packet();
}

static const PROGMEM u8 TBL_DATA_VALS[][2] = {
    {CYRF_05_RX_CTRL,       0x83},  //Initialize for reading RSSI
    {CYRF_29_RX_ABORT,      0x20},
    {CYRF_0F_XACT_CFG,      0x24},
    {CYRF_29_RX_ABORT,      0x00},
    {CYRF_03_TX_CFG,        0x08},
    {CYRF_10_FRAMING_CFG,   0xea},
    {CYRF_1F_TX_OVERRIDE,   0x00},
    {CYRF_1E_RX_OVERRIDE,   0x00},
    {CYRF_03_TX_CFG,        0x28},
    {CYRF_12_DATA64_THOLD,  0x3f},
    {CYRF_10_FRAMING_CFG,   0xff},
    {CYRF_0F_XACT_CFG,      0x24}, //Switch from reading RSSI to Writing
    {CYRF_29_RX_ABORT,      0x00},
    {CYRF_12_DATA64_THOLD,  0x0a},
    {CYRF_10_FRAMING_CFG,   0xea},
};

void RFProtocolDSM::cyrf_configdata(void)
{
    __PRINT_FUNC__;
    u8 reg, val;

    for (u8 i = 0; i < sizeof(TBL_DATA_VALS) / 2; i++) {
        reg = pgm_read_byte(&TBL_DATA_VALS[i][0]);
        val = pgm_read_byte(&TBL_DATA_VALS[i][1]);

        if (reg == CYRF_03_TX_CFG) {
            val |= getRFPower();
            mDev.writeReg(reg, val);
        } else {
            mDev.writeReg(reg, val);
        }
    }
}

void RFProtocolDSM::set_sop_data_crc(void)
{
    u8 pn_row = get_pn_row(mRFChanBufs[mChanIdx]);
    //printf("Ch: %d Row: %d SOP: %d Data: %d\n", ch[mChanIdx], pn_row, mSOPCol, mDataCol);
    mDev.setRFChannel(mRFChanBufs[mChanIdx]);
    mDev.setCRCSeed(mCRC);
    mCRC = ~mCRC;

    mDev.setSOPCode_P(TBL_PNCODES[pn_row][mSOPCol], 8);
    mDev.setDataCode_P(TBL_PNCODES[pn_row][mDataCol], 16);

    /* setup for next iteration */
    if (getProtocolOpt() & PROTOOPTS_DSMX)
        mChanIdx = (mChanIdx + 1) % 23;
    else
        mChanIdx = (mChanIdx + 1) % 2;
}

void RFProtocolDSM::calc_dsmx_channel(void)
{
    __PRINT_FUNC__;
    int idx = 0;
    u32 id = ~((mMfgIDBuf[0] << 24) | (mMfgIDBuf[1] << 16) | (mMfgIDBuf[2] << 8) | (mMfgIDBuf[3] << 0));
    u32 id_tmp = id;

    while(idx < 23) {
        int i;
        int count_3_27 = 0, count_28_51 = 0, count_52_76 = 0;
        id_tmp = id_tmp * 0x0019660D + 0x3C6EF35F;      // Randomization
        u8 next_ch = ((id_tmp >> 8) % 0x49) + 3;        // Use least-significant byte and must be larger than 3

        if (((next_ch ^ id) & 0x01 )== 0)
            continue;
        for (i = 0; i < idx; i++) {
            if(mRFChanBufs[i] == next_ch)
                break;
            if(mRFChanBufs[i] <= 27)
                count_3_27++;
            else if (mRFChanBufs[i] <= 51)
                count_28_51++;
            else
                count_52_76++;
        }
        if (i != idx)
            continue;
        if ((next_ch < 28 && count_3_27 < 8)
          ||(next_ch >= 28 && next_ch < 52 && count_28_51 < 7)
          ||(next_ch >= 52 && count_52_76 < 8))
        {
            mRFChanBufs[idx++] = next_ch;
        }
    }
}


#if 0
u32 RFProtocolDSM::bcd_to_int(u32 data)
{
    u32 value = 0, multi = 1;
    while (data) {
        value += (data & 15U) * multi;
        multi *= 10;
        data >>= 4;
    }
    return value;
}

int RFProtocolDSM::pkt32_to_coord(u8 *ptr)
{
    // (decimal, format DD MM.MMMM)
    return bcd_to_int(ptr[3]) * 3600000
         + bcd_to_int(((u32)ptr[2] << 16) | ((u32)ptr[1] << 8) | ptr[0]) * 6;
}

NO_INLINE static void parse_telemetry_packet()
{
    static u8 altitude; // byte from first GPS mPacketBuf
#if HAS_DSM_EXTENDED_TELEMETRY
    static const u8 update0a[] = { TELEM_DSM_PBOX_VOLT1, TELEM_DSM_PBOX_VOLT2,
                                   TELEM_DSM_PBOX_CAPACITY1, TELEM_DSM_PBOX_CAPACITY2,
                                   TELEM_DSM_PBOX_ALARMV1, TELEM_DSM_PBOX_ALARMV2,
                                   TELEM_DSM_PBOX_ALARMC1, TELEM_DSM_PBOX_ALARMC2, 0};
    static const u8 update15[] = { TELEM_DSM_JETCAT_STATUS, TELEM_DSM_JETCAT_THROTTLE,
                                   TELEM_DSM_JETCAT_PACKVOLT, TELEM_DSM_JETCAT_PUMPVOLT,
                                   TELEM_DSM_JETCAT_RPM, TELEM_DSM_JETCAT_TEMPEGT,
                                   TELEM_DSM_JETCAT_OFFCOND, 0};
    static const u8 update20[] = { TELEM_DSM_HYPOTHETIC_RPM, TELEM_DSM_HYPOTHETIC_VOLT1,
                                   TELEM_DSM_HYPOTHETIC_TEMP1, TELEM_DSM_HYPOTHETIC_AMPS1,
                                   TELEM_DSM_HYPOTHETIC_TEMP2, TELEM_DSM_HYPOTHETIC_AMPS2,
                                   TELEM_DSM_HYPOTHETIC_VOLT2, TELEM_DSM_HYPOTHETIC_THROTTLE,
                                   TELEM_DSM_HYPOTHETIC_OUTPUT, 0};
    static const u8 update18[] = { TELEM_DSM_RXPCAP_AMPS, TELEM_DSM_RXPCAP_CAPACITY, TELEM_DSM_RXPCAP_VOLT, 0};
    static const u8 update34[] = { TELEM_DSM_FPCAP_AMPS, TELEM_DSM_FPCAP_CAPACITY, TELEM_DSM_FPCAP_TEMP, 0};
#endif
    static const u8 update16[] = { TELEM_GPS_ALT, TELEM_GPS_LAT, TELEM_GPS_LONG, TELEM_GPS_HEADING, 0};
    static const u8 update17[] = { TELEM_GPS_SPEED, TELEM_GPS_TIME, TELEM_GPS_SATCOUNT, 0};
    static const u8 update7f[] = { TELEM_DSM_FLOG_FADESA, TELEM_DSM_FLOG_FADESB,
                                   TELEM_DSM_FLOG_FADESL, TELEM_DSM_FLOG_FADESR,
                                   TELEM_DSM_FLOG_FRAMELOSS, TELEM_DSM_FLOG_HOLDS,
                                   TELEM_DSM_FLOG_VOLT1, 0};
    static const u8 update7e[] = { TELEM_DSM_FLOG_RPM1, TELEM_DSM_FLOG_VOLT2, TELEM_DSM_FLOG_TEMP1, 0};
    static const u8 update03[] = { TELEM_DSM_AMPS1, 0};
    static const u8 update11[] = { TELEM_DSM_AIRSPEED, 0};
    static const u8 update12[] = { TELEM_DSM_ALTITUDE, TELEM_DSM_ALTITUDE_MAX, 0};
    static const u8 update14[] = { TELEM_DSM_GFORCE_X, TELEM_DSM_GFORCE_Y, TELEM_DSM_GFORCE_Z,
                                   TELEM_DSM_GFORCE_XMAX, TELEM_DSM_GFORCE_YMAX, TELEM_DSM_GFORCE_ZMAX,
                                   TELEM_DSM_GFORCE_ZMIN, 0};
    static const u8 update40[] = { TELEM_DSM_VARIO_ALTITUDE, TELEM_DSM_VARIO_CLIMBRATE1,
                                   TELEM_DSM_VARIO_CLIMBRATE2, TELEM_DSM_VARIO_CLIMBRATE3,
                                   TELEM_DSM_VARIO_CLIMBRATE4, TELEM_DSM_VARIO_CLIMBRATE5,
                                   TELEM_DSM_VARIO_CLIMBRATE6, 0};
    const u8 *update = &update7f[7];
    unsigned idx = 0;

#define data_type  mPacketBuf[0]
#define end_byte   mPacketBuf[15]
#define LSB_1st    ((data_type >= 0x15 && data_type <= 0x18) || (data_type == 0x34))

    // Convert 8bit mPacketBuf into 16bit equivalent
    static u16 pktTelem[8];
    if (LSB_1st) {
        for(u8 i=1; i < 8; ++i) {
            pktTelem[i] = (mPacketBuf[i*2+1] <<8) | mPacketBuf[i*2];
        }
    } else {
        for(u8 i=1; i < 8; ++i) {
            pktTelem[i] = (mPacketBuf[i*2] <<8) | mPacketBuf[i*2+1];
        }
    }
    switch(data_type) {
        case 0x7f: //TM1000 Flight log
        case 0xff: //TM1100 Flight log
            update = update7f;
            break;
        case 0x7e: //TM1000
        case 0xfe: //TM1100
            update = update7e;
            break;
        case 0x03: //High Current sensor
            update = update03;
            break;
        case 0x11: //AirSpeed sensor
            update = update11;
            break;
        case 0x12: //Altimeter sensor
            update = update12;
            break;
        case 0x14: //G-Force sensor
            update = update14;
            break;
#if HAS_DSM_EXTENDED_TELEMETRY
        case 0x18: //RX Pack Cap sensor (SPMA9604)
            update = update18;
            break;
        case 0x34: //Flight Pack Cap sensor (SPMA9605)
            update = update34;
            break;
#endif
        case 0x40: //Variometer sensor (SPMA9589)
            update = update40;
            break;
    }
    if (*update) {
        while (*update) {
            Telemetry.value[*update] = pktTelem[++idx];
            if (pktTelem[idx] != 0xffff)
                TELEMETRY_SetUpdated(*update);
            update++;
        }
        return;
    }
    switch(data_type) {
#if HAS_DSM_EXTENDED_TELEMETRY
        case 0x0a: //Powerbox sensor
            update = update0a;
            Telemetry.value[TELEM_DSM_PBOX_VOLT1] = pktTelem[1]; //In 1/100 of Volts
            Telemetry.value[TELEM_DSM_PBOX_VOLT2] = pktTelem[2]; //In 1/100 of Volts
            Telemetry.value[TELEM_DSM_PBOX_CAPACITY1] = pktTelem[3]; //In mAh
            Telemetry.value[TELEM_DSM_PBOX_CAPACITY2] = pktTelem[4]; //In mAh
            Telemetry.value[TELEM_DSM_PBOX_ALARMV1] = end_byte & 0x01; //0 = disable, 1 = enable
            Telemetry.value[TELEM_DSM_PBOX_ALARMV2] = end_byte & 0x02; //0 = disable, 1 = enable
            Telemetry.value[TELEM_DSM_PBOX_ALARMC1] = end_byte & 0x04; //0 = disable, 1 = enable
            Telemetry.value[TELEM_DSM_PBOX_ALARMC2] = end_byte & 0x08; //0 = disable, 1 = enable
            break;
        case 0x15: //JetCat sensor
            update = update15;
            Telemetry.value[TELEM_DSM_JETCAT_STATUS] = mPacketBuf[2];
            Telemetry.value[TELEM_DSM_JETCAT_THROTTLE] = bcd_to_int(mPacketBuf[3]); //up to 159% (the upper nibble is 0-f, the lower nibble 0-9)
            Telemetry.value[TELEM_DSM_JETCAT_PACKVOLT] = bcd_to_int(pktTelem[2]); //In 1/100 of Volts
            Telemetry.value[TELEM_DSM_JETCAT_PUMPVOLT] = bcd_to_int(pktTelem[3]); //In 1/100 of Volts (low voltage)
            Telemetry.value[TELEM_DSM_JETCAT_RPM] =      bcd_to_int(pktTelem[4] & 0x0fff); //RPM up to 999999
            Telemetry.value[TELEM_DSM_JETCAT_TEMPEGT] =  bcd_to_int(pktTelem[6]); //EGT temp up to 999°C
            Telemetry.value[TELEM_DSM_JETCAT_OFFCOND] = end_byte;
            break;
        case 0x20: //Hypothetic sensor
            update = update20;
            Telemetry.value[TELEM_DSM_HYPOTHETIC_RPM] =   pktTelem[1]; //In 10 rpm
            Telemetry.value[TELEM_DSM_HYPOTHETIC_VOLT1] = pktTelem[2]; //Batt in 1/100 of Volts (Volt2)
            Telemetry.value[TELEM_DSM_HYPOTHETIC_TEMP1] = pktTelem[3]; //FET Temp in 1/10 of degree (Fahrenheit???)
            Telemetry.value[TELEM_DSM_HYPOTHETIC_AMPS1] = pktTelem[4]; //In 1/100 Amp
            Telemetry.value[TELEM_DSM_HYPOTHETIC_TEMP2] = pktTelem[5]; //BEC Temp in 1/10 of degree (Fahrenheit???)
            Telemetry.value[TELEM_DSM_HYPOTHETIC_AMPS2] = mPacketBuf[12];     //BEC current in 1/10 Amp
            Telemetry.value[TELEM_DSM_HYPOTHETIC_VOLT2] = mPacketBuf[13] * 5; //BEC voltage in 1/100 of Volt
            Telemetry.value[TELEM_DSM_HYPOTHETIC_THROTTLE] = mPacketBuf[14]* 5; //Throttle % in 0.1%
            Telemetry.value[TELEM_DSM_HYPOTHETIC_OUTPUT] = end_byte * 5; //Output % in 0.1%
            break;
#endif //HAS_DSM_EXTENDED_TELEMETRY
        case 0x16: //GPS sensor (always second GPS mPacketBuf)
            update = update16;
            Telemetry.gps.altitude  = bcd_to_int((altitude << 24) | ((u32)pktTelem[1] << 8)); //In m * 1000 (16Bit decimal, 1 unit is 0.1m)
            Telemetry.gps.latitude  =  pkt32_to_coord(&mPacketBuf[4]) * ((end_byte & 0x01)? 1: -1); //1=N(+), 0=S(-)
            Telemetry.gps.longitude = (pkt32_to_coord(&mPacketBuf[8]) + ((end_byte & 0x04)? 360000000: 0)) //1=+100 degrees
                                                                  * ((end_byte & 0x02)? 1: -1); //1=E(+), 0=W(-)
            Telemetry.gps.heading = bcd_to_int(pktTelem[6]); //In degrees (16Bit decimal, 1 unit is 0.1 degree)
            break;
        case 0x17: //GPS sensor (always first GPS mPacketBuf)
            update = update17;
            Telemetry.gps.velocity = bcd_to_int(pktTelem[1]) * 5556 / 108; //In m/s * 1000
            //u8 ssec  = bcd_to_int(mPacketBuf[4]);
            u8 sec   = bcd_to_int(mPacketBuf[5]);
            u8 min   = bcd_to_int(mPacketBuf[6]);
            u8 hour  = bcd_to_int(mPacketBuf[7]);
            u8 day   = 0;
            u8 month = 0;
            u8 year  = 0; // + 2000
            Telemetry.gps.time = ((year & 0x3F) << 26)
                               | ((month & 0x0F) << 22)
                               | ((day & 0x1F) << 17)
                               | ((hour & 0x1F) << 12)
                               | ((min & 0x3F) << 6)
                               | ((sec & 0x3F) << 0);
            Telemetry.gps.satcount = bcd_to_int(mPacketBuf[8]);
            altitude = mPacketBuf[9];
            break;
    }
    idx = 0;
    while (*update) {
        if (pktTelem[++idx] != 0xffff)
            TELEMETRY_SetUpdated(*update);
        update++;
    }
}
#endif


u16 RFProtocolDSM::dsm2_cb(void)
{
#define CH1_CH2_DELAY 4010  // Time between write of channel 1 and channel 2
//#define CH1_CH2_DELAY 3000  // Time between write of channel 1 and channel 2
#define WRITE_DELAY   1550  // Time after write to verify write complete
//#define WRITE_DELAY   1000  // Time after write to verify write complete
#define READ_DELAY     400  // Time before write to check read state, and switch channel

    if(mState < DSM2_CHANSEL) {
        //Binding
        mState++;
        if(mState & 1) {
            //Send packet on even states
            //Note mState has already incremented,
            // so this is actually 'even' mState
            mDev.writePayload(mPacketBuf, MAX_PACKET_SIZE);
            return 8500;
        } else {
            u8  irq;
            int i = 0;

            do {
                irq = mDev.readReg(CYRF_04_TX_IRQ_STATUS);
                if(++i > NUM_WAIT_LOOPS) {
                    LOG(F("MAX WAIT IRQ : %x\n"), irq);
                    break;
                }
            } while (!(irq & 0x02));
            return 1500;
        }
    } else if(mState < DSM2_CH1_WRITE_A) {
        //Select mRFChanBufs and configure for writing data
        //CYRF_FindBestChannels(ch, 2, 10, 1, 79);
        cyrf_configdata();
        mDev.setRFMode(RF_TX);
        mChanIdx = 0;
        mState = DSM2_CH1_WRITE_A;
        set_sop_data_crc();
        return 10000;
    } else if(mState == DSM2_CH1_WRITE_A || mState == DSM2_CH1_WRITE_B
           || mState == DSM2_CH2_WRITE_A || mState == DSM2_CH2_WRITE_B)
    {
        if (mState == DSM2_CH1_WRITE_A || mState == DSM2_CH1_WRITE_B)
            build_data_packet(mState == DSM2_CH1_WRITE_B);
        mDev.writePayload(mPacketBuf, MAX_PACKET_SIZE);
        mState++;
        return WRITE_DELAY;
    } else if(mState == DSM2_CH1_CHECK_A || mState == DSM2_CH1_CHECK_B) {
        int i = 0;
        while (! (mDev.readReg(CYRF_04_TX_IRQ_STATUS) & 0x02)) {
            if(++i > NUM_WAIT_LOOPS)
                break;
        }
        set_sop_data_crc();
        mState++;
        return CH1_CH2_DELAY - WRITE_DELAY;
    } else if(mState == DSM2_CH2_CHECK_A || mState == DSM2_CH2_CHECK_B) {
        int i = 0;
        while (! (mDev.readReg(CYRF_04_TX_IRQ_STATUS) & 0x02)) {
            if(++i > NUM_WAIT_LOOPS)
                break;
        }
        if (mState == DSM2_CH2_CHECK_A) {
            //Keep transmit power in sync
            mDev.writeReg(CYRF_03_TX_CFG, 0x28 | getRFPower());
        }
        if ((getProtocolOpt() & PROTOOPTS_TELEMETRY) == TELEM_OFF) {
            set_sop_data_crc();
            if (mState == DSM2_CH2_CHECK_A) {
                if(mChanCnt < 8) {
                    mState = DSM2_CH1_WRITE_A;
                    return 22000 - CH1_CH2_DELAY - WRITE_DELAY;
                }
                mState = DSM2_CH1_WRITE_B;
            } else {
                mState = DSM2_CH1_WRITE_A;
            }
            return 11000 - CH1_CH2_DELAY - WRITE_DELAY;
        } else {
            mState++;
            mDev.setRFMode(RF_RX); //Receive mode
            mDev.writeReg(CYRF_05_RX_CTRL, 0x80); //Prepare to receive
            return 11000 - CH1_CH2_DELAY - WRITE_DELAY - READ_DELAY;
        }
    } else if(mState == DSM2_CH2_READ_A || mState == DSM2_CH2_READ_B) {
        //Read telemetry if needed
        u8 rx_state = mDev.readReg(CYRF_07_RX_IRQ_STATUS);
        if((rx_state & 0x03) == 0x02) {  // RXC=1, RXE=0 then 2nd check is required (debouncing)
            rx_state |= mDev.readReg(CYRF_07_RX_IRQ_STATUS);
        }
        if((rx_state & 0x07) == 0x02) { // good data (complete with no errors)
            mDev.writeReg(CYRF_07_RX_IRQ_STATUS, 0x80); // need to set RXOW before data read
            mDev.writePayload(mPacketBuf, mDev.readReg(CYRF_09_RX_COUNT));
            //parse_telemetry_packet();
        }
        if (mState == DSM2_CH2_READ_A && mChanCnt < 8) {
            mState = DSM2_CH2_READ_B;
            //Reseat RX mode just in case any error
            mDev.writeReg(CYRF_0F_XACT_CFG, (mDev.readReg(CYRF_0F_XACT_CFG) | 0x20));  // Force end mState
            int i = 0;
            while (mDev.readReg(CYRF_0F_XACT_CFG) & 0x20) {
                if(++i > NUM_WAIT_LOOPS)
                    break;
            }
            mDev.writeReg(CYRF_05_RX_CTRL, 0x80); //Prepare to receive
            return 11000;
        }
        if (mState == DSM2_CH2_READ_A)
            mState = DSM2_CH1_WRITE_B;
        else
            mState = DSM2_CH1_WRITE_A;
        mDev.setRFMode(RF_TX); //Write mode
        set_sop_data_crc();
        return READ_DELAY;
    }
    return 0;
}

void RFProtocolDSM::initialize(u8 bind)
{
    cyrf_config();

    if (getProtocolOpt() & PROTOOPTS_DSMX) {
        calc_dsmx_channel();
    } else {
        if (RANDOM_CHANNELS) {
            u8 tmpch[10];

            mDev.findBestChannels(tmpch, 10, 5, 3, 75);
            u8 idx = rand32() % 10;
            mRFChanBufs[0] = tmpch[idx];

            while (1) {
               idx = rand32() % 10;
               if (tmpch[idx] != mRFChanBufs[0])
                   break;
            }
            mRFChanBufs[1] = tmpch[idx];
        } else {
            mRFChanBufs[0] = (mMfgIDBuf[0] + mMfgIDBuf[2] + mMfgIDBuf[4]
                          + ((mFixedID >> 0) & 0xff) + ((mFixedID >> 16) & 0xff)) % 39 + 1;
            mRFChanBufs[1] = (mMfgIDBuf[1] + mMfgIDBuf[3] + mMfgIDBuf[5]
                          + ((mFixedID >> 8) & 0xff) + ((mFixedID >> 8) & 0xff)) % 40 + 40;
        }
    }
    /*
    mRFChanBufs[0] = 0;
    mRFChanBufs[1] = 0;
    if (mFixedID == 0)
        mFixedID = 0x2b9d2952;
    mMfgIDBuf[0] = 0xff ^ ((mFixedID >> 24) & 0xff);
    mMfgIDBuf[1] = 0xff ^ ((mFixedID >> 16) & 0xff);
    mMfgIDBuf[2] = 0xff ^ ((mFixedID >> 8) & 0xff);
    mMfgIDBuf[3] = 0xff ^ ((mFixedID >> 0) & 0xff);
    printf("DSM2 Channels: %02x %02x\n", mRFChanBufs[0], mRFChanBufs[1]);
    */

    mCRC     = ~((mMfgIDBuf[0] << 8) + mMfgIDBuf[1]);
    mSOPCol  = (mMfgIDBuf[0] + mMfgIDBuf[1] + mMfgIDBuf[2] + 2) & 0x07;
    mDataCol = 7 - mSOPCol;
    mChanCnt = 7;
    if (mChanCnt < 6)
        mChanCnt = 6;
    else if (mChanCnt > 12)
        mChanCnt = 12;

    //memset(&Telemetry, 0, sizeof(Telemetry));
    //TELEMETRY_SetType(TELEM_DSM);

    mDev.setRFMode(RF_TX);
    if (bind) {
        mState = DSM2_BIND;
        initialize_bind_state();
        mIsBinding = 1;
    } else {
        mState = DSM2_CHANSEL;
        mIsBinding = 0;
    }
}

u16 RFProtocolDSM::callState(void)
{
    return dsm2_cb();
}

int RFProtocolDSM::init(void)
{
    mChanIdx  = 0;
    mPacketCtr   = 0;
    mFixedID     = 0;

    isStickMoved(1);

    /* Initialise CYRF chip */
    mDev.initialize();
    mDev.reset();
    mDev.readMfgID(mMfgIDBuf);

    initialize(1);
    startState(10000);

    return 0;
}

int RFProtocolDSM::close(void)
{
    RFProtocol::close();
    mDev.initialize();
    return (mDev.reset() ? 1L : -1L);
}

int RFProtocolDSM::reset(void)
{
    return close();
}

int RFProtocolDSM::getInfo(s8 id, u8 *data)
{
    u8 size;

    size = RFProtocol::getInfo(id, data);
    if (size == 0) {
        switch (id) {
            case INFO_STATE:
                *data = mState;
                size = 1;
                break;

            case INFO_CHANNEL:
                *data = mRFChanBufs[0];
                size = 1;
                break;

            case INFO_PACKET_CTR:
                size = sizeof(mPacketCtr);
                *((u32*)data) = mPacketCtr;
                break;
        }
    }
    return size;
}

