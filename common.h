#ifndef _COMMON_H_
#define _COMMON_H_

#include <stdint.h>
#include <string.h>
#include <inttypes.h>


//Magic macro to check enum size
//#define ctassert(n,e) extern unsigned char n[(e)?0:-1]
#define ctassert(COND,MSG) typedef char static_assertion_##MSG[(COND)?1:-1]

typedef int8_t s8;
typedef int16_t s16;
typedef int32_t s32;
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;

#ifndef NULL
#define NULL    0
#endif

#ifndef TRUE
#define TRUE    1
#endif

#ifndef FALSE
#define FALSE   0
#endif


typedef enum TxPower {
    TXPOWER_100uW,
    TXPOWER_300uW,
    TXPOWER_1mW,
    TXPOWER_3mW,
    TXPOWER_10mW,
    TXPOWER_30mW,
    TXPOWER_100mW,
    TXPOWER_150mW,
    TXPOWER_LAST,
} TXPOWER_T;

enum TXRX_State {
    TXRX_OFF,
    TX_EN,
    RX_EN,
};

#endif
