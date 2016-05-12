/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 see <http://www.gnu.org/licenses/>
*/

// For Arduino 1.0 and earlier
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "SerialProtocol.h"
#include "utils.h"

#define MAX_BUF_SIZE 64

struct ringBuf {
    u8 buffer[MAX_BUF_SIZE];
    u8 head;
    u8 tail;
};

struct ringBuf mRxRingBuf = { {0}, 0, 0 };
struct ringBuf mTxRingBuf = { {0}, 0, 0 };
static u8 chkSumTX;

static void putChar(struct ringBuf *buf, u8 data)
{
    u8 head = buf->head;

    buf->buffer[head] = data;
    if (++head >= MAX_BUF_SIZE)
        head = 0;
    buf->head = head;
}

static u8 getChar(struct ringBuf *buf)
{
    u8 tail = buf->tail;
    u8 ch   = buf->buffer[tail];
    if (buf->head != tail) {
        if (++tail >= MAX_BUF_SIZE)
            tail = 0;
        buf->tail = tail;
    }
    return ch;
}

static void putChar2TX(u8 data)
{
    chkSumTX ^= data;
    putChar(&mTxRingBuf, data);
}

static __inline void flushTX(void)
{
    UCSR0B |= (1<<UDRIE0);
}

static u8 sAvailable(struct ringBuf *buf)
{
    return ((u8)(buf->head - buf->tail)) % MAX_BUF_SIZE;
}

#if !__STD_SERIAL__
ISR(USART_RX_vect)
{
    putChar(&mRxRingBuf, UDR0);
}

ISR(USART_UDRE_vect)
{
    struct ringBuf *buf = &mTxRingBuf;

    u8 tail = buf->tail;
    if (buf->head != tail) {
        UDR0 = buf->buffer[tail];
        if (++tail >= MAX_BUF_SIZE)
            tail = 0;
        buf->tail = tail;
    }

    // disable transmitter UDRE interrupt
    if (tail == buf->head)
        UCSR0B &= ~(1<<UDRIE0);
}
#endif

SerialProtocol::SerialProtocol()
{
}

SerialProtocol::~SerialProtocol()
{
    UCSR0B &= ~((1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0)|(1<<UDRIE0));
}

void SerialProtocol::begin(u32 baud)
{
    u8 h = ((F_CPU  / 4 / baud -1) / 2) >> 8;
    u8 l = ((F_CPU  / 4 / baud -1) / 2);

    cli();
    UCSR0B = 0;
    memset(&mRxRingBuf, 0, sizeof(mRxRingBuf));
    memset(&mTxRingBuf, 0, sizeof(mTxRingBuf));

    u8 data;
    for (u8 i = 0; i < 32; i++)
        data = UDR0;

    UCSR0A = (1<<U2X0);
    UBRR0H = h;
    UBRR0L = l;
    UCSR0B |= (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0);
    UCSR0C = (1<<UCSZ00) | (1<<UCSZ01);
    sei();
}

void SerialProtocol::clearTX(void)
{
    cli();
    UCSR0B &= ~(1<<UDRIE0);
    memset(&mTxRingBuf, 0, sizeof(mTxRingBuf));
    sei();
}

void SerialProtocol::clearRX(void)
{
    cli();
    memset(&mRxRingBuf, 0, sizeof(mRxRingBuf));
    sei();
}

void SerialProtocol::printf(const __FlashStringHelper *fmt, ...)
{
    char buf[128]; // resulting string limited to 128 chars

    va_list args;
    va_start (args, fmt);

#ifdef __AVR__
    vsnprintf_P(buf, sizeof(buf), (const char *)fmt, args); // progmem for AVR
#else
    vsnprintf(buf, sizeof(buf), (const char *)fmt, args); // for the rest of the world
#endif
    va_end(args);

    for (u8 i = 0; i < strlen(buf); i++)
        putChar(&mTxRingBuf, buf[i]);
    flushTX();
}

void SerialProtocol::printf(char *fmt, ...)
{
    char buf[128]; // resulting string limited to 128 chars
    va_list args;

    va_start (args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    for (u8 i = 0; i < strlen(buf); i++)
        putChar(&mTxRingBuf, buf[i]);
    flushTX();
}

void SerialProtocol::dumpHex(char *name, u8 *data, u16 cnt)
{
    u8  i;
    u8  b;
    u16 addr = 0;

    LOG("-- %s buf size : %d -- \n", name, cnt);
    while (cnt) {
        LOG("%08x - ", addr);

        for (i = 0; (i < 16) && (i < cnt); i ++) {
            b = *(data + i);
            LOG("%02x ", b);
        }

        LOG(" : ");
        for (i = 0; (i < 16) && (i < cnt); i ++) {
            b = *(data + i);
            if ((b > 0x1f) && (b < 0x7f))
                LOG("%c", b);
            else
                LOG(".");
        }
        LOG("\n");
        data += i;
        addr += i;
        cnt  -= i;
    }
}

u8 SerialProtocol::getString(u8 *buf)
{
    u8 size = sAvailable(&mRxRingBuf);

    for (u8 i = 0; i < size; i++)
        *buf++ = getChar(&mRxRingBuf);

    return size;
}

u8 SerialProtocol::available(void)
{
    return sAvailable(&mRxRingBuf);
}

u8 SerialProtocol::read(void)
{
    return getChar(&mRxRingBuf);
}

void SerialProtocol::setCallback(u32 (*callback)(u8 cmd, u8 *data, u8 size))
{
    mCallback = callback;
}

void SerialProtocol::sendResponse(bool ok, u8 cmd, u8 *data, u8 size)
{
    putChar2TX('$');
    putChar2TX('M');
    putChar2TX((ok ? '>' : '!'));
    chkSumTX = 0;
    putChar2TX(size);
    putChar2TX(cmd);
    for (u8 i = 0; i < size; i++)
        putChar2TX(*data++);
    putChar2TX(chkSumTX);
    flushTX();
}

void SerialProtocol::evalCommand(u8 cmd, u8 *data, u8 size)
{
    static u8 batt = 0;

    switch (cmd) {
        case CMD_TEST:
            u8 buf[7];
            buf[0] = batt++;
            sendResponse(true, cmd, buf, 7);
            break;

        default:
            if (mCallback)
                (*mCallback)(cmd, data, size);
            break;
    }
}

void SerialProtocol::handleRX(void)
{
    u8 rxSize = sAvailable(&mRxRingBuf);

    if (rxSize == 0)
        return;

    while (rxSize--) {
        u8 ch = getChar(&mRxRingBuf);

        switch (mState) {
            case STATE_IDLE:
                if (ch == '$')
                    mState = STATE_HEADER_START;
                break;

            case STATE_HEADER_START:
                mState = (ch == 'M') ? STATE_HEADER_M : STATE_IDLE;
                break;

            case STATE_HEADER_M:
                mState = (ch == '<') ? STATE_HEADER_ARROW : STATE_IDLE;
                break;

            case STATE_HEADER_ARROW:
                if (ch > MAX_PACKET_SIZE) { // now we are expecting the payload size
                    mState = STATE_IDLE;
                    continue;
                }
                mDataSize = ch;
                mCheckSum = ch;
                mOffset   = 0;
                mState    = STATE_HEADER_SIZE;
                break;

            case STATE_HEADER_SIZE:
                mCmd       = ch;
                mCheckSum ^= ch;
                mState     = STATE_HEADER_CMD;
                break;

            case STATE_HEADER_CMD:
                if (mOffset < mDataSize) {
                    mCheckSum           ^= ch;
                    mRxPacket[mOffset++] = ch;
                } else {
                    if (mCheckSum == ch)
                        evalCommand(mCmd, mRxPacket, mDataSize);
                    mState = STATE_IDLE;
                    rxSize = 0;             // no more than one command per cycle
                }
                break;
        }
    }
}
