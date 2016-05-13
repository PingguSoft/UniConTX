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

static __inline void putChar2TX(u8 data)
{
    putChar(&mTxRingBuf, data);
}

static __inline void flushTX(void)
{
    UCSR0B |= BV(UDRIE0);
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
        UCSR0B &= ~BV(UDRIE0);
}
#endif

SerialProtocol::SerialProtocol()
{
}

SerialProtocol::~SerialProtocol()
{
    UCSR0B &= ~(BV(RXEN0) | BV(TXEN0) | BV(RXCIE0) | BV(UDRIE0));
}

void SerialProtocol::begin(u32 baud, u8 config)
{
    cli();
    memset(&mRxRingBuf, 0, sizeof(mRxRingBuf));
    memset(&mTxRingBuf, 0, sizeof(mTxRingBuf));

    
#if 0    
    u8 h = ((F_CPU  / 4 / baud -1) / 2) >> 8;
    u8 l = ((F_CPU  / 4 / baud -1) / 2);

    UCSR0B = 0;

    u8 data;
    for (u8 i = 0; i < 32; i++)
        data = UDR0;

    UCSR0A = (1<<U2X0);
    UBRR0H = h;
    UBRR0L = l;
    UCSR0B |= (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0);
    UCSR0C = conig; //(1<<UCSZ00) | (1<<UCSZ01);
#else
    u16 ubrr = (((F_CPU) + 8UL * (baud)) / (16UL * (baud)) -1UL);
    u8  use_2x = 0;
    u8  baud_tol = 2;

    if (100 * (F_CPU) > (16 * ((ubrr) + 1)) * (100 * (baud) + (baud) * (baud_tol))) {
        use_2x = 1;
    } else if (100 * (F_CPU) < (16 * ((ubrr) + 1)) * (100 * (baud) - (baud) * (baud_tol))) {
        use_2x = 1;
    }

    if (use_2x) {
        ubrr =  (((F_CPU) + 4UL * (baud)) / (8UL * (baud)) -1UL);
        if (100 * (F_CPU) > (8 * ((ubrr) + 1)) * (100 * (baud) + (baud) * (baud_tol))) {
            // Baud rate achieved is higher than allowed !!!
        } else if (100 * (F_CPU) < (8 * ((ubrr) + 1)) * (100 * (baud) - (baud) * (baud_tol))) {
            // Baud rate achieved is lower than allowed !!!
        }
        UCSR0A |= BV(U2X0);
    } else {
        UCSR0A &= ~BV(U2X0);
    }

    UBRR0L = (ubrr & 0xff);
    UBRR0H = (ubrr >> 8);

	UCSR0C = config; //BV(UPM01) | BV(USBS0) | BV(UCSZ01) | BV(UCSZ00);
	while (UCSR0A & BV(RXC0) )                      //flush receive buffer
		UDR0;
	//enable reception and RC complete interrupt
	UCSR0B = BV(RXEN0) | BV(RXCIE0) | BV(TXEN0);    //rx enable and interrupt
#endif
    sei();
}

void SerialProtocol::clearTX(void)
{
    cli();
    UCSR0B &= ~BV(UDRIE0);
    memset(&mTxRingBuf, 0, sizeof(mTxRingBuf));
    sei();
}

void SerialProtocol::clearRX(void)
{
    cli();
    memset(&mRxRingBuf, 0, sizeof(mRxRingBuf));
    sei();
}

u8 SerialProtocol::available(void)
{
    return sAvailable(&mRxRingBuf);
}

u8 SerialProtocol::read(void)
{
    return getChar(&mRxRingBuf);
}

u8 SerialProtocol::read(u8 *buf)
{
    u8 size = sAvailable(&mRxRingBuf);

    for (u8 i = 0; i < size; i++)
        *buf++ = getChar(&mRxRingBuf);

    return size;
}

void SerialProtocol::setCallback(u32 (*callback)(u8 cmd, u8 *data, u8 size))
{
    mCallback = callback;
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
                if (ch == 0x55) {
                    mState = STATE_HEADER_CMD;
                    mOffset = 0;
                    mDataSize = 25;
                }
                break;

            case STATE_HEADER_CMD:
                if (mOffset < mDataSize) {
                    mRxPacket[mOffset++] = ch;
                } else {
                    if (mCallback)
                        (*mCallback)(mRxPacket, mDataSize);
                    mState = STATE_IDLE;
                    rxSize = 0;             // no more than one command per cycle
                }
                break;
        }
    }
}


//
// Utility functions for debugging
//
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

