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

#include "common.h"
#include "utils.h"
#include "RFProtocol.h"
#include "RCRcvrPWM.h"

#define PIN_ELEVATOR                3
#define PIN_AILERON                 4
#define PIN_THROTTLE                5
#define PIN_RUDDER                  6
#define PIN_AUX1                    7
#define PIN_AUX2                    9
#define PIN_AUX3                    10

#define PCINT_RX1_PIN_COUNT        5
#define PCINT_RX1_PORT             PORTD
#define PCINT_RX1_DDR              DDRD
#define PCINT_RX1_MASK             PCMSK2
#define PCINT_RX1                  PCINT2_vect
#define PCINT_RX1_PINS             PIND
#define PCINT_RX1_IR_BIT           BV(2)

#define PCINT_RX2_PIN_COUNT        2
#define PCINT_RX2_PORT             PORTB
#define PCINT_RX2_DDR              DDRB
#define PCINT_RX2_MASK             PCMSK0
#define PCINT_RX2                  PCINT0_vect
#define PCINT_RX2_PINS             PINB
#define PCINT_RX2_IR_BIT           BV(0)

#define MAX_RC_PIN_CNT             (PCINT_RX1_PIN_COUNT + PCINT_RX2_PIN_COUNT)


static const PROGMEM u8 TBL_PINS_RX1[] = {
    PIN_THROTTLE, PIN_RUDDER, PIN_ELEVATOR, PIN_AILERON, PIN_AUX1
};

static const PROGMEM u8 TBL_PINS_RX2[] = {
    PIN_AUX2, PIN_AUX3
};

static u16 wPrevTime[MAX_RC_PIN_CNT];
static s16 sRC[MAX_RC_PIN_CNT];

s16 RCRcvrPWM::getRC(u8 ch)
{
    if (ch > MAX_RC_PIN_CNT)
        return -500;

    return sRC[ch];
}

s16 *RCRcvrPWM::getRCs(void)
{
    return sRC;
}

u8 RCRcvrPWM::getMaxCh(void)
{
    return MAX_RC_PIN_CNT;
}

void RCRcvrPWM::init(void)
{
    memset(sRC, 0, sizeof(sRC));
    sRC[0] = -500;  // throttle min

    for (u8 i = 0; i < PCINT_RX1_PIN_COUNT; i++) {
        u8 pin = pgm_read_byte(TBL_PINS_RX1 + i);

        PCINT_RX1_PORT |= BV(pin);
        PCINT_RX1_MASK |= BV(pin);
        PCINT_RX1_DDR  &= ~BV(pin);
    }

    for (u8 i = 0; i < PCINT_RX2_PIN_COUNT; i++) {
        u8 pin = pgm_read_byte(TBL_PINS_RX2 + i) - 8;

        PCINT_RX2_PORT |= BV(pin);
        PCINT_RX2_MASK |= BV(pin);
        PCINT_RX2_DDR  &= ~BV(pin);
    }

    PCICR = PCINT_RX1_IR_BIT | PCINT_RX2_IR_BIT;
}

void RCRcvrPWM::close(void)
{
    PCICR = ~(PCINT_RX1_IR_BIT | PCINT_RX2_IR_BIT);
}

ISR(PCINT_RX1)
{
    u8  mask;
    u8  pins;
    u8  bv;
    u16 wTime;
    u16 wDiff;
    static u8 ucLastPin;

    pins  = PCINT_RX1_PINS;
    mask  = pins ^ ucLastPin;
    wTime = micros();
    sei();
    ucLastPin = pins;

    for (u8 i = 0; i < PCINT_RX1_PIN_COUNT; i++) {
        bv = BV(pgm_read_byte(TBL_PINS_RX1 + i));

        if (mask & bv) {
            if (!(pins & bv)) {
                wDiff = constrain(wTime - wPrevTime[i], 1000, 2000);
                sRC[i]= map(wDiff, 1000, 2000, -500, 500);
                if (i & 0x01)
                    sRC[i] = -sRC[i];
            } else {
                wPrevTime[i] = wTime;
            }
        }
    }
}

ISR(PCINT_RX2)
{
    u8  mask;
    u8  pins;
    u8  bv;
    u16 wTime;
    u16 wDiff;
    static u8 ucLastPin;

    pins  = PCINT_RX2_PINS;
    mask  = pins ^ ucLastPin;
    wTime = micros();
    sei();
    ucLastPin = pins;

    for (u8 i = 0; i < PCINT_RX2_PIN_COUNT; i++) {
        bv = BV(pgm_read_byte(TBL_PINS_RX2 + i) - 8);

        if (mask & bv) {
            if (!(pins & bv)) {
                wDiff = constrain(wTime - wPrevTime[PCINT_RX1_PIN_COUNT + i], 1000, 2000);
                sRC[PCINT_RX1_PIN_COUNT + i] = map(wDiff, 1000, 2000, -500, 500);
            } else {
                wPrevTime[PCINT_RX1_PIN_COUNT + i] = wTime;
            }
        }
    }
}
