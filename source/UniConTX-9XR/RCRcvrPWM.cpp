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
#include "common.h"
#include "utils.h"
#include "RCRcvrPWM.h"

#define CHANEL_NUMBER               8       //set the number of chanels
#define DEFAULT_SERVO_VALUE         1500    //set the default servo value
#define PPM_FRLEN                   22500   //set the PPM frame length in microseconds (1ms = 1000µs)
#define PPM_PULSELEN                300     //set the pulse length
#define ON_STATE                    1       //set polarity of the pulses: 1 is positive, 0 is negative
#define PIN_PPM                     10      //set PPM signal output pin on the arduino


#define PIN_ELEVATOR                2
#define PIN_AILERON                 3
#define PIN_THROTTLE                4
#define PIN_RUDDER                  5
#define PIN_AUX1                    6
#define PIN_AUX2                    7
#define PIN_AUX3                    8
#define PIN_AUX4                    9

#define PCINT_RX1_IDX               0
#define PCINT_RX1_PORT              PORTD
#define PCINT_RX1_DDR               DDRD
#define PCINT_RX1_MASK              PCMSK2
#define PCINT_RX1                   PCINT2_vect
#define PCINT_RX1_PINS              PIND
#define PCINT_RX1_IR_BIT            BV(2)

#define PCINT_RX2_IDX               1
#define PCINT_RX2_PORT              PORTB
#define PCINT_RX2_DDR               DDRB
#define PCINT_RX2_MASK              PCMSK0
#define PCINT_RX2                   PCINT0_vect
#define PCINT_RX2_PINS              PINB
#define PCINT_RX2_IR_BIT            BV(0)

static const PROGMEM u8 TBL_PINS_RX1[] = {
    PIN_THROTTLE, PIN_RUDDER, PIN_ELEVATOR, PIN_AILERON, PIN_AUX1, PIN_AUX2
};

static const PROGMEM u8 TBL_PINS_RX2[] = {
    PIN_AUX3, PIN_AUX4
};

// AETR1234
static const u8 TBL_CH_MAP[] = {
    3, 2, 0, 1, 4, 5, 6, 7
};

static u16 wPrevTime[sizeof(TBL_PINS_RX1) + sizeof(TBL_PINS_RX2)];
static s16 sRC[sizeof(TBL_PINS_RX1) + sizeof(TBL_PINS_RX2)];

s16 RCRcvrPWM::getRC(u8 ch)
{
    if (ch >= getChCnt())
        return 1000;

    return sRC[ch];
}

s16 *RCRcvrPWM::getRCs(void)
{
    return sRC;
}

u8 RCRcvrPWM::getChCnt(void)
{
    return sizeof(TBL_PINS_RX1) + sizeof(TBL_PINS_RX2);
}


void RCRcvrPWM::init(void)
{
    for (u8 i = 1; i < sizeof(sRC); i++)
        sRC[i] = 1500;

    sRC[0] = 1000;  // throttle min

    for (u8 i = 0; i < sizeof(TBL_PINS_RX1); i++) {
        u8 pin = pgm_read_byte(TBL_PINS_RX1 + i);

        PCINT_RX1_PORT |= BV(pin);
        PCINT_RX1_MASK |= BV(pin);
        PCINT_RX1_DDR  &= ~BV(pin);
    }

    for (u8 i = 0; i < sizeof(TBL_PINS_RX2); i++) {
        u8 pin = pgm_read_byte(TBL_PINS_RX2 + i) - 8;

        PCINT_RX2_PORT |= BV(pin);
        PCINT_RX2_MASK |= BV(pin);
        PCINT_RX2_DDR  &= ~BV(pin);
    }
    PCICR = PCICR | (PCINT_RX1_IR_BIT | PCINT_RX2_IR_BIT);

    pinMode(PIN_PPM, OUTPUT);
    digitalWrite(PIN_PPM, !ON_STATE);  //set the PPM signal pin to the default state (off)

    cli();
    TCCR1A = 0; // set entire TCCR1 register to 0
    TCCR1B = 0;

    OCR1A = 100;  // compare match register, change this
    TCCR1B |= (1 << WGM12);  // turn on CTC mode
    TCCR1B |= (1 << CS11);  // 8 prescaler: 0,5 microseconds at 16mhz, 1ms at 8mhz
    TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
    sei();
}

void RCRcvrPWM::close(void)
{
    PCICR = PCICR & ~(PCINT_RX1_IR_BIT | PCINT_RX2_IR_BIT);
}

void calcPeriod(u8 idx, u16 ts, u8 mask, u8 pins)
{
    u8  bv;
    u8  *tbl;
    u8  size;
    u8  start;
    u8  bit;
    u16 wDiff;

    if (idx == PCINT_RX1_IDX) {
        tbl   = (u8*)TBL_PINS_RX1;
        size  = sizeof(TBL_PINS_RX1);
        start = 0;
        bit   = 0;
    } else {
        tbl   = (u8*)TBL_PINS_RX2;
        size  = sizeof(TBL_PINS_RX2);
        start = sizeof(TBL_PINS_RX1);
        bit   = 8;
    }

    for (u8 i = 0; i < size; i++) {
        bv = BV(pgm_read_byte(tbl + i) - bit);
        if (mask & bv) {
            if (!(pins & bv)) {
                wDiff  = constrain(ts - wPrevTime[start + i], 1000, 2000);
                sRC[start + i] = wDiff;
            } else {
                wPrevTime[start + i] = ts;
            }
        }
    }
}

ISR(PCINT_RX1)
{
    u8  mask;
    u8  pins;
    u16 wTS;
    static u8 ucLastPin;

    pins      = PCINT_RX1_PINS;
    mask      = pins ^ ucLastPin;
    wTS       = micros();
    ucLastPin = pins;

    sei();
    calcPeriod(PCINT_RX1_IDX, wTS, mask, pins);
}

ISR(PCINT_RX2)
{
    u8  mask;
    u8  pins;
    u16 wTS;
    static u8 ucLastPin;

    pins      = PCINT_RX2_PINS;
    mask      = pins ^ ucLastPin;
    wTS       = micros();
    ucLastPin = pins;

    sei();
    calcPeriod(PCINT_RX2_IDX, wTS, mask, pins);
}

ISR(TIMER1_COMPA_vect)
{
    static boolean state = true;

    TCNT1 = 0;

    if (state) {    // start pulse
        digitalWrite(PIN_PPM, ON_STATE);
        OCR1A = PPM_PULSELEN;
        state = false;
    } else {        // end pulse and calculate when to start the next pulse
        static byte cur_chan_numb;
        static unsigned int calc_rest;

        digitalWrite(PIN_PPM, !ON_STATE);
        state = true;

        if(cur_chan_numb >= CHANEL_NUMBER){
            cur_chan_numb = 0;
            calc_rest = calc_rest + PPM_PULSELEN;
            OCR1A = (PPM_FRLEN - calc_rest);
            calc_rest = 0;
        } else {
            OCR1A = (sRC[TBL_CH_MAP[cur_chan_numb]] - PPM_PULSELEN);
            calc_rest = calc_rest + sRC[TBL_CH_MAP[cur_chan_numb]];
            cur_chan_numb++;
        }
    }
}

