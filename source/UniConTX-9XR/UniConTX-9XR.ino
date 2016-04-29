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
#include "SerialProtocol.h"

#define PIN_BEAT    13
#define PIN_PPM     2
#define MULTIPLIER (F_CPU / 8000000)

static SerialProtocol  mSerial;
static int  ppm[16];

#define DEBUG   1

void read_ppm()
{
    static u32  pulse;
    static u32  counter;
    static u8   channel;
    static u32  last_micros;

    counter = TCNT1;
    TCNT1 = 0;

    if (counter < 710 * MULTIPLIER) {         //must be a pulse if less than 710us
        pulse = counter;
    } else if (counter > 1910 * MULTIPLIER) { //sync pulses over 1910us
        channel = 0;
//        Serial.print("PPM Frame Len: ");
//        Serial.println(micros() - last_micros);
        last_micros = micros();
    } else{                                 //servo values between 710us and 2420us will end up here
        ppm[channel] = (counter + pulse) / MULTIPLIER;
        channel++;
    }
}

void setup()
{
    pinMode(PIN_BEAT, OUTPUT);

    mSerial.begin(100000);
    mSerial.sendString("HELLO !!!!!!!!\n");

#if 0
    pinMode(PIN_PPM, INPUT);
    attachInterrupt(PIN_PPM - 2, read_ppm, CHANGE);

    TCCR1A = 0;  //reset timer1
    TCCR1B = 0;
    TCCR1B |= (1 << CS11);  //set timer1 to increment every 0,5 us    
#endif
}

static u8   beatStatus = 0;

u8 buf[100];

void loop()
{
    beatStatus = !beatStatus;
    digitalWrite(PIN_BEAT, beatStatus);

#if 0
    sprintf(buf, "%4d %4d %4d %4d %4d %4d %4d %4d\n", ppm[0], ppm[1], ppm[2], ppm[3], ppm[4],
        ppm[5], ppm[6], ppm[7], ppm[8]);
    Serial.print(buf);
#endif    
    u8 len = mSerial.getString(buf);
    if (len > 0) {
        mSerial.dump(buf, len);
    }
}

