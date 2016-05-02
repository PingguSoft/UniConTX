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


#ifndef _DEVICE_COMMON_H_
#define _DEVICE_COMMON_H_

#include "Common.h"
#include <Arduino.h>
#include <avr/pgmspace.h>


// RFX2401 TX/RX switch, high active
#define PIN_RXEN      6
#define PIN_TXEN      7
#define TX_HI() digitalWrite(PIN_TXEN, HIGH);
#define TX_LO() digitalWrite(PIN_TXEN, LOW);
#define RX_HI() digitalWrite(PIN_RXEN, HIGH);
#define RX_LO() digitalWrite(PIN_RXEN, LOW);
#define RFX_TX()    TX_HI();    RX_LO();
#define RFX_RX()    TX_LO();    RX_HI();
#define RFX_IDLE()  TX_LO();    RX_LO();


// 4052 RF switch
#define PIN_RF_PE1   A3
#define PIN_RF_PE2   A2
#define RF_SEL_7105()   digitalWrite(PIN_RF_PE1, LOW);  digitalWrite(PIN_RF_PE2, LOW);
#define RF_SEL_2500()   digitalWrite(PIN_RF_PE1, LOW);  digitalWrite(PIN_RF_PE2, HIGH);
#define RF_SEL_2401()   digitalWrite(PIN_RF_PE1, HIGH); digitalWrite(PIN_RF_PE2, LOW);
#define RF_SEL_6936()   digitalWrite(PIN_RF_PE1, HIGH); digitalWrite(PIN_RF_PE2, HIGH);

// INIT
#define INIT_COMMON()   pinMode(PIN_RXEN, OUTPUT); pinMode(PIN_TXEN, OUTPUT); pinMode(PIN_RF_PE1, OUTPUT); pinMode(PIN_RF_PE2, OUTPUT); RFX_IDLE();

#endif
