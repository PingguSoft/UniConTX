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

#define PIN_CC_CSN      A0
#define NRF_CC_CFG()    pinMode(PIN_CC_CSN, OUTPUT);
#define CC_CS_HI()      digitalWrite(PIN_CC_CSN, HIGH);
#define CC_CS_LO()      digitalWrite(PIN_CC_CSN, LOW);

#define PIN_CSN_7105    A1
#define A7105_CS_CFG()  pinMode(PIN_CSN_7105,  OUTPUT);
#define A7105_CS_HI()   digitalWrite(PIN_CSN_7105, HIGH);
#define A7105_CS_LO()   digitalWrite(PIN_CSN_7105, LOW);

#define PIN_NRF_CSN     A4
#define NRF_CS_CFG()    pinMode(PIN_NRF_CSN, OUTPUT);
#define NRF_CS_HI()     digitalWrite(PIN_NRF_CSN, HIGH);
#define NRF_CS_LO()     digitalWrite(PIN_NRF_CSN, LOW);

#define PIN_CYRF_CSN    A5
#define PIN_CYRF_RESET  9
#define CYRF_CS_CFG()   pinMode(PIN_CYRF_CSN, OUTPUT);  pinMode(PIN_CYRF_RESET, OUTPUT);
#define CYRF_CS_HI()    digitalWrite(PIN_CYRF_CSN, HIGH);
#define CYRF_CS_LO()    digitalWrite(PIN_CYRF_CSN, LOW);
#define CYRF_RST_HI()   digitalWrite(PIN_CYRF_RESET, HIGH);
#define CYRF_RST_LO()   digitalWrite(PIN_CYRF_RESET, LOW);


// RFX2401 TX/RX switch, high active
#define PIN_TXEN        4
#define PIN_RXEN        5
#define RFX_CFG()       pinMode(PIN_TXEN, OUTPUT);  pinMode(PIN_RXEN, OUTPUT);
#define TX_HI()         digitalWrite(PIN_TXEN, HIGH);
#define TX_LO()         digitalWrite(PIN_TXEN, LOW);
#define RX_HI()         digitalWrite(PIN_RXEN, HIGH);
#define RX_LO()         digitalWrite(PIN_RXEN, LOW);
#define RFX_TX()        TX_HI();    RX_LO();
#define RFX_RX()        TX_LO();    RX_HI();
#define RFX_IDLE()      TX_LO();    RX_LO();


// 4052 RF switch
#define PIN_RF_PE1      A3
#define PIN_RF_PE2      A2
#define RF_SEL_CFG()    pinMode(PIN_RF_PE1, OUTPUT);    pinMode(PIN_RF_PE2, OUTPUT);
#define RF_SEL_7105()   digitalWrite(PIN_RF_PE1, LOW);  digitalWrite(PIN_RF_PE2, LOW);
#define RF_SEL_2500()   digitalWrite(PIN_RF_PE1, LOW);  digitalWrite(PIN_RF_PE2, HIGH);
#define RF_SEL_2401()   digitalWrite(PIN_RF_PE1, HIGH); digitalWrite(PIN_RF_PE2, LOW);
#define RF_SEL_6936()   digitalWrite(PIN_RF_PE1, HIGH); digitalWrite(PIN_RF_PE2, HIGH);

// INIT
#define INIT_COMMON()   RFX_CFG();      RFX_IDLE();     \
                        RF_SEL_CFG();   RF_SEL_7105();  \
                        A7105_CS_CFG(); A7105_CS_HI();  \
                        NRF_CC_CFG();   CC_CS_HI();     \
                        NRF_CS_CFG();   NRF_CS_HI();    \
                        CYRF_CS_CFG();  CYRF_CS_HI();   CYRF_RST_HI();

typedef enum TxPower {
    TXPOWER_100uW = 0,      // -35dBm
    TXPOWER_300uW,          // -30dBm
    TXPOWER_1mW,            // -24dBm
    TXPOWER_3mW,            // -18dBm
    TXPOWER_10mW,           // -13dBm
    TXPOWER_30mW,           // - 5dBm
    TXPOWER_100mW,          //   0dBm
    TXPOWER_150mW,          // + 4dBm
    TXPOWER_LAST,
} TXPOWER_T;

enum TXRX_State {
    TXRX_OFF,
    TX_EN,
    RX_EN,
};

#endif
