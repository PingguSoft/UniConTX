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
#include "Common.h"
#include "DeviceCommon.h"
#include "Utils.h"

DeviceCommon::DeviceCommon()
{
    SPI.begin();
    init();
}

DeviceCommon::~DeviceCommon()
{
    SPI.end();
    init();
}

void DeviceCommon::init(void)
{
    pinMode(PIN_TXEN, OUTPUT);
    pinMode(PIN_RXEN, OUTPUT);
    setRFMode(RF_IDLE, TRUE);

    pinMode(PIN_RF_PE1, OUTPUT);
    pinMode(PIN_RF_PE2, OUTPUT);

    pinMode(PIN_CC_CSN, OUTPUT);
    digitalWrite(PIN_CC_CSN, HIGH);

    pinMode(PIN_CSN_7105,  OUTPUT);
    digitalWrite(PIN_CSN_7105, HIGH);

    pinMode(PIN_NRF_CSN, OUTPUT);
    digitalWrite(PIN_NRF_CSN, HIGH);

    pinMode(PIN_CYRF_CSN, OUTPUT);
    pinMode(PIN_CYRF_RESET, OUTPUT);
    digitalWrite(PIN_CYRF_CSN, HIGH);
    digitalWrite(PIN_CYRF_RESET, HIGH);
}

void DeviceCommon::setRFMode(enum RF_MODE mode, u8 skipImpl)
{
    switch(mode) {
        case RF_IDLE:
            digitalWrite(PIN_TXEN, LOW);
            digitalWrite(PIN_RXEN, LOW);
            break;

        case RF_TX:
            digitalWrite(PIN_TXEN, HIGH);
            digitalWrite(PIN_RXEN, LOW);
            break;

        case RF_RX:
            digitalWrite(PIN_TXEN, LOW);
            digitalWrite(PIN_RXEN, HIGH);
            break;
    }
    if (!skipImpl)
        setRFModeImpl(mode);
}

void DeviceCommon::setRFSwitch(enum TX_CHIP chip)
{
    switch(chip) {
        case TX_CC2500:
            digitalWrite(PIN_RF_PE1, LOW);
            digitalWrite(PIN_RF_PE2, HIGH);
            break;

        case TX_A7105:
            digitalWrite(PIN_RF_PE1, LOW);
            digitalWrite(PIN_RF_PE2, LOW);
            break;

        case TX_NRF24L01:
            digitalWrite(PIN_RF_PE1, HIGH);
            digitalWrite(PIN_RF_PE2, LOW);
            break;

        case TX_CYRF6936:
            digitalWrite(PIN_RF_PE1, HIGH);
            digitalWrite(PIN_RF_PE2, HIGH);
            break;
    }
}

