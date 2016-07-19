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


void setup()
{
    pinMode(13, OUTPUT);
    pinMode(A1, INPUT);
    Serial.begin(115200);
}

long lLastTS = 0;

void loop()
{
	long	lTS;
	uint8_t	led = 1;
	
	while(1) {
		if (Serial.available()) {
			Serial.write(Serial.read());
		}
		lTS = millis();
		if (lTS - lLastTS > 300) {
//            Serial.println("HELLO");
			digitalWrite(13, led);
			led = !led;
			lLastTS = lTS;
		}
	}
}

