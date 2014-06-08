/*
 * Bluetooth.h
 *
 *  Created on: 16.01.2012
 *      Author: nicolas
 */

#ifndef BLUETOOTH_H_
#define BLUETOOTH_H_

#include <avr/io.h>
#include "USART.h"
#include <stdio.h>
#include <stdlib.h>

class Bluetooth {
	public:
		Bluetooth(uint32_t baud);

		~Bluetooth() {
		}

		bool setBaud(uint32_t baud);

		void writeChar(char c) {
			USART_putchar(c);
		}

		void writeFloat(float f) {
			char fstr[8];

			dtostrf(f, 8, 4, fstr);
			writeString(fstr);
		}

		void writeFloatRaw(float f) {
			union {
			    unsigned char c[4];
			    float f;
			} pun;

			pun.f = f;
			writeChar(pun.c[0]);
			writeChar(pun.c[1]);
			writeChar(pun.c[2]);
			writeChar(pun.c[3]);
		}

		uint8_t writeString(const char* s);

		char getChar(void)	{
		    return USART_getchar();
		}

		char getCharAsync(void)	{
			if (USART_dataAvailable())
				return USART_getchar();
			return '\0';
		}

		void getString(char* buffer, uint8_t length);

		void changeDeviceName(const char* deviceName);

		bool isDeviceOk() {
			writeString("AT");
			if (getChar() != 'O') {
				return false;
			}
			if (getChar() != 'K') {
				return false;
			}
			return true;
		}
};

#endif /* BLUETOOTH_H_ */
