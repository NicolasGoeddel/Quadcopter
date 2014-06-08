/*
 * Bluetooth.cpp
 *
 *  Created on: 16.01.2012
 *      Author: nicolas
 */

#include "Bluetooth.h"

Bluetooth::Bluetooth(uint32_t baud) {
	USART_init(baud);
}

uint8_t Bluetooth::writeString(const char* s) {
	uint8_t length = 0;
	while (*s) {
		writeChar(*s);
		s++;
		length++;
	}
	return length;
}

void Bluetooth::getString(char* buffer, uint8_t length) {
	while (length) {
		*buffer = getChar();
		if (*buffer == '\r' || *buffer == '\n') {
			break;
		}
		buffer++;
		length--;
	}
	*buffer = '\0';
}

void Bluetooth::changeDeviceName(const char* deviceName) {
	writeString("AT+NAME");
	uint8_t length = writeString(deviceName) + 2;
	while (length--) {
		getChar();
	}
}
