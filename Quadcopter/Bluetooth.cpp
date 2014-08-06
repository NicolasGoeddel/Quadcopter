/*
 * Bluetooth.cpp
 *
 *  Created on: 16.01.2012
 *      Author: nicolas
 */

#include "Bluetooth.h"

#include "DEBUG.h"

Bluetooth::Bluetooth(USART_t* usart, PORT_t* port, uint32_t baud) : USART(usart, port, true) {
	setBaudrate(baud);
}

void Bluetooth::getData(uint8_t* buffer, uint8_t length) {
	while (length) {
		*buffer = receiveChar();
		buffer++;
		length--;
	}
}

uint8_t Bluetooth::getString(char* buffer, uint8_t length) {
	uint8_t read = 0;
	while (length) {
		*buffer = receiveChar();
		if ((*buffer == 13) || (*buffer == 10) || (*buffer == 0)) {
			break;
		}
		buffer++;
		length--;
		read++;
	}
	if (length > 0) {
		*buffer = '\0';
	}
	return read;
}

void Bluetooth::changeDeviceName(const char* deviceName) {
	write("AT+NAME")->write(deviceName);
	while (*deviceName) {
		receiveChar();
		deviceName++;
	}
}

bool Bluetooth::isDeviceOk() {
	write("AT");
	if (receiveChar() != 'O') {
		return false;
	}
	if (receiveChar() != 'K') {
		return false;
	}
	return true;
}

bool Bluetooth::setBaud(uint32_t baud) {
	// Standardbefehl
	char command[] = "AT+BAUD4";
	char p = '4';

	// Gibt die höchste Stelligkeit der Baudrate an.
	uint32_t h = 1000;

	// Wähle entsprechende Nummer laut Datenblatt
	switch (baud) {
		case 1200:    p = '1'; h = 1000; break;
		case 2400:    p = '2'; h = 1000; break;
		case 4800:    p = '3'; h = 1000; break;
		case 9600:    p = '4'; h = 1000; break;
		case 19200:   p = '5'; h = 10000; break;
		case 38400:   p = '6'; h = 10000; break;
		case 57600:   p = '7'; h = 10000; break;
		case 115200:  p = '8'; h = 100000; break;
		case 230400:  p = '9'; h = 100000; break;
		case 460800:  p = 'A'; h = 100000; break;
		case 921600:  p = 'B'; h = 100000; break;
		case 1382400: p = 'C'; h = 1000000; break;
		default:
			return false;
	}
	command[7] = p;
	write(command);

	/* Überprüfe Rückgabewert, OK<r>, wobei <r> gleich die Baudrate
	 * in Dezimalform sein muss
	 */
	if (receiveChar() != 'O') {
		return false;
	}
	if (receiveChar() != 'K') {
		return false;
	}

	while (h > 0) {
		/* Hole zunächst die erste Ziffer der Baudrate,
		 * dann die jeweils nächste.
		 */
		char expected = baud / h;
		// Überprüfe, ob die Ziffer auch zurück gegeben wurde.
		if (receiveChar() != expected) {
			return false;
		}
		// Lösche die höchste Ziffer
		baud -= expected * h;
		// Gehe eine Dezimalstelle tiefer
		h /= 10;
	}

	// Setze jetzt auch die Baudrate vom USART neu
	setBaudrate(baud);
	//USART_setBaudrate(baud);

	return true;
}
