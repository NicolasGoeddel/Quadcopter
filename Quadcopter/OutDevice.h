/*
 * IODevice.h
 *
 *  Created on: 10.06.2014
 *      Author: nicolas
 */

#ifndef IODEVICE_H_
#define IODEVICE_H_

#include <avr/io.h>
#include <stdlib.h>		// wg. itoa
#include "libstdcpp.h"	// für new und delete

class OutDevice {
	private:
		OutDevice* outDevice;

	public:
		OutDevice();
		OutDevice(OutDevice* outDevice);

		~OutDevice() {};

		OutDevice* write(const char c) {
			outDevice->write(c);
			return this;
		}

		/**
		 * Schreibt einen kompletten String zum Ausgabegerät.
		 *
		 * @param s Der zu sendende String.
		 */
		OutDevice* write(const char * c);

		OutDevice* writeInt(const int16_t i);

		OutDevice* writeInt4(const int16_t i);

		OutDevice* writeInt4x3(const int16_t x, const int16_t y, const int16_t z);

		OutDevice* writeUint(const uint16_t i);

		/**
		 * Schreibt eine Fließkommazahl mit einer Genauigkeit von 4
		 * Nachkommastellen als Dezimalzahl zum Ausgabegerät.
		 *
		 * @param f Der zu schreibende Floatwert.
		 */
		OutDevice* writeFloat(float f);

		/**
		 * Schreibt eine Fließkommazahl binär zum Ausgabegerät.
		 * Damit kann man die volle Genauigkeit beibehalten und braucht
		 * dennoch nur 4 Bytes für die Übertragung.
		 *
		 * @param f Der zu schreibende Floatwert.
		 */
		OutDevice* writeFloatRaw(float f);

		OutDevice* writeUint3(const uint16_t i);

};

#endif /* IODEVICE_H_ */
