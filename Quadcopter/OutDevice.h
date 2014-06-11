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

template <class ReturnType>
class OutDevice {
	public:
		virtual ~OutDevice() {};

		virtual ReturnType* write(const char c) = 0;

		/**
		 * Schreibt einen kompletten String zum Ausgabegerät.
		 *
		 * @param s Der zu sendende String.
		 */
		ReturnType* write(const char * c) {
			while (*c) {
				write(*c++);
			}
			return (ReturnType*) this;
		}

		ReturnType* writeInt(const int16_t i) {
			char buf[8];
			ltoa(i, buf, 10);
			write(buf);
			return (ReturnType*) this;
		}

		ReturnType* writeInt4(const int16_t i) {
			if (i >= 0) write(' ');
			int16_t j = (i < 0) ? -i : i;
			if (j < 100) write(' ');
			if (j < 10) write(' ');
			char buf[8];
			ltoa(i, buf, 10);
			write(buf);
			return (ReturnType*) this;
		}

		ReturnType* writeInt4x3(const int16_t x, const int16_t y, const int16_t z) {
			writeInt4(x);
			write(',');
			writeInt4(y);
			write(',');
			writeInt4(z);
			return (ReturnType*) this;
		}

		ReturnType* writeUint(const uint16_t i) {
			char buf[7];
			ultoa(i, buf, 10);
			write(buf);
			return (ReturnType*) this;
		}

		/**
		 * Schreibt eine Fließkommazahl mit einer Genauigkeit von 4
		 * Nachkommastellen als Dezimalzahl zum Ausgabegerät.
		 *
		 * @param f Der zu schreibende Floatwert.
		 */
		ReturnType* writeFloat(float f) {
			char fstr[8];

			dtostrf(f, 8, 4, fstr);
			write(fstr);
			return (ReturnType*) this;
		}

		/**
		 * Schreibt eine Fließkommazahl binär zum Ausgabegerät.
		 * Damit kann man die volle Genauigkeit beibehalten und braucht
		 * dennoch nur 4 Bytes für die Übertragung.
		 *
		 * @param f Der zu schreibende Floatwert.
		 */
		void writeFloatRaw(float f) {
			union {
			    unsigned char c[4];
			    float f;
			} pun;

			pun.f = f;
			write(pun.c[0]);
			write(pun.c[1]);
			write(pun.c[2]);
			write(pun.c[3]);
		}

		ReturnType* writeUint3(const uint16_t i) {
			if (i < 100) write(' ');
			if (i < 10) write(' ');
			char buf[7];
			ultoa(i, buf, 10);
			write(buf);
			return (ReturnType*) this;
		}

};

#endif /* IODEVICE_H_ */
