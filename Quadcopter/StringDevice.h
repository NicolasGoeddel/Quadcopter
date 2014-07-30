/*
 * StringDevice.h
 *
 *  Created on: 11.06.2014
 *      Author: nicolas
 */

#ifndef STRINGDEVICE_H_
#define STRINGDEVICE_H_

#include <avr/io.h>
#include <stdlib.h>		// wg. itoa
#include "libstdcpp.h"	// für new und delete
#include <string.h>

template <class ReturnType>
class StringDeviceOut {
	private:
		void static zeroString(char * buf, uint8_t length) {
			for (uint8_t i = 0; i < length; i++) {
				buf[i] = 0;
			}
		}
	public:
		virtual ~StringDeviceOut() {};

		virtual ReturnType* writeChar(const char c) = 0;

		/**
		 * Schreibt einen kompletten String zum Ausgabegerät.
		 *
		 * @param s Der zu sendende String.
		 */
		ReturnType* write(const char * c) {
			while (*c) {
				writeChar(*c++);
			}
			return (ReturnType*) this;
		}

		ReturnType* writeInt(const int16_t i) {
			char buf[8];
			zeroString(buf, 8);
			ltoa(i, buf, 10);
			write(buf);
			return (ReturnType*) this;
		}

		ReturnType* writeInt4(const int16_t i) {
			if (i >= 0) writeChar(' ');
			int16_t j = (i < 0) ? -i : i;
			if (j < 100) writeChar(' ');
			if (j < 10) writeChar(' ');
			char buf[8];
			zeroString(buf, 8);
			ltoa(i, buf, 10);
			write(buf);
			return (ReturnType*) this;
		}

		ReturnType* writeInt4(const int16_t x, const int16_t y, const int16_t z) {
			writeInt4(x);
			writeChar(',');
			writeInt4(y);
			writeChar(',');
			writeInt4(z);
			return (ReturnType*) this;
		}

		ReturnType* writeUint(const uint16_t i) {
			char buf[7];
			zeroString(buf, 7);
			ultoa(i, buf, 10);
			write(buf);
			return (ReturnType*) this;
		}

		ReturnType* writeUint32(const uint32_t i) {
			char buf[11];
			zeroString(buf, 11);
			ultoa(i, buf, 10);
			write(buf);
			return (ReturnType*) this;
		}

		void writeUint32Raw(const uint32_t i) {
			union {
			    unsigned char c[4];
			    uint32_t i;
			} pun;

			pun.i = i;
			writeChar(pun.c[0]);
			writeChar(pun.c[1]);
			writeChar(pun.c[2]);
			writeChar(pun.c[3]);
		}

		/**
		 * Schreibt eine Fließkommazahl mit einer Genauigkeit von 4
		 * Nachkommastellen als Dezimalzahl zum Ausgabegerät.
		 *
		 * @param f Der zu schreibende Floatwert.
		 */
		ReturnType* writeFloat(const float f) {
			char fstr[12];
			zeroString(fstr, 12);

			dtostrf(f, 8, 6, fstr);
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
		void writeFloatRaw(const float f) {
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

		ReturnType* writeUint3(const uint16_t i) {
			if (i < 100) writeChar(' ');
			if (i < 10) writeChar(' ');
			char buf[7];
			zeroString(buf, 7);
			ultoa(i, buf, 10);
			write(buf);
			return (ReturnType*) this;
		}

};

template <class ReturnType>
class StringDevice : public StringDeviceOut<ReturnType> {
	public:
		virtual ~StringDevice() {};
};

#endif /* STRINGDEVICE_H_ */
