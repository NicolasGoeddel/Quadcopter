/*
 * IODevice.cpp
 *
 *  Created on: 10.06.2014
 *      Author: nicolas
 */

#include "OutDevice.h"

OutDevice::OutDevice(OutDevice* outDevice) {
	this->outDevice = outDevice;
}

OutDevice* OutDevice::write(const char * c) {
	while (*c) {
		write(*c++);
	}
	return this;
}

OutDevice* OutDevice::writeInt(const int16_t i) {
	char buf[8];
	ltoa(i, buf, 10);
	write(buf);
	return this;
}

OutDevice* OutDevice::writeInt4(const int16_t i) {
	if (i >= 0) write(' ');
	int16_t j = (i < 0) ? -i : i;
	if (j < 100) write(' ');
	if (j < 10) write(' ');
	char buf[8];
	ltoa(i, buf, 10);
	write(buf);
	return this;
}

OutDevice* OutDevice::writeInt4x3(const int16_t x, const int16_t y, const int16_t z) {
	writeInt4(x);
	write(',');
	writeInt4(y);
	write(',');
	writeInt4(z);
	return  this;
}

OutDevice* OutDevice::writeUint(const uint16_t i) {
	char buf[7];
	ultoa(i, buf, 10);
	write(buf);
	return this;
}

OutDevice* OutDevice::writeFloat(float f) {
	char fstr[8];

	dtostrf(f, 8, 4, fstr);
	write(fstr);
	return this;
}

OutDevice* OutDevice::writeFloatRaw(float f) {
	union {
	    unsigned char c[4];
	    float f;
	} pun;

	pun.f = f;
	write(pun.c[0]);
	write(pun.c[1]);
	write(pun.c[2]);
	write(pun.c[3]);
	return this;
}

OutDevice* OutDevice::writeUint3(const uint16_t i) {
	if (i < 100) write(' ');
	if (i < 10) write(' ');
	char buf[7];
	ultoa(i, buf, 10);
	write(buf);
	return this;
}
