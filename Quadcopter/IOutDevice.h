/*
 * IODevice.h
 *
 *  Created on: 10.06.2014
 *      Author: nicolas
 */

#ifndef IOUTDEVICE_H_
#define IOUTDEVICE_H_

#include <avr/io.h>
#include <stdlib.h>		// wg. itoa
#include "libstdcpp.h"	// für new und delete

template <class ReturnType>
class IOutDevice {
	public:
		virtual ~IOutDevice() {};

		virtual ReturnType* write(const char c) = 0;
};

#endif /* IOUTDEVICE_H_ */
