/*
 * Clock.h
 *
 *  Created on: 21.05.2014
 *      Author: nicolas
 */

#ifndef CLOCK_H_
#define CLOCK_H_

#include "XMEGA_helper.h"
#include <avr/io.h>

#define TIMER_FREQUENCY 50

class Clock {
	public:
		volatile bool eventInterrupt;
		volatile uint32_t milliSeconds;
		Clock();
		~Clock();
};

#endif /* CLOCK_H_ */
