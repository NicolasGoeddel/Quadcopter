/*
 * Clock.cpp
 *
 *  Created on: 21.05.2014
 *      Author: nicolas
 */

#include "Clock.h"

volatile Clock* object_Clock = 0;

ISR(TCD1_OVF_vect) {
	object_Clock->eventInterrupt = 1;
	object_Clock->milliSeconds += 1000 / TIMER_FREQUENCY;
}

Clock::Clock() {
	object_Clock = this;
	milliSeconds = 0;
	eventInterrupt = 0;
	configureInterrupt(TCD1, TIMER_FREQUENCY);
}

Clock::~Clock() {
	//do nothing
}

