/*
 * Clock.cpp
 *
 *  Created on: 21.05.2014
 *      Author: nicolas
 */

#include "Clock.h"

volatile Clock* object_Clock = 0;

Clock::Clock() {
	object_Clock = this;
	milliSeconds = 0;
	eventInterrupt = 0;
	configureInterrupt(TCD1, TIMER_FREQUENCY);
	addToInterrupt(TCD1_OVF_vect_num);
}
