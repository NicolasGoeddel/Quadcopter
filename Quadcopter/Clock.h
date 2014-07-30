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
#include "DEBUG.h"
#include "InterruptHelper.h"

#define TIMER_FREQUENCY 50

/**
 * Diese Klasse hilft die Zeit zu messen, sobald man sie
 * initialisiert. Außerdem triggert sie aktuell noch mit
 * einer volatile-Variablen die Hauptschleife vom ganzen
 * Programm. Das muss zukünftig noch irgendwie anders
 * gelöst werden, damit die Hauptschleife sauber läuft.
 */
class Clock : Interrupt {
	public:
		volatile bool eventInterrupt;
		volatile uint32_t milliSeconds;
		Clock();
		~Clock() {};

		/**
		 * Der Interrupt, der mit einer Frequenz von
		 * TIMER_FREQUENCY ausgelöst wird.
		 */
		void interrupt() {
			eventInterrupt = 1;
			milliSeconds += 1000 / TIMER_FREQUENCY;
		}
};

#endif /* CLOCK_H_ */
