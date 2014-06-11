/*
 * DEBUG.h
 *
 *  Created on: 11.06.2014
 *      Author: nicolas
 */

#ifndef DEBUG_H_
#define DEBUG_H_

#include <avr/io.h>
#include <util/delay.h>

/**
 * Initialisiert Eingang für Taster und Ausgänge für LED:
 *
 * PH0 - PH1 : Taster
 * PH2 - LEDGND
 * PH3 - LED+
 */
void DEBUG_init() {
	// Debuggig-Taster und -LED
	PORTH.DIR = _BV(1) | _BV(2) | _BV(3);
	PORTH.PIN0CTRL = PORT_OPC_PULLDOWN_gc;

	PORTH.PIN3CTRL = PORT_OPC_WIREDOR_gc;

	PORTH.PIN1CTRL = PORT_OPC_WIREDOR_gc;	//Setzt PIN1 fest auf Vcc
	PORTH.OUTSET = _BV(1);

	PORTH.PIN2CTRL = PORT_OPC_WIREDAND_gc;	//Setzt PIN2 fest auf GND
	PORTH.OUTCLR = _BV(2);

	PORTH.OUTCLR = _BV(3);
}

/**
 * Setzt den Zustand der Debugging LED.
 *
 * @param state Neuer Zustand der LED:
 *              0 - LED aus.
 *              1 - LED an.
 *              sonst - Toggle LED.
 */
void DEBUG_LED(int8_t state) {
	switch (state) {
		case 0: PORTH.OUTCLR = _BV(3); break;
		case 1: PORTH.OUTSET = _BV(3); break;
		default: PORTH.OUTTGL = _BV(3); break;
	}
}

/**
 * Lässt die LED in einem festen Intervall von 300 ms
 * mehrmals blinken.
 *
 * @param times Wie oft die LED blinken soll.
 */
void DEBUG_LED_blink(uint8_t times) {
	if (times == 0) return;

	while (times) {
		PORTH.OUTTGL = _BV(3);
		_delay_ms(150);
		PORTH.OUTTGL = _BV(3);
		_delay_ms(150);
		times--;
	}
}

/**
 * Gibt den Zustand des Debugging Tasters zurück
 *
 * @return Zustand des Tasters:
 *         false - Taster nicht gedrückt.
 *         true - Taster gedrückt.
 */
bool DEBUG_Button() {
	return !!(PORTH.IN & _BV(0));
}

#endif /* DEBUG_H_ */
