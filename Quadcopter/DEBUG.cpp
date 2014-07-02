/*
 * DEBUG.cpp
 *
 *  Created on: 11.06.2014
 *      Author: nicolas
 */

#include "DEBUG.h"

void DEBUG_init() {
	// LED an Pin 2 und 3
	PORTH.DIRSET = _BV(2) | _BV(3);
	PORTH.PIN3CTRL = PORT_OPC_WIREDOR_gc;
	PORTH.PIN2CTRL = PORT_OPC_WIREDAND_gc;	//Setzt PIN2 fest auf GND
	PORTH.OUTCLR = _BV(2);
	PORTH.OUTCLR = _BV(3);

	// Button an Pin 0 und 1
	PORTH.DIRCLR = _BV(0);
	PORTH.DIRSET = _BV(1);
	PORTH.PIN0CTRL = PORT_OPC_PULLDOWN_gc;
	PORTH.PIN1CTRL = PORT_OPC_WIREDOR_gc;	//Setzt PIN1 fest auf Vcc
	PORTH.OUTSET = _BV(1);

	// Switch an Pin 4 und 5
	PORTH.DIRCLR = _BV(4);
	PORTH.DIRSET = _BV(5);
	PORTH.PIN4CTRL = PORT_OPC_PULLDOWN_gc;
	PORTH.PIN5CTRL = PORT_OPC_WIREDOR_gc;	//Setzt PIN5 fest auf Vcc
	PORTH.OUTSET = _BV(5);
}

void DEBUG_LED(int8_t state) {
	switch (state) {
		case 0: PORTH.OUTCLR = _BV(3); break;
		case 1: PORTH.OUTSET = _BV(3); break;
		default: PORTH.OUTTGL = _BV(3); break;
	}
}

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

bool DEBUG_Button() {
	return !!(PORTH.IN & _BV(0));
}

bool DEBUG_Switch() {
	return !!(PORTH.IN & _BV(4));
}
