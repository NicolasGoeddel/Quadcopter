/**
 * Copyright 2012-2013 Nicolas Göddel
 *
 * This file is part of the Quadcopter Project.
 *
 * The Quadcopter Project is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The Quadcopter Project is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Diese Datei ist Teil des Quadcopter Projekts.
 *
 * Das Quadcopter Projekt ist Freie Software: Sie können es unter den Bedingungen
 * der GNU General Public License, wie von der Free Software Foundation,
 * Version 3 der Lizenz oder (nach Ihrer Option) jeder späteren
 * veröffentlichten Version, weiterverbreiten und/oder modifizieren.
 *
 * Das Quadcopter Projekt wird in der Hoffnung, dass es nützlich sein wird, aber
 * OHNE JEDE GEWÄHELEISTUNG, bereitgestellt; sogar ohne die implizite
 * Gewährleistung der MARKTFÄHIGKEIT oder EIGNUNG FÜR EINEN BESTIMMTEN ZWECK.
 * Siehe die GNU General Public License für weitere Details.
 *
 * Sie sollten eine Kopie der GNU General Public License zusammen mit diesem
 * Programm erhalten haben. Wenn nicht, siehe <http://www.gnu.org/licenses/>.
 */

#ifndef XMEGA_HELPER_H_
#define XMEGA_HELPER_H_

#include <avr/io.h>
#include <avr/interrupt.h>

void set32MHz();

void inline activateInterrupts() {
	PMIC.CTRL = PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm | PMIC_HILVLEN_bm;
	sei();
}

void inline setPinPullDown(register8_t & pincfg) {
	pincfg = PORT_OPC_PULLDOWN_gc;
}

void inline setPinPullDown(PORT_t &port, uint8_t pin) {
	PORTCFG.MPCMASK = _BV(pin);
	port.PIN0CTRL = PORT_OPC_PULLDOWN_gc;
}

void inline setPinPullUp(PORT_t &port, uint8_t pin) {
	PORTCFG.MPCMASK = _BV(pin);
	port.PIN0CTRL = PORT_OPC_PULLUP_gc;
}

void inline setPinPullUp(register8_t & pincfg) {
	pincfg = PORT_OPC_PULLUP_gc;
}

/*
 * TCC0 = 0x800		PORTC = 0x640
 * TCD0 = 0x900		PORTD = 0x660
 * TCE0 = 0xA00		PORTE = 0x680
 * TCF0 = 0xB00		PORTF = 0x6A0
 */

uint16_t init4ChanPWM(PORT_t & port, TC0_t & timer, uint32_t frequency, uint16_t max);

uint16_t init4ChanPWM(PORT_t & port, TC0_t & timer, uint32_t frequency);

void set4ChanPWM(TC0_t & timer, uint16_t A, uint16_t B, uint16_t C, uint16_t D);

void setTimer(TC0_t & timer, PORT_t & port);

uint16_t configureInterrupt(TC1_t & timer, uint32_t frequency);


#endif /* XMEGA_HELPER_H_ */
