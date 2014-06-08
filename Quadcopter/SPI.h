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

#ifndef SPI_H_
#define SPI_H_

#include <avr/io.h>
#include <util/delay.h>

/**
 * Template Parameter für die SPI-Klasse
 * SCLK		Pinnummer am gewählten Port für den Clock
 * MISO		Pinnummer am gewählten Port für die einkommenden Daten
 * MOSI		Pinnummer am gewählten Port für die ausgehenden Daten
 * DELAY	Verzögerung in Mikrosekunden nach jeder Clock-Flanke.
 * 			Wird hier 0 angegeben, greift das zweite Template, in dem
 * 			_delay_us() gar nicht erst aufgerufen wird.
 */
template <uint8_t SCLK, uint8_t MISO, uint8_t MOSI, uint8_t DELAY>
class SPI {
	private:
		PORT_t * port;

	public:
		/**
		 * PORT Port, der benutzt werden soll. Auf diesen beziehen sich die Pinnummern.
		 */
		SPI(PORT_t & PORT) {
			port = &PORT;

			port->DIRSET = _BV(SCLK) | _BV(MOSI);
			port->DIRCLR = _BV(MISO);
			port->OUTSET = _BV(SCLK);
			port->OUTCLR = _BV(MOSI);
			PORTCFG.MPCMASK = _BV(MISO) | _BV(MOSI) | _BV(SCLK);
			/* Mögliche Werte für port->PIN0CTRL
			 * PORT_OPC_PULLUP_gc	PullUp-Widerstand am Input
			 * PORT_OPC_PULLDOWN_gc	PullDown-Widerstand am Input
			 * PORT_OPC_TOTEM_gc	Weder PullUp- noch PullDown-Widerstand am Input
			 */
			port->PIN0CTRL = PORT_OPC_TOTEM_gc;
		}
		~SPI() {}

		/**
		 * 8 Bit Wert über SPI schreiben.
		 */
		void write(uint8_t value) {
			for (uint8_t bit = 128; bit > 0; bit >>= 1) {
				port->OUTCLR = _BV(SCLK);
				if (value & bit) {
					port->OUTSET = _BV(MOSI);
				} else {
					port->OUTCLR = _BV(MOSI);
				}
				_delay_us(DELAY);
				port->OUTSET = _BV(SCLK);
				_delay_us(DELAY);
			}
		}

		/**
		 * 8 Bit Wert über SPI empfangen.
		 */
		uint8_t read() {
			uint8_t value = 0;
			for (uint8_t bit = 128; bit > 0; bit >>= 1) {
				port->OUTCLR = _BV(SCLK);
				_delay_us(DELAY);
				port->OUTSET = _BV(SCLK);
				if (port->IN & _BV(MISO)) {
					value |= bit;
				}
				_delay_us(DELAY);
			}
			return value;
		}
};

template <uint8_t SCLK, uint8_t MISO, uint8_t MOSI>
class SPI <SCLK, MISO, MOSI, 0> {
	private:
		PORT_t * port;

	public:
		/**
		 * PORT Port, der benutzt werden soll. Auf diesen beziehen sich die Pinnummern.
		 */
		SPI(PORT_t & PORT) {
			port = &PORT;

			port->DIR = (port->DIR & ~(_BV(MISO))) | _BV(SCLK) | _BV(MOSI);
			port->OUT = (port->OUT & ~(_BV(MOSI))) | _BV(SCLK);
			PORTCFG.MPCMASK = _BV(MISO);
			/* Mögliche Werte für port->PIN0CTRL
			 * PORT_OPC_PULLUP_gc	PullUp-Widerstand am Input
			 * PORT_OPC_PULLDOWN_gc	PullDown-Widerstand am Input
			 * PORT_OPC_TOTEM_gc	Weder PullUp- noch PullDown-Widerstand am Input
			 */
			port->PIN0CTRL = PORT_OPC_TOTEM_gc;
		}
		~SPI() {}

		/**
		 * 8 Bit Wert über SPI schreiben.
		 */
		void write(uint8_t value) {
			for (uint8_t bit = 128; bit > 0; bit >>= 1) {
				port->OUTCLR = _BV(SCLK);
				if (value & bit) {
					port->OUTSET = _BV(MOSI);
				} else {
					port->OUTCLR = _BV(MOSI);
				}
				port->OUTSET = _BV(SCLK);
			}
		}

		/**
		 * 8 Bit Wert über SPI empfangen.
		 */
		uint8_t read() {
			uint8_t value = 0;
			for (uint8_t bit = 128; bit > 0; bit >>= 1) {
				port->OUTCLR = _BV(SCLK);
				port->OUTSET = _BV(SCLK);
				if (port->IN & _BV(MISO)) {
					value |= bit;
				}
			}
			return value;
		}
};


#endif /* SPI_H_ */
