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

/* Pinbelegung für den Port
 *
 * Bit	Displayport
 * 0	DB4
 * 1	DB5
 * 2	DB6
 * 3	DB7
 * 4	RW
 * 5	I/OC1
 * 6	I/OC2
 * 7	EX
 */

#ifndef DISPLAY_H_
#define DISPLAY_H_

#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>      // wg. itoa

	// display commands
#define LCD_CMD_SETFUNCTIONMODE          0xc0  // SF
#define LCD_CMD_SETENTRYMODE             0x40  // SE
#define LCD_CMD_SETDISPLAYMODE           0x20  // SD
#define LCD_CMD_SHIFTCURDISPADDRESS      0x10  // MA
#define LCD_CMD_SETUNDERLINEMODE         0x0C  // SU
#define LCD_CMD_WRITERAMUNDERLINEBIT     0x08  // WU
#define LCD_CMD_SETBLINKINGFREQUENCY     0x04  // SB
#define LCD_CMD_CURSORHOMESTARTADDR      0x03  // MH
#define LCD_CMD_CLEARCURSORDATAADDRHOME  0x01  // CH
#define LCD_CMD_NOOPERATION              0x00  // NOP

	// SF - Set Function Mode constants
#define sfIO8BIT         0x20  // bit 5
#define sfIO4BIT         0x00
#define sfFont5x8        0x10  // bit 4
#define sfFont5x12       0x00

	// SD - Set Display Mode constants
#define dmDisplayOn      0x10
#define dmCursorOn       0x08
#define dmUnderline      0x04
#define dmCursorBlink    0x02
#define dmCharacterBlink 0x01

	// SU - Set Underline Mode constants
#define suSetUnderline   0x03
#define suResetUnderline 0x02
#define suNoUnderline    0x00

	// SB - Set Blinking Frequency constants
#define sbSlow           0x00
#define sbNormal         0x01
#define sbFast           0x02
#define sbFastest        0x03

	// SE - Set Entry Mode constants

	// base definitions cursor
#define LCD_EM_CursorRead 0x08
#define LCD_EM_CursorWrite 0x10
#define LCD_EM_CursorIncrement 0x00
#define LCD_EM_CursorDecrement 0x20

	// base definitions display
#define LCD_EM_DisplayRead 0x01
#define LCD_EM_DisplayWrite 0x02
#define LCD_EM_DisplayIncrement 0x00
#define LCD_EM_DisplayDecrement 0x04

	// constants for cursor
#define emCursorNoChange       0x00
#define emCursorIncRead        LCD_EM_CursorIncrement | LCD_EM_CursorRead
#define emCursorIncWrite       LCD_EM_CursorIncrement | LCD_EM_CursorWrite
#define emCursorIncReadWrite   LCD_EM_CursorIncrement | LCD_EM_CursorRead | LCD_EM_CursorWrite
#define emCursorDecRead        LCD_EM_CursorDecrement | LCD_EM_CursorRead
#define emCursorDecWrite       LCD_EM_CursorDecrement | LCD_EM_CursorWrite
#define emCursorDecReadWrite   LCD_EM_CursorDecrement | LCD_EM_CursorRead | LCD_EM_CursorWrite

	// constants for display start address
#define emDisplayNoChange      0x00
#define emDisplayIncRead       LCD_EM_DisplayIncrement | LCD_EM_DisplayRead
#define emDisplayIncWrite      LCD_EM_DisplayIncrement | LCD_EM_DisplayWrite
#define emDisplayIncReadWrite  LCD_EM_DisplayIncrement | LCD_EM_DisplayRead | LCD_EM_DisplayWrite
#define emDisplayDecRead       LCD_EM_DisplayDecrement | LCD_EM_DisplayRead
#define emDisplayDecWrite      LCD_EM_DisplayDecrement | LCD_EM_DisplayWrite
#define emDisplayDecReadWrite  LCD_EM_DisplayDecrement | LCD_EM_DisplayRead | LCD_EM_DisplayWrite

//#define Display_delay for (uint8_t i = 0; i < 254; i++) { asm("nop"); }

class Display {
	private:
		PORT_t * port;

		enum PORT {
			DB4 = 0,	// DB4-DB7 müssen auf PIN0-PIN3 gemappt bleiben
			DB5 = 1,
			DB6 = 2,
			DB7 = 3,
			IOC1 = 5,
			IOC2 = 4,
			EX = 6
		};

		void send(uint8_t control, uint8_t data) {

			// Control-Bits setzen und obere 4 Datenbit setzen.
			port->OUT = control | (data >> 4);
			// 4 Bit rein schieben.
			_delay_us(8);
			port->OUTSET = _BV(EX);
			_delay_us(8);
			port->OUTCLR = _BV(EX);
			// Control-Bits setzen und untere 4 Datenbit setzen.
			port->OUT = control | (data & 0x0f);
			// 4 Bit rein schieben.
			_delay_us(8);
			port->OUTSET = _BV(EX);
			_delay_us(8);
			// Port nullen.
			port->OUTCLR = _BV(EX);
			_delay_us(40);
//			port->OUTCLR = 0x0f; //_BV(EX);
//			_delay_us(1);
//			port->OUTCLR = 0xf0;
		}
	public:
		Display(PORT_t & PORT) {
			port = &PORT;
			port->DIR = 0xff;
			port->OUTCLR = 0xff;
		}
		~Display() { }

		void init() {
			// SF - Set Function Mode
			//  >> 4 Bit I/O, 5x8 Font, 1/32 duty (corresponding for 4 lines)
			//	>> 4 lines x 40 words, 96 words CG RAM
			send(0, LCD_CMD_SETFUNCTIONMODE | sfIO4BIT | sfFont5x8 | 0x0B);

			// SE - Set Entry Mode
			send(0, LCD_CMD_SETENTRYMODE | emCursorIncWrite );

			// SU - Set Underline Mode
			send(0, LCD_CMD_SETUNDERLINEMODE | suNoUnderline );

			// SB - Set Blinking Frequency
			send(0, LCD_CMD_SETBLINKINGFREQUENCY | sbFastest );

			// SD - Set Display Mode
			send(0, LCD_CMD_SETDISPLAYMODE | dmDisplayOn
					/*| dmCursorOn */ | dmCursorBlink);

			// CH - Clear Cursor of DD RAM display data home & display start address home
			send(0, LCD_CMD_CLEARCURSORDATAADDRHOME);

			_delay_ms(10);
		}

		void setCursorPos(uint8_t row, uint8_t column) {
			send(_BV(IOC1) | _BV(IOC2), column + (row * 40));
		}

		Display* write(const char * c) {
			while (*c) {
				write(*c++);
			}
			return this;
		}

		void write(char c) {
			send(_BV(IOC2), c);
		}

		void clear() {
			send(0, LCD_CMD_CLEARCURSORDATAADDRHOME);
			setCursorPos(0, 0);
		}

		Display* write(const uint8_t row, const uint8_t column, const char* text) {
			setCursorPos(row, column);
			write(text);
			return this;
		}

		Display* writeInt(const int16_t i) {
			char buf[8];
			ltoa(i, buf, 10);
			write(buf);
			return this;
		}

		Display* writeInt4(const int16_t i) {
			if (i >= 0) write(" ");
			int16_t j = (i < 0) ? -i : i;
			if (j < 100) write(" ");
			if (j < 10) write(" ");
			char buf[8];
			ltoa(i, buf, 10);
			write(buf);
			return this;
		}

		Display* writeInt4x3(const int16_t x, const int16_t y, const int16_t z) {
			writeInt4(x);
			write(",");
			writeInt4(y);
			write(",");
			writeInt4(z);
			return this;
		}

		Display* writeInt(const uint8_t row, const uint8_t column, const int16_t i) {
			setCursorPos(row, column);
			writeInt(i);
			return this;
		}

		Display* writeUint(const uint16_t i) {
			char buf[7];
			ultoa(i, buf, 10);
			write(buf);
			return this;
		}

		Display* writeFloat(float f) {
			char fstr[8];

			dtostrf(f, 8, 4, fstr);
			write(fstr);
			return this;
		}

		Display* writeUint3(const uint16_t i) {
			if (i < 100) write(" ");
			if (i < 10) write(" ");
			char buf[7];
			ultoa(i, buf, 10);
			write(buf);
			return this;
		}

		Display* writeUint3x3(const uint16_t x, const uint16_t y, const uint16_t z) {
			writeUint3(x);
			write(",");
			writeUint3(y);
			write(",");
			writeUint3(z);
			return this;
		}

		Display* writeUInt(const uint8_t row, const uint8_t column, const uint16_t i) {
			setCursorPos(row, column);
			writeUint(i);
			return this;
		}
};

#endif /* DISPLAY_H_ */
