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

#ifndef L3G4200D_H_
#define L3G4200D_H_

#include "timer.h"
#include "myMath.h"

/*
 * Includefile for Gyrometer L3G4200
 */

//Register Map							Type	Default
#define GYRO_WHO_AM_I		0x0F 	//	r		11010011
#define GYRO_CTRL_REG1		0x20	//	rw		00000111
#define GYRO_CTRL_REG2		0x21	//	rw		00000000
#define GYRO_CTRL_REG3		0x22	//	rw		00000000
#define GYRO_CTRL_REG4		0x23	//	rw		00000000
#define GYRO_CTRL_REG5		0x24	//	rw		00000000
#define GYRO_REFERENCE		0x25	//	rw		00000000
#define GYRO_OUT_TEMP		0x26	//	r		output
#define GYRO_STATUS_REG		0x27	//	r		output
#define GYRO_OUT_X_L		0x28	//	r		output
#define GYRO_OUT_X_H		0x29	//	r		output
#define GYRO_OUT_Y_L		0x2A	//	r		output
#define GYRO_OUT_Y_H		0x2B	//	r		output
#define GYRO_OUT_Z_L		0x2C	//	r		output
#define GYRO_OUT_Z_H		0x2D	//	r		output
#define GYRO_FIFO_CTRL_REG	0x2E	//	rw		00000000
#define GYRO_FIFO_SRC_REG	0x2F	//	r		output
#define GYRO_INT1_CFG		0x30	//	rw		00000000
#define GYRO_INT1_SRC		0x31	//	r		output
#define GYRO_INT1_TSH_XH	0x32	//	rw		00000000
#define GYRO_INT1_TSH_XL	0x33	//	rw		00000000
#define GYRO_INT1_TSH_YH	0x34	//	rw		00000000
#define GYRO_INT1_TSH_YL	0x35	//	rw		00000000
#define GYRO_INT1_TSH_ZH	0x36	//	rw		00000000
#define GYRO_INT1_TSH_ZL	0x37	//	rw		00000000
#define GYRO_INT1_DURATION	0x38	//	rw		00000000

#define GYRO_RW_BIT			0x80
#define GYRO_MS_BIT			0x40

// CTRL_REG1
#define GYRO_PD_NormalMode	_BV(3)
#define GYRO_PD_PowerDown	0
#define GYRO_X_Axis_Enable	_BV(0)
#define GYRO_Y_Axis_Enable	_BV(1)
#define GYRO_Z_Axis_Enable	_BV(2)

// CTRL_REG5
#define GYRO_CTRL5_FIFO_EN	_BV(6)
#define GYRO_CTRL5_HP_EN	_BV(5)

// FIFO_CTRL_REG
#define GYRO_FIFO_MODE_Bypass	0
#define GYRO_FIFO_MODE_Fifo		_BV(5)
#define GYRO_FIFO_MODE_Stream	_BV(6)

#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include "SPI.h"

template <uint8_t CS, uint8_t SCKL, uint8_t DOUT, uint8_t DIN>
class L3G4200D {
	private:
		/**
		 * Deklariere die SPI-Klasse, die benutzt werden soll. In diesem
		 * Fall ist es eine Software-SPI.
		 */
		SPI<SCKL, DIN, DOUT, 0> spi;

		/**
		 * Port, an dem der ADXL345 angeschlossen ist.
		 */
		PORT_t * port;

		float gyroRate[3];
		float gyroScaleFactor;

		bool useFIFO;

		/**
		 * Diese Array enthält alle letzten n Werte für das Smoothing-Verfahren und
		 * arbeitet wie ein Ringpuffer.
		 */
		MeanValue<int16_t, int32_t> smoothValues[3];

		uint8_t read(char register_address) {
			uint8_t read_address = GYRO_RW_BIT | register_address;
			uint8_t register_value = 0;

			port->OUTCLR = _BV(CS);

			spi.write(read_address);
			register_value = spi.read();

			port->OUTSET = _BV(CS);

			return register_value;
		}

		void write(uint8_t register_address, uint8_t register_value) {
			port->OUTCLR = _BV(CS);

			spi.write(register_address);
			spi.write(register_value);

			port->OUTSET = _BV(CS);
		}

		/**
		 * Liest die Rohdaten der drei Achsen aus den normalen
		 * Registern aus.
		 *
		 * @param xyz Enthält nach dem Aufruf die Rotationsänderung aller Achsen.
		 */
		void readData(int16_t* xyz) {
			uint8_t read_address;
			struct {
					union {
							int16_t i;
							uint16_t u;
					};
			} in;
			read_address = ACC_RW_BIT | ACC_MB_BIT | ACC_DATAX0;

			port->OUTCLR = _BV(CS);

			spi.write(read_address);
			in.u = spi.read();
			in.u |= spi.read() << 8;
			xyz[0] = in.i;
			in.u = spi.read();
			in.u |= spi.read() << 8;
			xyz[1] = in.i;
			in.u = spi.read();
			in.u |= spi.read() << 8;
			xyz[2] = in.i;

			port->OUTSET = _BV(CS);
		}

		void readXYZ(int16_t* xyz) {
			if (!useFIFO) {
				readData(xyz);
				return;
			}
			uint8_t entries = read(GYRO_FIFO_SRC_REG) & 0x1f; // Nur die letzten 5 Bits lesen
			xyz[0] = xyz[1] = xyz[2] = 0;

			for (uint8_t i = 0; i < entries; i++) {
				readData(xyzTmp);
				xyz[0] += xyzTmp[0];
				xyz[1] += xyzTmp[1];
				xyz[2] += xyzTmp[2];
			}
		}

	public:
		enum Bandwidth {
			Bandwidth_100 = (0b00 << 6),
			Bandwidth_200 = (0b01 << 6),
			Bandwidth_400 = (0b10 << 6),
			Bandwidth_800 = (0b11 << 6)
		};

		enum CutOff {
			CutOff_12_5 = (0b00 << 4),
			CutOff_20   = (0b00 << 4),
			CutOff_30   = (0b00 << 4),
			CutOff_25   = (0b01 << 4),
			CutOff_35   = (0b01 << 4),
			CutOff_50   = (0b10 << 4),
			CutOff_70   = (0b11 << 4),
			CutOff_110  = (0b11 << 4)
		};

		int16_t gyroZero[3];

		L3G4200D(PORT_t & PORT) : spi(PORT) {
			port = &PORT;
			port->DIRSET = _BV(CS);
			port->OUTSET = _BV(CS);

			for (uint8_t i = 0; i < 3; i++) {
				gyroRate[i] = 0.0;
				gyroZero[i] = 0;
			}

			// Umrechnung für Fullscale select 2000 dps
			gyroScaleFactor = 2000.0 * M_PI / (32768.0 * 180.0);

			enableFIFO(false);

			_delay_ms(1);
		}

		~L3G4200D() {
		}

		/**
		 * Stellt ein über wie viele Werte der Mittelwert gebildet werden soll, wenn
		 * die Funktion measureSmooth() aufgerufen wird. Der Mittelwert wird gebildet,
		 * indem measureSmooth() einfach mehrfach aufgerufen wird.
		 *
		 * @param smoothCount Die Anzahl der Werte, über die der Mittelwert gebildet
		 *        werden soll.
		 */
		void setSmooth(uint8_t smoothCount) {
			for (uint8_t axis = 0; axis < 3; axis++) {
				smoothValues[axis].setCapacity(smoothCount);
			}
		}

		float x() {
			return gyroRate[0];
		}
		float y() {
			return gyroRate[1];
		}
		float z() {
			return gyroRate[2];
		}

		bool isDeviceOk() {
			return (read(GYRO_WHO_AM_I) == 0b11010011);
		}

		void setBandwidth(Bandwidth bandwith, CutOff cutOff) {
			write(GYRO_CTRL_REG1, bandwith | cutOff | GYRO_PD_NormalMode | GYRO_X_Axis_Enable | GYRO_Y_Axis_Enable | GYRO_Z_Axis_Enable);
		}

		/**
		 * Aktiviert oder deaktiviert den im L3G4200D enthaltenen FIFO-Puffer
		 * und setzt ihn in den Stream-Modus, sodass immer die aktuellsten
		 * Daten vorhanden sind, auch wenn der Puffer überläuft.
		 *
		 * @param state
		 */
		void enableFIFO(bool state = true) {
			useFIFO = state;
			if (state) {
				write(GYRO_CTRL_REG5, GYRO_CTRL5_FIFO_EN);
				write(GYRO_FIFO_CTRL_REG, GYRO_FIFO_MODE_Stream);
			} else {
				write(GYRO_CTRL_REG5, 0);
				write(GYRO_FIFO_CTRL_REG, GYRO_FIFO_MODE_Bypass);
			}
		}

		void setDefaults() {
			setBandwidth(Bandwidth_100, CutOff_12_5);

			write(GYRO_CTRL_REG2, 0b00000000);
			write(GYRO_CTRL_REG3, 0b00000000);
			write(GYRO_CTRL_REG4, 0b10110000);	//Full Scale Selection:2000dps

			enableFIFO(true);
		}

		/**
		 * Berechnet den Mittelwert über die letzten n Werte. Über wie
		 * viele Werte der Mittelwert gebildet werden soll, lässt sich
		 * mit setSmooth() einstellen. Die gemittelten Werte können mit
		 * den entsprechenden Gettern ausgelesen werden.
		 */
		void measureSmooth() {
			int16_t values[3];

			readXYZ(values);

			for (uint8_t axis = 0; axis < 3; axis++) {
				gyroRate[axis] = smoothValues[axis](values[axis] - gyroZero[axis]) * gyroScaleFactor;
			}
		}

		void measure() {
			int16_t values[3];

			readXYZ(values);

			for (uint8_t axis = 0; axis < 3; axis++) {
				gyroRate[axis] = (values[axis] - gyroZero[axis]) * gyroScaleFactor;
			}
		}

		void calibrate(uint8_t samples = 100) {
			int32_t zero[3] = {0, 0, 0};

			for (uint8_t i = 0; i < samples; i++) {
				int16_t values[3];
				readXYZ(values);
				for (uint8_t axis = 0; axis < 3; axis++) {
					zero[axis] += values[axis];
				}
				_delay_ms(10);
			}

			gyroZero[0] = zero[0] / samples;
			gyroZero[1] = zero[1] / samples;
			gyroZero[2] = zero[2] / samples;
		}
};

#endif /* L3G4200D_H_ */
