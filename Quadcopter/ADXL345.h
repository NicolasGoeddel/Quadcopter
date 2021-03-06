/**
 * Copyright 2012-2014 Nicolas Göddel
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
 * along with Quadcopter. If not, see <http://www.gnu.org/licenses/>.
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

#ifndef ADXL345_H_
#define ADXL345_H_

#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include "SPI.h"
#include "myMath.h"
#include <avr/interrupt.h>
#include "MeanValue.h"

/* PACKAGE
 *
 * +---------+
 * | o       |         △ +x
 * | 3 4 5 B |         ⎟
 * | # y w w |   +y    ⎟
 * | v v v v |   ◁⎯⎯⎯⎯⎯◎     z geht aus der Ebene raus.
 * | C N T Y |
 * +---------+
 *
 * Wenn der Pfeil Richtung Erde zeigt, sollte die Beschleunigung -9,81 m/s² betragen.
 *
 */

/**
 * Wenn USE_OFFSET auf 1 steht, dann werden die Hardware-Offsets benutzt,
 * andernfalls wird der Bias softwaremäßig berechnet. Wir benutzen die
 * Möglichkeit mit den Offset-Register im ADXL345 nicht, da sie scheinbar
 * nicht korrekt geschrieben oder gelesen werden können. Entweder ist unser
 * Chip kaputt oder es liegt an anderen unerklärlichen Gründen.
 */
#define USE_OFFSET 0

//*******************************************************
//						ADXL345 Definitions
//*******************************************************
#define READ	0x8000

//ADXL Register Map
#define ACC_DEVID			0x00	//Device ID Register
#define ACC_THRESH_TAP		0x1D	//Tap Threshold
#define ACC_OFSX			0x1E	//X-axis offset
#define ACC_OFSY			0x1F	//Y-axis offset
#define ACC_OFSZ			0x20	//Z-axis offset
#define ACC_DUR				0x21	//Tap Duration
#define ACC_Latent			0x22	//Tap latency
#define ACC_Window			0x23	//Tap window
#define ACC_THRESH_ACT		0x24	//Activity Threshold
#define ACC_THRESH_INACT	0x25	//Inactivity Threshold
#define ACC_TIME_INACT		0x26	//Inactivity Time
#define ACC_ACT_INACT_CTL	0x27	//Axis enable control for activity and inactivity detection
#define ACC_THRESH_FF		0x28	//free-fall threshold
#define ACC_TIME_FF			0x29	//Free-Fall Time
#define ACC_TAP_AXES		0x2A	//Axis control for tap/double tap
#define ACC_ACT_TAP_STATUS	0x2B	//Source of tap/double tap
#define ACC_BW_RATE			0x2C	//Data rate and power mode control
#define ACC_POWER_CTL		0x2D	//Power Control Register
#define ACC_INT_ENABLE		0x2E	//Interrupt Enable Control
#define ACC_INT_MAP			0x2F	//Interrupt Mapping Control
#define ACC_INT_SOURCE		0x30	//Source of interrupts
#define ACC_DATA_FORMAT		0x31	//Data format control
#define ACC_DATAX0			0x32	//X-Axis Data 0
#define ACC_DATAX1			0x33	//X-Axis Data 1
#define ACC_DATAY0			0x34	//Y-Axis Data 0
#define ACC_DATAY1			0x35	//Y-Axis Data 1
#define ACC_DATAZ0			0x36	//Z-Axis Data 0
#define ACC_DATAZ1			0x37	//Z-Axis Data 1
#define ACC_FIFO_CTL		0x38	//FIFO control
#define ACC_FIFO_STATUS		0x39	//FIFO status

//Power Control Register Bits
#define ACC_WU_0			_BV(0)	//Wake Up Mode - Bit 0
#define ACC_WU_1			_BV(1)	//Wake Up mode - Bit 1
#define ACC_SLEEP			_BV(2)	//Sleep Mode
#define ACC_MEASURE			_BV(3)	//Measurement Mode
#define ACC_AUTO_SLP		_BV(4)	//Auto Sleep Mode bit
#define ACC_LINK			_BV(5)	//Link bit

//Interrupt Enable/Interrupt Map/Interrupt Source Register Bits
#define ACC_OVERRUN			_BV(0)
#define ACC_WATERMARK		_BV(1)
#define ACC_FREE_FALL		_BV(2)
#define ACC_INACTIVITY		_BV(3)
#define ACC_ACTIVITY		_BV(4)
#define ACC_DOUBLE_TAP		_BV(5)
#define ACC_SINGLE_TAP		_BV(6)
#define ACC_DATA_READY		_BV(7)

//FIFO Modes
#define ACC_FIFO_Bypass		0
#define ACC_FIFO_FIFO		_BV(6)
#define ACC_FIFO_Stream		_BV(7)
#define ACC_FIFO_Trigger	(_BV(6) | _BV(7))

//Data Format Bits
#define ACC_RANGE_0			_BV(0)
#define ACC_RANGE_1			_BV(1)
#define ACC_JUSTIFY			_BV(2)
#define ACC_FULL_RES		_BV(3)

#define ACC_INT_INVERT		_BV(5)
#define ACC_USE_3WIRE_SPI	_BV(6)
#define ACC_SELF_TEST		_BV(7)

//	RANGE SHORTCUTS
#define ACC_RANGE_2G		0
#define ACC_RANGE_4G		ACC_RANGE_0
#define ACC_RANGE_8G		ACC_RANGE_1
#define ACC_RANGE_16G		(ACC_RANGE_0 | ACC_RANGE_1)

#define ACC_RW_BIT			0x80
#define ACC_MB_BIT			0x40

/**
 * Die Templateparameter definieren die Pin-Nummern des Ports,
 * der im Konstruktor übergeben wird.
 */
template <uint8_t CS, uint8_t MOSI, uint8_t MISO, uint8_t SCLK>
class ADXL345 {
	private:
		/**
		 * Deklariere die SPI-Klasse, die benutzt werden soll. In diesem
		 * Fall ist es eine Software-SPI.
		 */
		SPI<SCLK, MISO, MOSI, 0> spi;

		/**
		 * Port, an dem der ADXL345 angeschlossen ist.
		 */
		PORT_t * port;

		/**
		 * Der Skalierungsfaktor um die originalen Ganzzahlwerte aus dem Chip
		 * in m/s² umzurechnen. Er ändert sich je nachdem wie @range eingestellt
		 * wurde.
		 */
		float scaleFactor;

		/**
		 * Dieses Array enthält die Beschleunigung in alle drei Richtungen in m/s²,
		 * nachdem measure() oder measureSmooth() aufgerufen wurde.
		 */
		float meterPerSecSec[3];

		/**
		 * Hier werden die gemittelten Werte der drei Beschleunigungsachsen
		 * berechnet und gespeichert. Sie werden genutzt bei measureSmooth().
		 */
		MeanValue<int16_t, int32_t> smoothValues[3];

		/**
		 * Der eingestellte Messbereich. Mögliche Werte sind 2, 4, 8 und 16.
		 */
		uint8_t range;

		/**
		 * Offset-Werte für die Offsetregister im ADX345.
		 */
		int32_t offset[3];
		int8_t offset8[3];

		/**
		 * Wenn diese Variable auf true steht, dann wird der FIFO-Puffer
		 * benutzt, indem alle im FIFO enthaltenen Werte ausgelesen und
		 * gemittelt werden. So kann man die Samplerate maximal 32 mal
		 * höher setzen als die Geschwindigkeit, mit der man measure()
		 * bzw. measureSmooth() aufruft. Auf diese Weise verliert man auch
		 * keine Werte, die bei einer Auslesefrequenz, die niedriger als
		 * die SampleRate ist, verloren gingen.
		 * Standardmäßig ist dieser Wert false und muss mit enableFIFO(true)
		 * zunächst auf true gesetzt werden. Mit enableFIFO(false) kann man
		 * den FIFO wieder deaktivieren.
		 * Genauer gesagt wird hier der Stream-Mode aus dem Datenblatt
		 * benutzt, da der beim Überlaufen des FIFOs immer die aktuellsten
		 * Werte beinhaltet.
		 */
		bool useFIFO;

		/**
		 * Die Geschwindigkeit, in der intern neue Samples acquiriert werden
		 * in Hz.
		 */
		enum SampleRate {
			SampleRate_3200	= 0b1111,//!< SampleRate_3200
			SampleRate_1600	= 0b1110,//!< SampleRate_1600
			SampleRate_800	= 0b1101, //!< SampleRate_800
			SampleRate_400	= 0b1100, //!< SampleRate_400
			SampleRate_200	= 0b1011, //!< SampleRate_200
			SampleRate_100	= 0b1010, //!< SampleRate_100
			SampleRate_50	= 0b1001,  //!< SampleRate_50
			SampleRate_25	= 0b1000,  //!< SampleRate_25
			SampleRate_12_5	= 0b0111,//!< SampleRate_12_5
			SampleRate_6_25 = 0b0110,//!< SampleRate_6_25
			SampleRate_3_13 = 0b0101,//!< SampleRate_3_13
			SampleRate_1_56	= 0b0100,//!< SampleRate_1_56
			SampleRate_0_78	= 0b0011,//!< SampleRate_0_78
			SampleRate_0_39	= 0b0010,//!< SampleRate_0_39
			SampleRate_0_20	= 0b0001,//!< SampleRate_0_20
			SampleRate_0_10	= 0b0000 //!< SampleRate_0_10
		};
		uint16_t sampleRate;

		/**
		 * Gibt den Inhalt eines bestimmten Registers vom ADXL345 zurück.
		 * @param register_address Das auszulesende Register.
		 * @return Der Inhalt des Registers.
		 */
		uint8_t read(uint8_t register_address) {
			uint8_t read_address = ACC_RW_BIT | register_address;
			uint8_t register_value = 0;

			port->OUTCLR = _BV(CS);

			spi.write(read_address);
			register_value = spi.read();

			port->OUTSET = _BV(CS);

			return register_value;
		}

		/**
		 * Schreibt einen Wert in ein bestimmtes Register.
		 * @param register_address Das Register, in das geschrieben werden soll.
		 * @param register_value Der Wert, der in das Register geschrieben werden soll.
		 */
		void write(uint8_t register_address, uint8_t register_value) {
			port->OUTCLR = _BV(CS);

			spi.write(register_address);
			spi.write(register_value);

			port->OUTSET = _BV(CS);
		}

		void writeOffset(int8_t* xyz) {
			uint8_t write_address = ACC_MB_BIT | ACC_OFSX;
			port->OUTCLR = _BV(CS);

			spi.write(write_address);
			spi.write(xyz[0]);
			spi.write(xyz[1]);
			spi.write(xyz[2]);

			port->OUTSET = _BV(CS);
		}

		/**
		 * Liest die Rohdaten der drei Achsen aus den normalen
		 * Registern aus.
		 *
		 * @param xyz Enthält nach dem Aufruf die Beschleunigung aller Achsen.
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

		/**
		 * Liest die Rohdaten der drei Achsen aus den normalen
		 * Registern aus und achtet bei aktiviertem FIFO-Puffer darauf
		 * alle Werte aus dem Puffer zu lesen und zu mitteln, falls
		 * mehr als einer drin steht.
		 *
		 * @param xyz Enthält nach dem Aufruf die Beschleunigung aller Achsen.
		 */
		void readXYZ(int16_t* xyz) {
			if (!useFIFO) {
				readData(xyz);
				return;
			}
			uint8_t entries = read(ACC_FIFO_STATUS) & 0x3f; // Nur die letzten 6 Bits lesen
			int32_t xyzSum[3] = {0, 0, 0};
			/* Theoretisch könnte man hier auch einfach xyz wieder verwenden
			 * und Stackspeicher sparen.
			 */
			int16_t xyzTmp[3];

			for (uint8_t i = 0; i < entries; i++) {
				readData(xyzTmp);
				xyzSum[0] += xyzTmp[0];
				xyzSum[1] += xyzTmp[1];
				xyzSum[2] += xyzTmp[2];

				/* Zwischen dem Auslesen einzelner Samples aus dem FIFO
				 * müssen mindestens 5 µs gewartet werden. Nach dem
				 * letzten Sample müssen wir allerdings nicht mehr warten.
				 */
				if (i < entries) {
					_delay_us(5);
				}
			}
			xyz[0] = xyzSum[0] / entries;
			xyz[1] = xyzSum[1] / entries;
			xyz[2] = xyzSum[2] / entries;
		}
#endif
	public:
		/**
		 * Initialisiert den ADXL345 und setzt die Standardwerte.
		 *
		 * @param PORT Der Port, an dem der ADXL345 angeschlossen ist.
		 */
		ADXL345(PORT_t & PORT) : spi(PORT) {
			port = &PORT;
			port->DIRSET = _BV(CS);
			port->OUTSET = _BV(CS);
			// Set CS as TOTEM Pole
			PORTCFG.MPCMASK = _BV(CS);
			port->PIN0CTRL = PORT_OPC_TOTEM_gc;

			for (uint8_t i = 0; i < 3; i++) {
				meterPerSecSec[i] = 0.0;
			}

			sampleRate = SampleRate_100;
			range = 2;
			scaleFactor = ENV_G * (float)range / 32768.0;

			enableFIFO(false);

			_delay_ms(1);
		}

		~ADXL345() {}

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

		/**
		 * Legt den Messbereich in g fest. Der maximale Wert ist 16 g (+/- 16 * 9,81 m/s²).
		 * Abstufungen sind 2, 4, 8 und 16. Wird ein Wert zwischen zwei Stufen gewählt,
		 * wird der nächst höhere benutzt. Wird ein Wert größer als 16 übergeben, wird
		 * 16 genommen.
		 * Wenn man die Range umgestellt hat, ist eine erneute Kalibrierung
		 * nicht notwendig, da die Kalibrierungsdaten im ADXL345 automatisch
		 * an verschiedene Bereiche angepasst wird.
		 *
		 * @param g Der gewünschte Messbereich in 9,81 m/s².
		 */
		void setRange(uint8_t g) {
			uint8_t mode;
			if (g > 8) {
				range = 16;
				mode = ACC_RANGE_16G;
			} else if (g > 4) {
				range = 8;
				mode = ACC_RANGE_8G;
			} else if (g > 2) {
				range = 4;
				mode = ACC_RANGE_4G;
			} else {
				range = 2;
				mode = ACC_RANGE_2G;
			}
			scaleFactor = ENV_G * (float)range / 32768.0;
			write(ACC_DATA_FORMAT, /* _BV(ACC_SELF_TEST) | ACC_USE_3WIRE_SPI | */ ACC_FULL_RES | ACC_JUSTIFY | mode);
		}

		/**
		 * Legt die Samplingfrequenz in Hz fest. Hiermit wird bestimmt,
		 * wie oft pro Sekunde die Daten im internen Register aktualisiert
		 * werden. Wenn man den internen FIFO nutzen möchte (was in dieser
		 * Klasse bisher nicht unterstützt wird), dann gibt diese Frequenz
		 * auch an, wie häufig neue Werte in den FIFO gepusht werden.
		 *
		 * @param sr Die Samplingfrequenz aus der entsprechenden Enumeration.
		 */
		void setSampleRate(SampleRate sr) {
			sampleRate = sr;

			write(ACC_BW_RATE, sr);
		}

		/**
		 * Aktiviert oder deaktiviert den im ADXL345 enthaltenen FIFO-Puffer
		 * und setzt ihn in den Stream-Modus, sodass immer die aktuellsten
		 * Daten vorhanden sind, auch wenn der Puffer überläuft.
		 *
		 * @param state
		 */
		void enableFIFO(bool state = true) {
			useFIFO = state;
			if (state) {
				write(ACC_FIFO_CTL, ACC_FIFO_Stream | 32);
			} else {
				write(ACC_FIFO_CTL, ACC_FIFO_Bypass | 32);
			}
		}

		/**
		 * Gibt die aktuelle Beschleunigung auf der X-Achse zurück.
		 *
		 * @return Beschleunigung auf der X-Achse.
		 */
		float inline x() {
			return meterPerSecSec[0];
		}
		/**
		 * Gibt die aktuelle Beschleunigung auf der Y-Achse zurück.
		 *
		 * @return Beschleunigung auf der Y-Achse.
		 */
		float inline y() {
			return meterPerSecSec[1];
		}
		/**
		 * Gibt die aktuelle Beschleunigung auf der Z-Achse zurück.
		 *
		 * @return Beschleunigung auf der Z-Achse.
		 */
		float inline z() {
			return meterPerSecSec[2];
		}

		/**
		 * Gibt den absoluten Drehwinkel um die X-Achse in Rad an.
		 *
		 * @return Drehwinkel um die X-Achse in Rad.
		 */
		float inline angleX() {	// returns 0 if z=1 and y=0
			return myAtan2(meterPerSecSec[1], meterPerSecSec[2]);
		}
		/**
		 * Gibt den absoluten Drehwinkel um die Y-Achse in Rad an.
		 *
		 * @return Drehwinkel um die Y-Achse in Rad.
		 */
		float inline angleY() {
			return -myAtan2(meterPerSecSec[0], meterPerSecSec[2]);
		}
		/**
		 * Gibt den absoluten Drehwinkel um die Z-Achse in Rad an.
		 *
		 * @return Drehwinkel um die Z-Achse in Rad.
		 */
		float inline angleZ() { //TODO
			return -myAtan2(meterPerSecSec[1], meterPerSecSec[0]);
		}

		float inline angleCosXY() {
			float x = angleX();
			float y = angleY();
			float a = x * x + y * y;
			return 1.0 - a * (0.5 + 0.041666 * a); // Series expansion for sqrt(cos(a))
		}

		/**
		 * Stellt die Standardeinstellungen für den ADXL345 ein. Diese
		 * Funktion kann man je nach Projekt anpassen, wenn man will.
		 * Wenn man die Range umgestellt hat, ist eine erneute Kalibrierung
		 * nicht notwendig, da die Kalibrierungsdaten im ADXL345 automatisch
		 * an verschiedene Bereiche angepasst wird.
		 */
		void setDefaults() {
			write(ACC_POWER_CTL, 0);

			setRange(2);

			setSampleRate(SampleRate_800);

			enableFIFO(true);

			write(ACC_POWER_CTL, ACC_MEASURE);	//Put the Accelerometer into measurement mode
			_delay_ms(10);
		}

		/**
		 * Stellt fest, ob die Device-ID korrekt ausgelesen werden kann.
		 *
		 * @return Gibt true zurück, wenn die Device-ID stimmt, sonst false.
		 */
		bool inline isDeviceOk() {
			return (read(ACC_DEVID) == 0b11100101);
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
#if USE_OFFSET
				meterPerSecSec[axis] = smoothValues[axis](values[axis]) * scaleFactor;
#else
				meterPerSecSec[axis] = smoothValues[axis](values[axis] - offset[axis]) * scaleFactor;
#endif
			}
		}

		/**
		 * Berechnet einen LowPass-Filter über die gemessenen Werte.
		 * Die Werte können dann mit den entsprechenden Gettern
		 * ausgelesen werden.
		 * Siehe auch: https://en.wikipedia.org/wiki/Low-pass_filter#Simple_infinite_impulse_response_filter
		 */
		void measureLowPass() {
			int16_t values[3];

			readXYZ(values);

			for (uint8_t axis = 0; axis < 3; axis++) {
#if USE_OFFSET
				meterPerSecSec[axis] = meterPerSecSec[axis] + 0.25 * (values[axis] * scaleFactor - meterPerSecSec[axis]);
#else
				meterPerSecSec[axis] = meterPerSecSec[axis] + 0.25 * ((values[axis]  - offset[axis]) * scaleFactor - meterPerSecSec[axis]);
				// Keine Ahnung, ob diese Zeile vielleicht auch schneller ist
				//meterPerSecSec[axis] = 0.25 * (values[axis]  - offset[axis]) * scaleFactor + 0.75 * meterPerSecSec[axis];
#endif
			}
		}

		/**
		 * Liest die aktuelle Beschleunigung aus dem ADXL345 aus, ohne
		 * den Wert über vorherige Werte zu mitteln. Die ermittelten
		 * Werte lassen sich dann mit den passenden Gettern auslesen.
		 */
		void measure() {
			int16_t values[3];

			readXYZ(values);

			for (uint8_t axis = 0; axis < 3; axis++) {
#if USE_OFFSET
				meterPerSecSec[axis] = values[axis] * scaleFactor;
#else
				meterPerSecSec[axis] = (values[axis] - offset[axis]) * scaleFactor;
#endif
			}
		}

		/**
		 * Diese Funktion sollte einmalig aufgerufen werden, wenn der
		 * Sensor in der Horizontalen liegt. Es werden insgesamt 100
		 * Werte im zeitlichen Abstand von ca. 10 ms acquiriert,
		 * anschließend gemittelt und als Bias verwendet.
		 *
		 * @return	Gibt true zurück, wenn der berechnete Offset in
		 *          8-Bit-Offset-Register passt, sonst false.
		 */
		bool calibrate(uint8_t samples = 100) {
			bool error = false;
#if USE_OFFSET
			// Setze Offsetregister auf 0
			writeOffset(0, 0, 0);
#endif

			int16_t values[3];

			// Berechne den Mittelwert über 100 Samples
			int32_t accelSampleSum[3] = {0, 0, 0};
			for (uint8_t sample = 0; sample < samples; sample++) {

				readXYZ(values);
				for (uint8_t axis = 0; axis < 3; axis++) {
					accelSampleSum[axis] += values[axis];
				}
				_delay_ms(10);
			}

			for (uint8_t i = 0; i < 3; i++) {
				/* Diese Offsets arbeiten im vollen Bereich, also 16 Bit, während
				 * die Offsetregister nur mit 8 Bit arbeiten und nur von -2g bis 2g
				 * laufen.
				 */
				offset[i] = accelSampleSum[i] / samples;
				if (i == 2) {
					offset[2] -= (32768 / range);
				}
				/* Deswegen müssen wir die berechneten Offsets jetzt auf 8 Bit runter
				 * brechen und noch die eingestellte Range beachten.
				 */
#if USE_OFFSET
				offset[i] = -offset[i] / (256 / (range / 2));
				if (offset[i] > 127) {
					error = true;
					offset[i] = 127;
				} else if (offset[i] < -128) {
					error = true;
					offset[i] = -128;
				}
				offset8[i] = offset[i];
#endif
			}

#if USE_OFFSET
			// Schreibe die neuen Offsets in das entsprechende Register.
			writeOffset(offset8);
#endif

			return !error;
		}

		int8_t getOffset(uint8_t axis) {
			if (axis > 2) {
				return 0;
			}
			return offset8[axis];
		}

		int32_t getOffset32(uint8_t axis) {
			if (axis > 2) {
				return 0;
			}
			return offset[axis];
		}

};
