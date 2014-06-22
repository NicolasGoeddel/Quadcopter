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

#include "DEBUG.h"
#include <util/delay.h>

/**
 * Siehe: http://stackoverflow.com/questions/920500/what-is-the-purpose-of-cxa-pure-virtual
 */
extern "C" void __cxa_pure_virtual() {
	while (true) {
		DEBUG_LED(-1);
		_delay_ms(100);
	}
}

#include <stdlib.h>
#include <avr/io.h>
#include "XMEGA_helper.h"
#include "Display.h"
#include "Bluetooth.h"
#include "ADXL345.h"
#include "L3G4200D.h"
#include "PID.h"
#include "CFilter.h"
#include "myMath.h"
#include "DMAController.h"
#include "Transmitter.h"
#include "Clock.h"
#include "myMath.h"

//#define DEBUG_DMA
//#define DEBUG_RF
//#define DEBUG_USART
//#define DEBUG_TEST
#define MAIN_PROGRAM

#define DISPLAY
#define REMOTE
#define BLUETOOTH

/* Port-Map
 *	PORT	BITS	PIN		Beschreibung
 *	C		0-7		15-22	Display
 *	D		0-3		25-28	Motorsteuerung
 *	D		6-7		31-32	Bluetooth-Adapter
 *	E		4-7		39-42	Beschleunigungssensor (SCLK, MOSI, MISO, CS)
 *	E		0-3		35-38	Gyrosensor
 *	F		3-7		48-32	nRF24L01+
 *	H		0-1				Debug-Taster
 *	H		2-3				Debug-LED
 *
 */

const float dT = 1.0 / TIMER_FREQUENCY;

struct Motor_t {
		int16_t m[4];
		Motor_t(int16_t speed) {
			m[0] = m[1] = m[2] = m[3] = speed;
		}
		void set(int16_t speed) {
			m[0] = m[1] = m[2] = m[3] = speed;
		}
		void cutBorders(int16_t low, int16_t high) {
			for (uint8_t i = 0; i < 4; i++) {
				if (m[i] < low) m[i] = low;
				if (m[i] > high) m[i] = high;
			}
		}
		void setPWM(TC0_t &timer) {
			set4ChanPWM(timer, m[0], m[1], m[2], m[3]);
		}
};

Motor_t motor(0);

#ifdef DEBUG_TEST
int main() {
	set32MHz();
	_delay_ms(10);

	// Für LED und Taster.
	DEBUG_init();

	while(1) {
		DEBUG_LED(-1);
		_delay_ms(250);
	}
}
#endif

#ifdef MAIN_PROGRAM
int main() {
	set32MHz();
	_delay_ms(10);

	// Für LED und Taster.
	DEBUG_init();

	// Display initialisieren
	Display display(PORTC);
	display.init();
	_delay_ms(50);

	display.clear();
	_delay_ms(100);

	// init4ChanPWM gibt als Rückgabewert den maximalen DutyCycle zurück
	uint16_t max = init4ChanPWM(PORTD, TCD0, 100);
	display.setCursorPos(0, 0)->write("PWM max=")->writeUint(max);

	//Aktiviere die Interrupts
	activateInterrupts();

	uint8_t errorLine = 0;

	// Initialisiere Beschleunigungssensor
	//template <uint8_t CS, uint8_t MOSI, uint8_t MISO, uint8_t SCLK>
	ADXL345<4, 5, 6, 7> acc(PORTE);
	if (!acc.isDeviceOk()) {
		display.setCursorPos(errorLine++, 0)->write("ACC ERROR!");
	} else {
		acc.setDefaults();
		if (!acc.calibrate()) {
			display.setCursorPos(1, 0)->write("ACC CAL WARN!");
		} else {
			//display.write(1, 0, "ACC:")->writeInt(acc.getOffset32(0))->write(",")->writeInt(acc.getOffset32(1))->write(",")->writeInt(acc.getOffset32(2));
			display.setCursorPos(1, 0)->write("ACC:")->writeInt(acc.getOffset(0))->write(",")->writeInt(acc.getOffset(1))->write(",")->writeInt(acc.getOffset(2));
		}
		acc.setSmooth(10);
	}

	// Initialisiere Gyro
	L3G4200D<2, 1, 0, 3> gyro(PORTE);
	if (!gyro.isDeviceOk()) {
		display.setCursorPos(errorLine++, 0)->write("GYRO ERROR!");
	} else {
		gyro.setDefaults();
		gyro.calibrate();
		gyro.setSmooth(3);
		display.setCursorPos(2, 0)->write("GYRO ok.");
	}

#	ifdef REMOTE
	// Initialisiere die Fernbedienung
	Transmitter remote;
	if (remote.getConfigStatus() != 0) {
		display.setCursorPos(errorLine++, 0)->write("RF Error: ")->writeUint(remote.getConfigStatus());
	} else {
		display.setCursorPos(3, 0)->write("Remote ok.");
	}
#	endif

#	ifdef BLUETOOTH
	// Initialisiere Bluetooth Debugger
	Bluetooth bt(9600);
	if (!bt.isDeviceOk()) {
		display.setCursorPos(errorLine++, 0)->write("BLUETOOTH ERROR!");
	} else {
		display.setCursorPos(0, 0)->write("Bluetooth ok.");
	}
#	endif

	// Wenn ein Fehler aufgetreten ist, beende das Programm, damit man die Fehlermeldung sehen kann.
	if (errorLine > 0) {
		display.setCursorPos(3, 0)->write("STOPPING DEVICE!");
		while (true);
	} else {
		_delay_ms(500);
		display.clear();
		display.setCursorPos(0, 0)->write("Calibrating done");
		_delay_ms(1000);
		display.clear();
	}

	// END INITIALISATION

	// Duty Cycles für verschiedene Geschwindigkeiten
	uint16_t zero     = ((uint32_t) max * 10 / 100);		// 10%
	uint16_t standby  = ((uint32_t) max * 125 / 1000);		// 12.5%
	uint16_t power    = ((uint32_t) max * 130 / 1000);		// 13%
	uint16_t maxPower = ((uint32_t) max * 165 / 1000);		// 17.5%
	uint16_t speed    = zero;

	/*float x, y, z;
	x = y = z = 0; */

	// Einstellung für Remote
	#define MAX_ANGLE_DEG 45
	#define MAX_ANGLE_RAD (MAX_ANGLE_DEG * 0.017453293)

	// Einstellung für die PID-Regler
	#define PID_P 0.2
	#define PID_I 0.5
	#define PID_D 0.0576
	#define PID_I_Min -5.0
	#define PID_I_Max 5.0
	#define PID_I_Band 10.0
	#define PID_Scale 0.12
	PID pidX(PID_P * PID_Scale, PID_I * PID_Scale, PID_D * PID_Scale, dT);
	pidX.setIBand(PID_I_Band, PID_I_Min, PID_I_Max);
	PID pidY(PID_P * PID_Scale, PID_I * PID_Scale, PID_D * PID_Scale, dT);
	pidY.setIBand(PID_I_Band, PID_I_Min, PID_I_Max);

	CFilter filterX(0.045, dT);
	CFilter filterY(0.045, dT);

	float angleX, angleY, angleZ = 0.0;
	float pidAngleX, pidAngleY;
	float accZ = 0.0;
	float speedZ = 0.0;
	float posZ = 0.0;

	motor.set(zero);
	_delay_ms(400);

	Clock clock;

	for (;;) {
		if (clock.eventInterrupt) {
			// Debug LED an
			DEBUG_LED(1);

			// Gyro und Beschleunigungssensor auslesen
			gyro.measure();
			acc.measure();

			// Beschleunigung "normalisieren"
			accZ = (-acc.z() - ENV_G);
			speedZ += accZ * dT;
			posZ += speedZ * dT;
			speedZ *= 0.98;

			// Winkel aus Beschleunigung und Gyro berechnen
			angleX = filterX(acc.angleX(), gyro.x());
			angleY = filterY(acc.angleY(), gyro.y());
			angleZ += gyro.z() * dT;

			// Fernbedienung auslesen
			uint8_t remoteX, remoteY, remoteState, remotePushed;
#			ifdef REMOTE
			remote.getState(remoteX, remoteY, remoteState, remotePushed);
#			else
			remoteX = 128;
			remoteY = 128;
			remoteState = 0;
			remotePushed = 0;
#			endif

			// X-Y-Werte auf -1..1 normalisieren
			float rX = (remoteX - 128) / 128.0;
			float rY = (remoteY - 128) / 128.0;

			/* Linearität der X-Y-Werte aufheben: f(x) = (x^3 + x/2) / 1.5
			 * Das macht kleine Auslenkungen genau und größere stärker.
			 */
			//rX = rX * (rX * rX + 0.5) * 0.666666;
			//rY = rY * (rY * rY + 0.5) * 0.666666;

			// X-Y-Werte auf MAX_ANGLE_RAD strecken
			rX *= MAX_ANGLE_RAD;
			rY *= MAX_ANGLE_RAD;

			// Korrekturwinkel berechnen mit PID-Regler
			pidAngleX = pidX(rX, angleX);
			pidAngleY = pidY(rY, angleY);

			int16_t pwmX = pidAngleX * (maxPower - zero);// * PID_Scale;
			int16_t pwmY = pidAngleY * (maxPower - zero);// * PID_Scale;

			// Debug-Infos ausgeben
#			ifdef DISPLAY
			display.setCursorPos(0, 0)->write("a:")->writeInt4x3(acc.x() * 100, acc.y() * 100, acc.z() * 100);
			display.setCursorPos(1, 0)->write("w:")->writeInt4x3(DEG(angleX), DEG(angleY), DEG(angleZ));
//			display.setCursorPos(2, 0)->write("p:")->writeInt4x3(DEG(pidAngleX), DEG(pidAngleY), DEG(angleZ));
			display.setCursorPos(2, 0)->write("g:")->writeInt4x3(DEG(gyro.x()), DEG(gyro.y()), DEG(gyro.z()));
//			display.setCursorPos(2, 0)->write("r:")->writeInt4x3(remoteX, remoteY, remoteState);
#			endif

#			ifdef BLUETOOTH
			if (remote.getButton(Transmitter::btnFire)) {
				bt.write("Fire!\r\n");
			}
			if (remote.getButton(Transmitter::btnTop)) {
				bt.write("a:")->writeFloat(acc.x())->writeFloat(acc.y())->writeFloat(acc.z());
				bt.write(" g:")->writeFloat(gyro.x())->writeFloat(gyro.y())->writeFloat(gyro.z());
				bt.writeChar(13);
//				bt.writeFloatRaw(NAN);
//				bt.writeFloatRaw(acc.x());
//				bt.writeFloatRaw(acc.y());
//				bt.writeFloatRaw(acc.z());
//				bt.writeFloatRaw(gyro.x());
//				bt.writeFloatRaw(gyro.y());
//				bt.writeFloatRaw(gyro.z());
			}
#			endif

			if (DEBUG_Button()) {
				speed = standby;
#			ifndef REMOTE
			} else {
				speed = zero;
#			endif
			}

#			ifdef REMOTE
			if (remote.getButton(Transmitter::btnFire) || (DEBUG_Button())) {
				speed = standby;
			} else {
				speed = zero;
			}
			if (remote.getButton(Transmitter::btnMiddle)) {
				if (remote.getButton(Transmitter::btnFire)) {
					speed = maxPower;
				} else {
					speed = power;
				}
			}

			if (remote.getButton(Transmitter::btnBottom)) {
				display.clear();
				display.setCursorPos(0, 0)->write("Calibrating...");
				uint8_t tmp = (acc.calibrate()) ? 1 : 0;
				gyro.calibrate();
				display.setCursorPos(1, 0)->write("GYRO calibrated.");
				display.setCursorPos(2, 0)->write("ACC error= ")->writeUint(tmp);
				pidX.reset();
				pidY.reset();
				posZ = 0.0;
				_delay_ms(200);
			}
#			endif

			if (speed != zero) {
				// Motormischer
				motor.m[0] = speed - pwmY;
				motor.m[2] = speed + pwmY;
				motor.m[1] = speed + pwmX;
				motor.m[3] = speed - pwmX;
				motor.cutBorders(0, maxPower);
			} else {
				motor.set(zero);
			}

			motor.setPWM(TCD0);

			clock.eventInterrupt = false;

			// Debug LED aus
			DEBUG_LED(0);
		}

#		ifdef DISPLAY
		int16_t perc;
//		perc = ((int32_t)m1 * 100) / max;
//		display.write(2, 0, "m1: ")->writeInt(perc)->write("%  ");
//		perc = ((int32_t)m3 * 100) / max;
//		display.write(2, 8, "m3: ")->writeInt(perc)->write("%  ");
		//perc = ((int32_t)(motor.m[1] - zero) * 100) / (maxPower - zero);
		perc = ((int32_t)motor.m[1] * 100) / max;
		display.setCursorPos(3, 0)->write("m1:")->writeInt4(perc)->write("%");
		//perc = ((int32_t)(motor.m[3] - zero) * 100) / (maxPower - zero);
		perc = ((int32_t)motor.m[3] * 100) / max;
		display.setCursorPos(3, 0)->write("m3:")->writeInt4(perc)->write("%");
#		endif
	}
}
#endif

#ifdef DEBUG_USART
void shiftBytes(Display* d, Bluetooth* bt, uint8_t chars) {
	for (uint8_t i = 0; i < chars; i++) {
		d->write(bt->getChar());
	}
}

int main() {
	set32MHz();
	_delay_ms(100);

	// Display initialisieren
	Display display(PORTC);
	display.init();
	_delay_ms(50);

	display.clear();
	_delay_ms(100);

	Bluetooth bt(9600);

	display.write(0, 1, "init...");

	bt.writeString("AT");
	display.setCursorPos(1, 1);
	shiftBytes(&display, &bt, 2);

	bt.writeString("AT+BAUD4");
	display.setCursorPos(2, 1);
	shiftBytes(&display, &bt, 6);

	display.write(3, 1, "done.");

	_delay_ms(250);

	uint8_t c = 0;

	while(true) {
		bt.writeChar(c++);

		char input = bt.getCharAsync();
		if (input) {
			display.write(input);
		}
	}
}
#endif

#ifdef DEBUG_DMA
int main() {
	set32MHz();
	_delay_ms(100);
	DMAController::reset();

	Display display(PORTC);
	display.init();

	display.write(0, 0, "DMA Test.");
	display.write(1, 0, "Copy a to b");
	_delay_ms(100);

	char* a = "Hallo Welt!12345";
	char* b = "----------------";

	DMAController::enable();
	DMAController::useChannel(0);
	DMAController::setBlockSize(0, 16);

	DMAController::setSource(0, a + 15, DMAController::SrcDirectionDec, DMAController::SrcReloadBlock);

	DMAController::setDestination(0, b, DMAController::DestDirectionInc, DMAController::DestReloadBlock);

	DMAController::setRepeatCount(0, 1);
	DMAController::setBurstLength(0, DMAController::BurstLength1Byte);
	DMAController::useSingleShot(0, true);
	DMAController::setTriggerSource(0, DMA_CH_TRIGSRC_OFF_gc);

	DMAController::enable(0);

	for (uint8_t i = 0; i < 16; i++) {
		DMAController::trigger(0);
		display.write(2, 0, a);
		display.write(3, 0, b);
		_delay_ms(250);
	}

	//Warte auf das Beenden der kompletten Transaktion
	while (!DMAController::isTransactionComplete(0));

	DMAController::useChannel(0, false);

	display.write(1, 0, "done 1!       ");
	_delay_ms(500);
	display.write(1, 0, "Copy fast...");
	_delay_ms(100);
	display.write(2, 0, a);
	display.write(3, 0, b);

	if (DMAController::copyMemory(a, b, 16)) {
		display.write(1, 0, "done 2!       ");
		display.write(2, 0, a);
		display.write(3, 0, b);
	} else {
		display.write(1, 0, "fail!");
	}

	while (true);
}
#endif

#ifdef DEBUG_RF
int main() {
	set32MHz();
	useInterruptPriorities();
	_delay_ms(500);

	DEBUG_init();

	Display display(PORTC);
	//Display display(PORT_FROM_NUMBER(2));
	display.init();
	_delay_ms(100);
	display.write(0, 0, "Starting...");

	cli();
	activateInterrupts();

	Transmitter remote;

	if (remote.getConfigStatus() != 0) {
		display.write(1, 0, "RF Error: ")->writeUint(remote.getConfigStatus());
		while (1);
	}

	display.clear();

	DEBUG_LED(1);

	uint8_t i = 0;
	for (;;) {
		uint8_t axisX = 0, axisY = 0, buttonState = 0, buttonPushed = 0;

		remote.getState(axisX, axisY, buttonState, buttonPushed);

		display.write(0, 0, "X-Axis: ")->writeInt(axisX)->write("  ");
		display.write(1, 0, "Y-Axis: ")->writeInt(axisY)->write("  ");
		display.write(2, 0, " state: ")->writeInt(buttonState)->write("  ");
		display.write(3, 0, "pushed: ")->writeInt(buttonPushed)->write("  ");
		i++;

		_delay_ms(10);
	}
}
#endif
