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
		DEBUG_LED();
		_delay_ms(100);
	}
}

#define DISPLAY
#define REMOTE		// Disables DEBUG_Button for standby speed.
//#define BLUETOOTH
#define MOTOR

#include <stdlib.h>
#include <avr/io.h>
// Ein paar Hilfsfunktionen, die später nochmal heraus "operiert" werden
#include "XMEGA_helper.h"

#ifdef DISPLAY
#	include "Display.h"
#endif

#ifdef BLUETOOTH
#	include "Bluetooth.h"
#endif

// Die Klassen für die Sensoren
#include "ADXL345.h"
#include "L3G4200D.h"

// Der PID-Regler
#include "PID.h"

// Der Komplementärfilter
#include "CFilter.h"

// Ein paar Mathefunktionen
#include "myMath.h"

// Die DMA-Controller-Klasse
#include "DMAController.h"

// Die Klasse für die Fernbedienung
#ifdef REMOTE
#	include "Transmitter.h"
#endif

// Die Klasse für die Zeitsteuerung
#include "Clock.h"

// Die Klasse um die Motoren zu steuern
#include "Motor.h"

//#define DEBUG_DMA
//#define DEBUG_RF
//#define DEBUG_USART
//#define DEBUG_TEST
//#define DEBUG_DMA2
//#define DEBUG_DEBUG

#if !(defined(DEBUG_DMA) || defined(DEBUG_RF) || defined(DEBUG_USART) || defined(DEBUG_TEST) || defined(DEBUG_DMA2) || defined(DEBUG_DEBUG))
#	define MAIN_PROGRAM
#endif

/* Port-Map
 *	PORT	BITS	PIN		Beschreibung
 *	A		0-1
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

bool strEqual(char * a, char * b, uint8_t length = 255) {
	while (length && *a && *b) {
		if (*a != *b) {
			return false;
		}
		a++;
		b++;
		length--;
	}

	return (*a == *b) || (length == 0);
}

#ifdef BLUETOOTH
bool readPID(Bluetooth* bt, float & p, float & i, float & d) {
	if (bt->isDataAvailable()) {
		char c = bt->receiveChar();
		if (c == 's') {
			//DEBUG_LED(1);
			struct {
					float p, i, d;
			} pidBuffer;
			bt->getData((uint8_t*) &pidBuffer, 12);
			//DEBUG_LED(0);
			p = pidBuffer.p;
			i = pidBuffer.i;
			d = pidBuffer.d;
			return true;
		} else if (c == 'g') {
			bt->write("pid\n");
			bt->writeFloatRaw(p);
			bt->writeFloatRaw(i);
			bt->writeFloatRaw(d);
		}
	}
	return false;
}
#endif

int main() {
	disableJTAG();
	set32MHz();
	_delay_ms(10);

	// Für LED und Taster.
	DEBUG_init();

	// Display initialisieren
#	ifdef DISPLAY
	Display display(PORTC);
	display.init();
	_delay_ms(50);

	display.clear();
	_delay_ms(100);
#	endif

#	ifdef MOTOR
	/* 0.1 ist der Dutycycle für die Motorcontroller,
	 * bei dem die Motoren gerade aus sein sollten.
	 * 0.2 ist der Dutycycle, bei dem die Motoren auf
	 * Vollgas laufen sollen.
	 */
	Motor motor(PORTD, TCD0, 0.1, 0.20);
#		ifdef DISPLAY
	display.setCursorPos(0, 0)->write("PWM max=")->writeUint(motor.getMaxDC());
#		endif
#	else
#		ifdef DISPLAY
	display.setCursorPos(0, 0)->write("Motor off!")->writeUint(motor.getMaxDC());
#		endif
#	endif

	//Aktiviere die Interrupts
	activateInterrupts();

	uint8_t errorLine = 0;

	// Initialisiere Beschleunigungssensor
	//template <uint8_t CS, uint8_t MOSI, uint8_t MISO, uint8_t SCLK>
	ADXL345<4, 5, 6, 7> acc(PORTE);
	if (!acc.isDeviceOk()) {
#		ifdef DISPLAY
		display.setCursorPos(errorLine++, 0)->write("ACC ERROR!");
#		endif
	} else {
		acc.setDefaults();
		if (!acc.calibrate()) {
#			ifdef DISPLAY
			display.setCursorPos(1, 0)->write("ACC CAL WARN!");
#			endif
		} else {
#			ifdef DISPLAY
			//display.write(1, 0, "ACC:")->writeInt(acc.getOffset32(0))->write(",")->writeInt(acc.getOffset32(1))->write(",")->writeInt(acc.getOffset32(2));
			display.setCursorPos(1, 0)->write("ACC:")->writeInt(acc.getOffset(0))->write(",")->writeInt(acc.getOffset(1))->write(",")->writeInt(acc.getOffset(2));
#			endif
		}
		acc.setSmooth(5);
	}

	// Initialisiere Gyro
	L3G4200D<2, 1, 0, 3> gyro(PORTE);
	if (!gyro.isDeviceOk()) {
#		ifdef DISPLAY
		display.setCursorPos(errorLine++, 0)->write("GYRO ERROR!");
#		endif
	} else {
		gyro.setDefaults();
		gyro.calibrate();
		gyro.setSmooth(3);
#		ifdef DISPLAY
		display.setCursorPos(2, 0)->write("GYRO ok.");
#		endif
	}

#	ifdef REMOTE
	// Initialisiere die Fernbedienung
	Transmitter remote;
#		ifdef DISPLAY
	if (remote.getConfigStatus() != 0) {
		display.setCursorPos(errorLine++, 0)->write("RF Error: ")->writeUint(remote.getConfigStatus());
	} else {
		display.setCursorPos(3, 0)->write("Remote ok.");
	}
#		endif
#	else
#		ifdef DISPLAY
	display.setCursorPos(3, 0)->write("Remote off!");
#		endif
#	endif

#	ifdef BLUETOOTH
	// Initialisiere Bluetooth Debugger
	Bluetooth bt(&USARTD1, &PORTD, 9600);
	if (!bt.isDeviceOk()) {
#		ifdef DISPLAY
		display.setCursorPos(errorLine++, 0)->write("BLUETOOTH ERROR!");
#		endif
	} else {
#		ifdef DISPLAY
		display.setCursorPos(0, 0)->write("Bluetooth ok.");
#		endif
	}
#	else
#		ifdef DISPLAY
	display.setCursorPos(0, 0)->write("Bluetooth off!");
#		endif
#	endif

	// Wenn ein Fehler aufgetreten ist, beende das Programm, damit man die Fehlermeldung sehen kann.
	if (errorLine > 0) {
#		ifdef DISPLAY
		display.setCursorPos(3, 0)->write("STOPPING DEVICE!");
#		endif
		while (true);
	} else {
#		ifdef DISPLAY
		_delay_ms(500);
		display.clear();
		display.setCursorPos(0, 0)->write("Calibrating done");
		_delay_ms(1000);
		display.clear();
#		endif
	}

	// END INITIALISATION

	// Duty Cycles für verschiedene Geschwindigkeiten
	float standby = 0.15;	// regelbar mit Cursor hoch und runter
	float power = 0.35;

	float speed = 0.0;

	/*float x, y, z;
	x = y = z = 0; */

	// Einstellung für Remote
	#define MAX_ANGLE_DEG 45
	#define MAX_ANGLE_RAD (MAX_ANGLE_DEG * 0.017453293)

	// Einstellung für die PID-Regler
	#define PID_P 0.0
	#define PID_I 0.0
	#define PID_D 0.0 //0.0024
	#define PID_I_Min -1.0
	#define PID_I_Max 1.0
	#define PID_I_Band 1.0
	#define PID_Scale 1.0 //0.06

	PID pidX(PID_P * PID_Scale, PID_I * PID_Scale, PID_D * PID_Scale, dT);
	pidX.setIBand(PID_I_Band, PID_I_Min, PID_I_Max);

	PID pidY(PID_P * PID_Scale, PID_I * PID_Scale, PID_D * PID_Scale, dT);
	pidY.setIBand(PID_I_Band, PID_I_Min, PID_I_Max);

	CFilter filterX(0.045, dT);
	CFilter filterY(0.045, dT);

	struct Vector2f {
			Vector2f() {
				x = y = 0.0f;
			}
			float x, y;
	};

	struct Vector3f {
			Vector3f() {
				x = y = z = 0.0f;
			}
			float x, y, z;
	};

	Vector3f angle;
	Vector3f pidAngle;
	float accZ = 0.0;
	float speedZ = 0.0;
	float posZ = 0.0;

	motor.setSpeed(0);
	_delay_ms(2000);

	DEBUG_LED(1);
	Clock clock;

	enum {
		DISPLAY_MOTORS = 0,
		DISPLAY_P,
		DISPLAY_I,
		DISPLAY_D,
		DISPLAY_SENSORS,
		DISPLAY_GYRO,
		DISPLAY_LAST
	};

	uint8_t displayState = DISPLAY_MOTORS;

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
			angle.x = filterX(acc.angleX(), gyro.x());
			angle.y = filterY(acc.angleY(), gyro.y());
			if (fabs(gyro.z()) > 10.0f) {
				angle.z += gyro.z() * dT;
			}

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
			Vector2f rc;
			rc.x = (float) (remoteX - 128) / 128.0;
			rc.y = (float) (remoteY - 128) / 128.0;

			/* Linearität der X-Y-Werte aufheben: f(x) = (x^3 + x/2) / 1.5
			 * Das macht kleine Auslenkungen genau und größere stärker.
			 */
			//rc.x = rc.x * (rc.x * rc.x + 0.5) * 0.666666;
			//rc.y = rc.y * (rc.y * rc.y + 0.5) * 0.666666;

			// X-Y-Werte auf MAX_ANGLE_RAD strecken
			rc.x *= MAX_ANGLE_RAD;
			rc.y *= MAX_ANGLE_RAD;

			// Korrekturwinkel berechnen mit PID-Regler
			pidAngle.x = pidX(rc.x, angle.x);
			pidAngle.y = pidY(rc.y, angle.y);

			Vector3f pwm;
			pwm.x = pidAngle.x * 10.0; // * PID_Scale;
			pwm.y = pidAngle.y * 10.0; // * PID_Scale;
			pwm.z = 0.0;				// YAW

			// Debug-Infos ausgeben
			float p, i, d;
			pidX.getPID(&p, &i, &d);

#			ifdef DISPLAY
			switch (displayState) {
				case DISPLAY_P:
				case DISPLAY_I:
				case DISPLAY_D:
					display.setCursorPos(0, 0);
					if (displayState == DISPLAY_P) {
						display.write("P:");
					} else {
						display.write("p:");
					}
					display.writeFloat(p);
					display.setCursorPos(1, 0);
					if (displayState == DISPLAY_I) {
						display.write("I:");
					} else {
						display.write("i:");
					}
					display.writeFloat(i);
					display.setCursorPos(2, 0);
					if (displayState == DISPLAY_D) {
						display.write("D:");
					} else {
						display.write("d:");
					}
					display.writeFloat(d);
					display.setCursorPos(3, 0)->write("p:")->writeInt4(10 * DEG(pidAngle.x), 10 * DEG(pidAngle.y), DEG(angle.z));
					break;
				case DISPLAY_SENSORS:
					display.setCursorPos(0, 0)->write("a:")->writeInt4(acc.x() * 100, acc.y() * 100, acc.z() * 100);
					display.setCursorPos(1, 0)->write("g:")->writeInt4(gyro.x() * 100, gyro.y() * 100, gyro.z() * 100);
					display.setCursorPos(2, 0)->write("w:")->writeInt4(DEG(angle.x), DEG(angle.y), DEG(angle.z));
					display.setCursorPos(3, 0)->write("t:")->writeUint32(clock.milliSeconds);
					break;
				case DISPLAY_GYRO:
					display.setCursorPos(0, 0)->write("gx:")->writeInt(gyro.x() * 100);
					display.setCursorPos(1, 0)->write("gy:")->writeInt(gyro.y() * 100);
					display.setCursorPos(2, 0)->write("gz:")->writeInt(gyro.z() * 100);
					display.setCursorPos(3, 0)->write("t:")->writeUint32(clock.milliSeconds);
					break;
			}

#			endif

#			if defined(DISPLAY) && defined(REMOTE) && defined(BLUETOOTH)
			if (remote.getButton(Transmitter::btnFire)) {
				bt.write("Fire!\r\n");
			}
#			endif
#			ifdef BLUETOOTH
			if (DEBUG_Switch()
#				ifdef DISPLAY
					|| remote.getButton(Transmitter::btnTop)
#				endif
			) {
//				bt.write("a:")->writeFloat(acc.x())->writeFloat(acc.y())->writeFloat(acc.z());
//				bt.write(" g:")->writeFloat(gyro.x())->writeFloat(gyro.y())->writeFloat(gyro.z());
//				bt.writeChar(13);
				bt.write("sensors\n");
				//bt.writeFloatRaw(NAN);
				bt.writeUint32Raw(clock.milliSeconds);
				bt.writeFloatRaw(acc.x());
				bt.writeFloatRaw(acc.y());
				bt.writeFloatRaw(acc.z());
				bt.writeFloatRaw(gyro.x());
				bt.writeFloatRaw(gyro.y());
				bt.writeFloatRaw(gyro.z());
				bt.writeFloatRaw(angleX);
				bt.writeFloatRaw(angleY);
				bt.writeFloatRaw(angleZ);
				bt.writeFloatRaw(pidAngleX);
				bt.writeFloatRaw(pidAngleY);
				bt.writeFloatRaw(0.0);

				if (readPID(&bt, p, i, d)) {
					pidX.setPID(p, i, d);
					pidY.setPID(p, i, d);
				}
			}
#			endif

#			ifdef REMOTE
#				ifdef DISPLAY
			if (remote.wasButtonPushed(Transmitter::curDown)) {
				displayState++;
				if (displayState == DISPLAY_LAST) {
					displayState = 0;
				}
				display.clear();
			}
			if (remote.wasButtonPushed(Transmitter::curUp)) {
				if (displayState == 0) {
					displayState = DISPLAY_LAST;
				}
				displayState--;
				display.clear();
			}
#				endif

			if (remote.getButton(Transmitter::btnFire)) {
				speed = standby;
			} else {
				speed = 0;
			}
			if (remote.getButton(Transmitter::btnMiddle)) {
				if (remote.getButton(Transmitter::btnFire)) {
					speed = 0.46;
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
			if (remote.getButton(Transmitter::curLeft)) {
				//yaw = 0.01;
				switch (displayState) {
					case DISPLAY_P: p -= 0.0001; break;
					case DISPLAY_I: i -= 0.0001; break;
					case DISPLAY_D: d -= 0.0001; break;
				}
			}
			if (remote.getButton(Transmitter::curRight)) {
				//yaw = -0.01;
				switch (displayState) {
					case DISPLAY_P: p += 0.0001; break;
					case DISPLAY_I: i += 0.0001; break;
					case DISPLAY_D: d += 0.0001; break;
				}
			}
			pidX.setPID(p, i, d);
			pidY.setPID(p, i, d);
#			endif

			if (DEBUG_Button()) {
				speed = standby;
#			ifndef REMOTE
			} else {
				speed = 0;
#			endif
			}

#			ifdef MOTOR
			motor.setSpeed(speed - pwm.y + pwm.z,
						   speed + pwm.x - pwm.z,
						   speed + pwm.y + pwm.z,
					       speed - pwm.x - pwm.z);
#			endif
			clock.eventInterrupt = false;

			// Debug LED aus
			DEBUG_LED(0);

			if (displayState == DISPLAY_MOTORS) {
				display.setCursorPos(2, 0)->write("speed: ")->writeFloat(speed);
				display.setCursorPos(0, 0)->write("mf:")->writeInt4(motor.getSpeedI(0))->write("%");
				display.setCursorPos(0, 8)->write("ml:")->writeInt4(motor.getSpeedI(1))->write("%");
				display.setCursorPos(1, 0)->write("mb:")->writeInt4(motor.getSpeedI(2))->write("%");
				display.setCursorPos(1, 8)->write("mr:")->writeInt4(motor.getSpeedI(3))->write("%");
				display.setCursorPos(3, 0)->write("r:")->writeInt4(DEG(rc.x), DEG(rc.y), remoteState);
			}
		}
	}
}
#endif

#ifdef DEBUG_DEBUG
int main() {
	set32MHz();
	_delay_ms(10);

	// Für LED und Taster.
	DEBUG_init();

	while (true) {
		if (DEBUG_Button()) {
			DEBUG_LED(1);
		} else {
			DEBUG_LED(0);
		}
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

#ifdef DEBUG_DMA2
int main() {
	set32MHz();
	_delay_ms(100);

	DEBUG_init();

	DEBUG_LED_blink(4);

	DEBUG_LED(1);

	Display display(PORTC);
	display.init();

	// Reset DMA
	DMA.CTRL = 0;
	DMA.CTRL = DMA_RESET_bm;
	while ((DMA.CTRL & DMA_RESET_bm) != 0);

	activateInterrupts();

	display.setCursorPos(0, 0)->write("Init Bluetooth");
	Bluetooth bt(&USARTD1, &PORTD, 9600);
	//bt.write("Hello World.");
	if (!bt.isDeviceOk()) {
		display.setCursorPos(1, 0)->write("BLUETOOTH ERROR!");
	} else {
		display.setCursorPos(1, 0)->write("Bluetooth ok.");
	}

	while (true);
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
	_delay_ms(500);

	DEBUG_init();

	Display display(PORTC);
	//Display display(PORT_FROM_NUMBER(2));
	display.init();
	_delay_ms(100);
	display.setCursorPos(0, 0)->write("Starting...");

	cli();
	activateInterrupts();

	Transmitter remote;

	if (remote.getConfigStatus() != 0) {
		display.setCursorPos(1, 0)->write("RF Error: ")->writeUint(remote.getConfigStatus());
		while (1);
	}

	display.clear();

	DEBUG_LED(1);

	uint8_t i = 0;
	for (;;) {
		uint8_t axisX = 0, axisY = 0, buttonState = 0, buttonPushed = 0;

		remote.getState(axisX, axisY, buttonState, buttonPushed);

		display.setCursorPos(0, 0)->write("X-Axis: ")->writeInt(axisX)->write("  ");
		display.setCursorPos(1, 0)->write("Y-Axis: ")->writeInt(axisY)->write("  ");
		display.setCursorPos(2, 0)->write(" state: ")->writeInt(buttonState)->write("  ");
		display.setCursorPos(3, 0)->write("pushed: ")->writeInt(buttonPushed)->write("  ");
		i++;

		_delay_ms(10);
	}
}
#endif
