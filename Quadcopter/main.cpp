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

#include <avr/eeprom.h>

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
//#define CONFIGURE_MOTOR_CONTROLLER

#if !(defined(DEBUG_DMA) || defined(DEBUG_RF) || defined(DEBUG_USART) || defined(DEBUG_TEST) || defined(DEBUG_DMA2) || defined(DEBUG_DEBUG) || defined(CONFIGURE_MOTOR_CONTROLLER))
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
 *	F		2-7		47-52	nRF24L01+
 *	H		0-1				Debug-Taster
 *	H		2-3				Debug-LED
 *	H		4-5				Debug-Schalter für Bluetooth
 *
 */

const float dT = 1.0 / TIMER_FREQUENCY;

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

void saveEEPROM(PID& pidX, PID& pidY, PID& pidZ, float cursorSpeed, Motor& motor) {
	float* eeprom = 0;
	eeprom_write_float(eeprom++, pidX.getP());
	eeprom_write_float(eeprom++, pidX.getI());
	eeprom_write_float(eeprom++, pidX.getD());
	eeprom_write_float(eeprom++, pidZ.getP());
	eeprom_write_float(eeprom++, pidZ.getI());
	eeprom_write_float(eeprom++, pidZ.getD());
	eeprom_write_float(eeprom++, cursorSpeed);
	eeprom_write_float(eeprom++, motor.getZeroDC(0));
	eeprom_write_float(eeprom++, motor.getZeroDC(1));
	eeprom_write_float(eeprom++, motor.getZeroDC(2));
	eeprom_write_float(eeprom++, motor.getZeroDC(3));
}

void loadEEPROM(PID& pidX, PID& pidY, PID& pidZ, float& cursorSpeed, Motor& motor) {
	float* eeprom = 0;
	float p = eeprom_read_float(eeprom++);
	float i = eeprom_read_float(eeprom++);
	float d = eeprom_read_float(eeprom++);
	pidX.setPID(p, i, d);
	pidY.setPID(p, i, d);
	p = eeprom_read_float(eeprom++);
	i = eeprom_read_float(eeprom++);
	d = eeprom_read_float(eeprom++);
	pidZ.setPID(p, i, d);
	cursorSpeed = eeprom_read_float(eeprom++);
	motor.setZero(0, eeprom_read_float(eeprom++));
	motor.setZero(1, eeprom_read_float(eeprom++));
	motor.setZero(2, eeprom_read_float(eeprom++));
	motor.setZero(3, eeprom_read_float(eeprom++));
}

int main() {
	disableJTAG();
	set32MHz();
	_delay_ms(10);

	//Aktiviere die Interrupts
	//FIXME oh oh
	activateInterrupts();

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
	display.write("INIT Motor...");
	Motor motor(PORTD, TCD0, 0.1, 0.20);
#		ifdef DISPLAY
	display.setCursorPos(1, 0)->write("Motors ok.")->writeUint(motor.getMaxDC());
	display.setCursorPos(2, 0)->write("PWM max=")->writeUint(motor.getMaxDC());
#		endif
#	else
#		ifdef DISPLAY
	display.setCursorPos(0, 0)->write("Motors off!")->writeUint(motor.getMaxDC());
#		endif
#	endif

	uint8_t errors = 0;

	// Initialisiere Beschleunigungssensor
	//template <uint8_t CS, uint8_t MOSI, uint8_t MISO, uint8_t SCLK>
#	ifdef DISPLAY
	_delay_ms(750);
	display.clear();
	_delay_ms(50);
	display.write("INIT Acc...");
#	endif
	ADXL345<4, 5, 6, 7> acc(PORTE);
	if (!acc.isDeviceOk()) {
#		ifdef DISPLAY
		display.setCursorPos(1, 0)->write("Acc ERROR!");
		_delay_ms(2000);
		errors++;
#		endif
	} else {
		acc.setDefaults();
		if (!acc.calibrate()) {
#			ifdef DISPLAY
			display.setCursorPos(1, 0)->write("Calib WARN!");
#			endif
		} else {
#			ifdef DISPLAY
			display.setCursorPos(1, 0)->write("Acc ok.")->writeUint(motor.getMaxDC());
			display.setCursorPos(2, 0)->write("OFF:")->writeInt(acc.getOffset(0))->write(",")->writeInt(acc.getOffset(1))->write(",")->writeInt(acc.getOffset(2));
#			endif
		}
		acc.setSmooth(8);
	}

	// Initialisiere Gyro
#	ifdef DISPLAY
	_delay_ms(750);
	display.clear();
	_delay_ms(50);
	display.write("INIT Gyro...");
#	endif
	L3G4200D<2, 1, 0, 3> gyro(PORTE);
	if (!gyro.isDeviceOk()) {
#		ifdef DISPLAY
		display.setCursorPos(1, 0)->write("Gyro ERR!");
		_delay_ms(2000);
		errors++;
#		endif
	} else {
		gyro.setDefaults();
		gyro.calibrate();
		gyro.setSmooth(3);
#		ifdef DISPLAY
		display.setCursorPos(1, 0)->write("Gyro ok.");
#		endif
	}

#	ifdef DISPLAY
	_delay_ms(750);
	display.clear();
	_delay_ms(50);
#	endif
#	ifdef REMOTE
	// Initialisiere die Fernbedienung
#		ifdef DISPLAY
	display.write("INIT Remote...");
#		endif
	Transmitter remote;
#		ifdef DISPLAY
	if (remote.getConfigStatus() != 0) {
		display.setCursorPos(1, 0)->write("Remote ERR (")->writeUint(remote.getConfigStatus())->write(")");
		errors++;
		_delay_ms(2000);
	} else {
		display.setCursorPos(1, 0)->write("Remote ok.");
	}
#		endif
#	else
#		ifdef DISPLAY
	display.setCursorPos(0, 0)->write("Remote off!");
#		endif
#	endif

#	ifdef DISPLAY
	_delay_ms(750);
	display.clear();
	_delay_ms(50);
#	endif

#	ifdef BLUETOOTH
	// Initialisiere Bluetooth Debugger
#		ifdef DISPLAY
	display.write("INIT BT...");
#		endif
	Bluetooth bt(&USARTD1, &PORTD, 9600);
	if (!bt.isDeviceOk()) {
#		ifdef DISPLAY
		display.setCursorPos(1, 0)->write("BT ERROR!");
		_delay_ms(2000);
		errors++;
#		endif
	} else {
#		ifdef DISPLAY
		display.setCursorPos(1, 0)->write("BT ok.");
#		endif
	}
#	else
#		ifdef DISPLAY
	display.setCursorPos(0, 0)->write("BT off!");
#		endif
#	endif

#		ifdef DISPLAY
	_delay_ms(750);
	display.clear();
	_delay_ms(50);
	display.write("Status");
#		endif

	// Wenn ein Fehler aufgetreten ist, beende das Programm, damit man die Fehlermeldung sehen kann.
	if (errors > 0) {
#		ifdef DISPLAY
		display.setCursorPos(1, 0)->write("Errors: ")->writeUint(errors);
		display.setCursorPos(3, 0)->write("STOPPING DEVICE!");
#		endif
		while (true);
	} else {
#		ifdef DISPLAY
		display.setCursorPos(1, 0)->write("Ok.");
		display.setCursorPos(2, 0)->write("Waiting 1 s...");
		_delay_ms(1000);
		display.clear();
#		endif
	}

	// END INITIALISATION

	// Duty Cycles für verschiedene Geschwindigkeiten
	float standby = 0.15;	// regelbar mit Cursor hoch und runter
	float power = 0.35;

	float speed = 0.0;

	bool fireStatus = false;

	// Einstellung für Remote
	#define MAX_ANGLE_DEG 45
	#define MAX_ANGLE_RAD (MAX_ANGLE_DEG * 0.017453293)

	// Einstellung für die PID-Regler
	#define PID_P 0.054
	#define PID_I 0.02
	#define PID_D 0.022
	#define PID_I_Min -1.0
	#define PID_I_Max 1.0
	#define PID_I_Band 1.0

	PID pidX(PID_P, PID_I, PID_D, dT);
	pidX.setIBand(PID_I_Band, PID_I_Min, PID_I_Max);

	PID pidY(PID_P, PID_I, PID_D, dT);
	pidY.setIBand(PID_I_Band, PID_I_Min, PID_I_Max);

	PID pidZ(0.0000, 0.0, 0.0000, dT);
	float accZ = 0.0;
	float cursorSpeed = 1.0;

	CFilter filterX(0.0045, dT);
	CFilter filterY(0.0045, dT);

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
	Vector3f position;
	Vector3f velocity;
	Vector3f acceleration;

	loadEEPROM(pidX, pidY, pidZ, cursorSpeed, motor);

	motor.setSpeed(0);
	_delay_ms(2000);

	//DEBUG_LED(1);
	Clock clock;

	enum {
		DISPLAY_MOTORS = 0,
		DISPLAY_P,
		DISPLAY_I,
		DISPLAY_D,
		DISPLAY_P_Z,
		DISPLAY_I_Z,
		DISPLAY_D_Z,
		DISPLAY_CURSOR_SPEED,
		DISPLAY_SENSORS,
		DISPLAY_SPEED,
		DISPLAY_GYRO,
		DISPLAY_ZERO_DC_0,
		DISPLAY_ZERO_DC_1,
		DISPLAY_ZERO_DC_2,
		DISPLAY_ZERO_DC_3,
		DISPLAY_MEMORY,
		DISPLAY_LAST
	};

	enum {
		MODE_CONFIGURE = 0,
		MODE_FLY
	};

	enum {
		DISPLAY_ACTION_NONE = 0,
		DISPLAY_ACTION_SAVE,
		DISPLAY_ACTION_LOAD
	};

	uint32_t displayLastActionTime = 0;
	uint8_t displayLastAction = DISPLAY_ACTION_NONE;
	const uint16_t displayLastActionDelay = 1000;
	uint8_t displayState = DISPLAY_MOTORS;
	uint8_t lastDisplayState = displayState;
	uint8_t workingMode = MODE_CONFIGURE;

	for (;;) {
		if (clock.eventInterrupt) {
			// Debug LED an
			//DEBUG_LED(1);

			/* ========================= SENSOR AUSWERTUNG ========================= */
			// Gyro und Beschleunigungssensor auslesen
			gyro.measureLowPass();
			acc.measureLowPass();

			// Winkel aus Beschleunigung und Gyro berechnen
			angle.x = filterX(acc.angleX(), gyro.x());
			angle.y = filterY(acc.angleY(), gyro.y());
			if (fabs(gyro.z()) > 1.0f) {
				angle.z += gyro.z() * dT;
			}

			// Beschleunigung relativ zum Quadcopter
			acceleration.z = (acc.z() - ENV_G * acc.angleCosXY());
			velocity.z += acceleration.z * dT;
			position.z += velocity.z * dT;
			velocity.z *= 0.95;

			// Speed für die einzelnen Axen
			Vector3f motorSpeed;

			// X-Y-Werte auf -1..1 normalisieren
			Vector2f remoteAngle;

			/* ========================= REMOTE AUSWERTUNG ========================= */
#			ifdef REMOTE
#				ifdef DISPLAY
			if (workingMode == MODE_CONFIGURE) {
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
#				endif //DISPLAY

				if (remote.getButton(Transmitter::curLeft)) {
					switch (displayState) {
						case DISPLAY_P: pidX.addP(-0.0001); pidY.addP(-0.0001); break;
						case DISPLAY_I: pidX.addI(-0.0001); pidY.addI(-0.0001); break;
						case DISPLAY_D: pidX.addD(-0.0001); pidY.addD(-0.0001); break;
						case DISPLAY_P_Z: pidZ.addP(-0.0001); break;
						case DISPLAY_I_Z: pidZ.addP(-0.0001); break;
						case DISPLAY_D_Z: pidZ.addP(-0.0001); break;
						case DISPLAY_CURSOR_SPEED: cursorSpeed -= 1.0; break;
						case DISPLAY_ZERO_DC_0: motor.setZero(0, motor.getZeroDC(0) - 0.0001); break;
						case DISPLAY_ZERO_DC_1: motor.setZero(1, motor.getZeroDC(1) - 0.0001); break;
						case DISPLAY_ZERO_DC_2: motor.setZero(2, motor.getZeroDC(2) - 0.0001); break;
						case DISPLAY_ZERO_DC_3: motor.setZero(3, motor.getZeroDC(3) - 0.0001); break;
						case DISPLAY_MEMORY:
							loadEEPROM(pidX, pidY, pidZ, cursorSpeed, motor);
							displayLastAction = DISPLAY_ACTION_LOAD;
							displayLastActionTime = clock.milliSeconds;
							break;
					}
				}
				if (remote.getButton(Transmitter::curRight)) {
					switch (displayState) {
						case DISPLAY_P: pidX.addP(0.0001); pidY.addP(0.0001); break;
						case DISPLAY_I: pidX.addI(0.0001); pidY.addI(0.0001); break;
						case DISPLAY_D: pidX.addD(0.0001); pidY.addD(0.0001); break;
						case DISPLAY_P_Z: pidZ.addP(0.0001); break;
						case DISPLAY_I_Z: pidZ.addP(0.0001); break;
						case DISPLAY_D_Z: pidZ.addP(0.0001); break;
						case DISPLAY_CURSOR_SPEED: cursorSpeed += 1.0; break;
						case DISPLAY_ZERO_DC_0: motor.setZero(0, motor.getZeroDC(0) + 0.0001); break;
						case DISPLAY_ZERO_DC_1: motor.setZero(1, motor.getZeroDC(1) + 0.0001); break;
						case DISPLAY_ZERO_DC_2: motor.setZero(2, motor.getZeroDC(2) + 0.0001); break;
						case DISPLAY_ZERO_DC_3: motor.setZero(3, motor.getZeroDC(3) + 0.0001); break;
						case DISPLAY_MEMORY:
							saveEEPROM(pidX, pidY, pidZ, cursorSpeed, motor);
							displayLastAction = DISPLAY_ACTION_SAVE;
							displayLastActionTime = clock.milliSeconds;
							break;
					}
				}

				if (DEBUG_Button()) {
					speed = standby;
#				ifndef REMOTE
					} else {
						speed = 0;
#				endif //REMOTE
				}

			} else { // MODE_FLY
				if (remote.getButton(Transmitter::curLeft)) {
					motorSpeed.z = 0.01;
				}
				if (remote.getButton(Transmitter::curRight)) {
					motorSpeed.z = -0.01;
				}

				if (remote.getButton(Transmitter::curUp)) {
					accZ = cursorSpeed;
				} else if (remote.getButton(Transmitter::curDown)) {
					accZ = -cursorSpeed;
				} else {
					accZ = 0.0;
				}

				speed = pidZ(accZ, acceleration.z);
				if (speed < 0.0) speed = 0.0;
				if (speed > 0.46) speed = 0.46;
			}

			if (remote.wasButtonPushed(Transmitter::btnBottom)) {
				if (workingMode == MODE_CONFIGURE) {
					workingMode = MODE_FLY;
					lastDisplayState = displayState;
					displayState = DISPLAY_MOTORS;
				} else {
					displayState = lastDisplayState;
					workingMode = MODE_CONFIGURE;
				}
			}

			// Fernbedienung auslesen
			uint8_t remoteX, remoteY, remoteThrottle, remoteState, remotePushed;
			remote.getState(remoteX, remoteY, remoteThrottle, remoteState, remotePushed);

			float remoteSpeed = (float) remoteThrottle / 255.0;
			speed = remoteSpeed;

			// Wenn der Top-Button gedrückt wird, kann sich um die eigene Achse drehen
			if (remote.getButton(Transmitter::btnTop)) {
				remoteAngle.x = 0;
				remoteAngle.y = 0;

				//float remoteSpeed = (float) (remoteY - 128) / 128.0;
				//standby += 0.075 * remoteSpeed * dT;

			} else {
				remoteAngle.x = (float) (remoteX - 128) / 128.0;
				remoteAngle.y = (float) (remoteY - 128) / 128.0;

				/* Linearität der X-Y-Werte aufheben: f(x) = (x^3 + x/2) / 1.5
				 * Das macht kleine Auslenkungen genau und größere stärker.
				 */
				remoteAngle.x = remoteAngle.x * (remoteAngle.x * remoteAngle.x + 0.5) * 0.666666;
				remoteAngle.y = remoteAngle.y * (remoteAngle.y * remoteAngle.y + 0.5) * 0.666666;

				// X-Y-Werte auf MAX_ANGLE_RAD strecken
				remoteAngle.x *= MAX_ANGLE_RAD;
				remoteAngle.y *= MAX_ANGLE_RAD;
			}

			if (not remote.isConnected()) {
				fireStatus = false;
			}
			if (remote.wasButtonPushed(Transmitter::btnFire)) {
				fireStatus = not fireStatus;
			}

#			endif //REMOTE

			/* ========================= PID Regler ========================= */
			// Korrekturwinkel berechnen mit PID-Regler
			pidAngle.x = pidX(remoteAngle.x, angle.x);
			pidAngle.y = pidY(remoteAngle.y, angle.y);

			motorSpeed.x = pidAngle.x * 1.0; // * PID_Scale;
			motorSpeed.y = pidAngle.y * 1.0; // * PID_Scale;

			/* ========================= BLUETOOTH DEBUGGING ========================= */
#			ifdef BLUETOOTH
			if (DEBUG_Switch()) {
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
#			endif //BLUETOOTH

			/* ========================= MOTOR MISCHER ========================= */
#			ifdef MOTOR
			if (displayState < DISPLAY_ZERO_DC_0 || displayState > DISPLAY_ZERO_DC_3) {
				if (speed < 0.05) {
					motor.setSpeed(0);
				} else {
					motor.setSpeed(speed - motorSpeed.y + motorSpeed.z,
								   speed + motorSpeed.x - motorSpeed.z,
								   speed + motorSpeed.y + motorSpeed.z,
								   speed - motorSpeed.x - motorSpeed.z);
				}
			} else {
				if (remote.getButton(Transmitter::btnFire)) {
					motor.setSpeed(0.2);
				} else {
					motor.setSpeed(0);
				}
			}
#			endif //MOTOR
			clock.eventInterrupt = false;

			/* ========================= DEBUG AUSGABEN ========================= */
#			ifdef DISPLAY
			switch (displayState) {
				case DISPLAY_P:
				case DISPLAY_I:
				case DISPLAY_D:
					display.setCursorPos(0, 0)->write(displayState == DISPLAY_P ? "P:" : "p:")->writeFloat(pidX.getP());
					display.setCursorPos(1, 0)->write(displayState == DISPLAY_I ? "I:" : "i:")->writeFloat(pidX.getI());
					display.setCursorPos(2, 0)->write(displayState == DISPLAY_D ? "D:" : "d:")->writeFloat(pidX.getD());
					display.setCursorPos(3, 0)->write("p:")->writeInt4(100 * DEG(pidAngle.x), 100 * DEG(pidAngle.y), DEG(angle.z));
					break;
				case DISPLAY_P_Z:
				case DISPLAY_I_Z:
				case DISPLAY_D_Z:
				case DISPLAY_CURSOR_SPEED:
					display.setCursorPos(0, 0)->write(displayState == DISPLAY_P_Z ? "Pz:" : "pz:")->writeFloat(pidZ.getP());
					display.setCursorPos(1, 0)->write(displayState == DISPLAY_I_Z ? "Iz:" : "iz:")->writeFloat(pidZ.getI());
					display.setCursorPos(2, 0)->write(displayState == DISPLAY_D_Z ? "Dz:" : "dz:")->writeFloat(pidZ.getD());
					display.setCursorPos(3, 0)->write(displayState == DISPLAY_CURSOR_SPEED ? "CURSPEED:" : "curSpeed:")->writeFloat(cursorSpeed);
					break;
				case DISPLAY_SENSORS:
					display.setCursorPos(0, 0)->write("a:")->writeInt4(acc.x() * 100, acc.y() * 100, acc.z() * 100);
					display.setCursorPos(1, 0)->write("g:")->writeInt4(gyro.x() * 100, gyro.y() * 100, gyro.z() * 100);
					display.setCursorPos(2, 0)->write("w:")->writeInt4(DEG(angle.x), DEG(angle.y), DEG(angle.z));
					display.setCursorPos(3, 0)->write("height:")->writeFloat(DEG(acceleration.z));
					break;
				case DISPLAY_SPEED:
					display.setCursorPos(0, 0)->write("a:")->writeFloat(acceleration.z);
					display.setCursorPos(1, 0)->write("v:")->writeFloat(velocity.z);
					display.setCursorPos(2, 0)->write("s:")->writeFloat(position.z);
					break;
				case DISPLAY_GYRO:
					display.setCursorPos(0, 0)->write("gx:")->writeInt(gyro.x() * 100);
					display.setCursorPos(1, 0)->write("gy:")->writeInt(gyro.y() * 100);
					display.setCursorPos(2, 0)->write("gz:")->writeInt(gyro.z() * 100);
					display.setCursorPos(3, 0)->write("t:")->writeUint32(clock.milliSeconds);
					break;
				case DISPLAY_MOTORS:
					display.setCursorPos(0, 0)->write("mf:")->writeInt4(motor.getSpeedI(0))->write("%");
					display.setCursorPos(0, 8)->write("ml:")->writeInt4(motor.getSpeedI(1))->write("%");
					display.setCursorPos(1, 0)->write("mb:")->writeInt4(motor.getSpeedI(2))->write("%");
					display.setCursorPos(1, 8)->write("mr:")->writeInt4(motor.getSpeedI(3))->write("%");
					display.setCursorPos(2, 0)->write(workingMode == MODE_CONFIGURE ? "speed: " : "flymode: ")->writeFloat(speed);
					if (remote.isConnected()) {
						display.setCursorPos(3, 0)->write("r:")->writeInt4(DEG(remoteAngle.x), DEG(remoteAngle.y), remoteState);
					} else {
						display.setCursorPos(3, 0)->write("r: not connected");
					}
					break;
				case DISPLAY_ZERO_DC_0:
				case DISPLAY_ZERO_DC_1:
				case DISPLAY_ZERO_DC_2:
				case DISPLAY_ZERO_DC_3:
					display.setCursorPos(0, 0)->write(displayState == DISPLAY_ZERO_DC_0 ? "ZERO 0: " : "zero 0: ")->writeFloat(motor.getZeroDC(0));
					display.setCursorPos(1, 0)->write(displayState == DISPLAY_ZERO_DC_1 ? "ZERO 1: " : "zero 1: ")->writeFloat(motor.getZeroDC(1));
					display.setCursorPos(2, 0)->write(displayState == DISPLAY_ZERO_DC_2 ? "ZERO 2: " : "zero 2: ")->writeFloat(motor.getZeroDC(2));
					display.setCursorPos(3, 0)->write(displayState == DISPLAY_ZERO_DC_3 ? "ZERO 3: " : "zero 3: ")->writeFloat(motor.getZeroDC(3));
					break;
				case DISPLAY_MEMORY:
					display.setCursorPos(0, 0)->write("Memory Menu");
					display.setCursorPos(1, 0)->write("================");
					if ((displayLastAction == DISPLAY_ACTION_NONE) || (displayLastActionTime + displayLastActionDelay < clock.milliSeconds)) {
						display.setCursorPos(2, 0)->write(" SAVE: Cursor > ");
						display.setCursorPos(3, 0)->write(" LOAD: Cursor < ");
					} else {
						switch (displayLastAction) {
							case DISPLAY_ACTION_LOAD:
								display.setCursorPos(3, 0)->write(" LOAD successful!");
								break;
							case DISPLAY_ACTION_SAVE:
								display.setCursorPos(2, 0)->write(" SAVE successful!");
								break;
						}
					}
					break;
			}
#			endif //DISPLAY

			// Debug LED aus
			//DEBUG_LED(0);
		}
	}
}
#endif

#ifdef CONFIGURE_MOTOR_CONTROLLER
int main() {
	set32MHz();
	_delay_ms(10);

	DEBUG_init();
	Motor motor(PORTD, TCD0, 0.1, 0.20);
	activateInterrupts();

	motor.setSpeed(0);
	_delay_ms(2000);

	while (true) {
		if (DEBUG_Button()) {
			motor.setSpeed(1.0);
			DEBUG_LED(1);
		} else {
			motor.setSpeed(0.0);
			DEBUG_LED(0);
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

// Die DMA-Controller-Klasse
#include "DMAController.h"

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
