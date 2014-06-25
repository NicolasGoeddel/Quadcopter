/*
 * Motor.h
 *
 *  Created on: 25.06.2014
 *      Author: nicolas
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include <avr/io.h>
#include "XMEGA_helper.h"
#include <util/delay.h>

class Motor {
	private:
		/**
		 * Der Dutycycle, bei dem sich die Motoren nicht drehen,
		 * also die Nullstellung.
		 */
		uint16_t zeroDC;

		/**
		 * Der Dutycycle, bei dem sich die Motoren mit voller
		 * Geschwindigkeit drehen.
		 */
		uint16_t fullDC;

		/**
		 * Differenz zwischen fullDC und zeroDC
		 */
		uint16_t diffDC;

		/**
		 * Der höchste Wert für den Dutycycle der verwendeten
		 * PWM.
		 */
		uint16_t maxDC;

		/**
		 * Der aktuelle Dutycycle von jedem Motor.
		 */
		uint16_t DC[4];

		/**
		 * Der Timer, der verwendet werden soll.
		 */
		TC0_t* timer;

		PORT_t* port;

		void inline setPWM() {
			set4ChanPWM(*timer, DC[0], DC[1], DC[2], DC[3]);
		}

		void inline setPWM(uint16_t pwm) {
			set4ChanPWM(*timer, pwm, pwm, pwm, pwm);
		}

		float inline cutBorders(float speed) {
			return (speed < 0.0) ? 0.0 : ((speed > 1.0) ? 1.0 : speed);
		}

	public:
		Motor(PORT_t & port, TC0_t & timer, float zero, float full) {
			maxDC = init4ChanPWM(port, timer, 100);
			this->timer = &timer;
			this->port = &port;
			zeroDC = zero * maxDC;
			fullDC = full * maxDC;
			diffDC = fullDC - zeroDC;
			DC[0] = DC[1] = DC[2] = DC[3] = zeroDC;
			setPWM(0);
			_delay_ms(3000);
		}

		~Motor();

		/**
		 * Stellt die Geschwindigkeit aller Motoren gleichzeitig
		 * ein.
		 *
		 * @param speed Die Geschwindigkeit der Motoren im Bereich 0..1.
		 */
		void setSpeed(float speed) {
			DC[0] = DC[1] = DC[2] = DC[3] = zeroDC + (fullDC - zeroDC) * cutBorders(speed);
			setPWM();
		}

		/**
		 * Stellt die Geschwindigkeit eines Motors ein.
		 *
		 * @param motor Die Nummer des Motors im Bereich 0..3.
		 * @param speed Die Geschwindigkeit der Motoren im Bereich 0..1.
		 */
		void setSpeed(uint8_t motor, float speed) {
			if (motor > 3) return;
			DC[motor] = zeroDC + diffDC * cutBorders(speed);
			setPWM();
		}

		void setSpeed(float m1, float m2, float m3, float m4) {
			DC[0] = zeroDC + diffDC * cutBorders(m1);
			DC[1] = zeroDC + diffDC * cutBorders(m2);
			DC[2] = zeroDC + diffDC * cutBorders(m3);
			DC[3] = zeroDC + diffDC * cutBorders(m4);
			setPWM();
		}

		uint16_t getMaxDC() {
			return maxDC;
		}

		uint8_t getSpeedI(uint8_t motor) {
			if (motor > 3) return 0;
			return (uint32_t)((DC[motor] - zeroDC) * 100 / diffDC);
		}

		uint16_t getDC(uint8_t motor) {
			if (motor > 3) return 0;
			return DC[motor];
		}
};

#endif /* MOTOR_H_ */
