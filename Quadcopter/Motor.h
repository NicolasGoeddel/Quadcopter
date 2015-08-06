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
		uint16_t zeroDC[4];

		/**
		 * Der Dutycycle, bei dem sich die Motoren mit voller
		 * Geschwindigkeit drehen.
		 */
		uint16_t fullDC;

		/**
		 * Differenz zwischen fullDC und zeroDC
		 */
		uint16_t diffDC[4];

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
			fullDC = full * maxDC;
			setZero(zero);
			setSpeed(0.0);
			setPWM(0);
			//FIXME Vielleicht muss da auch noch ein kleines Delay rein. Ich weiß nicht so genau...
			//_delay_ms(3000);
		}

		~Motor();

		/**
		 * Stellt die Geschwindigkeit aller Motoren gleichzeitig
		 * ein.
		 *
		 * @param speed Die Geschwindigkeit der Motoren im Bereich 0..1.
		 */
		void setSpeed(float speed) {
			speed = cutBorders(speed);
			DC[0] = zeroDC[0] + diffDC[0] * speed;
			DC[1] = zeroDC[1] + diffDC[1] * speed;
			DC[2] = zeroDC[2] + diffDC[2] * speed;
			DC[3] = zeroDC[3] + diffDC[3] * speed;
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
			DC[motor] = zeroDC[motor] + diffDC[motor] * cutBorders(speed);
			setPWM();
		}

		void setSpeed(float m1, float m2, float m3, float m4) {
			DC[0] = zeroDC[0] + diffDC[0] * cutBorders(m1);
			DC[1] = zeroDC[1] + diffDC[1] * cutBorders(m2);
			DC[2] = zeroDC[2] + diffDC[2] * cutBorders(m3);
			DC[3] = zeroDC[3] + diffDC[3] * cutBorders(m4);
			setPWM();
		}

		void setZero(uint8_t motor, float zero) {
			if (motor > 3) return;
			zero = cutBorders(zero);
			zeroDC[motor] = zero * maxDC;
			diffDC[motor] = fullDC - zeroDC[motor];
			setPWM();
		}

		void setZero(float zero) {
			zero = cutBorders(zero);
			zeroDC[0] = zero * maxDC;
			zeroDC[1] = zero * maxDC;
			zeroDC[2] = zero * maxDC;
			zeroDC[3] = zero * maxDC;
			diffDC[0] = fullDC - zeroDC[0];
			diffDC[1] = fullDC - zeroDC[1];
			diffDC[2] = fullDC - zeroDC[2];
			diffDC[3] = fullDC - zeroDC[3];
			setPWM();
		}

		uint16_t getMaxDC() {
			return maxDC;
		}

		uint8_t getSpeedI(uint8_t motor) {
			if (motor > 3) return 0;
			return (uint32_t)((DC[motor] - zeroDC[motor]) * 100 / diffDC[motor]);
		}

		uint16_t getDC(uint8_t motor) {
			if (motor > 3) return 0;
			return DC[motor];
		}

		float getZeroDC(uint8_t motor) {
			if (motor > 3) return 0;
			return (float) zeroDC[motor] / (float) maxDC;
		}
};

#endif /* MOTOR_H_ */
