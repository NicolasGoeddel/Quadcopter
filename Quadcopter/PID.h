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

#ifndef PID_H_
#define PID_H_

#include <avr/io.h>
#include "math.h"

#define RATE_XAXIS_PID_IDX          0
#define RATE_YAXIS_PID_IDX          1
#define ZAXIS_PID_IDX               2
#define ATTITUDE_XAXIS_PID_IDX      3
#define ATTITUDE_YAXIS_PID_IDX      4
#define HEADING_HOLD_PID_IDX        5
#define ATTITUDE_GYRO_XAXIS_PID_IDX 6
#define ATTITUDE_GYRO_YAXIS_PID_IDX 7
#define ALTITUDE_HOLD_PID_IDX       8
#define ZDAMPENING_PID_IDX          9

// This struct above declares the variable PID[] to hold each of the PID values for various functions
// The following constants are declared in AeroQuad.h
// ROLL = 0, PITCH = 1, YAW = 2 (used for Arcobatic Mode, gyros only)
// ROLLLEVEL = 3, PITCHLEVEL = 4, LEVELGYROROLL = 6, LEVELGYROPITCH = 7 (used for Stable Mode, accels + gyros)
// HEADING = 5 (used for heading hold)
// ALTITUDE = 8 (used for altitude hold)
// ZDAMPENING = 9 (used in altitude hold to dampen vertical accelerations)

/*
class PID {
		struct PIDdata {
			float P, I, D;
			float lastPosition;
			float previousPIDTime;
			float integratedError;
			float windupGuard;
		} pid[10];

		float windupGuard; //Read in from EEPROM
		float currentTime;
	public:
		PID() {
			windupGuard = 0.0;
			currentTime = 0.0;
		}
		~PID() {}

		float updatePID(float targetPosition, float currentPosition, PIDdata * PIDparameters) {
			  // AKA PID experiments
			  const float deltaPIDTime = (currentTime - PIDparameters->previousPIDTime) / 1000000.0;

			  PIDparameters->previousPIDTime = currentTime;  // AKA PID experiments
			  float error = targetPosition - currentPosition;

			  PIDparameters->integratedError += error * deltaPIDTime;
			  PIDparameters->integratedError = constrain(PIDparameters->integratedError, -PIDparameters->windupGuard, PIDparameters->windupGuard);
			  float dTerm = PIDparameters->D * (currentPosition - PIDparameters->lastPosition) / (deltaPIDTime * 100); // dT fix from Honk
			  PIDparameters->lastPosition = currentPosition;
			  return (PIDparameters->P * error) + (PIDparameters->I * (PIDparameters->integratedError)) + dTerm;
		}
		void zeroIntegralError() {
			for (uint8_t axis = 0; axis < ATTITUDE_YAXIS_PID_IDX; axis++) {
				pid[axis].integratedError = 0.0;
				pid[axis].previousPIDTime = currentTime;
			}
		}
};
*/

//#define OLD_IMPLEMENTATION

class PID {
#ifndef OLD_IMPLEMENTATION
	private:
		float iState;	// Integrator state
		// Maximum and minimum allowable integrator state
		float iMax;
		float iMin;
		float iBand;	// Integral operating band
		float iGain;	// Integral gain
		float pGain;	// Proportional gain
		float dGain;	// Derivative gain
		float dT;
	public:
		PID(float p, float i, float d, float dT) {
			iGain = i;
			pGain = p;
			dGain = d;
			iState = 0.0;
			iMax = 1.0;
			iMin = -1.0;
			iBand = 2.0;
			this->dT = dT;
		}
		~PID() {};
		void setIBand(float iBand, float iMin, float iMax) {
			this->iBand = iBand;
			this->iMin = iMin;
			this->iMax = iMax;
		}
		float operator()(float error) {
			// calculate the integral state with approriate limiting
			float iState = 0.0;
			// only accumulate error while in band
			if (fabs(error) < iBand) {
				iState = iState + error;
			}
			// limit accumulator to bounds iMax, iMin
			if (iState > iMax) {
				iState = iMax;
			} else if (iState < iMin) {
				iState = iMin;
			}
			// calculate the integral term
			float iTerm = iGain * iState;
			return (pGain * error) + iTerm - (dGain * dT);
		}
		float operator()(float target, float current) {
			return (*this)(target - current);
		}
		void reset() {
			iState = 0.0;
		}
#else
	private:
		float q[3];
		float err[3];
		float last;
	public:
		PID(float p, float i, float d, float dT) {
			q[0] = p + i * dT + d / dT;
			q[1] = -p - 2 * d / dT;
			q[2] = d / dT;
			err[0] = err[1] = err[2] = 0;
			last = 0;
		}
		~PID() {}
		float operator()(float target, float current) {
			err[2] = err[1];
			err[1] = err[0];
			err[0] = target - current;

			last += q[0] * err[0] + q[1] * err[1] + q[2] * err[2];

			return last;
		}
		float operator()(float error) {
			err[2] = err[1];
			err[1] = err[0];
			err[0] = error;

			last += q[0] * err[0] + q[1] * err[1] + q[2] * err[2];

			return last;
		}
		void reset() {
			last = 0;
			err[0] = err[1] = err[2] = 0;
		}
#endif
};

#endif /* PID_H_ */
