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

#include "myMath.h"
#include <errno.h>
#include <math.h>

// Alternate method to calculate arctangent from: http://www.dspguru.com/comp.dsp/tricks/alg/fxdatan2.htm
/*
float arctan2(float x, float y) {
	float coeff_1 = 0.785398163;	//Pi / 4
	float coeff_2 = 2.35619449;		//Pi * 3 / 4
	float abs_y = ((y < 0) ? -y : y) + 1e-10;	// kludge to prevent 0/0 condition
	float r, angle;

	if (x >= 0) {
		r = (x - abs_y) / (x + abs_y);
		angle = coeff_1 - coeff_1 * r;
	} else {
		r = (x + abs_y) / (x - abs_y);
		angle = coeff_2 + coeff_1 * r;
	}
	if (y < 0) {
		return -angle;
	} else {
		return angle;
	}
}
*/

/*
float atan(double x) {
//	      Algorithm and coefficients from:
//	      "Software manual for the elementary functions"
//	      by W.J. Cody and W. Waite, Prentice-Hall, 1980

	static double p[] = {
			-0.13688768894191926929e+2,
			-0.20505855195861651981e+2,
			-0.84946240351320683534e+1,
			-0.83758299368150059274e+0
		};
	static float q[] = {
			0.41066306682575781263e+2,
			0.86157349597130242515e+2,
			0.59578436142597344465e+2,
			0.15024001160028576121e+2,
			1.0
		};
	static float a[] = {
			0.0,
			0.52359877559829887307710723554658381,  // pi/6
			M_PI_2,
			1.04719755119659774615421446109316763   // pi/3
		};

	int neg = x < 0;
	int n;
	double  g;

//	if (__IsNan(x)) {
//		errno = EDOM;
//		return x;
//	}
	if (neg) {
		x = -x;
	}
	if (x > 1.0) {
		x = 1.0 / x;
		n = 2;
	} else {
		n = 0;
	}

	if (x > 0.26794919243112270647) {       // 2 - sqtr(3)
		n = n + 1;
		x = (((0.73205080756887729353 * x - 0.5) - 0.5) + x) / (1.73205080756887729353 + x);
	}

	// ??? avoid underflow ???

	g = x * x;
	x += x * g * POLYNOM3(g, p) / POLYNOM4(g, q);
	if (n > 1)
		x = -x;
	x += a[n];
	return neg ? -x : x;
}
*/

float myAtan(float x) {
	if (x > 1.0) {
		return M_PI_2 - (x / (x * x + 0.28));
	} else if (x < -1.0) {
		return -M_PI_2 - (x / (x * x + 0.28));
	}
	return x / (1 + 0.28 * x * x);
}

/*
 * (c) copyright 1988 by the Vrije Universiteit, Amsterdam, The Netherlands.
 * See the copyright notice in the ACK home directory, in the file "Copyright".
 *
 * Author: Ceriel J.H. Jacobs
 */
/* $Header: /opt/proj/minix/cvsroot/src/lib/math/atan2.c,v 1.1.1.1 2005/04/21 14:56:24 beng Exp $ */

float myAtan2(float y, float x) {
	double absx, absy, val;

	if (x == 0 && y == 0) {
		errno = EDOM;
		return 0;
	}
	absy = y < 0 ? -y : y;
	absx = x < 0 ? -x : x;
	if (absx < 1e-10) {
		/* x negligible compared to y */
		return y < 0 ? -M_PI_2 : M_PI_2;
	}
	if (absy < 1e-10) {
		/* y negligible compared to x */
		val = 0.0;
	} else {
		val = myAtan(y / x);
	}
	if (x > 0) {
		/* first or fourth quadrant; already correct */
		return val;
	}
	if (y < 0) {
		/* third quadrant */
		return val - M_PI;
	}
	return val + M_PI;
}

float Myatof(char * value) {
	float f;
	// Skip all chars other to 0..9
	while (*value) {
		if ((*value >= '0' && *value <= '9') || *value == '.') break;
		value++;
	}
	// Read all digits before the point
	while (*value) {
		if (*value < '0' || *value > '9') break;
		f = f * 10.0 + (*value - '0');
		value++;
	}
	if (*value == '.') {
		value++;
		// Read all digits
		float mult = 0.1;
		while (*value) {
			if (*value < '0' || *value > '9') break;
			f += (*value - '0') * mult;
			mult *= 0.1;
			value++;
		}
	}
	return f;
}
