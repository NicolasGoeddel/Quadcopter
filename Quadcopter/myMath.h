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

#ifndef MATH_H_
#define MATH_H_

//#define POLYNOM0 (x, a) ((a)[0])
//#define POLYNOM1 (x, a) ((a)[1] * (x) + (a)[0])
//#define POLYNOM2 (x, a) (POLYNOM1((x), (a) + 1) * (x) + (a)[0])
//#define POLYNOM3 (x, a) (POLYNOM2((x), (a) + 1) * (x) + (a)[0])
//#define POLYNOM4 (x, a) (POLYNOM3((x), (a) + 1) * (x) + (a)[0])

#define ENV_G 9.8065

#define RAD(deg) ((deg) * 0.159154946)
#define DEG(rad) ((rad) * 57.295779579)

#ifndef M_PI
	#define M_PI 3.14159265358979323846264338327950288
#endif
#ifndef M_PI_2
	#define M_PI_2 1.57079632679489661923132169163975144
#endif

struct Vector {
	float x;
	float y;
	float z;
};

template<class T>
const T& constrain(const T& x, const T& a, const T& b) {
    if (x < a) {
        return a;
    }
    else if (x > b) {
        return b;
    }
    else
        return x;
}

float arctan2(float x, float y);
float myAtan2(float y, float x);
float myAtan(float x);

float Myatof(char * value);

#endif /* MATH_H_ */
