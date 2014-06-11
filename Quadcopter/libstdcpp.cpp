/*
 * libstdcpp.cpp
 *
 *  Created on: 11.06.2014
 *      Author: nicolas
 */

#include "libstdcpp.h"

void * operator new(size_t n) {
	void * const p = malloc(n);
	// handle p == 0
	return p;
}

void operator delete(void * p) {
	free(p);
}
