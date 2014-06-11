/*
 * libstdcpp.h
 *
 *  Created on: 11.06.2014
 *      Author: nicolas
 */

#ifndef LIBSTDCPP_H_
#define LIBSTDCPP_H_

#include <stdlib.h>

void * operator new(size_t n);

void operator delete(void * p);

#endif /* LIBSTDCPP_H_ */
