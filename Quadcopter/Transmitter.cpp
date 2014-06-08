/*
 * Transmitter.cpp
 *
 *  Created on: 07.05.2013
 *      Author: nicolas
 */

#include "Transmitter.h"

Transmitter* transmitter = 0;

void ReceiverCallback(uint8_t* payload) {
	transmitter->addPayload(payload);
}
