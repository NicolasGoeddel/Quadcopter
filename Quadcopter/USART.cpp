/*
 * USART.cpp
 *
 *  Created on: 04.06.2014
 *      Author: nicolas
 */

#include "USART.h"

bool USART::init(uint32_t baudRate) {
	if (
#	ifdef USARTC0
	(usart == &USARTC0) ||
#	endif
#	ifdef USARTD0
	(usart == &USARTD0) ||
#	endif
#	ifdef USARTE0
	(usart == &USARTE0) ||
#	endif
#	ifdef USARTF0
	(usart == &USARTF0) ||
#	endif
	false) {
		port->OUTSET = _BV(3);
		port->DIRSET = _BV(3);
		port->OUTCLR = _BV(2);
		port->DIRCLR = _BV(2);
	}
	if (
#	ifdef USARTC1
	(usart == &USARTC1) ||
#	endif
#	ifdef USARTD1
	(usart == &USARTD1) ||
#	endif
#	ifdef USARTE1
	(usart == &USARTE1) ||
#	endif
#	ifdef USARTF1
	(usart == &USARTF1) ||
#	endif
	false) {
		port->OUTSET = _BV(7);
		port->DIRSET = _BV(7);
		port->OUTCLR = _BV(6);
		port->DIRCLR = _BV(6);
	}

	//Disable interrupts
	usart->CTRLA = 0;

	//8 data bits, even parity and 1 stop bit
	//USARTC0_CTRLC = USART_CMODE0_bm | USART_PMODE0_bm | USART_CHSIZE_8BIT_gc;
	usart->CTRLC = USART_CMODE_ASYNCHRONOUS_gc | USART_CHSIZE_8BIT_gc | USART_PMODE_DISABLED_gc;

	//Enable receive and transmit and high speed mode
	usart->CTRLB = USART_TXEN_bm | USART_CLK2X_bm | USART_RXEN_bm; //

	// Baud rate selection
	return setBaudrate(baudRate);
}

#define GET_TRIGGER_SOURCE(name) if (usart == &name) return DMA_CH_TRIGSRC_##name##_DRE_gc;
register8_t USART::getTriggerSource(USART_t* usart) {
#	ifdef USARTC0
	GET_TRIGGER_SOURCE(USARTC0)
#	endif
#	ifdef USARTC1
	GET_TRIGGER_SOURCE(USARTC1)
#	endif
#	ifdef USARTD0
	GET_TRIGGER_SOURCE(USARTD0)
#	endif
#	ifdef USARTD1
	GET_TRIGGER_SOURCE(USARTD1)
#	endif
#	ifdef USARTE0
	GET_TRIGGER_SOURCE(USARTE0)
#	endif
#	ifdef USARTE1
	GET_TRIGGER_SOURCE(USARTE1)
#	endif
#	ifdef USARTF0
	GET_TRIGGER_SOURCE(USARTF0)
#	endif
#	ifdef USARTF1
	GET_TRIGGER_SOURCE(USARTF1)
#	endif
	return 0;
}
#undef GET_TRIGGER_SOURCE

bool USART::setBaudrate(uint32_t baudRate) {
	this->baudRate = baudRate;

	int8_t exp;
	uint32_t div;
	uint32_t limit;
	uint32_t ratio;
	uint32_t min_rate;
	uint32_t max_rate;

	/*
	 * Check if the hardware supports the given baud rate
	 */
	/* 8 = (2^0) * 8 * (2^0) = (2^BSCALE_MIN) * 8 * (BSEL_MIN) */
	max_rate = cpuFreq / 8;
	/* 4194304 = (2^7) * 8 * (2^12) = (2^BSCALE_MAX) * 8 * (BSEL_MAX+1) */
	min_rate = cpuFreq / 4194304;

	if (!((usart)->CTRLB & USART_CLK2X_bm)) {
		max_rate /= 2;
		min_rate /= 2;
	}

	if ((baudRate > max_rate) || (baudRate < min_rate)) {
		return false;
	}

	/* Check if double speed is enabled. */
	if (!((usart)->CTRLB & USART_CLK2X_bm)) {
		baudRate *= 2;
	}

	/* Find the lowest possible exponent. */
	limit = 0xfffU >> 4;
	ratio = cpuFreq / baudRate;

	for (exp = -7; exp < 7; exp++) {
		if (ratio < limit) {
			break;
		}

		limit <<= 1;

		if (exp < -3) {
			limit |= 1;
		}
	}

	/*
	 * Depending on the value of exp, scale either the input frequency or
	 * the target baud rate. By always scaling upwards, we never introduce
	 * any additional inaccuracy.
	 *
	 * We are including the final divide-by-8 (aka. right-shift-by-3) in
	 * this operation as it ensures that we never exceeed 2**32 at any
	 * point.
	 *
	 * The formula for calculating BSEL is slightly different when exp is
	 * negative than it is when exp is positive.
	 */
	if (exp < 0) {
		/* We are supposed to subtract 1, then apply BSCALE. We want to
		 * apply BSCALE first, so we need to turn everything inside the
		 * parenthesis into a single fractional expression.
		 */
		cpuFreq -= 8 * baudRate;

		/* If we end up with a left-shift after taking the final
		 * divide-by-8 into account, do the shift before the divide.
		 * Otherwise, left-shift the denominator instead (effectively
		 * resulting in an overall right shift.)
		 */
		if (exp <= -3) {
			div = ((cpuFreq << (-exp - 3)) + baudRate / 2) / baudRate;
		} else {
			baudRate <<= exp + 3;
			div = (cpuFreq + baudRate / 2) / baudRate;
		}
	} else {
		/* We will always do a right shift in this case, but we need to
		 * shift three extra positions because of the divide-by-8.
		 */
		baudRate <<= exp + 3;
		div = (cpuFreq + baudRate / 2) / baudRate - 1;
	}

	(usart)->BAUDCTRLB = (uint8_t)(((div >> 8) & 0X0F) | (exp << 4));
	(usart)->BAUDCTRLA = (uint8_t)div;

	return true;
}

