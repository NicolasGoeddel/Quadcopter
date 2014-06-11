/*
 * USART.cpp
 *
 *  Created on: 04.06.2014
 *      Author: nicolas
 */

#include "USART.h"

/**
 * \brief Set the baudrate value in the USART module
 *
 * This function sets the baudrate register with scaling regarding the CPU
 * frequency and makes sure the baud rate is supported by the hardware.
 * The function can be used if you don't want to calculate the settings
 * yourself or changes to baudrate at runtime is required.
 *
 * \param usart The USART module.
 * \param baud The baudrate.
 * \param cpu_hz The CPU frequency.
 *
 * \retval true if the hardware supports the baud rate
 * \retval false if the hardware does not support the baud rate (i.e. it's
 *               either too high or too low.)
 */
bool USART_setBaudrate(USART_t *usart, uint32_t baud, uint32_t cpu_hz) {
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
	max_rate = cpu_hz / 8;
	/* 4194304 = (2^7) * 8 * (2^12) = (2^BSCALE_MAX) * 8 * (BSEL_MAX+1) */
	min_rate = cpu_hz / 4194304;

	if (!((usart)->CTRLB & USART_CLK2X_bm)) {
		max_rate /= 2;
		min_rate /= 2;
	}

	if ((baud > max_rate) || (baud < min_rate)) {
		return false;
	}

	/* Check if double speed is enabled. */
	if (!((usart)->CTRLB & USART_CLK2X_bm)) {
		baud *= 2;
	}

	/* Find the lowest possible exponent. */
	limit = 0xfffU >> 4;
	ratio = cpu_hz / baud;

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
		cpu_hz -= 8 * baud;

		/* If we end up with a left-shift after taking the final
		 * divide-by-8 into account, do the shift before the divide.
		 * Otherwise, left-shift the denominator instead (effectively
		 * resulting in an overall right shift.)
		 */
		if (exp <= -3) {
			div = ((cpu_hz << (-exp - 3)) + baud / 2) / baud;
		} else {
			baud <<= exp + 3;
			div = (cpu_hz + baud / 2) / baud;
		}
	} else {
		/* We will always do a right shift in this case, but we need to
		 * shift three extra positions because of the divide-by-8.
		 */
		baud <<= exp + 3;
		div = (cpu_hz + baud / 2) / baud - 1;
	}

	(usart)->BAUDCTRLB = (uint8_t)(((div >> 8) & 0X0F) | (exp << 4));
	(usart)->BAUDCTRLA = (uint8_t)div;

	return true;
}

void USART_init(uint32_t baud) {
	//For the sake of example, I'll just REMAP the USART pins from PC3 and PC2 to PC7 and PC6
	PORTD.OUTSET = _BV(7); //Let's make PC7 as TX
	PORTD.DIRSET = _BV(7); //TX pin as output

	PORTD.OUTCLR = _BV(6);
	PORTD.DIRCLR = _BV(6); //PC6 as RX

	//Disable interrupts
	USART_PORT.CTRLA = 0;
	//8 data bits, even parity and 1 stop bit
	//USARTC0_CTRLC = USART_CMODE0_bm | USART_PMODE0_bm | USART_CHSIZE_8BIT_gc;
	USART_PORT.CTRLC = USART_CMODE_ASYNCHRONOUS_gc | USART_CHSIZE_8BIT_gc | USART_PMODE_DISABLED_gc;

	//Enable receive and transmit
	USART_PORT.CTRLB = USART_TXEN_bm | USART_CLK2X_bm | USART_RXEN_bm; // And enable high speed mode

	// Baud rate selection
	USART_setBaudrate(&USART_PORT, baud, F_CPU);
}

void USART_setBaudrate(uint32_t baudrate) {
	USART_setBaudrate(&USART_PORT, baudrate, F_CPU);
}

void USART_putchar(uint8_t c) {
	while(!(USART_PORT.STATUS & USART_DREIF_bm));

	USARTD1.DATA = c;
}

void USART_putString(const char * c) {
	while (*c) {
		USART_putchar(*c++);
	}
}

/**
 * \brief Receive a data with the USART module
 *
 * This function returns the received data from the USART module.
 *
 * \param usart The USART module.
 *
 * \return The received data.
 */
uint8_t USART_getchar() {
	while (!(USART_PORT.STATUS & USART_RXCIF_bm));

	return ((uint8_t)USARTD1.DATA);
}

uint8_t USART_dataAvailable() {
	return (USART_PORT.STATUS & USART_RXCIF_bm) ? 1 : 0;
}

uint8_t USART_getcharAsync() {
	return ((uint8_t)USARTD1.DATA);
}
