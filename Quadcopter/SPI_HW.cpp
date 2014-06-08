/*
 * SPI_HW.c
 *
 *  Created on: 14.08.2013
 *      Author: nicolas
 */

#include "SPI_HW.h"

void SPI_init(HWSPI * spi, PORT_t * port, uint8_t lsbFirst, uint8_t master, uint8_t transferMode, uint8_t prescaler, uint8_t intLevel) {
	register8_t control = SPI_ENABLE_bm;
	if (lsbFirst) {
		/* DORD decides the data order when a byte is shifted out from the DATA register. When DORD is written to one,
		 * the least-significant bit (lsb) of the data byte is transmitted first, and when DORD is written to zero, the most-signif-
		 * icant bit (msb) of the data byte is transmitted first.
		 */
		control |= SPI_DORD_bm;
	}
	if (master) {
		/* This bit selects master mode when written to one, and slave mode when written to zero. If SS is configured as an
		 * input and driven low while master mode is set, master mode will be cleared.
		 */
		control |= SPI_MASTER_bm;

		if (prescaler >= 128) {
			control |= SPI_PRESCALER0_bm | SPI_PRESCALER1_bm;
		} else {
			uint8_t tmp = 0;	//Untere 2 Bits sind SPI_PRESCALER[0|1]_bm und drittes Bit ist der invertierte CLK2X
			while (prescaler > 2) {
				prescaler >>= 1;
				tmp++;
			}
			control |=  (((~tmp) & 1) << SPI_CLK2X_bp) | ((tmp >> 1) & 0x03);
		}
		port->DIRCLR = _BV(6);
		port->PIN6CTRL = PORT_OPC_PULLUP_gc;
		port->DIRSET = _BV(5) | _BV(7) | _BV(4);
	} else {
		port->DIRCLR = _BV(7) | _BV(5) | _BV(4);
		port->DIRSET = _BV(6);
	}
	control |= ((transferMode & 0x03) << 2);

	spi->port->CTRL = control;
	spi->port->INTCTRL = (intLevel & 0x03);
	spi->port->STATUS = 0;
	spi->free = 1;
}

uint8_t SPI_status(HWSPI * spi) {
	return (spi->port->STATUS & SPI_IF_bm) ? 1 : 0;
}

void SPI_wait(HWSPI * spi) {
	while (!(spi->port->STATUS & SPI_IF_bm));
	spi->free = 1;
}

void SPI_send(HWSPI * spi, uint8_t value) {
	if (!spi->free) {
		while (!(spi->port->STATUS & SPI_IF_bm));
	}
	spi->port->DATA = value;
	spi->free = 0;
}

uint8_t SPI_transmit(HWSPI * spi, uint8_t value) {
	if (!spi->free) {
		while (!(spi->port->STATUS & SPI_IF_bm));
	}
	spi->port->DATA = value;
	while (!(spi->port->STATUS & SPI_IF_bm));
	return spi->port->DATA;
}

uint8_t SPI_receive(HWSPI * spi) {
	if (!spi->free) {
		while (!(spi->port->STATUS & SPI_IF_bm));
	}
	spi->port->DATA = 0;
	while (!(spi->port->STATUS & SPI_IF_bm));
	return spi->port->DATA;
}

uint16_t SPI_receive16(HWSPI * spi) {
	uint8_t value;
	if (!spi->free) {
		while (!(spi->port->STATUS & SPI_IF_bm));
	}
	spi->port->DATA = 0;
	while (!(spi->port->STATUS & SPI_IF_bm));
	value = spi->port->DATA;
	spi->port->DATA = 0;
	while (!(spi->port->STATUS & SPI_IF_bm));
	return spi->port->DATA | (value << 8);
}
