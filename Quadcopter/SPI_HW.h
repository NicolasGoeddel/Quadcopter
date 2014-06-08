/*
 * SPI_HW.h
 *
 *  Created on: 14.08.2013
 *      Author: nicolas
 */

#ifndef SPI_HW_H_
#define SPI_HW_H_

#include <avr/io.h>

typedef struct HWSPI {
		SPI_t * port;
		uint8_t free;
} HWSPI;

extern void SPI_init(HWSPI * spi, PORT_t * port, uint8_t lsbFirst, uint8_t master, uint8_t transferMode, uint8_t prescaler, uint8_t intLevel);

extern uint8_t SPI_status(HWSPI * spi);
extern void SPI_wait(HWSPI * spi);
extern void SPI_send(HWSPI * spi, uint8_t value);
extern uint8_t SPI_transmit(HWSPI * spi, uint8_t value);
extern uint8_t SPI_receive(HWSPI * spi);
extern uint16_t SPI_receive16(HWSPI * spi);



#endif /* SPI_HW_H_ */
