/*
 * USART.h
 *
 *  Created on: 04.06.2014
 *      Author: nicolas
 */

#ifndef USART_H_
#define USART_H_

#include <avr/io.h>

#define USART_PORT USARTD1

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
bool USART_setBaudrate(USART_t *usart, uint32_t baud, uint32_t cpu_hz);
void USART_setBaudrate(uint32_t baudrate);

void USART_init(uint32_t baud);

void USART_putchar(uint8_t c);

void USART_putString(const char * c);

/**
 * \brief Receive a data with the USART module
 *
 * This function returns the received data from the USART module.
 *
 * \param usart The USART module.
 *
 * \return The received data.
 */
uint8_t USART_getchar();

uint8_t USART_dataAvailable();

uint8_t inline USART_getcharAsync();

#endif /* USART_H_ */
