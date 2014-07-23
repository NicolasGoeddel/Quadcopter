/*
 * USART.h
 *
 *  Created on: 04.06.2014
 *      Author: nicolas
 */

#ifndef USART_H_
#define USART_H_

#include <avr/io.h>
#include "StringDevice.h"

class USART : StringDevice<USART> {
	private:
		USART_t* usart;
		PORT_t* port;
		uint32_t baudRate;
		uint32_t cpuFreq;
	public:
		USART(USART_t* usart, PORT_t* port) {
			this->usart = usart;
			this->port = port;
			cpuFreq = F_CPU;
			init(9600);
		}

		~USART() {

		}

		void setCPUFrequency(uint32_t cpuFreq) {
			this->cpuFreq = cpuFreq;
			setBaudrate(baudRate);
		}

		/**
		 * Bisher ist nur die Baudrate variabel. 1 Stopbit, gerade Parität und 8 Bit Datengröße.
		 *
		 * @param baudrate
		 */
		bool init(uint32_t baudRate);

		/**
		 * \brief Set the baudrate value in the USART module
		 *
		 * This function sets the baudrate register with scaling regarding the CPU
		 * frequency and makes sure the baud rate is supported by the hardware.
		 * The function can be used if you don't want to calculate the settings
		 * yourself or changes to baudrate at runtime is required.
		 *
		 * \param baudRate The baudrate.
		 *
		 * \retval true if the hardware supports the baud rate.
		 * \retval false if the hardware does not support the baud rate (i.e. it's
		 *               either too high or too low).
		 */
		bool setBaudrate(uint32_t baudRate);

		using StringDevice<USART>::write;

		USART* writeChar(const char c) {
			while(!(usart->STATUS & USART_DREIF_bm));

			USARTD1.DATA = c;
			return this;
		}

		char receiveChar(){
			while (!(usart->STATUS & USART_RXCIF_bm));

			return ((uint8_t)usart->DATA);
		}

		char receiveCharAsync() {
			return ((uint8_t)usart->DATA);
		}

		bool isDataAvailable() {
			return (usart->STATUS & USART_RXCIF_bm) ? true : false;
		}
};

#endif /* USART_H_ */
