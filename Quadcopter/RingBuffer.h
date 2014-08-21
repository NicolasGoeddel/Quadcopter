/*
 * RingBuffer.h
 *
 *  Created on: 30.07.2014
 *      Author: nicolas
 */

#ifndef RINGBUFFER_H_
#define RINGBUFFER_H_

#include <avr/io.h>
#include <avr/interrupt.h>
#include "XMEGA_helper.h"
#include "InterruptHelper.h"
#include "StringDevice.h"
#include "DEBUG.h"

template <class CapacityType>
class RingBufferOutDMA : Interrupt, public StringDeviceOut<RingBufferOutDMA<CapacityType> > {
	private:
		CapacityType readIndex;
		CapacityType writeIndex;
		CapacityType newDataLen;
		CapacityType cFree;
		CapacityType size;
		uint8_t* buffer;
		volatile bool isrDone;
		DMA_CH_t* dma;
		uint8_t* destination;

	public:
		RingBufferOutDMA(CapacityType size, DMA_CH_t* DMAChan, uint8_t* destination, register8_t triggerSource) {
			readIndex = 0;
			writeIndex = 0;
			newDataLen = 0;
			this->size = size;
			cFree = size;
			isrDone = true;

			if (DMAChan == 0) {
				dma = getFreeDMAChannel();
			} else {
				dma = DMAChan;
			}

			INTERRUPT_NUM_t interruptNum = DMA_CH0_vect_num;
			if (dma == &DMA.CH1) {
				interruptNum = DMA_CH1_vect_num;
			} else if (dma == &DMA.CH2) {
				interruptNum = DMA_CH2_vect_num;
			} else if (dma ==  &DMA.CH3) {
				interruptNum = DMA_CH3_vect_num;
			}

			buffer = (uint8_t*) malloc(size);
			this->destination = destination;

			dma->SRCADDR0 = 0;
			dma->SRCADDR1 = 0;
			dma->SRCADDR2 = 0;

			dma->DESTADDR0 = ((uint16_t) destination) & 0xff;
			dma->DESTADDR1 = (((uint16_t) destination) >> 8) & 0xff;
			dma->DESTADDR2 = 0;

			dma->ADDRCTRL = DMA_CH_SRCRELOAD_NONE_gc | DMA_CH_SRCDIR_INC_gc | DMA_CH_DESTRELOAD_NONE_gc | DMA_CH_DESTDIR_FIXED_gc;

			dma->TRIGSRC = triggerSource;
			dma->TRFCNT = 0;
			dma->REPCNT = 0;
			dma->CTRLA = DMA_CH_SINGLE_bm | DMA_CH_BURSTLEN_1BYTE_gc;

			dma->CTRLB = DMA_CH_TRNINTLVL_LO_gc;

			DMA.CTRL |= DMA_ENABLE_bm;

			addToInterrupt(interruptNum);
		}

		~RingBufferOutDMA() {
			free(buffer);
		}

		CapacityType writeBuf(const char* buffer, CapacityType len = 0, bool inISR = false) {
			if (len > size) {
				return 0;
			}

			// len of zero means a null-terminated string was passed
			if (len == 0) {
				// calculate the length of the string
				while (*buffer) {
					len++;
					buffer++;
				}
				buffer -= len;
			}

			// busy wait until there is enough space in the ringbuffer
			if (inISR) {
				if (cFree < len) return 0;
			} else {
				while (cFree < len);
				// busy wait until ISR is done
				while (!isrDone);
			}

			for (CapacityType i = 0; i < len; i++) {
				this->buffer[(writeIndex + i) % size] = buffer[i];
			}

			volatile uint8_t savedSreg = SREG;
			if (!inISR) {
				// enter critical section
				cli();
			}

			writeIndex = (writeIndex + len) % size;
			cFree -= len;
			newDataLen += len;

			// if there is no DMA transfer in progress already, initiate a new one
			if (!(dma->CTRLB & DMA_CH_CHBUSY_bm) && !(dma->CTRLA & DMA_CH_ENABLE_bm) && isrDone) {
				CapacityType maxLen = size - readIndex; //maximum length for this transfer
				CapacityType transferLen;

				// only transfer data up to the buffer boundary
				if (newDataLen > maxLen) {
					transferLen = maxLen;
				} else {
					transferLen = newDataLen;
				}

				uint16_t srcAddr = ((uint16_t) this->buffer) + readIndex;
				dma->SRCADDR0 = srcAddr & 0xff;
				dma->SRCADDR1 = (srcAddr >> 8) & 0xff;
				dma->SRCADDR2 = 0;
				dma->TRFCNT = transferLen;
				dma->CTRLA |= DMA_CH_ENABLE_bm;
				//dma->CTRLA |= DMA_CH_TRFREQ_bm;

				isrDone = false;
			}

			if (!inISR) {
				// Leave critical section
				SREG = savedSreg;
			}

			return len;
		}

		using StringDeviceOut<RingBufferOutDMA>::write;

		RingBufferOutDMA* writeChar(char c) {
			writeBuf(&c, 1);
			return this;
		}

		RingBufferOutDMA* writeCharISR(char c) {
			writeBuf(&c, 1, true);
			return this;
		}

		void interrupt() {
			// Is DMA_TX_Channel still busy?
			if (dma->CTRLB & DMA_CH_CHBUSY_bm) {
				return;
			}

			// number of bytes transferred in the last DMA transaction
			CapacityType lastReadLen = dma->TRFCNT;

			// Das hier sollte auch nie passieren!
			if (lastReadLen > this->newDataLen) {
				return;
			}
			this->readIndex = (this->readIndex + lastReadLen) % size;

			this->newDataLen -= lastReadLen;
			cFree += lastReadLen;
			CapacityType readIndex = this->readIndex;
			CapacityType newDataLen = this->newDataLen;

			// if there is data left to transfer, initiate a new DMA transaction
			if (newDataLen > 0) {
				// maximum length for this transfer
				CapacityType maxLen = size - readIndex;
				CapacityType len;
				if (newDataLen > maxLen) {
					len = maxLen;
				} else {
					len = newDataLen;
				}

				// set source address
				uint16_t srcAddr = ((uint16_t) buffer) + readIndex;
				dma->SRCADDR0 = srcAddr & 0xff;
				dma->SRCADDR1 = (srcAddr >> 8) & 0xff;
				dma->SRCADDR2 = 0;
				dma->TRFCNT = len;
				dma->CTRLA |= DMA_CH_ENABLE_bm;
				//dma->CTRLA |= DMA_CH_TRFREQ_bm;
			} else {
				// otherwise reset the DMA channel
				dma->TRFCNT = 0;
				dma->CTRLA &= ~DMA_CH_ENABLE_bm;
			}

			// since ERRIF and TRNIF share the same interrupt, we need to clear
			// the interrupt flag manually by *setting* the corresponding flag bits
			dma->CTRLB |= (DMA_CH_TRNIF_bm | DMA_CH_ERRIF_bm);

			isrDone = true;
		}

};

template <class CapacityType>
class RingBuffer {
	private:
		volatile bool lock;
		CapacityType readIndex;
		CapacityType writeIndex;
		CapacityType size;
		CapacityType cFree;
		uint8_t* buffer;

	public:
		RingBuffer(CapacityType size) {
			this->size = size;
			lock = false;
			cFree = size;
			readIndex = 0;
			writeIndex = 0;

			buffer = (uint8_t*) malloc(size);
		}

		~RingBuffer() {
			free(buffer);
		}

		RingBuffer* writeChar(char c) {
//			volatile uint8_t savedSREG = SREG;
//			cli();

			if (cFree) {
				buffer[writeIndex++] = (uint8_t) c;
				if (writeIndex == size) {
					writeIndex = 0;
				}
				cFree--;
			}

//			SREG = savedSREG;

			return this;
		}

		uint8_t receiveChar() {
			while (cFree == size);
			uint8_t c = 0;

			if (cFree < size) {
				c = buffer[readIndex++];
				if (readIndex == size) {
					readIndex = 0;
				}
				cFree++;
			}

			return c;
		}
};

template <class CapacityType>
class RingBufferInDMA : Interrupt {
	private:
		CapacityType readIndex;
		CapacityType writeIndex;
		CapacityType read;
		CapacityType size;
		CapacityType chunkSize;
		uint8_t* buffer;
		volatile bool isrDone;
		DMA_CH_t* dma;
		uint8_t* source;

	public:
		RingBufferInDMA(CapacityType size, CapacityType chunkSize, DMA_CH_t* DMAChan, uint8_t* source, register8_t triggerSource) {
			readIndex = 0;
			writeIndex = 0;
			this->size = size;
			if (chunkSize > size) {
				chunkSize = size;
			}
			this->chunkSize = chunkSize;
			read = 0;
			isrDone = true;

			if (DMAChan == 0) {
				dma = getFreeDMAChannel();
			} else {
				dma = DMAChan;
			}

			INTERRUPT_NUM_t interruptNum = DMA_CH0_vect_num;
			if (dma == &DMA.CH1) {
				interruptNum = DMA_CH1_vect_num;
			} else if (dma == &DMA.CH2) {
				interruptNum = DMA_CH2_vect_num;
			} else if (dma ==  &DMA.CH3) {
				interruptNum = DMA_CH3_vect_num;
			}

			buffer = (uint8_t*) malloc(size);
			this->source = source;

			dma->SRCADDR0 = ((uint16_t) source) & 0xff;
			dma->SRCADDR1 = (((uint16_t) source) >> 8) & 0xff;
			dma->SRCADDR2 = 0;

			dma->DESTADDR0 = 0;
			dma->DESTADDR1 = 0;
			dma->DESTADDR2 = 0;

			dma->ADDRCTRL = DMA_CH_DESTRELOAD_NONE_gc | DMA_CH_DESTDIR_INC_gc | DMA_CH_SRCRELOAD_NONE_gc | DMA_CH_SRCDIR_FIXED_gc;

			dma->TRIGSRC = triggerSource;
			dma->TRFCNT = 0;
			dma->REPCNT = 0;
			dma->CTRLA = DMA_CH_SINGLE_bm | DMA_CH_BURSTLEN_1BYTE_gc;

			dma->CTRLB = DMA_CH_TRNINTLVL_LO_gc;

			DMA.CTRL |= DMA_ENABLE_bm;

			dma->TRFCNT = chunkSize;
			dma->CTRLA |= DMA_CH_ENABLE_bm;

			addToInterrupt(interruptNum);
		}

		~RingBufferInDMA() {}

		CapacityType receiveBuf(const char* buffer, CapacityType len = 0, bool inISR = false) {
			if (len > size) {
				return 0;
			}

			// len of zero means a null-terminated string was passed
//			if (len == 0) {
//				// calculate the length of the string
//				while (*buffer) {
//					len++;
//					buffer++;
//				}
//				buffer -= len;
//			}

			// busy wait until there was copied enough bytes to the ringbuffer
			if (inISR) {
				if (read < len) return 0;
			} else {
				// Ist genug im Puffer um es am Stück lesen zu können?
				while (read < len) {
					// Falls nicht, warte bis der ISR durch ist und teste erneut
					// busy wait until ISR is done
					//FIXM Braucht man isrDone überhaupt?
					while (!isrDone);
				}
			}

			// Schalte den ISR kurzzeitig aus
			volatile uint8_t savedSreg = SREG;
			if (!inISR) {
				// enter critical section
				cli();
			}

			// Kopiere die Daten
			for (CapacityType i = 0; i < len; i++) {
				buffer[i] = this->buffer[(readIndex + i) % size];
			}

			// Passe die Leseposition an und korrigiere die Anzahl Bytes, die zum Auslesen bereit sind
			readIndex = (readIndex + len) % size;
			read -= len;

			if (!inISR) {
				// Leave critical section
				SREG = savedSreg;
			}

			return len;
		}

		uint8_t receiveChar() {
			uint8_t c;
			receiveBuf((char*) &c, 1);
			return c;
		}

		uint8_t receiveCharISR() {
			uint8_t c;
			receiveBuf((char*) &c, 1, true);
			return c;
		}

		void interrupt() {
			// Is DMA_TX_Channel still busy? (Sollte eigentlich nie passieren)
			if (dma->CTRLB & DMA_CH_CHBUSY_bm) {
				return;
			}

			// number of bytes transferred in the last DMA transaction
			CapacityType lastWriteLen = dma->TRFCNT;

			this->writeIndex = (this->writeIndex + lastWriteLen) % size;

			read += lastWriteLen;
			CapacityType writeIndex = this->writeIndex;

			// maximum length for this transfer
			CapacityType maxLen = size - writeIndex;
			CapacityType len = chunkSize;

			if (len > maxLen) {
				len = maxLen;
			}

			// set source address
			uint16_t destAddr = ((uint16_t) buffer) + writeIndex;
			dma->DESTADDR0 = destAddr & 0xff;
			dma->DESTADDR1 = (destAddr >> 8) & 0xff;
			dma->DESTADDR2 = 0;
			dma->TRFCNT = len;
			dma->CTRLA |= DMA_CH_ENABLE_bm;

			// since ERRIF and TRNIF share the same interrupt, we need to clear
			// the interrupt flag manually by *setting* the corresponding flag bits
			dma->CTRLB |= (DMA_CH_TRNIF_bm | DMA_CH_ERRIF_bm);

			isrDone = true;
		}
};

#endif /* RINGBUFFER_H_ */
