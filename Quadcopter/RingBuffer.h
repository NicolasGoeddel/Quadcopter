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

template <class CapacityType>
class RingBufferOutDMA : Interrupt, public StringDeviceOut<RingBufferOutDMA<CapacityType> > {
	private:
		CapacityType readIndex;
		CapacityType writeIndex;
		CapacityType newDataLen;
		CapacityType free;
		CapacityType size;
		uint8_t* buffer;
		bool isrDone;
		DMA_CH_t* dma;
		uint8_t* destination;
		register8_t triggerSource;

		CapacityType writeBuf(char* buf, CapacityType len) {
			if (len > size) {
				return 0;
			}

			// busy wait until there is enough space in the ringbuffer
			while (free < len);

			// busy wait until ISR is done
			while (!isrDone);

			for (CapacityType i = 0; i < len; i++) {
				buffer[(writeIndex + i) % size] = buf[i];
			}

			// enter critical section
			volatile uint8_t savedSreg = SREG;
			cli();

			writeIndex = (writeIndex + len) % size;
			free -= len;
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

				uint16_t srcAddr = ((uint16_t) buf) + readIndex;
				dma->SRCADDR0 = srcAddr & 0xff;
				dma->SRCADDR1 = (srcAddr >> 8) & 0xff;
				dma->SRCADDR2 = 0;
				dma->TRFCNT = transferLen;
				dma->CTRLA |= DMA_CH_ENABLE_bm;

				isrDone = false;
			}

			// Leave critical section
			SREG = savedSreg;

			return len;
		}

		CapacityType writeBufISR(char* buf, CapacityType len) {
			if (len > size) {
				return 0;
			}

			if (free < len) {
				return 0;
			}

			for (CapacityType i = 0; i < len; i++) {
				buffer[(writeIndex + i) % size] = buf[i];
			}

			writeIndex = (writeIndex + len) % size;
			free -= len;
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

				uint16_t srcAddr = ((uint16_t) buf) + readIndex;
				dma->SRCADDR0 = srcAddr & 0xff;
				dma->SRCADDR1 = (srcAddr >> 8) & 0xff;
				dma->SRCADDR2 = 0;
				dma->TRFCNT = transferLen;
				dma->CTRLA |= DMA_CH_ENABLE_bm;

				isrDone = false;
			}

			return len;
		}

	public:
		RingBufferOutDMA(CapacityType size, DMA_CH_t* DMAChan, uint8_t* destination, register8_t triggerSource) {
			readIndex = 0;
			writeIndex = 0;
			newDataLen = 0;
			this->size = size;
			free = size;
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

			buffer = new uint8_t(size);
			this->destination = destination;
			this->triggerSource = triggerSource;

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

		~RingBufferOutDMA() {}

		using StringDeviceOut<RingBufferOutDMA>::write;

		RingBufferOutDMA* writeChar(char c) {
			writeBuf(&c, 1);
			return this;
		}

		void writeCharISR(char c) {
			writeBufISR(&c, 1);
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
			free += lastReadLen;
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
			} else {
				// otherwise reset the DMA channel
				dma->TRFCNT = 0;
				dma->CTRLA &= DMA_CH_ENABLE_bm;
			}

			// since ERRIF and TRNIF share the same interrupt, we need to clear
			// the interrupt flag manually by *setting* the corresponding flag bits
			dma->CTRLB |= (DMA_CH_TRNIF_bm | DMA_CH_ERRIF_bm);

			isrDone = true;
		}
};

#endif /* RINGBUFFER_H_ */
