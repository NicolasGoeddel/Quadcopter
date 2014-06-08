/**
 * Copyright 2012-2013 Nicolas Göddel
 *
 * This file is part of the AVR Includes Project.
 *
 * The AVR Includes Project is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The AVR Includes Project Project is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Diese Datei ist Teil des AVR Include Projekts.
 *
 * Das AVR Include Projekt ist Freie Software: Sie können es unter den Bedingungen
 * der GNU General Public License, wie von der Free Software Foundation,
 * Version 3 der Lizenz oder (nach Ihrer Option) jeder späteren
 * veröffentlichten Version, weiterverbreiten und/oder modifizieren.
 *
 * Das AVR Include Projekt wird in der Hoffnung, dass es nützlich sein wird, aber
 * OHNE JEDE GEWÄHELEISTUNG, bereitgestellt; sogar ohne die implizite
 * Gewährleistung der MARKTFÄHIGKEIT oder EIGNUNG FÜR EINEN BESTIMMTEN ZWECK.
 * Siehe die GNU General Public License für weitere Details.
 *
 * Sie sollten eine Kopie der GNU General Public License zusammen mit diesem
 * Programm erhalten haben. Wenn nicht, siehe <http://www.gnu.org/licenses/>.
 */

#include "DMAController.h"

#define DOUBLE_BUFFERED(channel) (doubleBuffered & _BV((channel) >> 1))
#define CHANNEL(channel) (((DMA_CH_t*)&DMA.CH0)[channel])

uint8_t DMAController::usedChannels = 0;
uint8_t DMAController::doubleBuffered = 0;
uint8_t DMAController::autoDestBuffer = 0;
uint16_t DMAController::blockSizes[4] = {0, 0, 0, 0};
uint8_t* DMAController::buffer[4] = {0, 0, 0, 0};

uint8_t* DMAController::updateAutoBuffer(uint8_t channel, uint16_t blockSize) {
	/*
	 * Falls bei diesem Kanal der Buffer nicht automatisiert alloziert werden soll
	 * oder blockSize = 0 ist, lösche ihn.
	 */

	if (!(autoDestBuffer & _BV(channel)) || blockSize == 0) {
		if ( buffer[channel]) {
			free(buffer[channel]);
			buffer[channel] = 0;
		}
	} else {
		/*
		 * Ist schon ein Buffer mit anderer Größe alloziert, lösche den alten
		 * und alloziere einen neuen.
		 */
		if ((buffer[channel]) && (blockSizes[channel] != blockSize)) {
			free(buffer[channel]);
			blockSizes[channel] = blockSize;
			buffer[channel] = (uint8_t*) malloc(blockSize);
			if (buffer[channel] == 0) {
				return 0;
			}
		}
	}

	return buffer[channel];
}

void DMAController::setBuffer(uint8_t channel) {
	if (autoDestBuffer & _BV(channel)) {
		CHANNEL(channel).DESTADDR0 = ((uintptr_t) buffer[channel] & 0xff) ;
		CHANNEL(channel).DESTADDR1 = ((uintptr_t) buffer[channel] >> 8) & 0xff;
		CHANNEL(channel).DESTADDR2 = (((uintptr_t) buffer[channel]) >> 16) & 0xff;
	}
}

void DMAController::reset() {
	// reset DMA controller
	DMA.CTRL = 0;
	DMA.CTRL = DMA_RESET_bm;
	while ((DMA.CTRL & DMA_RESET_bm) != 0);
	usedChannels = 0;
	doubleBuffered = 0;
	autoDestBuffer = 0;
	for (uint8_t i = 0; i < 3; i++) {
		blockSizes[i] = 0;
		if (buffer[i])
			free(buffer[i]);
		buffer[i] = 0;
	}
}

void DMAController::setBlockSize(uint8_t channel, uint16_t blockSize) {
	if (DOUBLE_BUFFERED(channel)) {
		channel &= ~1;
		updateAutoBuffer(channel, blockSize);
		updateAutoBuffer(channel + 1, blockSize);
		setBuffer(channel);
		setBuffer(channel + 1);
		CHANNEL(channel).TRFCNT = blockSize;
		CHANNEL(channel + 1).TRFCNT = blockSize;
	} else {
		updateAutoBuffer(channel, blockSize);
		setBuffer(channel);
		CHANNEL(channel).TRFCNT = blockSize;

	}
}

void DMAController::useChannel(uint8_t channel, uint8_t use) {
	usedChannels = (usedChannels & ~_BV(channel)) | (use << channel);
	if (!use) {
		disable(channel);
		if (DOUBLE_BUFFERED(channel)) {
			doubleBuffered &= ~_BV(channel >> 1);
			useDoubleBuffering(channel, false);
		}
	}
}

void DMAController::setSource(uint8_t channel, void* source) {
	CHANNEL(channel).SRCADDR0 = (uintptr_t) source & 0xff;
	CHANNEL(channel).SRCADDR1 = ((uintptr_t) source >> 8) & 0xff;
	CHANNEL(channel).SRCADDR2 = ((uintptr_t) source >> 16) & 0xff;
}

void DMAController::setSource(uint8_t channel, void* source, SrcDirection_enum direction, SrcReload_enum mode) {
	setSource(channel, source, source, direction, mode);
}

void DMAController::setSource(uint8_t channel, void* source1, void* source2, SrcDirection_enum direction, SrcReload_enum mode) {
	if (DOUBLE_BUFFERED(channel)) {
		channel &= ~1;

		CHANNEL(channel).SRCADDR0 = (uintptr_t) source1 & 0xff;
		CHANNEL(channel).SRCADDR1 = ((uintptr_t) source1 >> 8) & 0xff;
		CHANNEL(channel).SRCADDR2 = ((uintptr_t) source1 >> 16) & 0xff;
		CHANNEL(channel + 1).SRCADDR0 = (uintptr_t) source2 & 0xff;
		CHANNEL(channel + 1).SRCADDR1 = ((uintptr_t) source2 >> 8) & 0xff;
		CHANNEL(channel + 1).SRCADDR2 = ((uintptr_t) source2 >> 16) & 0xff;

		CHANNEL(channel).ADDRCTRL = (CHANNEL(channel).ADDRCTRL & ~SrcDirection_bm) | direction;
		CHANNEL(channel + 1).ADDRCTRL = (CHANNEL(channel + 1).ADDRCTRL & ~SrcDirection_bm) | direction;

		CHANNEL(channel).ADDRCTRL = (CHANNEL(channel).ADDRCTRL & ~SrcReload_bm) | mode;
		CHANNEL(channel + 1).ADDRCTRL = (CHANNEL(channel + 1).ADDRCTRL & ~SrcReload_bm) | mode;

	} else {
		CHANNEL(channel).SRCADDR0 = (uintptr_t) source1 & 0xff;
		CHANNEL(channel).SRCADDR1 = ((uintptr_t) source1 >> 8) & 0xff;
		CHANNEL(channel).SRCADDR2 = ((uintptr_t) source1 >> 16) & 0xff;
		CHANNEL(channel).ADDRCTRL = (CHANNEL(channel).ADDRCTRL & ~SrcReload_bm) | mode;

		CHANNEL(channel).ADDRCTRL = (CHANNEL(channel).ADDRCTRL & ~SrcDirection_bm) | direction;
	}
}

void DMAController::setSourceDirection(uint8_t channel, SrcDirection_enum direction) {
	if (DOUBLE_BUFFERED(channel)) {
		channel &= ~1;
		CHANNEL(channel).ADDRCTRL = (CHANNEL(channel).ADDRCTRL & ~SrcDirection_bm) | direction;
		CHANNEL(channel + 1).ADDRCTRL = (CHANNEL(channel + 1).ADDRCTRL & ~SrcDirection_bm) | direction;
	} else {
		CHANNEL(channel).ADDRCTRL = (CHANNEL(channel).ADDRCTRL & ~SrcDirection_bm) | direction;
	}
}

void DMAController::setSourceReload(uint8_t channel, SrcReload_enum mode) {
	if (DOUBLE_BUFFERED(channel)) {
		channel &= ~1;
		CHANNEL(channel).ADDRCTRL = (CHANNEL(channel).ADDRCTRL & ~SrcReload_bm) | mode;
		CHANNEL(channel + 1).ADDRCTRL = (CHANNEL(channel + 1).ADDRCTRL & ~SrcReload_bm) | mode;
	} else {
		CHANNEL(channel).ADDRCTRL = (CHANNEL(channel).ADDRCTRL & ~SrcReload_bm) | mode;
	}
}

void* DMAController::setDestination(uint8_t channel, void* destination) {
	if (destination) {
		autoDestBuffer &= ~_BV(channel);
		updateAutoBuffer(channel, blockSizes[channel]);
		CHANNEL(channel).DESTADDR0 = (uintptr_t) destination & 0xff;
		CHANNEL(channel).DESTADDR1 = ((uintptr_t) destination >> 8) & 0xff;
		CHANNEL(channel).DESTADDR2 = ((uintptr_t) destination >> 16) & 0xff;
		return destination;

	} else {
		autoDestBuffer |= _BV(channel);
		if (updateAutoBuffer(channel, blockSizes[channel])) {
			setBuffer(channel);
			return buffer[channel];
		}
		return 0;
	}
}

void* DMAController::setDestination(uint8_t channel, void* destination, DestDirection_enum direction, DestReload_enum mode) {
	setDestination(channel, destination, destination, direction, mode);
	if (autoDestBuffer & _BV(channel)) {
		return buffer[channel];
	}
	return destination;
}

void DMAController::setDestination(uint8_t channel, void* destination1, void* destination2, DestDirection_enum direction, DestReload_enum mode) {
	if (DOUBLE_BUFFERED(channel)) {
		channel &= ~1;
		CHANNEL(channel).ADDRCTRL = (CHANNEL(channel).ADDRCTRL & ~DestDirection_bm) | direction;
		CHANNEL(channel + 1).ADDRCTRL = (CHANNEL(channel + 1).ADDRCTRL & ~DestDirection_bm) | direction;

		CHANNEL(channel).ADDRCTRL = (CHANNEL(channel).ADDRCTRL & ~DestReload_bm) | mode;
		CHANNEL(channel + 1).ADDRCTRL = (CHANNEL(channel + 1).ADDRCTRL & ~DestReload_bm) | mode;

		if (destination1) {
			autoDestBuffer &= ~_BV(channel);
			updateAutoBuffer(channel, blockSizes[channel]);
			CHANNEL(channel).DESTADDR0 = (uintptr_t) destination1 & 0xff;
			CHANNEL(channel).DESTADDR1 = ((uintptr_t) destination1 >> 8) & 0xff;
			CHANNEL(channel).DESTADDR2 = ((uintptr_t) destination1 >> 16) & 0xff;
		} else {
			autoDestBuffer |= _BV(channel);
			if (updateAutoBuffer(channel, blockSizes[channel])) {
				setBuffer(channel);
			}
		}

		if (destination2) {
			autoDestBuffer &= ~_BV(channel + 1);
			updateAutoBuffer(channel + 1, blockSizes[channel + 1]);
			CHANNEL(channel + 1).DESTADDR0 = (uintptr_t) destination2 & 0xff;
			CHANNEL(channel + 1).DESTADDR1 = ((uintptr_t) destination2 >> 8) & 0xff;
			CHANNEL(channel + 1).DESTADDR2 = ((uintptr_t) destination2 >> 16) & 0xff;
		} else {
			autoDestBuffer |= _BV(channel + 1);
			if (updateAutoBuffer(channel + 1, blockSizes[channel + 1])) {
				setBuffer(channel + 1);
			}
		}


	} else {
		CHANNEL(channel).ADDRCTRL = (CHANNEL(channel).ADDRCTRL & ~DestDirection_bm) | direction;

		CHANNEL(channel).ADDRCTRL = (CHANNEL(channel).ADDRCTRL & ~DestReload_bm) | mode;

		if (destination1) {
			autoDestBuffer &= ~_BV(channel);
			updateAutoBuffer(channel, blockSizes[channel]);
			CHANNEL(channel).DESTADDR0 = (uintptr_t) destination1 & 0xff;
			CHANNEL(channel).DESTADDR1 = ((uintptr_t) destination1 >> 8) & 0xff;
			CHANNEL(channel).DESTADDR2 = ((uintptr_t) destination1 >> 16) & 0xff;
		} else {
			autoDestBuffer |= _BV(channel);
			if (updateAutoBuffer(channel, blockSizes[channel])) {
				setBuffer(channel);
			}
		}
	}
}

void DMAController::setDestDirection(uint8_t channel, DestDirection_enum direction) {
	if (DOUBLE_BUFFERED(channel)) {
		channel &= ~1;
		CHANNEL(channel).ADDRCTRL = (CHANNEL(channel).ADDRCTRL & ~DestDirection_bm) | direction;
		CHANNEL(channel + 1).ADDRCTRL = (CHANNEL(channel + 1).ADDRCTRL & ~DestDirection_bm) | direction;
	} else {
		CHANNEL(channel).ADDRCTRL = (CHANNEL(channel).ADDRCTRL & ~DestDirection_bm) | direction;
	}
}

void DMAController::setDestReload(uint8_t channel, DestReload_enum mode) {
	if (DOUBLE_BUFFERED(channel)) {
		channel &= ~1;
		CHANNEL(channel).ADDRCTRL = (CHANNEL(channel).ADDRCTRL & ~DestReload_bm) | mode;
		CHANNEL(channel + 1).ADDRCTRL = (CHANNEL(channel + 1).ADDRCTRL & ~DestReload_bm) | mode;
	} else {
		CHANNEL(channel).ADDRCTRL = (CHANNEL(channel).ADDRCTRL & ~DestReload_bm) | mode;
	}
}

void DMAController::setBurstLength(uint8_t channel, BurstLength_enum burstLength) {
	if (DOUBLE_BUFFERED(channel)) {
		channel &= ~1;
		CHANNEL(channel).CTRLA = (CHANNEL(channel).CTRLA & ~BurstLength_bm) | burstLength;
		CHANNEL(channel + 1).CTRLA = (CHANNEL(channel + 1).CTRLA & ~BurstLength_bm) | burstLength;
	} else {
		CHANNEL(channel).CTRLA = (CHANNEL(channel).CTRLA & ~BurstLength_bm) | burstLength;
	}
}

void DMAController::enable(uint8_t channel) {
	if (DOUBLE_BUFFERED(channel)) {
		channel &= ~1;
		CHANNEL(channel).CTRLA |= DMA_CH_ENABLE_bm;
		CHANNEL(channel + 1).CTRLA &= ~DMA_CH_ENABLE_bm;
	} else {
		CHANNEL(channel).CTRLA |= DMA_CH_ENABLE_bm;
	}
}

void DMAController::disable(uint8_t channel) {
	if (DOUBLE_BUFFERED(channel)) {
		channel &= ~1;
		CHANNEL(channel).CTRLA &= ~DMA_CH_ENABLE_bm;
		CHANNEL(channel + 1).CTRLA &= ~DMA_CH_ENABLE_bm;
	} else {
		CHANNEL(channel).CTRLA &= ~DMA_CH_ENABLE_bm;
	}
}

bool DMAController::isTransactionComplete(uint8_t channel) {
	bool result = (DMA.INTFLAGS & _BV(channel)) == _BV(channel);
	if (result) {
		DMA.INTFLAGS |= _BV(channel);
	}
	return result && !isBlockTransferBusy(channel);
}

bool DMAController::isBlockTransferBusy(uint8_t channel) {
	return (CHANNEL(channel).CTRLB & (DMA_CH_CHBUSY_bm | DMA_CH_CHPEND_bm));
}

void DMAController::trigger(uint8_t channel) {
	while (CHANNEL(channel).CTRLA & DMA_CH_TRFREQ_bm);
	CHANNEL(channel).CTRLA |= DMA_CH_TRFREQ_bm;
}

bool DMAController::getFreeChannel(uint8_t& channel) {
	uint8_t c = 0;
	while (c < 4) {
		if (usedChannels & _BV(c)) break;
		c++;
	}
	if (c == 4) return false;
	channel = c;
	return true;
}

bool DMAController::copyMemory(void* source, void* destination, uint16_t size) {
	uint8_t channel;
	if (!getFreeChannel(channel)) return false;

	if (!copyMemory(channel, source, destination, size))
		return false;

	while (!isTransactionComplete(channel));
	useChannel(channel, false);

	return true;
}

bool DMAController::copyMemory(uint8_t channel, void* source, void* destination, uint16_t size) {
	if (usedChannels & _BV(channel)) return false;

	disable(channel);

	useChannel(channel);
	useDoubleBuffering(channel, false);

	setBlockSize(channel, size);

	setSource(channel, source, SrcDirectionInc, SrcReloadTransaction);

	setDestination(channel, destination, DestDirectionInc, DestReloadTransaction);

	setBurstLength(channel, BurstLength1Byte);
	setRepeatCount(channel, 1);
	useSingleShot(channel, false);
	setTriggerSource(channel, DMA_CH_TRIGSRC_OFF_gc);

	enable(channel);
	trigger(channel);

	return true;
}

bool DMAController::fillMemory(void* destination, uint16_t size, uint8_t byte) {
	uint8_t channel;
	if (!getFreeChannel(channel)) return false;

	disable(channel);

	useChannel(channel);
	useDoubleBuffering(channel, false);

	setBlockSize(channel, size);

	setSource(channel, &byte);
	setSourceDirection(channel, SrcDirectionFixed);
	setSourceReload(channel, SrcReloadBurst);

	setDestination(channel, destination);
	setDestDirection(channel, DestDirectionInc);
	setDestReload(channel, DestReloadTransaction);

	setBurstLength(channel, BurstLength1Byte);
	setRepeatCount(channel, 1);
	useSingleShot(channel, false);
	setTriggerSource(channel, DMA_CH_TRIGSRC_OFF_gc);

	enable(channel);
	trigger(channel);

	while (!isTransactionComplete(channel));
	useChannel(channel, false);

	return true;
}

void DMAController::setRepeatCount(uint8_t channel, uint8_t repeatCount) {
	if (repeatCount != 1) {
		if (DOUBLE_BUFFERED(channel)) {
			channel &= ~1;
			CHANNEL(channel).REPCNT = repeatCount;
			CHANNEL(channel + 1).REPCNT = repeatCount;
			CHANNEL(channel).CTRLA |= DMA_CH_REPEAT_bm;
			CHANNEL(channel + 1).CTRLA |= DMA_CH_REPEAT_bm;
		} else {
			CHANNEL(channel).REPCNT = repeatCount;
			CHANNEL(channel).CTRLA |= DMA_CH_REPEAT_bm;
		}
	} else {
		if (DOUBLE_BUFFERED(channel)) {
			channel &= ~1;
			CHANNEL(channel).CTRLA &= ~DMA_CH_REPEAT_bm;
			CHANNEL(channel + 1).CTRLA &= ~DMA_CH_REPEAT_bm;
		} else {
			CHANNEL(channel).CTRLA &= ~DMA_CH_REPEAT_bm;
		}
	}
}

void DMAController::useSingleShot(uint8_t channel, bool use) {
	uint8_t iUse = (use) ? 1 : 0;
	if (DOUBLE_BUFFERED(channel)) {
		channel &= ~1;
		CHANNEL(channel).CTRLA = (CHANNEL(channel).CTRLA & ~DMA_CH_SINGLE_bm) | (iUse << DMA_CH_SINGLE_bp);
		CHANNEL(channel + 1).CTRLA = (CHANNEL(channel + 1).CTRLA & ~DMA_CH_SINGLE_bm) | (iUse << DMA_CH_SINGLE_bp);
	} else {
		CHANNEL(channel).CTRLA = (CHANNEL(channel).CTRLA & ~DMA_CH_SINGLE_bm) | (iUse << DMA_CH_SINGLE_bp);
	}
}

void DMAController::setTriggerSource(uint8_t channel, uint8_t triggerSource) {
	if (DOUBLE_BUFFERED(channel)) {
		channel &= ~1;
		CHANNEL(channel).TRIGSRC = triggerSource;
		CHANNEL(channel + 1).TRIGSRC = triggerSource;
	} else {
		CHANNEL(channel).TRIGSRC = triggerSource;
	}
}


#undef CHANNEL
#undef DOUBLE_BUFFERED
