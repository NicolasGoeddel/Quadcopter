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
#include <stdlib.h>
#include <avr/io.h>
//#include <stdint.h>

#ifndef DMAController_H_
#define DMAController_H_

/*
 * Singleton Klasse für den DMAController.
 */
class DMAController {

	private:
		DMAController() {}
		virtual ~DMAController() {}

		// Ein Bit pro genutztem Kanal
		static uint8_t usedChannels;
		// Ein Bit pro zwei genutzen Kanälen
		static uint8_t doubleBuffered;
		static uint8_t* buffer[4];
		static uint16_t blockSizes[4];
		static uint8_t autoDestBuffer;

		/**
		 * Ändert die Puffergröße eines Puffers auf die gewünschte Größe, falls
		 * das automatische Allozieren gewünscht ist.
		 */
		static uint8_t* updateAutoBuffer(uint8_t channel, uint16_t blockSize);

		static inline void setBuffer(uint8_t channel);

	public:

		enum SrcReload_enum {
			SrcReloadNone = DMA_CH_SRCRELOAD_NONE_gc,  /* No reload */
			SrcReloadBlock = DMA_CH_SRCRELOAD_BLOCK_gc,  /* Reload at end of block */
			SrcReloadBurst = DMA_CH_SRCRELOAD_BURST_gc,  /* Reload at end of burst */
			SrcReloadTransaction = DMA_CH_SRCRELOAD_TRANSACTION_gc,  /* Reload at end of transaction */
			SrcReload_bm = SrcReloadNone | SrcReloadBlock | SrcReloadBurst | SrcReloadTransaction
		};

		enum DestReload_enum {
			DestReloadNone = DMA_CH_DESTRELOAD_NONE_gc,  /* No reload */
			DestReloadBlock = DMA_CH_DESTRELOAD_BLOCK_gc,  /* Reload at end of block */
			DestReloadBurst = DMA_CH_DESTRELOAD_BURST_gc,  /* Reload at end of burst */
			DestReloadTransaction = DMA_CH_DESTRELOAD_TRANSACTION_gc,  /* Reload at end of transaction */
			DestReload_bm = DestReloadNone | DestReloadBlock | DestReloadBurst | DestReloadTransaction
		};

		enum SrcDirection_enum {
		    SrcDirectionFixed = DMA_CH_SRCDIR_FIXED_gc,  /* Fixed */
		    SrcDirectionInc = DMA_CH_SRCDIR_INC_gc,  /* Increment */
		    SrcDirectionDec = DMA_CH_SRCDIR_DEC_gc,  /* Decrement */
		    SrcDirection_bm = SrcDirectionFixed | SrcDirectionInc | SrcDirectionDec
		};

		enum DestDirection_enum {
		    DestDirectionFixed = DMA_CH_DESTDIR_FIXED_gc,  /* Fixed */
		    DestDirectionInc = DMA_CH_DESTDIR_INC_gc,  /* Increment */
		    DestDirectionDec = DMA_CH_DESTDIR_DEC_gc,  /* Decrement */
		    DestDirection_bm = DestDirectionFixed | DestDirectionInc | DestDirectionDec
		};

		enum BurstLength_enum {
		    BurstLength1Byte = DMA_CH_BURSTLEN_1BYTE_gc,  /* 1-byte burst mode */
		    BurstLength2Byte = DMA_CH_BURSTLEN_2BYTE_gc,  /* 2-byte burst mode */
		    BurstLength4Byte = DMA_CH_BURSTLEN_4BYTE_gc,  /* 4-byte burst mode */
		    BurstLength8Byte = DMA_CH_BURSTLEN_8BYTE_gc,  /* 8-byte burst mode */
		    BurstLength_bm = BurstLength1Byte | BurstLength2Byte | BurstLength4Byte | BurstLength8Byte
		};

		/**
		 * Resettet den DMA Controller. Automatisch allozierte Puffer werden zurück
		 * gesetzt.
		 */
		static void reset();

		/**
		 * Setzt die Blockgröße eines Kanals auf eine bestimmte Größe. Wird der
		 * Kanal doppelt gepuffert, wird automatisch der zweite entsprechende Kanal
		 * auch aktualisiert. Beim Ändern der Größe können sich die Zeiger auf
		 * die Ausgangspuffer ändern, falls sie automatisch alloziert wurden.
		 */
		static void setBlockSize(uint8_t channel, uint16_t blockSize);

		/**
		 * Gibt an, ob ein bestimmter Kanal genutzt werden soll.
		 * Wenn use = false ist, wird der Kanal nicht mehr benutzt. Wenn ein Kanal,
		 * der zum DoubleBuffering gehört, deaktiviert wird, wird auch das
		 * DoubleBuffering deaktiviert.
		 */
		static void useChannel(uint8_t channel, uint8_t use);

		/**
		 * Definiert, dass der entsprechende Kanal genutzt werden soll.
		 */
		static inline void useChannel(uint8_t channel) {
			usedChannels |= _BV(channel);
		}

		/**
		 * Wenn channel gleich 0 oder 1 ist, wird Doublebuffering für Kanal 0 und 1 aktiviert
		 * und die Buffergröße von beiden auf die des größeren Buffers gesetzt.
		 * Für Kanal 2 und 3 gilt das selbe.
		 */
		//TODO Andere Parameter automatisch auf beide Kanäle anwenden, wenn DoubleBuffering aktiviert wird.
		static void useDoubleBuffering(uint8_t channel, bool use) {
			channel >>= 1;

			if (use) {
				DMA.CTRL |= (channel + 1) << 2;	//DMA_DBUFMODE_CH01_gc oder DMA_DBUFMODE_CH23_gc
				usedChannels |= _BV(2 * channel) | _BV(2 * channel + 1);
				doubleBuffered = channel;
			} else {
				DMA.CTRL &= ~((channel + 1) << 2);
				doubleBuffered &= ~_BV(channel);
			}
		}

		/**
		 * Setzt den Eingangspuffer eines Kanals. Der Pointer zum Buffer sollte
		 * gültig sein.
		 */
		static void setSource(uint8_t channel, void* source);

		/**
		 * Vereint das Festlegen des Quellpuffers, seiner Bewegungsrichtung nach jedem Burst
		 * und des Zeitpunkts, an dem der interne Pointer zurück gesetzt werden soll.
		 */
		static void setSource(uint8_t channel, void* source, SrcDirection_enum direction, SrcReload_enum mode);

		/**
		 * Vereint das Festlegen der beiden Quellpuffer für Doublebuffering, der Bewegungsrichtung
		 * nach jedem Burst und des Zeitpunkts, an dem der interne Pointer zurück gesetzt werden soll.
		 */
		static void setSource(uint8_t channel, void* source1, void* source2, SrcDirection_enum direction, SrcReload_enum mode);

		/**
		 * Legt fest, in welche Richtung sich der Pointer des Quellpuffers nach jedem Burst
		 * bewegen soll oder ob er fixiert bleiben soll. Wird DoubleBuffering verwendet, wird
		 * der Wert für beide beteiligten Kanäle auf einmal geändert.
		 */
		static void setSourceDirection(uint8_t channel, SrcDirection_enum direction);

		/**
		 * Legt fest, wann wieder angefangen werden soll vom Anfang des Eingangsbuffers zu lesen.
		 * Falls die SourceDirection auf Fixed gestellt ist, hat diese Einstellung keine Wirkung.
		 * Wird DoubleBuffering verwendet, wird der Wert für beide beteiligten Kanäle auf einmal
		 * geändert.
		 */
		static void setSourceReload(uint8_t channel, SrcReload_enum mode);

		/**
		 * Setzt manuell den Ausgangspuffer eines Kanals. Wird als Puffer ein
		 * Nullpointer übergeben, wird der Ausgangspuffer automatisch alloziert.
		 */
		static void* setDestination(uint8_t channel, void* destination);

		/**
		 * Vereint das Festlegen des Zielpuffers, seiner Bewegungsrichtung nach jedem Burst
		 * und des Zeitpunkts, an dem der interne Pointer zurück gesetzt werden soll. Bei
		 * DoubleBuffering wird die Zieladresse beider Kanäle gesetzt. Der Rückgabewert
		 * entspricht bei automatischer Allozierung dem erstellten Puffer für den übergebenen Kanal.
		 * Um bei DoubleBuffering den Pointer zum zweiten Kanal zu bekommen, muss getAutoBuffer()
		 * genutzt werden.
		 */
		static void* setDestination(uint8_t channel, void* destination, DestDirection_enum direction, DestReload_enum mode);

		/**
		 * Vereint das Festlegen der beiden Zielpuffer für Doublebuffering, der Bewegungsrichtung
		 * nach jedem Burst und des Zeitpunkts, an dem der interne Pointer zurück gesetzt werden soll.
		 */
		static void setDestination(uint8_t channel, void* destination1, void* destination2, DestDirection_enum direction, DestReload_enum mode);

		/**
		 * Legt fest, in welche Richtung sich der Pointer des Zielpuffers nach jedem Burst
		 * bewegen soll oder ob er fixiert bleiben soll. Wird DoubleBuffering verwendet, wird
		 * der Wert für beide beteiligten Kanäle auf einmal geändert.
		 */
		static void setDestDirection(uint8_t channel, DestDirection_enum direction);

		/**
		 * Legt fest, wann wieder angefangen werden soll vom Anfang des Zielpuffers zu lesen.
		 * Falls die DestinationDirection auf Fixed gestellt ist, hat diese Einstellung keine Wirkung.
		 * Wird DoubleBuffering verwendet, wird der Wert für beide beteiligten Kanäle auf einmal
		 * geändert.
		 */
		static void setDestReload(uint8_t channel, DestReload_enum mode);

		/**
		 * Legt fest wie viele Bytes mit einem Burst, d.h. einer Triggerung, vom Quell- in den
		 * Zielpuffer kopiert werden sollen. Gleichzeitig bestimmt diese Einstellung um wie
		 * viele Bytes sich die internen Zeiger innerhalb des Quell- bzw. Zielpuffers bewegen sollen,
		 * falls ihre Direction nicht auf Fixed gestellt ist.
		 */
		static void setBurstLength(uint8_t channel, BurstLength_enum burstLength);

		/**
		 * Aktiviert einen Kanal bzw. den ersten von zwei, wenn DoubleBuffering verwendet wird.
		 * Bei DoubleBuffering wird der zweite Kanal automatisch aktiviert, wenn eine komplette
		 * Transaktion des ersten beendet ist und umgekehrt.
		 */
		static void enable(uint8_t channel);

		/**
		 * Deaktiviert einen Kanal oder zwei, wenn DoubleBuffering verwendet wird.
		 */
		static void disable(uint8_t channel);

		/**
		 * Aktiviert den DMA Controller.
		 */
		static inline void enable() {
			DMA.CTRL |= DMA_CH_ENABLE_bm;
		}

		/**
		 * Deaktiviert den DMA Controller.
		 */
		static inline void disable() {
			DMA.CTRL &= ~DMA_CH_ENABLE_bm;
		}

		/**
		 * Gibt true zurück, wenn eine Transaktion auf einem Kanal vollendet wurde
		 * und gleichzeitig kein Blocktransfer mehr läuft.
		 */
		static bool isTransactionComplete(uint8_t channel);

		/**
		 * Gibt true zurück, wenn ein Blocktransfer auf dem übergebenen Kanal noch
		 * aktiv ist oder auf seine Ausführung wartet.
		 */
		static inline bool isBlockTransferBusy(uint8_t channel);

		/**
		 * Wartet bis das Trigger-Bit des übergebenen Kanals auf 0 gesetzt wurde
		 * und setzt es dann wieder auf 1 um den nächsten Transfer zu starten.
		 */
		static void trigger(uint8_t channel);

		/**
		 * Kann benutzt werden um einen Kanal zu finden, der gerade nicht benutzt
		 * wird. copyMemory und fillMemory machen davon ebenfalls Gebrauch. Die
		 * Variable channel wird mit der Nummer eines freien Kanals gefüllt, wenn
		 * ein Kanal frei ist und der Rückgabewert wird true sein. Andernsfalls
		 * ändert sich an channel nichts und der Rückgabewert ist false.
		 */
		static bool getFreeChannel(uint8_t& channel);

		/**
		 * Kopiert einen Speicherbereich synchron in einen anderen. Dabei wird ein
		 * aktuell nicht genutzter Kanal verwendet. Wenn der Rückgabewert der
		 * Methode true ist, wurde der Speicher erfolgreich kopiert, andernfalls
		 * war kein Kanal frei.
		 */
		static bool copyMemory(void* source, void* destination, uint16_t size);

		/**
		 * Startet einen asynchronen Kopiervorgang von einem Speicherbereich in einen
		 * anderen. Der Kopiervorgang ist beendet, wenn isTransactionComplete(channel)
		 * true zurück gibt. Danach sollte der Kanal wieder deaktiviert werden.
		 */
		static bool copyMemory(uint8_t channel, void* source, void* destination, uint16_t size);

		/**
		 * Füllt einen Speicherbereich mit einem bestimmten Wert. War das Füllen
		 * des Speichers erfolgreich, wird true zurückgegeben, andernfalls war kein
		 * freier Kanal verfügbar.
		 */
		static bool fillMemory(void* destination, uint16_t size, uint8_t byte);

		/**
		 * Legt fest wie oft der Puffer von einem Kanal (oder zwei bei DoubleBuffering)
		 * nacheinander kopiert werden soll. Wird 'repeatCount' auf 0 gesetzt, wird
		 * unendlich oft kopiert.
		 */
		static void setRepeatCount(uint8_t channel, uint8_t repeatCount);

		/**
		 * Wird SingleShot für einen Kanal (zwei bei DoubleBuffering) aktiviert,
		 * wird immer nur ein Burst pro Triggerung kopiert. Für den SingleShot-Modus
		 * bietet sich ein RepeatCount von 0 an, damit unendlich oft getriggert werden
		 * kann.
		 */
		static void useSingleShot(uint8_t channel, bool use);

		/**
		 * Setzt die Triggerquelle für einen bzw. zwei Kanäle (bei DoubleBuffering) fest.
		 * Alle Triggerquellen des verwendeten ATXMega finden sich in der entsprechenden
		 * io-Header-Datei in der Enumeration 'DMA_CH_TRIGSRC_enum'. Für den ATXMega128A1
		 * findet man sie z.B. in /usr/lib/avr/include/avr/iox128a1.h
		 */
		static void setTriggerSource(uint8_t channel, uint8_t triggerSource);

		/**
		 * Gibt den automatisch allozierten Zielpuffer eines Kanals zurück, falls diese
		 * Option genutzt wurde. Sonst 0.
		 */
		static void* getAutoBuffer(uint8_t channel) {
			return buffer[channel];
		}
};

#endif /* DMAController_H_ */
