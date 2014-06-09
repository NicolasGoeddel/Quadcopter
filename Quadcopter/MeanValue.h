/*
 * MeanValue.h
 *
 *  Created on: 06.06.2014
 *      Author: nicolas
 */

#ifndef MEANVALUE_H_
#define MEANVALUE_H_

#include <avr/io.h>

template <class ValueType, class SumType, class CapacityType = uint8_t>
class MeanValue {
	private:
		/**
		 * Anzahl der Werte, über die gemittelt werden soll.
		 */
		CapacityType capacity;

		/**
		 * Zuletzt ersetztes Element im Ringpuffer
		 */
		CapacityType index;

		/**
		 * Gibt an wie viele Elemte tatsächlich im Ringpuffer sind.
		 * Gerade am Anfang können das weniger sein als capacity
		 * angibt.
		 */
		CapacityType filled;

		/**
		 * Summe aller Werte im Ringpuffer.
		 */
		SumType sum;

		ValueType* buffer;
	public:
		MeanValue(CapacityType capacity) {
			this->capacity = capacity;
			index = 0;
			filled = 0;
			sum = 0;
			buffer = (ValueType*) malloc(capacity * sizeof(ValueType));
		}
		~MeanValue() {
			free(buffer);
		}

		/**
		 * Setzt die neue Größe für den Ringpuffer fest.
		 * Hier wird explizit auf realloc() verzichtet, da dabei zeitweise
		 * zu viel Speicher allokiert werden müsste.
		 * @param capacity	Die neue Kapazität.
		 * @return			0, wenn es geklappt hat,
		 * 					1, wenn es fehlgeschlagen ist,
		 * 					2, falls jetzt auch der alte Puffer nicht mehr existiert.
		 */
		uint8_t setCapacity(CapacityType capacity) {
			reset();
			free(buffer);
			buffer = (ValueType*) malloc(capacity * sizeof(ValueType));
			if (buffer == 0) {
				buffer = (ValueType*) malloc(this->capacity * sizeof(ValueType));
				return (buffer != 0) ? 1 : 2;
			}
			this->capacity = capacity;
			return 0;
		}

		void reset() {
			index = 0;
			filled = 0;
			sum = 0;
		}

		ValueType operator()(ValueType newValue) {
			if (filled < capacity) {
				filled++;
			} else {
				sum -= buffer[index];
			}

			sum += newValue;
			buffer[index] = newValue;
			index++;
			if (index == capacity)
				index = 0;

			return sum / filled;
		}
};

#endif /* MEANVALUE_H_ */
