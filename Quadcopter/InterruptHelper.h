/*
 * InterruptHelper.h
 *
 *  Created on: 30.07.2014
 *      Author: nicolas
 */

#ifndef INTERRUPTHELPER_H_
#define INTERRUPTHELPER_H_

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>

#if (_VECTORS_SIZE / _VECTOR_SIZE > 255)
	typedef uint16_t INTERRUPT_NUM_t;
#else
	typedef uint8_t INTERRUPT_NUM_t;
#endif

class Interrupt;

// Eine Art LinkedList, falls mehrere Instanzen auf einen Interrupt reagieren sollen.
struct interruptFunc_t {
		Interrupt* interrupt;
		interruptFunc_t* next;
};

extern interruptFunc_t interruptFunctions[_VECTORS_SIZE / _VECTOR_SIZE];

class Interrupt {
	private:
		INTERRUPT_NUM_t vectorNum;

	public:
		Interrupt() {
			vectorNum = -1;
		}

		virtual ~Interrupt() {
			if (vectorNum >= 0) {
				removeFromInterrupt();
			}
		}

		void addToInterrupt(INTERRUPT_NUM_t vectorNum) {
			this->vectorNum = vectorNum;
			interruptFunc_t* e = &interruptFunctions[vectorNum];
			if (e->interrupt) {
				while (e->next) {
					e = e->next;
				}
				e->next = (interruptFunc_t*) malloc(sizeof(interruptFunc_t));
				e->next->interrupt = this;
				e->next->next = 0;
			} else {
				e->interrupt = this;
				e->next = 0;
			}
		}

		void removeFromInterrupt() {
			interruptFunc_t* e = &interruptFunctions[vectorNum];
			if (e->interrupt == this) {
				if (e->next) {
					e->interrupt = e->next->interrupt;
					interruptFunc_t* tmp = e->next->next;
					e->next = tmp;
					free(tmp);
				}
			} else {
				interruptFunc_t* previous = e;
				e = e->next;
				while (e) {
					if (e->interrupt == this) {
						previous->next = e->next;
						free(e);
						break;
					}
					previous = e;
					e = e->next;
				}
			}
		}

		virtual void interrupt() {
		}
};

#endif /* INTERRUPTHELPER_H_ */
