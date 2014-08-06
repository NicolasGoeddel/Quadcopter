/*
 * InterruptHelper.cpp
 *
 *  Created on: 30.07.2014
 *      Author: nicolas
 */

#include "InterruptHelper.h"

interruptFunc_t interruptFunctions[_VECTORS_SIZE / _VECTOR_SIZE];

#define declareISR(name)	ISR(name, ISR_BLOCK) { \
								interruptFunc_t* e = &interruptFunctions[name##_num]; \
								if (e->interrupt) { \
									e->interrupt->interrupt(); \
									while (e->next) { \
										e = e->next; \
										if (e->interrupt) { \
											e->interrupt->interrupt(); \
										} \
									} \
								} \
							} \


/*
 * Im folgenden Abschnitt einfach alle Intrrupts deklarieren, die später mal
 * genutzt werden sollen.
 */

declareISR(TCD1_OVF_vect)
declareISR(DMA_CH0_vect)
declareISR(DMA_CH1_vect)
declareISR(DMA_CH2_vect)
declareISR(DMA_CH3_vect)

// Nachfolgend ein Beispiel
#if false
class InterruptExample : Interrupt {
	InterruptExample() {
		addToInterrupt(TCD1_OVF_vect_num);
	}

	void interrupt() {
		/* Diese Methode wird ausgeführt, sobald der
		 * entsprechende Interrupt ausgeführt wird.
		 */
	}
};


/* Nachfolgend ein Beispiel wie man innerhalb einer Klasse
 * auf unterschiedliche Interrupts reagieren kann.
 */
class MyClass {
		class Interrupt1 : public Interrupt {
				MyClass* parent;

				Interrupt1(MyClass* _parent) : parent(_parent) {
					addToInterrupt(TCD1_OVF_vect_num);
				}

				void interrupt() {
					parent->interrupt1();
				}
		};

		class Interrupt2 : public Interrupt {
				MyClass* parent;

				Interrupt2(MyClass* _parent) : parent(_parent) {
					addToInterrupt(TCD1_ERR_vect_num);
				}

				void interrupt() {
					parent->interrupt2();
				}
		};

		Interrupt1* int1;
		Interrupt2* int2;

		MyClass() {
			int1 = new Interrupt1(this);
			int2 = new Interrupt2(this);
		}

		void interrupt1() {
			// Wird aufgerufen bei TCD1_OVF_vect
		}
		void interrupt2() {
			// Wird aufgerufen bei TCD1_ERR_vect
		}
};
#endif
