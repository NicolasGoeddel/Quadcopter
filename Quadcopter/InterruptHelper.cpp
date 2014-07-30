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
#endif
