/*
 * Transmitter.h
 *
 *  Created on: 07.05.2013
 *      Author: nicolas
 */

#ifndef TRANSMITTER_H_
#define TRANSMITTER_H_

#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "rf.h"

#define TRANS_MAGICNUMBER 0xAA

extern void ReceiverCallback(uint8_t* payload);

class Transmitter;

extern Transmitter* transmitter;

class Transmitter {
	private:
		uint8_t values[4];
		uint8_t configStatus;
		uint8_t getsWithoutNewData;
		uint8_t lastState;

	public:
		enum ButtonName {
			btnFire = 0,
			btnTop,
			btnMiddle,
			btnBottom,
			curDown,
			curUp,
			curLeft,
			curRight
		};
		Transmitter() {
			transmitter = this;
			getsWithoutNewData = 0;
			rf_setCallback(&ReceiverCallback);

			wl_module_init();
			_delay_ms(50);
			wl_module_config();
			configStatus = wl_module_config_verify();

			//wl_module_tx_config(wl_module_TX_NR_1);

			//wl_module_rx_config();
			values[0] = values[1] = values[2] = values[3] = 0;
			lastState = 0;
		};
		~Transmitter() {};

		uint8_t getConfigStatus() {
			return configStatus;
		}

		void addPayload(uint8_t* payload) {
			values[0] = payload[0];
			values[1] = payload[1];
			values[2] = payload[2];
			uint8_t curPushed = (values[2] ^ lastState) & values[2] & 0xf0;
			values[3] |= (payload[3] | curPushed);
			getsWithoutNewData = 0;
			lastState = values[2];
		}

		void newGet() {
			if (getsWithoutNewData == 30) {
				//values[0] = values[1] = values[2] = values[3] = 0;
			} else {
				getsWithoutNewData++;
			}
		}

		/**
		 * Gibt den aktuellen Status der Fernbedienung zurück
		 *
		 * @param axisX			Bewegung auf der X-Achse.
		 * @param axisY			Bewegung auf der Y-Achse.
		 * @param buttonState	Aktueller Status der Buttons.
		 * @param buttonPushed	Wenn ein Button in der letzten Zeit gedrückt wurde,
		 *                      wird das entsprechende Bit auf 1 gesetzt. Das Bit kann
		 *                      mit wasButtonPushed() zurück gesetzt werden.
		 */
		void getState(uint8_t & axisX, uint8_t & axisY, uint8_t & buttonState, uint8_t & buttonPushed) {
			//newGet();
			axisX = values[0];
			axisY = values[1];
			buttonState = values[2];
			buttonPushed = values[3];
		}

		bool getButton(ButtonName button) {
			//newGet();
			if (values[2] & _BV(button))
				return true;
			return false;
		}

		bool wasButtonPushed(ButtonName button) {
			if (values[3] & _BV(button)) {
				values[3] &= ~(_BV(button));
				return true;
			}
			return false;
		}
};

#endif /* TRANSMITTER_H_ */
