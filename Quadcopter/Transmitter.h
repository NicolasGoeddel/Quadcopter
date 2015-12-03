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
#include "DEBUG.h"


#define TRANS_MAGICNUMBER 0xAA

extern void ReceiverCallback(uint8_t* payload);

class Transmitter;

extern Transmitter* transmitter;

class Transmitter {
	private:
		/**
		 * Wenn sich hier etwas änder, muss auch die Konstante
		 * 'wl_module_PAYLOAD' in 'rf.h' entsprechend angepasst werden.
		 */
		enum {
			PAYLOAD_X = 0,   //!< PAYLOAD_X
			PAYLOAD_Y,       //!< PAYLOAD_Y
			PAYLOAD_THROTTLE,//!< PAYLOAD_THROTTLE
			PAYLOAD_BSTATE,  //!< PAYLOAD_BSTATE
			PAYLOAD_BPUSHED, //!< PAYLOAD_BPUSHED
			PAYLOAD_LAST     //!< PAYLOAD_LAST
		};

		uint8_t values[PAYLOAD_LAST];
		/**
		 * 0	No error.
		 * 1	RF_CH is wrong.
		 * 2	RF_SETUP is wrong.
 		 * 3	RX_PW_P0 is wrong.
		 */
		uint8_t configStatus;
		uint8_t getsWithoutNewData;
		uint8_t lastState;
		enum {
			THRESHOLD_NO_DATA = 30,  // max value = 255
			THRESHOLD_REINIT = 90 // max value = 254
		};



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

			rf_setCallback(&ReceiverCallback);

			wl_module_init();
			_delay_ms(50);

			init();
		};
		~Transmitter() {};

		void init() {
			getsWithoutNewData = 0;

			wl_module_power_down();
			_delay_ms(50);

			wl_module_config();
			configStatus = wl_module_config_verify();

			//wl_module_tx_config(wl_module_TX_NR_1);

			//wl_module_rx_config();
			values[PAYLOAD_X] = values[PAYLOAD_Y] = 128;
			values[PAYLOAD_THROTTLE] = 0;
			values[PAYLOAD_BSTATE] = values[PAYLOAD_BPUSHED] = 0;
			lastState = 0;
		}

		uint8_t getConfigStatus() {
			return configStatus;
		}

		bool isConnected() {
			if (getsWithoutNewData >= THRESHOLD_NO_DATA) {
				return false;
			}
			return true;
		}

		/**
		 * Wird von ReceiverCallback aufgerufen und erhält
		 * die Daten, die die Fernbedienung gesendet hat.
		 *
		 * @param payload Die Rohdaten aus der Fernbedienung.
		 */
		void addPayload(uint8_t* payload) {
			values[PAYLOAD_X] = payload[PAYLOAD_X];
			values[PAYLOAD_Y] = payload[PAYLOAD_Y];
			values[PAYLOAD_THROTTLE] = payload[PAYLOAD_THROTTLE];
			values[PAYLOAD_BSTATE] = payload[PAYLOAD_BSTATE];
			uint8_t curPushed = (values[PAYLOAD_BSTATE] ^ lastState) & values[PAYLOAD_BSTATE] & 0xf0;
			values[PAYLOAD_BPUSHED] |= (payload[PAYLOAD_BPUSHED] | curPushed);
			getsWithoutNewData = 0;
			lastState = values[PAYLOAD_BSTATE];
			DEBUG_LED(2);
		}

		/**
		 * Sollte einmal pro Get aufgerufen werden. Wird die
		 * Funktion 30 mal aufgerufen, ohne dass bisher neue
		 * Daten empfangen wurden, werden alle Daten zurück
		 * gesetzt.
		 */
		void newGet() {
			if (getsWithoutNewData >= THRESHOLD_NO_DATA) {
				//TODO Funktioniert das hier auch richtig?
				values[PAYLOAD_X] = values[PAYLOAD_Y] = 128;
				values[PAYLOAD_THROTTLE] = 0;
				values[PAYLOAD_BSTATE] = values[PAYLOAD_BPUSHED] = 0;
			}
//			if (getsWithoutNewData == THRESHOLD_REINIT) {
//				init();
//			}
			if (getsWithoutNewData < 254) {
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
		void getState(uint8_t & axisX, uint8_t & axisY, uint8_t & throttle, uint8_t & buttonState, uint8_t & buttonPushed) {
			newGet();
			axisX = values[PAYLOAD_X];
			axisY = values[PAYLOAD_Y];
			buttonState = values[PAYLOAD_BSTATE];
			throttle = values[PAYLOAD_THROTTLE];
			buttonPushed = values[PAYLOAD_BPUSHED];
		}

		/**
		 * Gibt den Status des entsprechenden Buttons zurück.
		 *
		 * @param button Der zu überprüfende Button.
		 * @return       true, wenn der Button gerade gedrückt wird,
		 *               sonst false.
		 */
		bool getButton(ButtonName button) {
			//newGet();
			if (values[PAYLOAD_BSTATE] & _BV(button))
				return true;
			return false;
		}

		/**
		 * Gibt zurück, ob der entsprechende Button seit dem
		 * letzten Mal zumindest einmal gedrückt wurde.
		 *
		 * @param button Der zu überprüfende Button.
		 * @return       true, wenn der Button gerdrückt wurden,
		 *               sonst false.
		 */
		bool wasButtonPushed(ButtonName button) {
			if (values[PAYLOAD_BPUSHED] & _BV(button)) {
				values[PAYLOAD_BPUSHED] &= ~(_BV(button));
				return true;
			}
			return false;
		}
};

#endif /* TRANSMITTER_H_ */
