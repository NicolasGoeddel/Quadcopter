/*
 * DEBUG.h
 *
 *  Created on: 11.06.2014
 *      Author: nicolas
 */

#ifndef DEBUG_H_
#define DEBUG_H_

#include <avr/io.h>
#include <util/delay.h>

/**
 * Initialisiert Eingang für Taster und Ausgänge für LED:
 *
 * PH0 - PH1 : Taster
 * PH2 - LEDGND
 * PH3 - LED+
 */
void DEBUG_init();

/**
 * Setzt den Zustand der Debugging LED.
 *
 * @param state Neuer Zustand der LED:
 *              0 - LED aus.
 *              1 - LED an.
 *              sonst - Toggle LED.
 */
void DEBUG_LED(int8_t state);

/**
 * Lässt die LED in einem festen Intervall von 300 ms
 * mehrmals blinken.
 *
 * @param times Wie oft die LED blinken soll.
 */
void DEBUG_LED_blink(uint8_t times);

/**
 * Gibt den Zustand des Debugging Tasters zurück.
 *
 * @return Zustand des Tasters:
 *         false - Taster nicht gedrückt.
 *         true - Taster gedrückt.
 */
bool DEBUG_Button();

/**
 * Gibt den Zustand des Debugging Schalters zurück.
 *
 * @return Zustand des Schalters:
 *         false - Schalters aus.
 *         true - Schalters an.
 */
bool DEBUG_Switch();

#endif /* DEBUG_H_ */
