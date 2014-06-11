/*
 * Bluetooth.h
 *
 *  Created on: 16.01.2012
 *      Author: nicolas
 */

#ifndef BLUETOOTH_H_
#define BLUETOOTH_H_

#include <avr/io.h>
#include "USART.h"
#include <stdio.h>
#include <stdlib.h>
#include "StringDevice.h"

/**
 * Diese Klasse nutzt den USART (aktuell noch ohne Klassenstruktur)
 * um eine Kommunikation über den Bluetoothadapter EGBT-045(M)S
 * herzustellen. Auf der anderen Seite kann ein Computer mit Bluetooth
 * sein oder ein zweiter Adapter, der als Master fungiert.
 *
 * Bei der Baudrate ist wichtig zu wissen, dass dieser Adapter von
 * Werk aus die Baudrate 9600 nutzt. Das heißt beim ersten Mal sollte
 * man die Klasse mit der Baudrate 9600 initialisieren.
 */
class Bluetooth : public StringDevice<Bluetooth> {
	public:
		/**
		 * Erstelle ein Objekt der Bluetooth-Klasse mit einer bestimmten
		 * Baudrate.
		 *
		 * @param baud Die zu verwendende Baudrate.
		 */
		Bluetooth(uint32_t baud);

		~Bluetooth() {
		}

		/**
		 * Setze eine neue Baudrate fest. Dabei wird eine neue Baudrate
		 * beim Bluetooth-Adapter festgelegt und gleichzeitig die Baudrate
		 * vom USART angepasst.
		 *
		 * @param baud Die neue Baudrate.
		 * @return true, wenn alles geklappt hat, sonst false.
		 */
		bool setBaud(uint32_t baud);

		using StringDevice<Bluetooth>::write;

		/**
		 * Wenn der Bluetooth-Adapter verbunden ist, wird dadurch
		 * ein Zeichen an den Empfänger gesendet. Ist er nicht verbunden
		 * wird das Zeichen als Befehl an den Adapter selbst abgegeben.
		 *
		 * @param c Das zu sendende Zeichen.
		 */
		Bluetooth* writeChar(const char c) {
			USART_putchar(c);
			return this;
		}

		/**
		 * Liest ein Zeichen vom Bluetooth-Adapter. Kann nichts
		 * gelesen werden, wird so lange gewartet, bis etwas
		 * empfangen wurde.
		 *
		 * @return Das empfangene Zeichen.
		 */
		char getChar(void)	{
		    return USART_getchar();
		}

		/**
		 * Liest ein Zeichen vom Bluetooth-Adapter bzw. direkt aus
		 * dem Datenregister vom genutzten USART. Diese Funktion gibt
		 * keine Garantie darauf, ob tatsächlich ein neues Zeichen
		 * gelesen wurde. Wenn keine Daten verfügbar waren, dann wird
		 * immer ein Nullbyte zurück gegeben.
		 *
		 * @return Das empfangene Zeichen oder 0, falls keins vorhanden
		 *         war.
		 */
		char getCharAsync(void)	{
			if (USART_dataAvailable())
				return USART_getchar();
			return '\0';
		}

		/**
		 * Liest einen Zeilen-terminierten String vom Bluetooth-Adapter
		 * aus. Bei \r oder \n wird aufgehört zu lesen. Wenn der
		 * übergebene Puffer nicht ausreicht, wird das Lesen ebenfalls
		 * abgebrochen. Ist im Puffer noch etwas Platz, wird der gelesene
		 * String außerdem noch null-terminiert.
		 *
		 * @param buffer Der Puffer, in den der String gespeichert werden
		 *               soll.
		 * @param length Die Größe des Puffers.
		 */
		void getString(char* buffer, uint8_t length);

		/**
		 * Ändert den Namen des Bluetooth-Adapters.
		 *
		 * @param deviceName Der neue Name des Adapters als null-terminierter
		 *                   String.
		 */
		void changeDeviceName(const char* deviceName);

		/**
		 * Versucht einen einfachen Befehl an den Bluetooth-Adapter
		 * abzusetzen und überprüft den Rückgabewert. Bei erfolgreicher
		 * Kommunikation, wird true zurück gegeben.
		 *
		 * @return true, wenn der Adapter korrekt antwortet, sonst false.
		 */
		bool isDeviceOk();
};

#endif /* BLUETOOTH_H_ */
