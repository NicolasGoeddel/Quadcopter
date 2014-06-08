/*
 * rf.cpp
 *
 *  Created on: 16.04.2014
 *      Author: nicolas
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "rf.h"
#include "SPI_HW.h"

HWSPI RF_hwspi;
uint8_t RF_payload[wl_module_PAYLOAD];
void (*RF_receiveCallback)(uint8_t *);

void rf_setCallback(void (*receiveCallback)(uint8_t *)) {
	RF_receiveCallback = receiveCallback;
}

// Initialize pins for spi communication
void rfspi_init() {
	RF_hwspi.port = &RF_SPI;
	SPI_init(&RF_hwspi, &RF_PORT, 0, 1, 0, 4, 0);
}

// Shift full array through target device
void rfspi_transfer_sync(uint8_t* dataout, uint8_t* datain, uint8_t len) {
	uint8_t i;
	for (i = 0; i < len; i++) {
		datain[i] = SPI_transmit(&RF_hwspi, dataout[i]);
	}
}

// Shift full array to target device without receiving any byte
void rfspi_transmit_sync (uint8_t * dataout, uint8_t len) {
	uint8_t i;
	for (i = 0; i < len; i++) {
		SPI_send(&RF_hwspi, dataout[i]);
	}
}

// Clocks only one byte to target device and returns the received one
uint8_t rfspi_fast_shift (uint8_t data) {
	return SPI_transmit(&RF_hwspi, data);
}

/*
	Copyright (c) 2011 by Ernst Buchmann

	Code based on the work of Stefan Engelke and Brennan Ball

    Permission is hereby granted, free of charge, to any person
    obtaining a copy of this software and associated documentation
    files (the "Software"), to deal in the Software without
    restriction, including without limitation the rights to use, copy,
    modify, merge, publish, distribute, sublicense, and/or sell copies
    of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be
    included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
    MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
    DEALINGS IN THE SOFTWARE.


*/

// Defines for setting the wl_module registers for transmitting or receiving mode
#define TX_POWERUP wl_module_config_register(CONFIG, wl_module_CONFIG | ( _BV(PWR_UP) /* | _BV(PRIM_RX) */ ) )
#define RX_POWERUP wl_module_config_register(CONFIG, wl_module_CONFIG | ( _BV(PWR_UP) | _BV(PRIM_RX) ) )


// Flag which denotes transmitting mode
volatile uint8_t PTX;

void wl_module_power_reset() {
	PORTCFG.MPCMASK = _BV(RF_GND);
	RF_PORT.PIN0CTRL = PORT_OPC_WIREDAND_gc;
	RF_PORT.OUTCLR = _BV(RF_VCC);
	_delay_ms(500);
	PORTCFG.MPCMASK = _BV(RF_VCC);
	RF_PORT.PIN0CTRL = PORT_OPC_WIREDOR_gc;
	RF_PORT.OUTSET = _BV(RF_VCC);
}

/**	Initializes pins and interrupt to communicate with the wl_module
 *	Should be called in the early initializing phase at startup.
 */
void wl_module_init() {
    // Initialize spi module
    rfspi_init();

//    // Define Vcc and GND Pins
//    RF_PORT.DIRSET = _BV(RF_VCC) | _BV(RF_GND);
//    // Wire RF_VCC hard to VCC if written a 1 to it
//    PORTCFG.MPCMASK = _BV(RF_VCC);
//    RF_PORT.PIN0CTRL = PORT_OPC_WIREDOR_gc;
//    // Wire RF_GND hard to GND if written a 0 to it
//	PORTCFG.MPCMASK = _BV(RF_GND);
//	RF_PORT.PIN0CTRL = PORT_OPC_WIREDAND_gc;
//
//	wl_module_power_reset();

    RF_PORT.OUTCLR = _BV(RF_GND);

	// Define CSN and CE as Output and set them to default
	RF_PORT.DIRSET = _BV(RF_CSN) | _BV(RF_CE);
    wl_module_CE_lo;
    wl_module_CSN_hi;

    // Initialize external interrupt 0 (PIN 2)
    RF_PORT.DIRCLR = _BV(RF_IRQ);
    RF_PORT.INT0MASK = _BV(RF_IRQ);
    RF_PORT.PIN2CTRL = PORT_OPC_WIREDORPULL_gc | PORT_ISC_FALLING_gc;
    RF_PORT.INTCTRL = PORT_INT0LVL_MED_gc;
    PMIC.CTRL |= PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm;

    sei();
}

uint8_t wl_module_verify_register(uint8_t reg, uint8_t value) {
	uint8_t read;
	wl_module_read_register(reg, &read, sizeof(read));
	if (read == value) {
		return 1;
	}
	return 0;
}

/**	Sets the important registers in the wl-module and powers the module
 *	in receiving mode
 */
void wl_module_config() {
	//uint8_t s = SREG;
	//cli();
    // Set RF channel
    wl_module_config_register(RF_CH, wl_module_CH);

    //sei();

    // Set data speed & Output Power configured in wl_module.h
	wl_module_config_register(RF_SETUP, wl_module_RF_SETUP);
	// Set length of incoming payload
    wl_module_config_register(RX_PW_P0, wl_module_PAYLOAD);

    // Start receiver
    PTX = 0;			// Start in receiving mode
    RX_POWERUP;			// Power up in receiving mode
    wl_module_CE_hi;	// Listening for packets
    _delay_us(130);
}

/**
 * Verify the config that was set by wl_module_config()
 * @return 0	No error.
 *         1	RF_CH is wrong.
 *         2	RF_SETUP is wrong.
 *         3	RX_PW_P0 is wrong.
 */
uint8_t wl_module_config_verify() {
	if (!wl_module_verify_register(RF_CH, wl_module_CH))
		return 1;
	if (!wl_module_verify_register(RF_SETUP, wl_module_RF_SETUP))
		return 2;
	if (!wl_module_verify_register(RX_PW_P0, wl_module_PAYLOAD))
		return 3;

	return 0;
}

/**	Sets the important registers in the wl-module and powers the module
 *	in receiving mode
 */
extern void wl_module_rx_config() {
	uint8_t data[5];
    // Set RF channel
    wl_module_config_register(RF_CH,wl_module_CH);
	// Set data speed & Output Power configured in wl_module.h
	wl_module_config_register(RF_SETUP, wl_module_RF_SETUP);
	//Enable all RX Data-Pipes
	wl_module_config_register(EN_RXADDR, EN_RXADDR_ERX_ALL);
	//Set RX_Address Pipe 0
	data[0] = data[1] = data[2] = data[3] = data[4] = RX_ADDR_P0_B0_DEFAULT_VAL;
	wl_module_set_rx_addr(data, 5, 0);
	//Set RX_Address Pipe 1
	data[0] = data[1] = data[2] = data[3] = data[4] = RX_ADDR_P1_B0_DEFAULT_VAL;
	wl_module_set_rx_addr(data, 5, 1);
	//Set RX_Address Pipe 2-5
	data[0] = RX_ADDR_P2_DEFAULT_VAL;
	wl_module_set_rx_addr(data, 1, 2);
	data[0] = RX_ADDR_P3_DEFAULT_VAL;
	wl_module_set_rx_addr(data, 1, 3);
	data[0] = RX_ADDR_P4_DEFAULT_VAL;
	wl_module_set_rx_addr(data, 1, 4);
	data[0] = RX_ADDR_P5_DEFAULT_VAL;
	wl_module_set_rx_addr(data, 1, 5);
    // Set length of incoming payload
    wl_module_config_register(RX_PW_P0, wl_module_PAYLOAD);
	wl_module_config_register(RX_PW_P1, wl_module_PAYLOAD);
	wl_module_config_register(RX_PW_P2, wl_module_PAYLOAD);
	wl_module_config_register(RX_PW_P3, wl_module_PAYLOAD);
	wl_module_config_register(RX_PW_P4, wl_module_PAYLOAD);
	wl_module_config_register(RX_PW_P5, wl_module_PAYLOAD);

    // Start receiver
    PTX = 0;        // Start in receiving mode
    RX_POWERUP;     // Power up in receiving mode
    wl_module_CE_hi;     // Listening for pakets
    _delay_us(130);
}

/**	Sets the wl-module as one of the six sender. Define for every sender a unique Number (wl_module_TX_NR_x)
 *	when you call this Function.
 *	Each TX will get a TX-Address corresponding to the RX-Device.
 *	RX_Address_Pipe_0 must be the same as the TX-Address
 */
extern void wl_module_tx_config(uint8_t tx_nr) {
	uint8_t tx_addr[5];

    // Set RF channel
    wl_module_config_register(RF_CH, wl_module_CH);
	// Set data speed & Output Power configured in wl_module.h
	wl_module_config_register(RF_SETUP, wl_module_RF_SETUP);
	//Config the CONFIG Register (Mask IRQ, CRC, etc)
	wl_module_config_register(CONFIG, wl_module_CONFIG);
    // Set length of incoming payload
    wl_module_config_register(RX_PW_P0, wl_module_PAYLOAD);

	wl_module_config_register(SETUP_RETR,(SETUP_RETR_ARD_750 | SETUP_RETR_ARC_15));

	//set the TX address for the pipe with the same number as the iteration
	switch(tx_nr) {
		case 0: //setup TX address as default RX address for pipe 0 (E7:E7:E7:E7:E7)
			tx_addr[0] = tx_addr[1] = tx_addr[2] = tx_addr[3] = tx_addr[4] = RX_ADDR_P0_B0_DEFAULT_VAL;
			wl_module_set_TADDR(tx_addr);
			wl_module_set_RADDR(tx_addr);
			break;
		case 1: //setup TX address as default RX address for pipe 1 (C2:C2:C2:C2:C2)
			tx_addr[0] = tx_addr[1] = tx_addr[2] = tx_addr[3] = tx_addr[4] = RX_ADDR_P1_B0_DEFAULT_VAL;
			wl_module_set_TADDR(tx_addr);
			wl_module_set_RADDR(tx_addr);
			break;
		case 2: //setup TX address as default RX address for pipe 2 (C2:C2:C2:C2:C3)
			tx_addr[1] = tx_addr[2] = tx_addr[3] = tx_addr[4] = RX_ADDR_P1_B0_DEFAULT_VAL;
			tx_addr[0] = RX_ADDR_P2_DEFAULT_VAL;
			wl_module_set_TADDR(tx_addr);
			wl_module_set_RADDR(tx_addr);
			break;
		case 3: //setup TX address as default RX address for pipe 3 (C2:C2:C2:C2:C4)
			tx_addr[1] = tx_addr[2] = tx_addr[3] = tx_addr[4] = RX_ADDR_P1_B0_DEFAULT_VAL;
			tx_addr[0] = RX_ADDR_P3_DEFAULT_VAL;
			wl_module_set_TADDR(tx_addr);
			wl_module_set_RADDR(tx_addr);
			break;
		case 4: //setup TX address as default RX address for pipe 4 (C2:C2:C2:C2:C5)
			tx_addr[1] = tx_addr[2] = tx_addr[3] = tx_addr[4] = RX_ADDR_P1_B0_DEFAULT_VAL;
			tx_addr[0] = RX_ADDR_P4_DEFAULT_VAL;
			wl_module_set_TADDR(tx_addr);
			wl_module_set_RADDR(tx_addr);
			break;
		case 5: //setup TX address as default RX address for pipe 5 (C2:C2:C2:C2:C6)
			tx_addr[1] = tx_addr[2] = tx_addr[3] = tx_addr[4] = RX_ADDR_P1_B0_DEFAULT_VAL;
			tx_addr[0] = RX_ADDR_P5_DEFAULT_VAL;
			wl_module_set_TADDR(tx_addr);
			wl_module_set_RADDR(tx_addr);
			break;
	}

	PTX = 0;
	TX_POWERUP;
	/*
    // Start receiver
    PTX = 0;        // Start in receiving mode
    RX_POWERUP;     // Power up in receiving mode
    wl_module_CE_hi;     // Listening for pakets
	*/
}

/**	sets the TX address in the TX_ADDR register
 *	unsigned char * address is the actual address to be used.  It should be sized
 *	according to the tx_addr length specified to the nrf24l01.
 *	unsigned int len is the length of the address.  Its value should be specified
 *	according to the tx_addr length specified to the nrf24l01.
 */
extern void wl_module_set_tx_addr(uint8_t * address, uint8_t len) {
	wl_module_write_register(TX_ADDR, address, len);
}

/**	sets up the 24L01 as a transmitter
 *	this function takes the existing contents of the CONFIG register and simply
 *	clears the PRIM_RX bit in the CONFIG register.
 *	note: if the read value of the CONFIG register already has the PRIM_RX bit cleared, this
 *	function exits in order to not make an unecessary register write.
 */
extern void wl_module_set_as_tx() {
	unsigned char config;

	wl_module_read_register(CONFIG, &config, 1);

	if((config & CONFIG_PRIM_RX) == 0)
		return;

	config &= (~CONFIG_PRIM_RX);

	wl_module_write_register(CONFIG, &config, 1);

	wl_module_CE_lo;
}

/**	powers down the 24L01
 *	this function takes the existing contents of the CONFIG register and simply
 *	clears the PWR_UP bit in the CONFIG register.
 *	note: if the read value of the CONFIG register already has the PWR_UP bit cleared, this
 *	function exits in order to not make an unecessary register write.
 */
extern void wl_module_power_down() {
	unsigned char config;

	wl_module_read_register(CONFIG, &config, 1);

	if ((config & CONFIG_PWR_UP) == 0)
		return;

	config &= (~CONFIG_PWR_UP);

	wl_module_write_register(CONFIG, &config, 1);

	wl_module_CE_lo;
}

/**	sets the RX address in the RX_ADDR register that is offset by rxpipenum
 *	unsigned char * address is the actual address to be used.  It should be sized
 *	according to the rx_addr length that is being filled.
 *	unsigned int len is the length of the address.  Its value should be specified
 *	according to the rx_addr length specified to the nrf24l01.
 *	unsigned char rxpipenum is the pipe number (zero to five) whose address is being
 *	specified.  If an invalid address (greater than five) is supplied, the function
 *	does nothing.
 */
extern void wl_module_set_rx_addr(uint8_t * address, uint8_t len, uint8_t rxpipenum) {
	if (rxpipenum > 5)
		return;

	wl_module_write_register(RX_ADDR_P0 + rxpipenum, address, len);
}

extern void wl_module_get_rx_addr(uint8_t *data, uint8_t rxpipenum, uint8_t len) {
	if ((rxpipenum > 5))
		return;

	wl_module_read_register(RX_ADDR_P0 + rxpipenum, data, len);
}

/**	sets the RX payload width on the pipe offset by rxpipenum
 *	unsigned char payloadwidth is the length of the payload for the pipe referenced in
 *	rxpipenum.  It must be less than or equal to 32.  If an invalid payload width is
 *	specified, the function does nothing.
 *	unsigned char rxpipenum is the pipe number (zero to five) whose address is being
 *	specified.  If an invalid address (greater than five) is supplied, the function
 *	does nothing.
 */
extern void wl_module_set_rx_pw(unsigned char payloadwidth, unsigned char rxpipenum) {
	if ((rxpipenum > 5) || (payloadwidth > 32))
		return;

	wl_module_write_register(RX_PW_P0 + rxpipenum, &payloadwidth, 1);
}

/**	gets the RX payload width on the pipe offset by rxpipenum
 *	unsigned char rxpipenum is the pipe number (zero to five) whose address is being
 *	specified.  If an invalid address (greater than five) is supplied, the function
 *	does nothing.
 */
extern uint8_t wl_module_get_rx_pw(uint8_t rxpipenum) {
	unsigned char data;

	if ((rxpipenum > 5))
		return 0;

	wl_module_read_register(RX_PW_P0 + rxpipenum, &data, 1);

	return data;
}

/**	returns the current pipe in the 24L01's STATUS register
 */
extern uint8_t wl_module_get_rx_pipe() {
	return wl_module_get_rx_pipe_from_status(wl_module_get_status());
}

extern uint8_t wl_module_get_rx_pipe_from_status(uint8_t status) {
	return ((status & 0xE) >> 1);
}

/** Sets the receiving address
 */
void wl_module_set_RADDR(uint8_t * adr) {
    wl_module_CE_lo;
    wl_module_write_register(RX_ADDR_P0,adr,5);
    wl_module_CE_hi;
}

/** Sets the transmitting address
 */
void wl_module_set_TADDR(uint8_t * adr) {
    wl_module_write_register(TX_ADDR, adr,5);
}

/** Checks if data is available for reading
 */
extern uint8_t wl_module_data_ready() {
    if (PTX) return 0;
    uint8_t status;
    // Read wl_module status
    wl_module_CSN_lo;					// Pull down chip select
    status = rfspi_fast_shift(NOP);		// Read status register
    wl_module_CSN_hi;					// Pull up chip select
    return status & _BV(RX_DR);
}

/** returns true if TX_EMPTY bit in FIFO_STATUS register is set, false otherwise
 */
extern uint8_t wl_module_fifo_tx_empty() {
	uint8_t data;

	wl_module_read_register(FIFO_STATUS, &data, 1);

	return (data & FIFO_STATUS_TX_EMPTY);
}

/** returns true if RX_EMPTY bit in FIFO_STATUS register is set, false otherwise
 */
extern uint8_t wl_module_fifo_rx_empty() {
	uint8_t data;

	wl_module_read_register(FIFO_STATUS, &data, 1);

	return (data & FIFO_STATUS_RX_EMPTY);
}

/** returns the current RF channel in RF_CH register
 */
extern uint8_t wl_module_get_rf_ch() {
	uint8_t data;

	wl_module_read_register(RF_CH, &data, 1);

	return data;
}

/** returns the current RF_SETUP Register
 */
extern uint8_t wl_module_get_rf_setup() {
	uint8_t data;

	wl_module_read_register(RF_SETUP, &data, 1);

	return data;
}

/** returns the current PLOS_CNT value in OBSERVE_TX register
 */
extern uint8_t wl_module_get_plos_cnt() {
	uint8_t data;

	wl_module_read_register(OBSERVE_TX, &data, 1);

	return ((data & OBSERVE_TX_PLOS_CNT) >> 4);
}

/** returns the current ARC_CNT value in OBSERVE_TX register
 */
extern uint8_t wl_module_get_arc_cnt() {
	uint8_t data;

	wl_module_read_register(OBSERVE_TX, &data, 1);

	return (data & OBSERVE_TX_ARC_CNT);
}

/** return the value of the status register
 */
extern uint8_t wl_module_get_status() {
	return wl_module_get_one_byte(NOP);
}

extern uint8_t wl_module_get_rx_pipe_reading_status() {
	uint8_t pipe;
	pipe = wl_module_get_one_byte(NOP);
	return ((pipe & 0x0E) >> 1);
}

extern uint8_t wl_module_get_one_byte(uint8_t command) {
	uint8_t status;

	wl_module_CSN_lo;
	status = rfspi_fast_shift(command);
	wl_module_CSN_hi;

	return status;
}

/**	Reads wl_module_PAYLOAD bytes into data array
 */
extern uint8_t wl_module_get_data(uint8_t * data) {
	uint8_t status;

    wl_module_CSN_lo;									// Pull down chip select
    status = rfspi_fast_shift(R_RX_PAYLOAD);			// Send cmd to read rx payload
    rfspi_transfer_sync(data, data, wl_module_PAYLOAD);	// Read payload
    wl_module_CSN_hi;									// Pull up chip select

    wl_module_config_register(STATUS, _BV(RX_DR));		// Reset status register

    return status;
}

uint8_t wl_module_get_data() {
	return wl_module_get_data(RF_payload);
}

/** Clocks only one byte into the given wl-module register
 */
void wl_module_config_register(uint8_t reg, uint8_t value) {
    wl_module_CSN_lo;
    rfspi_fast_shift(W_REGISTER | (REGISTER_MASK & reg));
    rfspi_fast_shift(value);
    wl_module_CSN_hi;
}

/** Reads an array of bytes from the given start position in the wl-module registers.
 */
void wl_module_read_register(uint8_t reg, uint8_t * value, uint8_t len) {
    wl_module_CSN_lo;
    rfspi_fast_shift(R_REGISTER | (REGISTER_MASK & reg));
    rfspi_transfer_sync(value, value, len);
    wl_module_CSN_hi;
}

/** Writes an array of bytes into inte the wl-module registers.
 */
void wl_module_write_register(uint8_t reg, uint8_t * value, uint8_t len) {
    wl_module_CSN_lo;
    rfspi_fast_shift(W_REGISTER | (REGISTER_MASK & reg));
    rfspi_transmit_sync(value,len);
    wl_module_CSN_hi;
}

/**	Sends a data package to the default address. Be sure to send the correct
 *	amount of bytes as configured as payload on the receiver.
 */
void wl_module_send(uint8_t * value, uint8_t len) {
    while (PTX) {}                  // Wait until last paket is send

    wl_module_CE_lo;

    PTX = 1;                        // Set to transmitter mode
    TX_POWERUP;                     // Power up

    wl_module_CSN_lo;                    // Pull down chip select
    rfspi_fast_shift( FLUSH_TX );     // Write cmd to flush tx fifo
    wl_module_CSN_hi;                    // Pull up chip select

    wl_module_CSN_lo;                    // Pull down chip select
    rfspi_fast_shift( W_TX_PAYLOAD ); // Write cmd to write payload
    rfspi_transmit_sync(value,len);   // Write payload
    wl_module_CSN_hi;                    // Pull up chip select

    wl_module_CE_hi;                     // Start transmission
	_delay_us(10);						// Grünes Modul funktioniert nicht mit 10µs delay
	wl_module_CE_lo;
}


//=========================================== EXTRA AUS TUTORIAL ==================================================

ISR(PORTF_INT0_vect) {
	//PORTF.INTFLAGS = _BV(0);
	PORTH.OUTTGL = _BV(3);
    uint8_t status;

	// Read wl_module status
	wl_module_CSN_lo;                               	// Pull down chip select
	status = rfspi_fast_shift(NOP);						// Read status register
	wl_module_CSN_hi;                               	// Pull up chip select

	if (status & _BV(TX_DS)) {							// IRQ: Package has been sent
		wl_module_config_register(STATUS, _BV(TX_DS));	// Clear Interrupt Bitwl_module_send
		PTX = 0;
	}

	if (status & _BV(RX_DR)) {
		if (!(status & STATUS_RX_P_NO)) {
			uint8_t nRF_status = wl_module_get_data(RF_payload);
			if (RF_receiveCallback) {
				RF_receiveCallback(RF_payload);
			}
		} else {
			wl_module_config_register(STATUS, _BV(RX_DR));	// Clear Interrupt Bit
		}

	}

	if (status & _BV(MAX_RT)) {							// IRQ: Package has not been sent, send NOT again
		wl_module_config_register(STATUS, _BV(MAX_RT));	// Clear Interrupt Bit
		PTX = 0;
//		wl_module_CE_hi;								// Start transmission
//		_delay_us(10);
//		wl_module_CE_lo;
	}

	if (status & _BV(TX_FULL)) {						// TX_FIFO Full <-- this is not an IRQ
		wl_module_CSN_lo;                               // Pull down chip select
		rfspi_fast_shift(FLUSH_TX);						// Flush TX-FIFO
		wl_module_CSN_hi;                               // Pull up chip select
	}
}
