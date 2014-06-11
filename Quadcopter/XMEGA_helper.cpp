/**
 * Copyright 2012-2013 Nicolas Göddel
 *
 * This file is part of the Quadcopter Project.
 *
 * The Quadcopter Project is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The Quadcopter Project is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Diese Datei ist Teil des Quadcopter Projekts.
 *
 * Das Quadcopter Projekt ist Freie Software: Sie können es unter den Bedingungen
 * der GNU General Public License, wie von der Free Software Foundation,
 * Version 3 der Lizenz oder (nach Ihrer Option) jeder späteren
 * veröffentlichten Version, weiterverbreiten und/oder modifizieren.
 *
 * Das Quadcopter Projekt wird in der Hoffnung, dass es nützlich sein wird, aber
 * OHNE JEDE GEWÄHELEISTUNG, bereitgestellt; sogar ohne die implizite
 * Gewährleistung der MARKTFÄHIGKEIT oder EIGNUNG FÜR EINEN BESTIMMTEN ZWECK.
 * Siehe die GNU General Public License für weitere Details.
 *
 * Sie sollten eine Kopie der GNU General Public License zusammen mit diesem
 * Programm erhalten haben. Wenn nicht, siehe <http://www.gnu.org/licenses/>.
 */

#include "XMEGA_helper.h"

void set32MHz() {
	OSC.CTRL = OSC_RC32MEN_bm; //enable 32MHz oscillator
	while(!(OSC.STATUS & OSC_RC32MRDY_bm));   //wait for stability
	CCP = CCP_IOREG_gc; //secured access
	CLK.CTRL = 0x01; //choose this osc source as clk
}

/*
 * TCC0 = 0x800		PORTC = 0x640
 * TCD0 = 0x900		PORTD = 0x660
 * TCE0 = 0xA00		PORTE = 0x680
 * TCF0 = 0xB00		PORTF = 0x6A0
 */

/**
 * Initialisiert eine 4-Kanal PWM auf einem bestimmten Timer und Port.
 * port			Definiert den Port, auf dem die PWM and Pin 0 bis 3 ausgegeben wird.
 * 				Mögliche Werte sind z.B. PORTC, PORTD, PORTE, PORTF.
 * timer		Definiert den Timer, der genutzt werden soll. Dies ist abhängig vom Port.
 * 				TCC0, TCD0, TCE0, TCF0 sind mögliche Werte.
 * frequency	Definiert die Periode der PWM mittels einer Frequenzangabe in Hz.
 * max			Definiert die maximale Auflösung für den Duty Cycle.
 * return		Die maximal zu nutzende Auflösung des Duty Cycles.
 */
uint16_t init4ChanPWM(PORT_t & port, TC0_t & timer, uint32_t frequency, uint16_t max) {
	uint32_t a = F_CPU / frequency;
	uint8_t clk_src = 1;
	while (a >= max) {
		a = (F_CPU / _BV(clk_src)) / frequency;
		clk_src++;
	}

	if (clk_src > 7) {
		return 0;
	}

	uint8_t s = SREG;
	cli();

	port.DIR |= 0x0F;

	PORTCFG.MPCMASK = 0x0f;
	port.PIN0CTRL |= PORT_INVEN_bm | PORT_OPC_TOTEM_gc;

	timer.CTRLA = (timer.CTRLA & (~TC0_CLKSEL_gm)) | clk_src;
	timer.CTRLB = (timer.CTRLB & (~TC0_WGMODE_gm)) | TC0_CCAEN_bm | TC0_CCBEN_bm | TC0_CCCEN_bm | TC0_CCDEN_bm | TC0_WGMODE0_bm | TC0_WGMODE1_bm;
	timer.CTRLD = (timer.CTRLD & (~(TC0_EVACT_gm | TC0_EVSEL_gm))) | TC_EVACT_OFF_gc | TC_EVSEL_OFF_gc;
	timer.INTCTRLA = (timer.INTCTRLA & (~(TC0_ERRINTLVL_gm | TC0_OVFINTLVL_gm))) | TC_ERRINTLVL_OFF_gc | TC_OVFINTLVL_OFF_gc;

	HIRESC.CTRLA &= ~HIRES_HREN0_bm;
#pragma optsize-
	uint8_t n = MCU.AWEXLOCK & (~MCU_AWEXCLOCK_bm);
	CCP = CCP_IOREG_gc;
	MCU.AWEXLOCK = n;
#pragma optsize_default

	AWEXC.CTRL &= ~(AWEX_PGM_bm | AWEX_CWCM_bm | AWEX_DTICCDEN_bm | AWEX_DTICCCEN_bm | AWEX_DTICCBEN_bm | AWEX_DTICCAEN_bm);
	AWEXC.FDCTRL = (AWEXC.FDCTRL & (~(AWEX_FDDBD_bm | AWEX_FDMODE_bm | AWEX_FDACT_gm))) | AWEX_FDACT_NONE_gc;
	AWEXC.FDEMASK = 0b00000000;
	AWEXC.STATUS |= AWEXC.STATUS & AWEX_FDF_bm;

	//timer.INTFLAGS = timer.INTFLAGS;
	timer.CNT = 0;
	timer.CTRLFCLR = TC0_DIR_bm;

	timer.PER = a;

	SREG = s;

	return a;
}

/**
 * Initialisiert eine 4-Kanal PWM auf einem bestimmten Timer und Port.
 * port			Definiert den Port, auf dem die PWM and Pin 0 bis 3 ausgegeben wird.
 * 				Mögliche Werte sind z.B. PORTC, PORTD, PORTE, PORTF.
 * timer		Definiert den Timer, der genutzt werden soll. Dies ist abhängig vom Port.
 * 				TCC0, TCD0, TCE0, TCF0 sind mögliche Werte.
 * frequency	Definiert die Periode der PWM mittels einer Frequenzangabe.
 * return		Die maximal zu nutzenden Auflösung des Duty Cycles.
 */
uint16_t init4ChanPWM(PORT_t & port, TC0_t & timer, uint32_t frequency) {
	return init4ChanPWM(port, timer, frequency, 65535);
}

void set4ChanPWM(TC0_t & timer, uint16_t A, uint16_t B, uint16_t C, uint16_t D) {
	timer.CCA = A;
	timer.CCB = B;
	timer.CCC = C;
	timer.CCD = D;
}

void setTimer(TC0_t & timer, PORT_t & port) {

	port.DIR |= 0x0F;
//	port.PIN0CTRL |= PORT_INVEN_bm;
//	port.PIN1CTRL |= PORT_INVEN_bm;
//	port.PIN2CTRL |= PORT_INVEN_bm;
//	port.PIN3CTRL |= PORT_INVEN_bm;

	uint8_t s = SREG;
	cli();

	//// Clock source: Peripheral Clock/1 (32 MHz / 1 = 32 MHz)
	timer.CTRLA = (timer.CTRLA & (~TC0_CLKSEL_gm)) | TC_CLKSEL_DIV1_gc;

	// Mode: PWM(SS) Operation, Overflow TOP/Event on CCA/CCB/CCC/CCD
	// Compare/Capture on channel A: On
	// Compare/Capture on channel B: On
	// Compare/Capture on channel C: On
	// Compare/Capture on channel D: On
	timer.CTRLB = (timer.CTRLB & (~(TC0_CCAEN_bm | TC0_CCBEN_bm | TC0_CCCEN_bm | TC0_CCDEN_bm | TC0_WGMODE_gm))) | TC0_CCAEN_bm | TC0_CCBEN_bm | TC0_CCCEN_bm | TC0_CCDEN_bm | TC0_WGMODE0_bm | TC0_WGMODE1_bm;

	// Capture event source: None
	// Capture event action: None
	timer.CTRLD = (timer.CTRLD & (~(TC0_EVACT_gm | TC0_EVSEL_gm))) | TC_EVACT_OFF_gc | TC_EVSEL_OFF_gc;

	// Overflow interrupt: Disabled
	// Error interrupt: Disabled
	timer.INTCTRLA = (timer.INTCTRLA & (~(TC0_ERRINTLVL_gm | TC0_OVFINTLVL_gm))) | TC_ERRINTLVL_OFF_gc | TC_OVFINTLVL_OFF_gc;

	// Compare/Capture channel A interrupt: Disabled
	// Compare/Capture channel B interrupt: Disabled
	// Compare/Capture channel C interrupt: Disabled
	// Compare/Capture channel D interrupt: Disabled
	timer.INTCTRLB = (timer.INTCTRLB & (~(TC0_CCDINTLVL_gm | TC0_CCCINTLVL_gm | TC0_CCBINTLVL_gm | TC0_CCAINTLVL_gm))) | TC_CCDINTLVL_OFF_gc | TC_CCCINTLVL_OFF_gc | TC_CCBINTLVL_OFF_gc | TC_CCAINTLVL_OFF_gc;

	// High resolution extension: Off
	HIRESC.CTRLA &= ~HIRES_HREN0_bm;

	// Advanced Waveform Extension initialization
	// Optimize for speed (scheint unnötig zu sein)
	#pragma optsize-

	// Disable locking the AWEX configuration registers just to be sure
	uint8_t n = MCU.AWEXLOCK & (~MCU_AWEXCLOCK_bm);
	CCP = CCP_IOREG_gc;
	MCU.AWEXLOCK = n;
	// Restore optimization for size if needed
	#pragma optsize_default

	// Pattern generation: Off
	// Dead time insertion: Off
	AWEXC.CTRL &= ~(AWEX_PGM_bm | AWEX_CWCM_bm | AWEX_DTICCDEN_bm | AWEX_DTICCCEN_bm | AWEX_DTICCBEN_bm | AWEX_DTICCAEN_bm);

	// Fault protection initialization
	// Fault detection on OCD Break detection: On
	// Fault detection restart mode: Latched Mode
	// Fault detection action: None (Fault protection disabled)
	AWEXC.FDCTRL = (AWEXC.FDCTRL & (~(AWEX_FDDBD_bm | AWEX_FDMODE_bm | AWEX_FDACT_gm))) | AWEX_FDACT_NONE_gc;

	// Fault detect events:
	// Event channel 0: Off
	// Event channel 1: Off
	// Event channel 2: Off
	// Event channel 3: Off
	// Event channel 4: Off
	// Event channel 5: Off
	// Event channel 6: Off
	// Event channel 7: Off
	AWEXC.FDEMASK = 0b00000000;

	// Make sure the fault detect flag is cleared
	AWEXC.STATUS |= AWEXC.STATUS & AWEX_FDF_bm;

	// Clear the interrupt flags
	//timer.INTFLAGS = timer.INTFLAGS;

	// Set counter register
	timer.CNT = 0x0000;

	// Set period register
	// In this case we have PWM frequency of 8 kHz
	timer.PER = 4000;

	// Timer counts upwards
	timer.CTRLFCLR = TC0_DIR_bm;

	timer.CCA = 0;
	timer.CCB = 0;
	timer.CCC = 0;
	timer.CCD = 0;

	SREG = s;
}

uint16_t configureInterrupt(TC1_t & timer, uint32_t frequency) {
	uint8_t prescale = 1;
	uint16_t prescaler[] = {0, 1, 2, 4, 8, 64, 256, 1024};

	uint32_t a = 65535;

	while (a >= 65535) {
		a = (F_CPU / prescaler[prescale]) / frequency;
		prescale += (a >= 65535) ? 1 : 0;
	}

	if (prescale > 7) {
		return 0;
	}

	uint8_t s = SREG;
	cli();

	timer.CTRLA = prescale; //(timer.CTRLA & (~TC1_CLKSEL_gm)) | clk_src;
	timer.CTRLB = 0; //(timer.CTRLB & (~(TC1_CCAEN_bm | TC1_CCBEN_bm | TC1_WGMODE_gm)));
	timer.CTRLC = 0;
	timer.CTRLD = 0; //(timer.CTRLD & (~(TC0_EVACT_gm | TC0_EVSEL_gm))) | TC_EVACT_OFF_gc | TC_EVSEL_OFF_gc;

	// Enable Overflow Interrupt
	timer.INTCTRLA = TC1_OVFINTLVL1_bm; //(timer.INTCTRLA & (~TC1_OVFINTLVL_gm)) | TC1_OVFINTLVL1_bm;
	timer.INTCTRLB = 0; //(timer.INTCTRLB & (~(TC1_CCAINTLVL_gm | TC1_CCBINTLVL_gm)));

	timer.PER = a;

	SREG = s;

	return a;
}
