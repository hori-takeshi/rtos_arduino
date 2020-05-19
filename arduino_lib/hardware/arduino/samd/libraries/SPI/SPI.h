/*
 * Copyright (c) 2010 by Cristian Maglie <c.maglie@bug.st>
 * SPI Master library for arduino.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

#ifndef _SPI_H_INCLUDED
#define _SPI_H_INCLUDED

#include "variant.h"
#include "wiring_constants.h"
#include <Arduino.h>
 
//#define LSBFIRST 0 //already defined in wiring_constants.h
//#define MSBFIRST 1 

#define SPI_MODE0 0x02
#define SPI_MODE1 0x00
#define SPI_MODE2 0x03
#define SPI_MODE3 0x01


#define SPI_CLOCK_DIV2 6       //8 MHz
#define SPI_CLOCK_DIV4 12      //4 MHz
#define SPI_CLOCK_DIV8 24      //2 MHz
#define SPI_CLOCK_DIV16 48     //1 MHz
#define SPI_CLOCK_DIV32 96     //500 KHz
#define SPI_CLOCK_DIV64 192    //250 KHz
#define SPI_CLOCK_DIV128 384   //125 KHz

#ifndef intStatus
#define intStatus() __intStatus()
static inline unsigned char __intStatus(void) __attribute__((always_inline, unused));
static inline unsigned char __intStatus(void) {
	unsigned int primask, faultmask;
	asm volatile ("mrs %0, primask" : "=r" (primask));
	if (primask) return 0;
	asm volatile ("mrs %0, faultmask" : "=r" (faultmask));
	if (faultmask) return 0;
	return 1;
}
#endif

class SPISettings {
	public:
		SPISettings(uint32_t clock, uint8_t bitOrder, uint8_t dataMode)
		{
			interface_clock = clock;
			bit_order = bitOrder;
			data_mode = dataMode;
			
			/*Serial.print("clock: ");
			Serial.println(interface_clock);
			Serial.print("bit order: ");
			Serial.println(bit_order);
			Serial.print("data mode: ");
			Serial.println(data_mode);*/
		}
		
		SPISettings(void)
		{
			interface_clock = 4000000;
			bit_order = MSBFIRST;
			data_mode = SPI_MODE0;
			
			/*Serial.print("clock: ");
			Serial.println(interface_clock);
			Serial.print("bit order: ");
			Serial.println(bit_order);
			Serial.print("data mode: ");
			Serial.println(data_mode);*/
		}
	private:
		uint32_t interface_clock;
		uint8_t bit_order;
		uint8_t data_mode;
	
	friend class SPIClass;	
};


class SPIClass {
  public:
	SPIClass(SERCOM *p_sercom, uint8_t uc_pinMISO, uint8_t uc_pinSCK, uint8_t uc_pinMOSI);

	byte transfer(uint8_t data);
	void transfer(void *data, size_t count);
	byte read(void);
	uint16_t transfer16(uint16_t data);
	void write(uint8_t data);
	// SPI Configuration methods
	void beginTransaction(SPISettings settings);
	void endTransaction(void);
	
	void attachInterrupt();
	void detachInterrupt();
	
	void usingInterrupt(uint8_t interruptNumber);

	void begin();
	void beginSlave();
	void end();
	
	void setBitOrder(BitOrder order);
	void setDataMode(uint8_t uc_mode);
	void setClockDivider(uint16_t uc_div);

  private:
	SERCOM *_p_sercom;
	uint8_t _uc_pinMiso;
	uint8_t _uc_pinMosi;
	uint8_t _uc_pinSCK;
	uint8_t interruptMode;
	uint32_t interruptMask;
	uint8_t interruptSave;
	uint8_t inTransactionFlag;
};

#if SPI_INTERFACES_COUNT > 0
  extern SPIClass SPI;
#endif

#endif