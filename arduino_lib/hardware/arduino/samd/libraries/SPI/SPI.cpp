/*
 * Copyright (c) 2010 by Cristian Maglie <c.maglie@bug.st>
 * SPI Master library for arduino.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

#include "SPI.h"
#include "wiring_digital.h"
#include "assert.h"
#include "variant.h"
#include <Arduino.h>

SPIClass::SPIClass(SERCOM *p_sercom, uint8_t uc_pinMISO, uint8_t uc_pinSCK, uint8_t uc_pinMOSI)
{
	assert(p_sercom != NULL );
	_p_sercom = p_sercom;

  _uc_pinMiso = uc_pinMISO;
  _uc_pinSCK = uc_pinSCK;
  _uc_pinMosi = uc_pinMOSI;
}

void SPIClass::begin()
{
  // PIO init
  pinPeripheral(_uc_pinMiso, g_APinDescription[_uc_pinMiso].ulPinType);
  pinPeripheral(_uc_pinSCK, g_APinDescription[_uc_pinSCK].ulPinType);
  pinPeripheral(_uc_pinMosi, g_APinDescription[_uc_pinMosi].ulPinType);

	// Default speed set to 4Mhz, SPI mode set to MODE 0 and Bit order set to MSB first.
	_p_sercom->initSPI(SPI_PAD_2_SCK_3, SERCOM_RX_PAD_0, SPI_CHAR_SIZE_8_BITS, MSB_FIRST);
	_p_sercom->initSPIClock(SERCOM_SPI_MODE_0, 4000000);
	
	_p_sercom->enableSPI();
}


void SPIClass::beginSlave()
{
  // PIO init
  pinPeripheral(_uc_pinMiso, g_APinDescription[_uc_pinMiso].ulPinType);
  pinPeripheral(_uc_pinSCK, g_APinDescription[_uc_pinSCK].ulPinType);
  pinPeripheral(_uc_pinMosi, g_APinDescription[_uc_pinMosi].ulPinType);

	// Default speed set to 4Mhz, SPI mode set to MODE 0 and Bit order set to MSB first.
	_p_sercom->initSPIslave(SPI_PAD_2_SCK_3, SERCOM_RX_PAD_0, SPI_CHAR_SIZE_8_BITS, MSB_FIRST);
	
	_p_sercom->enableSPI();
}

void SPIClass::end()
{
	_p_sercom->resetSPI();
}


void SPIClass::beginTransaction(SPISettings settings)
	{
		SercomDataOrder bitOrder;
		SercomSpiClockMode SpiClockMode;
		
		interruptSave = intStatus();
		noInterrupts();
		
	
		if(settings.bit_order==LSBFIRST) bitOrder = LSB_FIRST;
		else if(settings.bit_order==MSBFIRST) bitOrder = MSB_FIRST;
		else;
		
		if(settings.data_mode==SPI_MODE0) SpiClockMode = SERCOM_SPI_MODE_0;
		else if(settings.data_mode==SPI_MODE1) SpiClockMode = SERCOM_SPI_MODE_1;
		else if(settings.data_mode==SPI_MODE2) SpiClockMode = SERCOM_SPI_MODE_2;
		else if(settings.data_mode==SPI_MODE3) SpiClockMode = SERCOM_SPI_MODE_3;
		else;
		
		end();
		
		pinPeripheral(_uc_pinMiso, g_APinDescription[_uc_pinMiso].ulPinType);
		pinPeripheral(_uc_pinSCK, g_APinDescription[_uc_pinSCK].ulPinType);
		pinPeripheral(_uc_pinMosi, g_APinDescription[_uc_pinMosi].ulPinType);
		
		_p_sercom->initSPI(SPI_PAD_2_SCK_3, SERCOM_RX_PAD_0, SPI_CHAR_SIZE_8_BITS, bitOrder);
		_p_sercom->initSPIClock(SpiClockMode, settings.interface_clock);
	
	_p_sercom->enableSPI();
	inTransactionFlag=1;
	}
	
void SPIClass::endTransaction(void)
{
	inTransactionFlag=0;
	interrupts();
}	

void SPIClass::setBitOrder(BitOrder order)
{
	if(order == LSBFIRST)
		_p_sercom->setDataOrderSPI(LSB_FIRST);
	else
		_p_sercom->setDataOrderSPI(MSB_FIRST);
}

void SPIClass::setDataMode(uint8_t mode)
{
	switch(mode)
	{
		case SPI_MODE0:
			_p_sercom->setClockModeSPI(SERCOM_SPI_MODE_0);
			break;
			
		case SPI_MODE1:
			_p_sercom->setClockModeSPI(SERCOM_SPI_MODE_1);
			break;
			
		case SPI_MODE2:
			_p_sercom->setClockModeSPI(SERCOM_SPI_MODE_2);
			break;
			
		case SPI_MODE3:
			_p_sercom->setClockModeSPI(SERCOM_SPI_MODE_3);
			break;
		
		default:
			break;
	}
}

void SPIClass::setClockDivider(uint16_t div)
{
	_p_sercom->setBaudrateSPI(div);
}

byte SPIClass::transfer(uint8_t data)
{
	//Writing the data
	_p_sercom->writeDataSPI(data);
	
	//Read data
	return _p_sercom->readDataSPI();
}

byte SPIClass::read()
{
	return _p_sercom->readDataSPI();
}

void SPIClass::usingInterrupt(uint8_t intNum)
{
	uint8_t irestore;
	uint32_t mask;
	
	irestore=intStatus();
	noInterrupts();
	if(intNum > 13)
	{
		//Interrupts();
		return;
	}
	else
	{
		//Pio *pio=g_APinDescription[intNum].ulPort;
		mask=g_APinDescription[intNum].ulPin;
		interruptMode=1;
		interruptMask=mask;
	}
	if (irestore) interrupts();		
}

void SPIClass::write(uint8_t data)
{
	//Writing the data
	_p_sercom->writeDataSPI(data);
}

void SPIClass::transfer(void *data, size_t count)
{
	uint8_t *p;
	//int i=0;
		//Serial.println("dentro transfer");
	if(count==0) return;
	p=(uint8_t *)data;
	_p_sercom->writeDataSPI(*p);  //scrittura primo dato
	
	while(--count > 0)
	{
		//Serial.println("dentro while");
		uint8_t out=*(p+1); //copio in out il prossimo dato
		uint8_t in = _p_sercom->readDataSPI();  //leggo dato da SPI
		//Serial.println("UNO");
		//while(!(SERCOM4->SPI.INTFLAG.bit.RXC));  // controllo trasferimento  //vedere se necessario
		//Serial.println("DUE");
		//Serial.println(out);
		_p_sercom->writeDataSPI(out);   //scrivo il out su SPI
		*p++=in; //metto in p il dato letto da spi
	}
	while(!(SERCOM4->SPI.INTFLAG.bit.TXC));  // controllo trasferimrnto
	//Serial.print("*p: ");
	//Serial.println(*p);
	*p = _p_sercom->readDataSPI();
	
	
		
}

uint16_t SPIClass::transfer16(uint16_t data)
{
	union{
		uint16_t value;
		struct{
			uint8_t lsb;
			uint8_t msb;
		};
	}in, out;
	
	in.value = data;
	
	if(SERCOM4->SPI.CTRLA.bit.DORD==0)
	{
		_p_sercom->writeDataSPI(in.msb);
		while(!(SERCOM4->SPI.INTFLAG.bit.TXC));
		out.msb = _p_sercom->readDataSPI();
		//while(!(SERCOM4->SPI.INTFLAG.bit.RXC)); 
		_p_sercom->writeDataSPI(in.lsb);
		while(!(SERCOM4->SPI.INTFLAG.bit.TXC));
		out.lsb = _p_sercom->readDataSPI();
	}

	else
	{
		_p_sercom->writeDataSPI(in.lsb);
		while(!(SERCOM4->SPI.INTFLAG.bit.TXC));
		out.lsb = _p_sercom->readDataSPI();
		//while(!(SERCOM4->SPI.INTFLAG.bit.RXC)); 
		_p_sercom->writeDataSPI(in.msb);
		while(!(SERCOM4->SPI.INTFLAG.bit.TXC));
		out.msb = _p_sercom->readDataSPI();
	}

		return out.value;
}

void SPIClass::attachInterrupt() {
	// Should be enableInterrupt()
}

void SPIClass::detachInterrupt() {
	// Should be disableInterrupt()
}

SPIClass SPI(&sercom4, 18, 20, 21);