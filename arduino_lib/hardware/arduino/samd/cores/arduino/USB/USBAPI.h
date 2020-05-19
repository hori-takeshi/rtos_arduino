/*
  Copyright (c) 2015 Arduino LLC.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
/*
**Modified 04/04/2016 by Arduino.org development team
*/

#pragma once

#define WEAK __attribute__ ((weak))

#define HSTPIPCFG_PTYPE_BLK 1
#define HSTPIPCFG_PTOKEN_IN 2
#define HSTPIPCFG_PTOKEN_OUT 3
#define HSTPIPCFG_PBK_1_BANK 4
#define HSTPIPCFG_PTYPE_INTRPT 5

#define EP0      0
#define EPX_SIZE 64 // 64 for Full Speed, EPT size max is 1024

#if defined __cplusplus

#include "Stream.h"
#include "RingBuffer.h"

//================================================================================
// USB
//================================================================================
//================================================================================
// Low level API
typedef struct {
	union {
		uint8_t bmRequestType;
		struct {
			uint8_t direction : 5;
			uint8_t type : 2;
			uint8_t transferDirection : 1;
		};
	};
	uint8_t bRequest;
	uint8_t wValueL;
	uint8_t wValueH;
	uint16_t wIndex;
	uint16_t wLength;
} Setup;


class USBDevice_ {
public:
	USBDevice_() {};

	// USB Device API
	void init();
	bool attach();	// Serial port goes down too...
	bool detach();

	bool configured();

private:
	bool initialized;
};

extern USBDevice_ USBDevice;

//================================================================================
//	Serial over CDC (Serial1 is the physical port)

class Serial_ : public Stream
{
public:
	Serial_(USBDevice_ &_usb) : usb(_usb) { }
	void begin(uint32_t baud_count);
	void begin(unsigned long, uint8_t);
	void end(void);

	virtual int available(void);
	virtual void accept(void);
	virtual int peek(void);
	virtual int read(void);
	virtual void flush(void);
	virtual size_t write(uint8_t);
	virtual size_t write(const uint8_t *buffer, size_t size);
	using Print::write; // pull in write(str) from Print
	operator bool();

	// This method allows processing "SEND_BREAK" requests sent by
	// the USB host. Those requests indicate that the host wants to
	// send a BREAK signal and are accompanied by a single uint16_t
	// value, specifying the duration of the break. The value 0
	// means to end any current break, while the value 0xffff means
	// to start an indefinite break.
	// readBreak() will return the value of the most recent break
	// request, but will return it at most once, returning -1 when
	// readBreak() is called again (until another break request is
	// received, which is again returned once).
	// This also mean that if two break requests are received
	// without readBreak() being called in between, the value of the
	// first request is lost.
	// Note that the value returned is a long, so it can return
	// 0-0xffff as well as -1.
	int32_t readBreak();

	// These return the settings specified by the USB host for the
	// serial port. These aren't really used, but are offered here
	// in case a sketch wants to act on these settings.
	uint32_t baud();
	uint8_t stopbits();
	uint8_t paritytype();
	uint8_t numbits();
	bool dtr();
	bool rts();
	enum {
		ONE_STOP_BIT = 0,
		ONE_AND_HALF_STOP_BIT = 1,
		TWO_STOP_BITS = 2,
	};
	enum {
		NO_PARITY = 0,
		ODD_PARITY = 1,
		EVEN_PARITY = 2,
		MARK_PARITY = 3,
		SPACE_PARITY = 4,
	};

private:
	USBDevice_ &usb;
	RingBuffer *_cdc_rx_buffer;
};
extern Serial_ SerialUSB;

//================================================================================
//================================================================================
//	Mouse

#define MOUSE_LEFT 1
#define MOUSE_RIGHT 2
#define MOUSE_MIDDLE 4
#define MOUSE_ALL (MOUSE_LEFT | MOUSE_RIGHT | MOUSE_MIDDLE)

class Mouse_
{
private:
	uint8_t _buttons;
	void buttons(uint8_t b);
public:
	Mouse_(void);
	void begin(void);
	void end(void);
	void click(uint8_t b = MOUSE_LEFT);
	void move(signed char x, signed char y, signed char wheel = 0);
	void press(uint8_t b = MOUSE_LEFT);		// press LEFT by default
	void release(uint8_t b = MOUSE_LEFT);	// release LEFT by default
	bool isPressed(uint8_t b = MOUSE_ALL);	// check all buttons by default
};
extern Mouse_ Mouse;

//================================================================================
//================================================================================
//	Keyboard

#define KEY_LEFT_CTRL		0x80
#define KEY_LEFT_SHIFT		0x81
#define KEY_LEFT_ALT		0x82
#define KEY_LEFT_GUI		0x83
#define KEY_RIGHT_CTRL		0x84
#define KEY_RIGHT_SHIFT		0x85
#define KEY_RIGHT_ALT		0x86
#define KEY_RIGHT_GUI		0x87

#define KEY_UP_ARROW		0xDA
#define KEY_DOWN_ARROW		0xD9
#define KEY_LEFT_ARROW		0xD8
#define KEY_RIGHT_ARROW		0xD7
#define KEY_BACKSPACE		0xB2
#define KEY_TAB				0xB3
#define KEY_RETURN			0xB0
#define KEY_ESC				0xB1
#define KEY_INSERT			0xD1
#define KEY_DELETE			0xD4
#define KEY_PAGE_UP			0xD3
#define KEY_PAGE_DOWN		0xD6
#define KEY_HOME			0xD2
#define KEY_END				0xD5
#define KEY_CAPS_LOCK		0xC1
#define KEY_F1				0xC2
#define KEY_F2				0xC3
#define KEY_F3				0xC4
#define KEY_F4				0xC5
#define KEY_F5				0xC6
#define KEY_F6				0xC7
#define KEY_F7				0xC8
#define KEY_F8				0xC9
#define KEY_F9				0xCA
#define KEY_F10				0xCB
#define KEY_F11				0xCC
#define KEY_F12				0xCD

//	Low level key report: up to 6 keys and shift, ctrl etc at once
typedef struct
{
	uint8_t modifiers;
	uint8_t reserved;
	uint8_t keys[6];
} KeyReport;

class Keyboard_ : public Print
{
private:
	KeyReport _keyReport;
	void sendReport(KeyReport* keys);
public:
	Keyboard_(void);
	void begin(void);
	void end(void);
	virtual size_t write(uint8_t k);
	virtual size_t press(uint8_t k);
	virtual size_t release(uint8_t k);
	virtual void releaseAll(void);
};
extern Keyboard_ Keyboard;

//================================================================================
//================================================================================
//	HID 'Driver'

const void* WEAK HID_GetInterface(void);
uint32_t WEAK HID_GetInterfaceLength(void);
uint32_t HID_SizeReportDescriptor(void);

uint32_t		HID_GetDescriptor(void);
bool	HID_Setup(Setup& setup);
void	HID_SendReport(uint8_t id, const void* data, uint32_t len);

//================================================================================
//================================================================================
//	MSC 'Driver'

uint32_t		MSC_GetInterface(uint8_t* interfaceNum);
uint32_t		MSC_GetDescriptor(uint32_t i);
bool	MSC_Setup(Setup& pSetup);
bool	MSC_Data(uint8_t rx,uint8_t tx);

//================================================================================
//================================================================================
//	CDC 'Driver'

int CDC_GetInterface(uint8_t* interfaceNum);
const void* _CDC_GetInterface(void);
uint32_t _CDC_GetInterfaceLength(void);
uint32_t		CDC_GetOtherInterface(uint8_t* interfaceNum);
uint32_t		CDC_GetDescriptor(uint32_t i);
bool	CDC_Setup(Setup& pSetup);


//================================================================================
//================================================================================

uint32_t USBD_SendControl(const void* _data, uint32_t len);
uint32_t USBD_RecvControl(void* d, uint32_t len);
void USBD_Calibrate();
uint32_t USBD_SendInterfaces(void);
bool USBD_ClassInterfaceRequest(Setup& setup);

uint32_t USBD_Available(uint32_t ep);
uint32_t USBD_SendSpace(uint32_t ep);
uint32_t USBD_Send(uint32_t ep, const void* d, uint32_t len);
uint32_t USBD_Recv(uint32_t ep, void* data, uint32_t len);		// non-blocking
uint32_t USBD_Recv(uint32_t ep);							// non-blocking
uint8_t USBD_armRecv(uint32_t ep);
void USBD_Flush(uint32_t ep);
uint32_t USBD_Connected(void);

#endif  // __cplusplus
