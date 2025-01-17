/**********************************************************************
 * 
 * This is the C++ part of the USBKeyboard library.
 * See USBKeyboard.h and the example files for a full documentation.
 * 
 *********************************************************************/

#include "USBKeyboard.h"

/* set USB HID report descriptor for boot protocol keyboard */
// from: https://github.com/adafruit/Adafruit-Trinket-USB/blob/master/TrinketHidCombo/TrinketHidComboC.c
PROGMEM const uint8_t usbHidReportDescriptor[USB_CFG_HID_REPORT_DESCRIPTOR_LENGTH] = {
	0x05, 0x01,           // USAGE_PAGE (Generic Desktop)
	0x09, 0x02,           // USAGE (Mouse)
	0xa1, 0x01,           // COLLECTION (Application)
	0x09, 0x01,           //   USAGE (Pointer)
	0xA1, 0x00,           //   COLLECTION (Physical)
	0x85, REPID_MOUSE,    //     REPORT_ID
	0x05, 0x09,           //     USAGE_PAGE (Button)
	0x19, 0x01,           //     USAGE_MINIMUM
	0x29, 0x03,           //     USAGE_MAXIMUM
	0x15, 0x00,           //     LOGICAL_MINIMUM (0)
	0x25, 0x01,           //     LOGICAL_MAXIMUM (1)
	0x95, 0x03,           //     REPORT_COUNT (3)
	0x75, 0x01,           //     REPORT_SIZE (1)
	0x81, 0x02,           //     INPUT (Data,Var,Abs)
	0x95, 0x01,           //     REPORT_COUNT (1)
	0x75, 0x05,           //     REPORT_SIZE (5)
	0x81, 0x03,           //     INPUT (Const,Var,Abs)
	0x05, 0x01,           //     USAGE_PAGE (Generic Desktop)
	0x09, 0x30,           //     USAGE (X)
	0x09, 0x31,           //     USAGE (Y)
	0x15, 0x81,           //     LOGICAL_MINIMUM (-127)
	0x25, 0x7F,           //     LOGICAL_MAXIMUM (127)
	0x75, 0x08,           //     REPORT_SIZE (8)
	0x95, 0x02,           //     REPORT_COUNT (2)
	0x81, 0x06,           //     INPUT (Data,Var,Rel)
	0xC0,                 //   END_COLLECTION
	0xC0,                 // END COLLECTION

	0x05, 0x01,           // USAGE_PAGE (Generic Desktop)
	0x09, 0x06,           // USAGE (Keyboard)
	0xA1, 0x01,           // COLLECTION (Application)
	0x85, REPID_KEYBOARD, // REPORT_ID
	0x75, 0x01,           //   REPORT_SIZE (1)
	0x95, 0x08,           //   REPORT_COUNT (8)
	0x05, 0x07,           //   USAGE_PAGE (Keyboard)(Key Codes)
	0x19, 0xE0,           //   USAGE_MINIMUM (Keyboard LeftControl)(224)
	0x29, 0xE7,           //   USAGE_MAXIMUM (Keyboard Right GUI)(231)
	0x15, 0x00,           //   LOGICAL_MINIMUM (0)
	0x25, 0x01,           //   LOGICAL_MAXIMUM (1)
	0x81, 0x02,           //   INPUT (Data,Var,Abs) ; Modifier byte
	0x95, 0x01,           //   REPORT_COUNT (1)
	0x75, 0x08,           //   REPORT_SIZE (8)
	0x81, 0x03,           //   INPUT (Cnst,Var,Abs) ; Reserved byte
	0x95, 0x05,           //   REPORT_COUNT (5)
	0x75, 0x01,           //   REPORT_SIZE (1)
	0x05, 0x08,           //   USAGE_PAGE (LEDs)
	0x19, 0x01,           //   USAGE_MINIMUM (Num Lock)
	0x29, 0x05,           //   USAGE_MAXIMUM (Kana)
	0x91, 0x02,           //   OUTPUT (Data,Var,Abs) ; LED report
	0x95, 0x01,           //   REPORT_COUNT (1)
	0x75, 0x03,           //   REPORT_SIZE (3)
	0x91, 0x03,           //   OUTPUT (Cnst,Var,Abs) ; LED report padding
	0x95, 0x05,           //   REPORT_COUNT (5)
	0x75, 0x08,           //   REPORT_SIZE (8)
	0x15, 0x00,           //   LOGICAL_MINIMUM (0)
	0x26, 0xA4, 0x00,     //   LOGICAL_MAXIMUM (164)
	0x05, 0x07,           //   USAGE_PAGE (Keyboard)(Key Codes)
	0x19, 0x00,           //   USAGE_MINIMUM (Reserved (no event indicated))(0)
	0x2A, 0xA4, 0x00,     //   USAGE_MAXIMUM (Keyboard Application)(164)
	0x81, 0x00,           //   INPUT (Data,Ary,Abs)
	0xC0,                 // END_COLLECTION

	// this second multimedia key report is what handles the multimedia keys
	0x05, 0x0C,           // USAGE_PAGE (Consumer Devices)
	0x09, 0x01,           // USAGE (Consumer Control)
	0xA1, 0x01,           // COLLECTION (Application)
	0x85, REPID_MMKEY,    //   REPORT_ID
	0x19, 0x00,           //   USAGE_MINIMUM (Unassigned)
	0x2A, 0x3C, 0x02,     //   USAGE_MAXIMUM
	0x15, 0x00,           //   LOGICAL_MINIMUM (0)
	0x26, 0x3C, 0x02,     //   LOGICAL_MAXIMUM
	0x95, 0x01,           //   REPORT_COUNT (1)
	0x75, 0x10,           //   REPORT_SIZE (16)
	0x81, 0x00,           //   INPUT (Data,Ary,Abs)
	0xC0,                 // END_COLLECTION

	// system controls, like power, needs a 3rd different report and report descriptor
	0x05, 0x01,             // USAGE_PAGE (Generic Desktop)
	0x09, 0x80,             // USAGE (System Control)
	0xA1, 0x01,             // COLLECTION (Application)
	0x85, REPID_SYSCTRLKEY, //   REPORT_ID
	0x95, 0x01,             //   REPORT_COUNT (1)
	0x75, 0x02,             //   REPORT_SIZE (2)
	0x15, 0x01,             //   LOGICAL_MINIMUM (1)
	0x25, 0x03,             //   LOGICAL_MAXIMUM (3)
	0x09, 0x82,             //   USAGE (System Sleep)
	0x09, 0x81,             //   USAGE (System Power)
	0x09, 0x83,             //   USAGE (System Wakeup)
	0x81, 0x60,             //   INPUT
	0x75, 0x06,             //   REPORT_SIZE (6)
	0x81, 0x03,             //   INPUT (Cnst,Var,Abs)
	0xC0,                   // END_COLLECTION
};



/*##################################### PUBLIC FUNCTIONS #####################################*/

void init22()
{
	cli();
	USBOUT &= ~USBMASK;
	USBDDR &= ~USBMASK;
	usbDeviceDisconnect();
	delayMicroseconds(250000);
	usbDeviceConnect();
	usbInit();
	sei();
}

/* constructor */
USBKeyboard::USBKeyboard() {
	init22();
}


/* constructor */
USBKeyboard::USBKeyboard(uint8_t layout) {
	init22();
	keyboard_layout = layout;
}


/* make usbPoll() accessable from the outside */
void USBKeyboard::update() {
	usbPoll();
}


/* Type a single char to the USB host  */
size_t USBKeyboard::write(uint8_t ascii) {
	sendKey(asciiToKeycode(ascii), asciiToShiftState(ascii));
	return 1; /* write successfull */
}


/* send a single key */
void USBKeyboard::sendKey(uint8_t keycode, uint8_t modifiers) {
	/* make sure Enter/Return is send properly */
	if (keycode == 0x28)
		USBKeyboard::sendReport(0, 0x2C, 0x2A, 0x28, 0, 0, 0);
	else
		USBKeyboard::sendReport(modifiers, keycode, 0, 0, 0, 0, 0);
	/* release all keys */
	USBKeyboard::sendReport(0, 0, 0, 0, 0, 0, 0);
}


/* send up to 6 keys */
void USBKeyboard::sendKeys(uint8_t keycode1, uint8_t keycode2, uint8_t keycode3, uint8_t keycode4, uint8_t keycode5, uint8_t keycode6, uint8_t modifiers) {
	/* press keys */
	USBKeyboard::sendReport(modifiers, keycode1, keycode2, keycode3, keycode4, keycode5, keycode6);
	/* release all keys */
	USBKeyboard::sendReport(0, 0, 0, 0, 0, 0, 0);
}


/* translate ASCII char to keycode */
uint8_t USBKeyboard::asciiToKeycode(char ascii) {
	/* check if input is valid ASCII char and translate it to keycode */
	if (ascii >= ' ' && ascii <= '~')
		//return keycodes[ascii - 32][keyboard_layout];
		return concat(KEYCODES_, LAYOUT)[ascii - 32];
	
	/* translate \t and \n */
	if (ascii == '\t') return 0x2B;
	if (ascii == '\n') return 0x28;
	
	/* char unknown */
	return 0;
}


/* translate ASCII char to shift state */
uint8_t USBKeyboard::asciiToShiftState(char ascii) {
	/* check if input is valid ASCII char and translate it to keycode */
	if (ascii >= ' ' && ascii <= '~') {
		if ((concat(MODIFIER_SHIFT_, LAYOUT)[11] & 1) || (ascii >= 'a' && ascii <= 'z') || (ascii >= 'A' && ascii <= 'Z')) {
			/* set shift depending on the Caps Lock state and input char */
			return (LED_states ^ ((concat(MODIFIER_SHIFT_, LAYOUT)[ascii/8 - 4] >> (7 - ((ascii - 32) % 8))) << 1)) & 2;
		}
		else {
			/* set shift depending on the input char */
			return (concat(MODIFIER_SHIFT_, LAYOUT)[ascii/8 - 4] >> (7 - ((ascii - 32) % 8))) << 1 & 2;
		}
	}
	
	/* char unknown or \t or \n, they need no Shift compensation for Caps Lock */
	return 0;
}


bool USBKeyboard::isCapsLockActivated() {
	 return (LED_states & 2);
}


uint8_t USBKeyboard::getCapsLockToggleCount() {
	return toggle_counter;
}


void USBKeyboard::resetCapsLockToggleCount() {
	toggle_counter = 0;
}

/* send the keyoard report */
void USBKeyboard::sendMultimediaKey(uint8_t key) {
	if (!usbInterruptIsReady()) return;
	/* perform usb background tasks until the report can be sent, then send it */
	while (!usbInterruptIsReady()) usbPoll();
	uint8_t report_buffer[3] = {REPID_MMKEY, key, 0};
	usbSetInterrupt(report_buffer, sizeof(report_buffer)); /* send */
	while (!usbInterruptIsReady()) usbPoll();
	report_buffer[1] = 0;
	usbSetInterrupt(report_buffer, sizeof(report_buffer)); /* send */
	/* see http://vusb.wikidot.com/driver-api */
}

/*#################################### PRIVATE FUNCTIONS #####################################*/

/* send the keyoard report */
void USBKeyboard::sendReport(uint8_t modifiers, uint8_t keycode1, uint8_t keycode2, uint8_t keycode3, uint8_t keycode4, uint8_t keycode5, uint8_t keycode6) {
	/* perform usb background tasks until the report can be sent, then send it */
	while (!usbInterruptIsReady()) usbPoll();
	uint8_t report_buffer[8] = {modifiers, 0, keycode1, keycode2, keycode3, keycode4, keycode5, keycode6};
	usbSetInterrupt(report_buffer, sizeof(report_buffer)); /* send */
	/* see http://vusb.wikidot.com/driver-api */
}


/*##################################### V-USB FUNCTIONS ######################################*/

/* declare the USB device as HID keyboard */
usbMsgLen_t usbFunctionSetup(uint8_t data[8]) {
	/* see HID1_11.pdf sect 7.2 and http://vusb.wikidot.com/driver-api */
	usbRequest_t *rq = (usbRequest_t *)data;
	
	if ((rq->bmRequestType & USBRQ_TYPE_MASK) != USBRQ_TYPE_CLASS)
		return 0; /* ignore request if it's not a class specific request */
	
	/* see HID1_11.pdf sect 7.2 */
	switch (rq->bRequest) {
		case USBRQ_HID_GET_IDLE:
			usbMsgPtr = &idle_rate; /* send data starting from this byte */
			return 1; /* send 1 byte */
		case USBRQ_HID_SET_IDLE:
			idle_rate = rq->wValue.bytes[1]; /* read in idle rate */
			return 0; /* send nothing */
		case USBRQ_HID_GET_PROTOCOL:
			usbMsgPtr = &protocol_version; /* send data starting from this byte */
			return 1; /* send 1 byte */
		case USBRQ_HID_SET_PROTOCOL:
			protocol_version = rq->wValue.bytes[1];
			return 0; /* send nothing */
		case USBRQ_HID_SET_REPORT:
			if (rq->wLength.word == 1) { /* check data is available
				1 byte, we don't check report type (it can only be output or feature)
				we never implemented "feature" reports so it can't be feature
				so assume "output" reports
				this means set LED status
				since it's the only one in the descriptor */
				return USB_NO_MSG; /* send nothing but call usbFunctionWrite */
			}
		default: /* do not understand data, ignore */
			return 0; /* send nothing */
	}
}


/* detect LED states (Caps Lock, Num Lock, Scroll Lcok) */
usbMsgLen_t usbFunctionWrite(uint8_t* data, uchar len) {
	if (data[0] == LED_states) /* return, if no LED states have changed */
		return 1;
	if (data[0] ^ LED_states & 2) /* increase counter, if Caps Lock state changed */
		toggle_counter++;
	LED_states = data[0];
	return 1; /* Data read, not expecting more */
}
