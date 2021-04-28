// Work Motherfucker PLEASE!!
#include <avr/io.h>
#include <avr/interrupt.h>
#include <uart.h>
#include <usart.h>
#include <stdio.h>

#define CLK_FREQ		16000000	// Define CPU clock frequency

#define BAUDRATE 57600		// Define baud rate for the radio transmitter
#define UBRRVAL ((CLK_FREQ/(BAUDRATE*16UL))-1)		// Calculate the baud rate setting
#define SYNC 0XFF		// SYNC byte for the packet
#define ADDR 0xAA		// Address byte for the packet

// Button Debouncing Parameters

#define timeout 30		// Timeout value for the state machine

// States for the state machine

#define NoPush 1 		// Wait for the button to be pressed
#define MaybePush 2		// Debounce the button press
#define Pushed 3		// Wait for the button to be released
#define MaybeNoPush 4	// Debounce the button release

// Button press/release parameters for sending to the base station

#define left_click	0x80
#define middle_click	0x40
#define right_click	0x20
#define rapid_fire	0x08
#define left_click_release	0x01
#define middle_click_release	0x02
#define right_click_release	0x04

unsigned char msg_C7;
unsigned char msg_C6;
unsigned char msg_C5;
unsigned char msg_C7_release;
unsigned char msg_C6_release;
unsigned char msg_C5_release;

// Debouncing parameters

volatile unsigned char tasktime;	// Timeout counter
unsigned char PushState;			// State machine for click buttons
unsigned char PushState_switch;		// State machine for move enable/disable button
unsigned char button_dbouncePressed;		// Stores which buttons were pressed for debounce
unsigned char currentButton;		// Which buttons are just pressed //the three task subroutines
unsigned char currentSwitch;		// Which switches are toggled
unsigned char switchToggled;		// Stores which switches were toggled for debounce
void task1(void);

// Mouse control parameters

volatile unsigned char xAxis;		// x-Axis of the mouse
volatile unsigned char yAxis;		// y-Axis of the mouse
volatile unsigned char wheel;		// Scroll position
volatile unsigned char click;		// Information for mouse button clicks

// Acceleration parameters

#define accel_timeout 2			// Timeout value for the state machine
char move_en;

volatile unsigned char aX;		// Register for ADC sample of x-axis tilt
volatile unsigned char aY;		// Register for ADC sample of y-axis tilt
volatile unsigned char aZ;		// Register for ADC sample of z-axis tilt

volatile signed long aX_ref;	// x-axis offset for calibration
volatile signed long aY_ref;	// y-axis offset for calibration
volatile signed long aZ_ref;	// z-axis offset for calibration

volatile signed int accel_x;	// Calibrated value of x-axis tilt
volatile signed int accel_y;	// Calibrated value of y-axis tilt

volatile signed int accel_x_prev;		// Store previous acceleration for cursor acceleration
volatile signed int accel_y_prev;		// Store previous acceleration for cursor acceleration

volatile signed int scale_x;	// Scale parameter for cursor acceleration
volatile signed int scale_y;	// Scale parameter for cursor acceleration

volatile signed int move_x;		// x-axis tilt to transmit
volatile signed int move_y;		// y-axis tilt to transmit

#define thresh 3
volatile unsigned char acceltime;

// Flag for scroll enable
char scroll_en;

// Time count for scrolling
volatile unsigned char scrolltime;

// Flag for enabling rapid fire
char rapid_fire_en;

// Flag for transmitting rapid fire
char send_rapid_fire;

// Flag for enabling invert_x
signed int invert_x;

// Flag for enabling invert_y
signed int invert_y;

// Mode for axis frame
char accel_mode;

// Flag for sensitivity of the cursor movement
unsigned char sensitivity;

// UART file descriptor
FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

// Transmit a byte to the transciever
void USART_txByte(unsigned char data)
{
	while (!(UCSR1A & (1 << UDRE1)));		// Wait until UDR is empty
	UDR1 = data;												// Load data into UDR
}

void tx_packet( uint8_t x, uint8_t y, uint8_t wh, uint8_t cl )
{
	USART_txByte(SYNC); // Send SYNC byte
	USART_txByte(ADDR); // Send address to recognize correct receiver
	USART_txByte(x); 	// Send x-axis of the mouse
	USART_txByte(y); 	// Send x-axis of the mouse
	USART_txByte(wh); 	// Send Scroll position
	USART_txByte(cl); 	// Send information for mouse button clicks
}

// Read the accelerometer data
void readAccel(void)
{
	if (accel_mode == 1)
	{
		// Reading z component (as x-axis)
		ADMUX = (1<<ADLAR)|(1<<REFS0);		// Select ADC channel0 (z component)
		ADCSRA |= (1 << ADSC);				// Enable ADC
		while (ADCSRA & (1 << ADSC));		// Wait until the ADC Data register becomes full
		aX = ADCH;							// Read high byte of the ADC Data Register

		// Reading y component (as y-axis)
		ADMUX = (1<<ADLAR)|(1<<REFS0)+1;	// Select ADC channel1 (y component)
		ADCSRA |= (1 << ADSC);				// Enable ADC
		while (ADCSRA & (1 << ADSC));		// Wait until ADC Data register is full
		aY = ADCH;							// Read high byte of ADC Data Register
    }
	else
	{
		// Reading y component (as y-axis)
		ADMUX = (1<<ADLAR)|(1<<REFS0)+1;	// Select ADC channel1 (y component)
		ADCSRA |= (1 << ADSC);				// Enable ADC
		while (ADCSRA & (1 << ADSC));		// Wait until ADC Data register is full
		aY = ADCH;							// Read high byte of ADC Data Register

		// Reading x component (as x-axis)
		ADMUX = (1<<ADLAR)|(1<<REFS0)+2;	// Select ADC channel2 (x component)
		ADCSRA |= (1 << ADSC);				// Enable ADC
		while (ADCSRA & (1 << ADSC));		// Wait until ADC Data register is full
		aX = ADCH;							// Read high byte of ADC Data Register
	}
}

/* Callibration:
 *		Read and average 1024 ADC samples to compute the tilt offset. */

void calibrate(void)
{
  unsigned int cnt;
  cnt = 0;

  // Take 1024 samples and then average them to get the offsets
  while (cnt < 1024)
  {
	ADMUX = (1<<ADLAR)|(1<<REFS0);		// Select ADC channel0
	ADCSRA |= (1 << ADSC);				// Enable ADC
	while (ADCSRA & (1 << ADSC));		// Wait until ADC Data register is full
	aZ = ADCH;							// Read high byte of ADC Data Register

	ADMUX = (1<<ADLAR)|(1<<REFS0)+1;	// Select ADC channel1
	ADCSRA |= (1 << ADSC);				// Enable ADC
	while (ADCSRA & (1 << ADSC));		// Wait until ADC Data register is full
	aY = ADCH;							// Read high byte of ADC Data Register

	ADMUX = (1<<ADLAR)|(1<<REFS0)+2;	// Select ADC channel2
	ADCSRA |= (1 << ADSC);				// Enable ADC
	while (ADCSRA & (1 << ADSC));		// Wait until ADC Data register is full
	aX = ADCH;							// Read high byte of ADC Data Register

	aX_ref = aX_ref + aX;
	aY_ref = aY_ref + aY;
	aZ_ref = aZ_ref + aZ;
	cnt++;
  }
  aX_ref = aX_ref >> 10;
  aY_ref = aY_ref >> 10;
  aZ_ref = aZ_ref >> 10;

  // Decide the axes based on the starting tilt postion
  // If the accelerometer is set handshake, use y and z axis.
  if ((60 < aZ_ref) && (aZ_ref < 100))
  {
   	accel_mode = 1;
	aX_ref = aZ_ref;
  }
  // Otherwise, normal mode (use x and y axis)
  else accel_mode = 0;

  // Initialize cursor acceleration parameters
  accel_x_prev = aX_ref;
  accel_y_prev = aY_ref;
}

/* Initialize UART:
 *		Set up the baud rate and USART settings for data transmission. */
void USART_init(void)
{
	// Baud rate setting
	UBRR1L = (uint8_t) UBRRVAL;				// Low byte
	UBRR1H = (uint8_t) (UBRRVAL>>8);		// High byte
	// Enable transimission
	UCSR1B = (1<<TXEN1);
	// USART setting: SPI mode, odd parity check enabeld, 2-bit stop bits, and 9-bit data size
	UCSR1C = (0<<UMSEL10) | (0<<UMSEL11) | (0<<UPM11) | (0<<UPM10) | (0<<USBS1) | (3<<UCSZ10);
}

/*
 *		Interrupt occurs every 4 ms.
 *		The ADC sample is read and the tilt is computed as cursor movement or scroll, then transmitted (every 8 ms).
 *		If in rapid fire mode, transmit a rapid fire packet every 30ms. */
ISR (TIMER0_COMPA_vect)
{
  // Decrement the task time counts if they are not already zero

	if (tasktime>0)	--tasktime;
  if (acceltime>0) --acceltime;

  // Every 8ms, read ADC value from accelerometer and compute the change of the cursor position

	if (acceltime==0)
	 {
  	acceltime = accel_timeout;	// Reset timeout counter
	readAccel();	// Read ADC value from accelerometer

	// Eliminate the offset by subracting the zero reference

	if (accel_mode == 1) accel_x = aX_ref - aX;
	else	accel_x = aX - aX_ref;
	accel_y = aY - aY_ref;

	// Set up threshhold to prevent drift

	if ((-thresh <= accel_x) && (accel_x <= thresh)) accel_x = 0;
	if ((-thresh <= accel_y) && (accel_y <= thresh)) accel_y = 0;

	// Compute scale parameters based on the cursor acceleration and position inversion

	if (accel_x > accel_x_prev) scale_x = invert_x * (accel_x - accel_x_prev + 8) >> 3;
	else scale_x = invert_x * (accel_x_prev - accel_x + 8) >> 3;
	if (accel_y > accel_y_prev) scale_y = invert_y * (accel_y - accel_y_prev + 8) >>3;
	else scale_y = invert_y * (accel_y_prev - accel_y + 8) >> 3;

	// Store the tilt value for computing cursor acceleration

	accel_x_prev = accel_x;
	accel_y_prev = accel_y;

	// Compute the change of the cursor postion to be sent to base station, based on scale and senstivity

	move_x = (accel_x >> sensitivity) * scale_x;
	move_y = (accel_y >> (sensitivity + scroll_en)) * scale_y;
  }

  // Every other 8ms, transmit the cursor position or scroll value to the base station
  else if (acceltime == 1)
	 {
	// If scroll is enabled, transmit the scroll value every 720ms
	if (scroll_en == 1)
	 {
		scrolltime++;		// Increase the time count for scroll value transmission

		//if (move_y > (35 >> (sensitivity + 1)) * scale_y) tx_packet(0x00, 0x00, -0x01, 0x00);
		//else if (move_y < - ((35 >> (sensitivity + 1)) * scale_y)) tx_packet(0x00, 0x00, 0x01, 0x00);

		if (scrolltime == 90)
		{
			scrolltime = 0;
			if (move_y > 0) tx_packet(0x00, 0x00, -0x01, 0x00);
			else if (move_y < 0) tx_packet(0x00, 0x00, 0x01, 0x00);
		}
	}
	// If move is enabled and scroll is disabled, transmit the cursor position to the base station
	else if (move_en == 1)	tx_packet(move_x, move_y, 0x00, 0x00);
  }

  // If in rapid fire mode, transmit the left button click every 30ms
  if (send_rapid_fire == 1)
	{
  		if (tasktime % 15 == 2)  tx_packet(0x00, 0x00, 0x00, rapid_fire);
  }
}

/*		Execute task-1 every 120ms. */
int main(void)
{
	// DEBUGGING:  on the external LEDs at Port D.7,5

	PORTD |= 0x00;	 // Turn off LEDs
	DDRD |= 0xA0;    // Output for LEDs at Port D.7,5

	// Set up switches

	DDRB = 0x00;		 // Port B pins are input
	PORTB = 0xff;		 // Turn on pullup registers on PORT B

	// Set up buttons

	DDRC = 0x00;		// Port C pins are input
	PORTC = 0xff;		// Turn on pullup registers on PORT C
	// Set up button related parameters

	currentButton = 0xff;
    button_dbouncePressed = 0xff;
	currentSwitch = 0xff;
	switchToggled = 0xff;
    PushState = NoPush;
	PushState_switch = NoPush;
    tasktime = timeout;
	msg_C7 = left_click;
	msg_C6 = middle_click;
	msg_C5 = right_click;
	msg_C7_release = left_click_release;
	msg_C6_release = middle_click_release;
	msg_C5_release = right_click_release;
	scrolltime = 0;

	// Set up timer 0 as 4 milliseconds timebase

		TIMSK0= (1<<OCIE0A);	// Turn on timer 0 cmp match ISR
  	OCR0A = 249;  			// Set the compare register to 250 time ticks
  	TCCR0B= 4; 				// Set prescalar to divide by 256
  	TCCR0A= (1<<WGM01) ;	// Turn on clear-on-match

	// ADC setup

		ADMUX = (1<<ADLAR)|(1<<REFS0);	// 1. Read ADCH (High byte of ADC data register) 2. Select VCC as voltage reference
  	ADCSRA = (1<<ADEN)|(1<<ADSC) + 7;	// Enable the ADC, start conversion, and select 128 as ADC prescalar
	// Port A is an input for ADC
	DDRA=0x00;
	// Initialize acceleration related parameters

	acceltime = 0;
	aX = 0;
	aY = 0;
	aZ = 0;
	accel_x = 0;
	accel_y = 0;
	scale_x = 0;
	scale_y = 0;

	// Initialize mode flags and parameters

	move_en = 0;
	scroll_en = 0;
	rapid_fire_en = 0;
	send_rapid_fire = 0;
	invert_x = 1;
	invert_y = 1;
	accel_mode = 0;

	// Initialize the UART1 for communicating with RF receiver
	USART_init();

	// Enable interrupt
  	sei();

	while (1)
	{
		// Call the task for debouncing
        if (tasktime == 0)	task1();
	}
}

/*		State machines for the button press debouncing and switch toggle debouncing.
 *		Update the flags and transmit packets corresponding to the states. */
void task1(void)
{
  tasktime = timeout;     // Reset the task timer
  currentButton = PINC;		// Read currently pressed buttons
  currentSwitch = PINB;		// Read currently toggled switches
  switch (PushState)
  {
     case NoPush:
	 	// If any button is pressed, it may be a valid press
        if ((currentButton != 0xff)) {
		    PushState=MaybePush;
		    button_dbouncePressed = currentButton;
		 }
        break;
     case MaybePush:
		// If same button is pressed, it is a valid press
        if (button_dbouncePressed == currentButton) {
           PushState=Pushed;
		   PORTD ^= 0x20; 	// DEBUGGING: Toggle LED at Port D.5 to debug

		   // For normal button operations
		   if (button_setting == AllSet) {
		   		// If Port C.7 button is pressed
           		if ((~currentButton & 0x80) == 0x80) {
					// Transmit a packet
  					cli();
					if (rapid_fire_en == 1) send_rapid_fire = 1;
		   			else 	tx_packet(0x00, 0x00, 0x00, msg_C7);
					sei();
		   		}
		   		// If Port C.6 button is pressed
		   		else if ((~currentButton & 0x40) == 0x40) {
  					cli();
		   			tx_packet(0x00, 0x00, 0x00, msg_C6); // Transmit a packet
					sei();
		   		}
		   		// If Port C.5 button is pressed
		   		else if ((~currentButton & 0x20) == 0x20) {
  					cli();
		   			tx_packet(0x00, 0x00, 0x00, msg_C5); // Transmit a packet
					sei();
		   		}
        	}
		}
		// Otherwise, it is not a valid press
        else PushState=NoPush;
        break;
     case Pushed:

		// If the button press has changed, it may be a valid release
        if (button_dbouncePressed != currentButton)		PushState=MaybeNoPush;
        break;
     case MaybeNoPush:

		// If the button press is actually not changed, it is not a valid release
        if (button_dbouncePressed == currentButton) PushState=Pushed;

		// Otherwise, it is a valid release
        else {
           PushState=NoPush;
		   PORTD ^= 0x20; 	// DEBUGGING: Toggle LED at Port D.5 to debug

		   // If Port C.7 button is released
           if ((~button_dbouncePressed & 0x80) == 0x80) {
  				cli();
		   		tx_packet(0x00, 0x00, 0x00, msg_C7_release); // Transmit a packet
				sei();
				send_rapid_fire = 0;
		   	}
		   	// If Port C.6 button is released
		   	else if ((~button_dbouncePressed & 0x40) == 0x40) {
  				cli();
		   		tx_packet(0x00, 0x00, 0x00, msg_C6_release); // Transmit a packet
				sei();
		   	}
		   	// If Port C.5 button is released
		  	else if ((~button_dbouncePressed & 0x20) == 0x20) {
  				cli();
		   		tx_packet(0x00, 0x00, 0x00, msg_C5_release); // Transmit a packet
				sei();
		   	}
			// If Port C.4 button is released (Move Enable)
			else if ((~button_dbouncePressed & 0x10) == 0x10) {
				move_en ^= 1;	// Toggle the flag for move enable
		   		PORTD ^= 0x80; 	// DEBUGGING: Toggle LED at Port D.7 to debug
		  		if (move_en == 1) {
					cli();
		   			calibrate();	// Set reference acceleration
					sei();
				}
		   	}
			// If Port C.3 button is released (Scroll Enable)
			else if ((~button_dbouncePressed & 0x08) == 0x08) {
				scroll_en ^= 1;	// Toggle the flag for scroll enable
				if (scroll_en == 1) {
					cli();
		   			calibrate();	// Set reference acceleration
					sei();
				}
			}
			button_dbouncePressed = 0xff;
        }
        break;
  }

  // State machine for switches
  switch (PushState_switch)
  {
     case NoPush:
	 	// If any switch is toggled, it may be a valid toggle
        if ((currentSwitch != switchToggled)) {
		    PushState_switch=MaybePush;
		    switchToggled = currentSwitch;
		 }
        break;
     case MaybePush:
		// If same switch is still toggled, it is a valid toggle
        if (currentSwitch == switchToggled) {
           PushState_switch=Pushed;

		   // If Port B.0 is on
           if ((~currentSwitch & 0x01) == 0x01) sensitivity = 4;

			 // If Port B.0 is off
		   else if ((~currentSwitch & 0x01) == 0x00) sensitivity = 3;

		   // If Port B.1 is on
		   if ((~currentSwitch & 0x02) == 0x02) rapid_fire_en = 0;

			 // If Port B.1 is off
		   else if ((~currentSwitch & 0x02) == 0x00) rapid_fire_en = 1;

		   // If Port B.2 is on
		   if ((~currentSwitch & 0x04) == 0x04) invert_x = 1;

			 // If Port B.2 is off
		   else if ((~currentSwitch & 0x04) == 0x00) invert_x = -1;

		   // If Port B.3 is on
		   if ((~currentSwitch & 0x08) == 0x08) invert_y = 1;

			 // If Port B.3 is off
		   else if ((~currentSwitch & 0x08) == 0x00) invert_y = -1;
		}
		// Otherwise, it is not a valid toggle
        else PushState_switch=NoPush;
        break;
     case Pushed:
	 	// If any switch is toggled, it may be a valid toggle
        if (currentSwitch != switchToggled) {
		   PushState_switch=MaybeNoPush;
		   switchToggled = currentSwitch;
		}
        break;
     case MaybeNoPush:
		// If same switch is still toggled, it is a valid toggle
        if (currentSwitch == switchToggled) {
           PushState_switch=NoPush;

		   // If Port B.0 is on
           if ((~currentSwitch & 0x01) == 0x01) sensitivity = 4;

			 // If Port B.0 is off
		   else if ((~currentSwitch & 0x01) == 0x00) sensitivity = 3;

		   // If Port B.1 is on
		   if ((~currentSwitch & 0x02) == 0x02) rapid_fire_en = 0;

			 // If Port B.1 is off
		   else if ((~currentSwitch & 0x02) == 0x00) rapid_fire_en = 1;

		   // If Port B.2 is on
		   if ((~currentSwitch & 0x04) == 0x04) invert_x = 1;

			 // If Port B.2 is off
		   else if ((~currentSwitch & 0x04) == 0x00) invert_x = -1;

		   // If Port B.3 is on
		   if ((~currentSwitch & 0x08) == 0x08) invert_y = 1;

			 // If Port B.3 is off
		   else if ((~currentSwitch & 0x08) == 0x00) invert_y = -1;
        }
		// Otherwise, it is not a valid toggle
		else PushState_switch=Pushed;
        break;
	}
}
