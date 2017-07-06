#include <SPI.h>
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif


// Pin map
//
// Nixie DIN (out):			11
// SRCK (out):				13
// RCK (out):				8
// Nixie /EN (out):			3
// Pixel DIN (out):			6
// HV /EN (out):			5

// WWVB Data (in):			7
// 60Hz heartbeat (out):	2

// Heartbeat indicator
const int PIN_HEARTBEAT = 2; // PORTD bit 2

const int PIN_PWM = 3;

const int PIN_HV = 5;

// Ring serial output
const int PIN_PIXEL = 6;

// SYMTRIK input pin
const int PIN_WWVB = 7;

const int PIN_RCK = 8;

// SPI pins
const int PIN_MOSI = 11;
const int PIN_MISO = 12;
const int PIN_SCK = 13;

// Heartbeat counter. Timer1 will produce interrupts at 60Hz, using
// "fractional PLL" technique.  Count heartbeat_short_periods (out of
// heartbeat_total_periods) using heartbeat_cycles, and the remainder
// using heartbeat_cycles+1.
// For 60Hz, count two periods of 33,333 cycles and one
// period of 33,334 cycles (with prescaler = 8, or 2,000,000Hz clock).

// No. of cycles in a short period
const uint16_t heartbeat_cycles = 33333;

// No. of short periods in full sequence
const uint8_t heartbeat_short_periods = 10;

// Total no. of periods in full sequence
const uint8_t heartbeat_total_periods = 16;

// Current period index
volatile uint8_t heartbeat_period = 0;

// Indicates the active pixel; updated at 60Hz by interrupt 
volatile uint8_t pixelIndex = 0;

// First 60 pixels are the ring; the final 10 are the on-board pixels
Adafruit_NeoPixel ring = Adafruit_NeoPixel(70, PIN_PIXEL, NEO_GRB + NEO_KHZ800);

const uint32_t OFF = 0L;
const uint32_t ORANGE = ring.Color(6, 1, 0);
const uint32_t GREEN = ring.Color(3, 10, 2);
const uint32_t PURPLE = ring.Color(5, 1, 8);
const uint32_t BLUE = ring.Color(4, 4, 20);

//const uint32_t ORANGE = ring.Color(6*25, 1*25, 0);
//const uint32_t GREEN = ring.Color(3*25, 10*25, 2*25);
//const uint32_t PURPLE = ring.Color(5*25, 1*25, 8*25);

// Six bytes of data for nixies
uint8_t nixieData[6];

void setup() {

	// Configure SPI pins
	SPI.begin();

	// Configure PWM
	pinMode(PIN_PWM, OUTPUT);

	pinMode(PIN_HV, OUTPUT);
	digitalWrite(PIN_HV, LOW);

	pinMode(PIN_RCK, OUTPUT);
	digitalWrite(PIN_RCK, LOW);

	pinMode(PIN_PIXEL, OUTPUT);
	pinMode(PIN_WWVB, INPUT);
	pinMode(PIN_HEARTBEAT, OUTPUT);

	configure_pwm();
	configure_heartbeat();

	ring.begin();

	// Backlight rainbow
	ring.setPixelColor(60, ring.Color(255,0,0));
	ring.setPixelColor(61, ring.Color(255,50,0));
	ring.setPixelColor(64, ring.Color(255,150,0));
	ring.setPixelColor(65, ring.Color(0,255,0));
	ring.setPixelColor(68, ring.Color(0,0,255));
	ring.setPixelColor(69, ring.Color(128,0,128));

	// Colons
	ring.setPixelColor(62, ring.Color(60, 0, 10));
	ring.setPixelColor(63, ring.Color(60, 0, 10));
	ring.setPixelColor(66, ring.Color(60, 0, 10));
	ring.setPixelColor(67, ring.Color(60, 0, 10));

	ring.show();

	setTubePwm(100);
	Serial.begin(115200);
}


void loop() {

	for (int hour=1;  hour<24;  hour++) {
		setHours(hour);
		for (int sec=0;  sec<60;  sec++) {
			setSeconds(sec);
			setMinutes(59 - sec);
			updateNixies();
			delay(25);
		}
	}
}

// Invoked at 60Hz.
void heartbeat() {

	// Set heartbeat pin high
	PORTD |= B00000100;

	// Sample the input
	bool input = digitalRead(PIN_WWVB);
	
	if (input) {
	  ring.setPixelColor(pixelIndex, ORANGE);
	}
	else {
		ring.setPixelColor(pixelIndex, PURPLE);
	}

  	pixelIndex+=1;
 	if (pixelIndex >= 60) {
  		pixelIndex -= 60;
  	}
	ring.setPixelColor(pixelIndex, GREEN);

	ring.show();

	// Set heartbeat pin low
	PORTD &= B11111011;
}

// Increment the time of day
void tick() {

}

void updateNixies() {
	digitalWrite(PIN_RCK, LOW);
	SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
	for (int i=0; i<6; i++) {
		SPI.transfer(nixieData[i]);
	}
	SPI.endTransaction();
	digitalWrite(PIN_RCK, HIGH);
}

void resetNixies() {
	for (int i=0; i<6; i++) {
		nixieData[i] = 0;
	}
}

// Set the value displayed on the seconds digits. No range check performed.
// 0 <= value <= 60
void setSeconds(int value) {
	int units;
	int tens;

	// Blank all seconds segments
	nixieData[5] = 0;
	nixieData[4] = 0;
	nixieData[3] &= B11111110;

    tens = value / 10;
	units = value % 10;

	if (units < 8) {
		nixieData[5] = (1 << units);
	}
	else {
		nixieData[4] |= (1 << (units-8));
	}

	if (tens < 6) {
		nixieData[4] |= (1 << (tens+2));
	}
	else {
		nixieData[3] |= B00000001;
	}
}

// Set the value displayed on the minutes digits. No range check performed.
// 0 <= value <= 59
void setMinutes(int value) {
	int units;
	int tens;

	// Blank all minutes segments
	nixieData[3] &= B00000001;
	nixieData[2] = 0;
	nixieData[1] &= B11111110;

	tens = value / 10;
	units = value % 10;

	if (units < 7) {
		nixieData[3] |= (1 << (units+1));
	}
	else {
		nixieData[2] |= (1 << (units-7));
	}

	if (tens < 5) {
		nixieData[2] |= (1 << (tens+3));
	}
	else {
		nixieData[1] |= 1;
	}
}

// Set the value displayed on the hours digits. No range check performed.
// 0 <= value <= 23
void setHours(int value) {
	int tens = value / 10;
	int units = value % 10;

	// Blank all hours segments
	nixieData[1] &= B00000001;
	nixieData[0] = 0;

	if (units < 7) {
		nixieData[1] |= (1 << (units+1));
	}
	else {
		nixieData[0] |= (1 << (units-7));
	}

	// Suppress leading zero
	if (tens > 0) {
		nixieData[0] |= (1 << (tens+3));
	}
}

void setTubePwm(uint8_t value) {
	OCR2B = value;
}

void configure_pwm() {
	// Configure timer 2 for PWM at 244Hz

	// Arduino pin 3 is OC2B
	TCCR2A = _BV(COM2B1) | _BV(COM2B0) | _BV(WGM21) | _BV(WGM20);
 	TCCR2B = _BV(CS22) | _BV(CS21);
	OCR2B = 255;
}
void configure_heartbeat() {

	// Configure timer 1 to interrupt at 60Hz
 	OCR1A = heartbeat_cycles;

	TCCR1A = 0;
    // Mode 4, CTC on OCR1A, prescaler 8
    TCCR1B = (1 << WGM12) | (1 << CS11);

    //Set interrupt on compare match
    TIMSK1 |= (1 << OCIE1A);
}

// Heartbeat interrupt.
ISR (TIMER1_COMPA_vect) {
	// Reset the CTC value. For a short period, count CTC_value cycles;
	// for a long period, count CTC_value+1. 
	if (++heartbeat_period >= heartbeat_total_periods)
		heartbeat_period = 0;

    // Set compare value for a short or long cycle. Must set OCR1A to n-1.
	if (heartbeat_period <= heartbeat_short_periods) {
		// Short period: n-1
		OCR1A = heartbeat_cycles-1;
	}
	else {
		// Long period: n+1-1
		OCR1A = heartbeat_cycles;
	}

	heartbeat();
}

