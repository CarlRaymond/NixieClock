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
const int PIN_WWVB = 7; // PORTD bit 7

const int PIN_RCK = 8;

// SPI pins
const int PIN_MOSI = 11;
const int PIN_MISO = 12;
const int PIN_SCK = 13;

// Heartbeat counter. Timer1 will produce interrupts at 60Hz, using
// "fractional PLL" technique.  Count heartbeat_short_periods (out of
// heartbeat_total_periods) using heartbeat_cycles, and the remainder
// using heartbeat_cycles+1.
// For 60Hz, count two periods of 33,333 cycles for each 
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

// First 60 pixels are the ring; the final 10 are the on-board backlight pixels
Adafruit_NeoPixel ring = Adafruit_NeoPixel(70, PIN_PIXEL, NEO_GRB + NEO_KHZ800);

const uint32_t OFF = 0L;
const uint32_t SAMPLE_ONE = ring.Color(6, 1, 0);
const uint32_t SAMPLE_CURSOR = ring.Color(3, 50, 2);
const uint32_t SAMPLE_ZERO = ring.Color(5, 1, 8);

const uint32_t RED = ring.Color(255,0,0);
const uint32_t ORANGE = ring.Color(255,50,0);
const uint32_t YELLOW = ring.Color(255,150,0);
const uint32_t GREEN = ring.Color(0,255,0);
const uint32_t BLUE = ring.Color(0,0,255);
const uint32_t PURPLE = ring.Color(128,0,128);
const uint32_t PINK = ring.Color(60, 0, 10);
const uint32_t FLASH = ring.Color(255, 128, 128);

// Six bytes of data for nixies
uint8_t nixieData[6];

// 72-bit long shift register for input samples. Little endian.
// Offset 0, bit 0 has most recent sample bit; offset 8 bit 7
// has oldest sample bit. Shifts left. 
uint8_t volatile samples[9];

// Correlation template for a zero bit. From newest to oldest:
// 48 0s, 12 1s, 12 0s.  Initialzing values start with LSB, which
// is the most recent bit w.r.t sampled bit values, and progress to
// the oldest bit.
uint8_t WWVB_ZERO[9] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x0f, 0x00};
// Correlaton template for one bit. From newest to oldest:
// 30 0s, 30 1s, 12 0s
//uint8_t WWVB_ONE[9] = { 0x00, 0x0F, 0xFF, 0xFF, 0xFF, 0xC0, 0x00, 0x00, 0x00 };
uint8_t WWVB_ONE[9] = { 0x00, 0x00, 0x00, 0xc0, 0xff, 0xff, 0xff, 0x0f, 0x00 };
// Correlation template for frame bit. From newest to oldest:
// 12 0s, 48 1s, 12 0s
//uint8_t WWVB_FRAME[9] = { 0x00, 0x0F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0, 0x00 };
uint8_t WWVB_FRAME[9] = { 0x00, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xff, 0x0f, 0x00 };

// Pattern matching threshold
int scoreThreshold = 48;

void setup() {

	// Configure SPI pins
	SPI.begin();

	// Configure PWM
	pinMode(PIN_PWM, OUTPUT);

	pinMode(PIN_HV, OUTPUT);
	digitalWrite(PIN_HV, HIGH); // HV off

	pinMode(PIN_RCK, OUTPUT);
	digitalWrite(PIN_RCK, LOW);

	pinMode(PIN_PIXEL, OUTPUT);
	pinMode(PIN_WWVB, INPUT);
	pinMode(PIN_HEARTBEAT, OUTPUT);

	configurePwm();
	configureHeartbeat();

	ring.begin();

	// Backlight rainbow
	ring.setPixelColor(60, RED);
	ring.setPixelColor(61, ORANGE);
	ring.setPixelColor(64, YELLOW);
	ring.setPixelColor(65, GREEN);
	ring.setPixelColor(68, BLUE);
	ring.setPixelColor(69, PURPLE);

	// Colons
	ring.setPixelColor(62, PINK);
	ring.setPixelColor(63, PINK);
	ring.setPixelColor(66, PINK);
	ring.setPixelColor(67, PINK);

	ring.show();

	setTubePwm(145);
	Serial.begin(230400);


	Serial.print(WWVB_ZERO[8], BIN);
	Serial.print(' ');
	Serial.print(WWVB_ZERO[7], BIN);
	Serial.print(' ');
	Serial.print(WWVB_ZERO[6], BIN);
	Serial.print(' ');
	Serial.print(WWVB_ZERO[5], BIN);
	Serial.print(' ');
	Serial.print(WWVB_ZERO[4], BIN);
	Serial.print(' ');
	Serial.print(WWVB_ZERO[3], BIN);
	Serial.print(' ');
	Serial.print(WWVB_ZERO[2], BIN);
	Serial.print(' ');
	Serial.print(WWVB_ZERO[1], BIN);
	Serial.print(' ');
	Serial.print(WWVB_ZERO[0], BIN);
	Serial.print('\n');

	Serial.print(WWVB_ONE[8], BIN);
	Serial.print(' ');
	Serial.print(WWVB_ONE[7], BIN);
	Serial.print(' ');
	Serial.print(WWVB_ONE[6], BIN);
	Serial.print(' ');
	Serial.print(WWVB_ONE[5], BIN);
	Serial.print(' ');
	Serial.print(WWVB_ONE[4], BIN);
	Serial.print(' ');
	Serial.print(WWVB_ONE[3], BIN);
	Serial.print(' ');
	Serial.print(WWVB_ONE[2], BIN);
	Serial.print(' ');
	Serial.print(WWVB_ONE[1], BIN);
	Serial.print(' ');
	Serial.print(WWVB_ONE[0], BIN);
	Serial.print('\n');

	Serial.print(WWVB_FRAME[8], BIN);
	Serial.print(' ');
	Serial.print(WWVB_FRAME[7], BIN);
	Serial.print(' ');
	Serial.print(WWVB_FRAME[6], BIN);
	Serial.print(' ');
	Serial.print(WWVB_FRAME[5], BIN);
	Serial.print(' ');
	Serial.print(WWVB_FRAME[4], BIN);
	Serial.print(' ');
	Serial.print(WWVB_FRAME[3], BIN);
	Serial.print(' ');
	Serial.print(WWVB_FRAME[2], BIN);
	Serial.print(' ');
	Serial.print(WWVB_FRAME[1], BIN);
	Serial.print(' ');
	Serial.print(WWVB_FRAME[0], BIN);
	Serial.print('\n');

}


void loop() {

	for (int hour=1;  hour<24;  hour++) {
		setHours(hour);
		for (int sec=0;  sec<60;  sec++) {
			setSeconds(sec);
			setMinutes(59 - sec);
			updateNixies();
			delay(17);
		}
	}
}

// Invoked at 60Hz.
void heartbeat() {

	// Set heartbeat pin high
	PORTD |= B00000100;

	// Sample the input - port D bit 7
	//bool input = (PORTD & B00000001) >> 7;
	uint8_t input = digitalRead(PIN_WWVB);
	shiftSample(input);

	//Serial.print(samples[8], BIN);
	//Serial.print(' ');
	//Serial.print(samples[7], BIN);
	//Serial.print(' ');
	//Serial.print(samples[6], BIN);
	//Serial.print(' ');
	//Serial.print(samples[5], BIN);
	//Serial.print(' ');
	//Serial.print(samples[4], BIN);
	//Serial.print(' ');
	//Serial.print(samples[3], BIN);
	//Serial.print(' ');
	//Serial.print(samples[2], BIN);
	//Serial.print(' ');
	//Serial.print(samples[1], BIN);
	//Serial.print(' ');
	//Serial.print(samples[0], BIN);
	//Serial.print('\n');

	int score_frame = score(WWVB_FRAME);
	int score_one = score(WWVB_ONE);
	int score_zero = score(WWVB_ZERO);

	flashZero(score_zero);
	flashOne(score_one);
	flashFrame(score_frame);

	if (score_zero > scoreThreshold || score_one > scoreThreshold || score_frame > scoreThreshold) {
		Serial.print(score_zero);
		if (score_zero > scoreThreshold)
			Serial.print("**  ");
		else
			Serial.print("    ");
	
		Serial.print(score_one);
		if (score_one > scoreThreshold)
			Serial.print("**  ");
		else
			Serial.print("    ");

		Serial.print(score_frame);
		if (score_frame > scoreThreshold)
			Serial.print("**\n");
		else
			Serial.print("\n");
	}

	if (input) {
	  ring.setPixelColor(pixelIndex, SAMPLE_ONE);
	}
	else {
		ring.setPixelColor(pixelIndex, SAMPLE_ZERO);
	}

  	pixelIndex+=1;
 	if (pixelIndex >= 60) {
  		pixelIndex -= 60;
  	}
	ring.setPixelColor(pixelIndex, SAMPLE_CURSOR);

	ring.show();

	// Set heartbeat pin low
	PORTD &= B11111011;
}

void flashZero(int score) {
	if (score > scoreThreshold) {
		ring.setPixelColor(68, FLASH);
		ring.setPixelColor(69, FLASH);
	}
	else {
		ring.setPixelColor(68, BLUE);
		ring.setPixelColor(69, PURPLE);
	}
}

void flashOne(int score) {
	if (score > scoreThreshold) {
		ring.setPixelColor(64, FLASH);
		ring.setPixelColor(65, FLASH);
	}
	else {
		ring.setPixelColor(64, YELLOW);
		ring.setPixelColor(65, GREEN);
	}
}

void flashFrame(int score) {
	if (score > scoreThreshold) {
		ring.setPixelColor(60, FLASH);
		ring.setPixelColor(61, FLASH);
	}
	else {
		ring.setPixelColor(60, RED);
		ring.setPixelColor(61, ORANGE);
	}
}

// Shifts in a new bit sample into the array. The new bit is the LSB of the value
void shiftSample(uint8_t value) {

	// Shift LSB of value into C flag
	asm volatile(
		"lsr %0 \n\t" : "=r" (value) : "0" (value) 
	);

	// Shift sample array left one bit
	asm volatile(

		// The two lines below are auto-generated by the compiler as a result
		// of the ' "e" (samples) ' constraint at the end of this asm block.
		// The compiler will select a base register (X, Y or Z) to hold the address
		// of 'samples', and the expression %a0 will be replaced with the selected
		// base register.

		//"ldi %A0, lo8(samples) \n\t"
		//"ldi %B0, hi8(samples) \n\t"

		// Unrolled loop to shift 9 bytes
		"ld __tmp_reg__, %a0 \n\t"
		"rol __tmp_reg__ \n\t"
		"st %a0+, __tmp_reg__ \n\t"

		"ld __tmp_reg__, %a0 \n\t"
		"rol __tmp_reg__ \n\t"
		"st %a0+, __tmp_reg__ \n\t"

		"ld __tmp_reg__, %a0 \n\t"
		"rol __tmp_reg__ \n\t"
		"st %a0+, __tmp_reg__ \n\t"

		"ld __tmp_reg__, %a0 \n\t"
		"rol __tmp_reg__ \n\t"
		"st %a0+, __tmp_reg__ \n\t"

		"ld __tmp_reg__, %a0 \n\t"
		"rol __tmp_reg__ \n\t"
		"st %a0+, __tmp_reg__ \n\t"

		"ld __tmp_reg__, %a0 \n\t"
		"rol __tmp_reg__ \n\t"
		"st %a0+, __tmp_reg__ \n\t"

		"ld __tmp_reg__, %a0 \n\t"
		"rol __tmp_reg__ \n\t"
		"st %a0+, __tmp_reg__ \n\t"

		"ld __tmp_reg__, %a0 \n\t"
		"rol __tmp_reg__ \n\t"
		"st %a0+, __tmp_reg__ \n\t"

		"ld __tmp_reg__, %a0 \n\t"
		"rol __tmp_reg__ \n\t"
		"st %a0+, __tmp_reg__ \n\t"
		:  : "e" (samples)
	);
}


// Array, indexed from 0..255, where each byte contains the number of 1 bits
// in the corresponding index. Used by score function to sum the number of
// matching bits in pattern comparisons.
uint8_t parity[256] = {
	0,	1,	1,	2,	1,	2,	2,	3,	1,	2,	2,	3,	2,	3,	3,	4,		// 0x00..0x0f
	1,	2,	2,	3,	2,	3,	3,	4,	2,	3,	3,	4,	3,	4,	4,	5,		// 0x10..0x1f (+1)
	1,	2,	2,	3,	2,	3,	3,	4,	2,	3,	3,	4,	3,	4,	4,	5,		// 0x20..0x2f (+1)
	2,	3,	3,	4,	3,	4,	4,	5,	3,	4,	4,	5,	4,	5,	5,	6,		// 0x30..0x3f (+2)

	1,	2,	2,	3,	2,	3,	3,	4,	2,	3,	3,	4,	3,	4,	4,	5,		// 0x40..0x4f (+1)
	2,	3,	3,	4,	3,	4,	4,	5,	3,	4,	4,	5,	4,	5,	5,	6,		// 0x50..0x5f (+2)
	2,	3,	3,	4,	3,	4,	4,	5,	3,	4,	4,	5,	4,	5,	5,	6,		// 0x60..0x6f (+2)
	3,	4,	4,	5,	4,	5,	5,	6,	4,	5,	5,	6,	5,	6,	6,	7,		// 0x70..0x7f (+3)

	1,	2,	2,	3,	2,	3,	3,	4,	2,	3,	3,	4,	3,	4,	4,	5,		// 0x80..0x8f (+1)
	2,	3,	3,	4,	3,	4,	4,	5,	3,	4,	4,	5,	4,	5,	5,	6,		// 0x90..0x9f (+2)
	2,	3,	3,	4,	3,	4,	4,	5,	3,	4,	4,	5,	4,	5,	5,	6,		// 0xa0..0xaf (+2)
	3,	4,	4,	5,	4,	5,	5,	6,	4,	5,	5,	6,	5,	6,	6,	7,		// 0xb0..0xbf (+3)

	2,	3,	3,	4,	3,	4,	4,	5,	3,	4,	4,	5,	4,	5,	5,	6,		// 0xc0..0xcf (+2)
	3,	4,	4,	5,	4,	5,	5,	6,	4,	5,	5,	6,	5,	6,	6,	7,		// 0xd0..0xdf (+3)
	3,	4,	4,	5,	4,	5,	5,	6,	4,	5,	5,	6,	5,	6,	6,	7,		// 0xe0..0xef (+3)
	4,	5,	5,	6,	5,	6,	6,	7,	5,	6,	6,	7,	6,	7,	7,	8,		// 0xf0..0xff (+4)
};


// Score the sample array bits against the supplied pattern. Result
// is number of matching bits between them.
int score(uint8_t *pattern) {

	int sum = 0;
	for (int i=0; i<9; i++) {
		// Compute matching bits: XOR pattern and samples, and complement
		uint8_t bits = ~(samples[i] ^ pattern[i]);
		sum += parity[bits];
	}

	return sum;
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

void configurePwm() {
	// Configure timer 2 for PWM at 244Hz

	// Arduino pin 3 is OC2B
	TCCR2A = _BV(COM2B1) | _BV(COM2B0) | _BV(WGM21) | _BV(WGM20);
 	TCCR2B = _BV(CS22) | _BV(CS21);
	OCR2B = 255;
}
void configureHeartbeat() {

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

