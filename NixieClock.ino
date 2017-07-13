#include <SPI.h>
#include "DataGenerator.h"
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


// Theory of operation:
// The WWVB bitstream consists of data bits which last one second each. A full word,
// containing complete time-of-day (TOD) data, is 60 bits, taking one minute,
// and is transmitted continuously. Each full frame gives the minute, hour, day, year
// (and some status bits) as of the moment that frame began. The frame does not include
// seconds; they are implcit. Second "0" begins at the start of the first bit of the frame.
//
// There are three possible symbol patterns within the word that can be received: a logic
// zero, a logic one, and a framing symbol that helps to determine begining and end of each
// frame. Each is a pulse with a varying high time and low time.
// WWVB_ZERO: 0.2s high, 0.8s low
// WWVB_ONE: 0.5s high, 0.5s low
// WWVB_FRAME: 0.8s high, 0.2s low
// 
// The incoming bitstream is sampled at 60 Hz, and and bits are transferred to a shift register.
// Symbols are detected by comparing the stored samples with ideal templates of
// the three possible symbols. A scoring function counts up the number of
// matching bits, and on each bit shift compares the sampled data to the three templates. At
// the 60Hz sample rate, a perfect bit pattern for WWVB_ZERO would consist of 12 high samples,
// followed by 48 low samples.
//
// The shift register and matching patterns are 80 bits instead of only 60. This allows
// capturing and comparing not just a full pulse, but some of the preceding pulse's tail end,
// and some of the following pulse's head end as well. Including these extra transitions will
// clearly delineate a pulse, and improve noise immunity. The matching patterns include 10 extra
// bits at the front and 10 bits at the back, bracketing the 60 samples for the symbol.
// Since all three symbols begin with at least 12 high samples, and end with at least 12 low
// samples, padding the patterns with only 10 bits prevents any chance of having an extra
// transition at either end of the sample buffer because of timing inaccuracy. Also, a buffer
// size that is a multiple of 8 bits is simpler to deal with.
// While the additional 20 bits do not help to distinguish one symbol from another (since they
// are the same for all symbols), they do help to distinguish a valid symbol from random
// noise.
// A match is detected when a symbols's score exceeds a threshold value. The threshold is below
// the maximum score of 80 to account for both noise in the received signal, and inaccuracy
// in the local sampling clock.
//
// This sample, shift, and score process happens continuously in the background, at 60Hz, and is
// the first phase in decoding the bitstream. The second phase consists of watching the
// constantly changing triples of symbol scores and detecting when they reach a peak.
// By the nature of the bit shifting, scores will follow a ramp pattern: as the incoming
// bits for a symbols get closer to perfect alignment with the pattern, the score will
// increase, reach a peak at perfect alignment, and then begin to drop as the incoming
// waveform pattern shifts past the ideal form of the template.
// 
// To find the peaks of the symbols scores, the previous n scores are kept in score history
// buffers (one buffer for each symbol). These are also shift registers, but hold bytes
// rather than bits. A peak in the scores is detected by looking for the ramp pattern in
// the score history. The history buffer size, n, should be an odd number (5? 7? 11?), and
// the peak finder looks for the case where the peak value is in the middle slot, with
// decreasing (or at least non-increasing) values on either side. (The case of two adjacent
// maximum values may have to be handled, as well.)
//
// At this point, the data bits can be recognized by the appearance of the peaks in the three
// symbol scoring buffers. Right after receiving a nice clean bit, one of the three buffers should
// show a score above the threshold, with a peak value. The other two buffers should show scores
// below the threshold, and not sharply peaked. After 60 ticks (one second), the following bit
// will have been recieved, and one of the three buffers should show a peak, ideally in the same
// position within the buffer. If the receiver should miss a bit, no buffer will show a peak.
//
// The task now is to keep the location of the peak from drifting as successive bits are decoded. Drift
// indicates that the tick interval timer's period is too short or too long. On noticing drift (after
// some low-pass filtering of the peak slot), the timer period is adjusted up or down slightly to
// compensate.
// 
// Timer1 interrupts at 60Hz (one tick), creating the heartbeat. An ISR is responsible
// for sampling the input signal and shifting it into the input shift register, scoring (x3),
// and shifting the symbol scores into their shift registers. 
//
// Timer2 is configured for PWM, at 244Hz, to control tube brightness.

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
const unsigned int heartbeat_cycles = 33333;

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

// Colors
const uint32_t OFF = 0L;
const uint32_t SAMPLE_ONE = ring.Color(6, 1, 0);
const uint32_t SAMPLE_ZERO = ring.Color(5, 1, 8);
const uint32_t SAMPLE_CURSOR = ring.Color(3, 50, 2);

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

// 80-bit long shift register for input samples. Little endian.
// Offset 0, bit 0 has most recent sample bit; offset 9 bit 7
// has oldest sample bit. Shifts left. 
uint8_t volatile samples[10];

// Correlation template for a zero bit, including the trailing 0s of the preceding symbol,
// and the leading 1s of the following symbol. From head end to tail end:
// 10 0s, 12 1s, 48 0s, 10 0s.  Initialzing values start with LSB, which
// is the most recent bit (the tail end of the pulse), and progress to
// the oldest bit (the head end).  (Remember: bytes are written LSB first, but bits
// in a byte are reversed!)
uint8_t WWVB_ZERO[10] = { 0xff, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0x3f, 0x00 };
// Correlaton template for one bit. From head to tail:
// 10 0s, 30 1s, 30 0s, 10 1s
uint8_t WWVB_ONE[10] = { 0xff, 0x03, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0x3f, 0x00 };
// Correlation template for frame bit. From head to tail:
// 10 0s, 48 1s, 12 0s, 10 1s
uint8_t WWVB_FRAME[10] = { 0xff, 0x03, 0xc0, 0xff, 0xff, 0xff, 0xff, 0xff, 0x3f, 0x00 };

// Pattern matching threshold
uint8_t scoreThreshold = 70;

// Score history buffers
const int historyLength = 7;
unsigned short history_ZERO[historyLength];
unsigned short history_ONE[historyLength];
unsigned short history_FRAME[historyLength];

// Current time of day, UTC
unsigned short volatile tod_ticks = 0;
unsigned short volatile tod_seconds = 0;
unsigned short volatile tod_minutes = 24;
unsigned short volatile tod_hours = 11;
unsigned int volatile tod_day = 0;
unsigned int volatile tod_year = 0;
unsigned int volatile tod_TZoffset = -4;
bool volatile tod_isdst = true;
bool volatile tod_isleapminute = false;
bool volatile tod_isleapyear = false;
bool volatile tod_changed = false;

// Prepare some fake data
uint8_t fakedata[] = {
	FRAME,	ZERO,	ZERO,	ONE,	ZERO,	ZERO,	ONE,	ZERO,	ZERO,	FRAME
};
DataGenerator fake_frame = DataGenerator(fakedata, 10);

void setup() {

	// Configure SPI pins
	SPI.begin();

	// Configure PWM for tube dimming
	pinMode(PIN_PWM, OUTPUT);

	// HV power supply
	pinMode(PIN_HV, OUTPUT);
	digitalWrite(PIN_HV, LOW); // HV on

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

	setTubePwm(150);
	Serial.begin(230400);


	test_showPatterns();
	test_shifter();

}

void test_showPatterns() {

	Serial.print("WWVB_ZERO: ");
	Serial.print(WWVB_ZERO[9], BIN);
	Serial.print(' ');
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

	Serial.print("WWVB_ONE: ");
	Serial.print(WWVB_ONE[9], BIN);
	Serial.print(' ');
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

	Serial.print("WWVB_FRAME: ");
	Serial.print(WWVB_FRAME[9], BIN);
	Serial.print(' ');
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

void test_shifter() {
	// Shift in a simulated WWVB_ZERO, then compare to the other patterns
	for (short i=0; i<10; i++) {
		shiftSample(0);
	}
	for (short i=0; i<12; i++) {
		shiftSample(1);
	}
	for (short i=0; i<48; i++) {
		shiftSample(0);
	}

	for (short i=0; i<10; i++) {
		shiftSample(1);
	}

	Serial.print("ZERO on ZERO: ");
	Serial.print(score(WWVB_ZERO));
	Serial.print("\nZERO on ONE: ");
	Serial.print(score(WWVB_ONE));
	Serial.print("\nZERO on FRAME: ");
	Serial.print(score(WWVB_FRAME));
	Serial.print("\n");

	// Shift in a simulated WWVB_ONE, then compare to the other patterns
	for (short i=0; i<10; i++) {
		shiftSample(0);
	}
	for (short i=0; i<30; i++) {
		shiftSample(1);
	}
	for (short i=0; i<30; i++) {
		shiftSample(0);
	}
	for (short i=0; i<10; i++) {
		shiftSample(1);
	}

	Serial.print("ONE on ZERO: ");
	Serial.print(score(WWVB_ZERO));
	Serial.print("\nONE on ONE: ");
	Serial.print(score(WWVB_ONE));
	Serial.print("\nONE on FRAME: ");
	Serial.print(score(WWVB_FRAME));
	Serial.print("\n");

	// Shift in a simulated WWVB_FRAME, then compare to the other patterns
	for (short i=0; i<10; i++) {
		shiftSample(0);
	}
	for (short i=0; i<48; i++) {
		shiftSample(1);
	}
	for (short i=0; i<12; i++) {
		shiftSample(0);
	}
	for (short i=0; i<10; i++) {
		shiftSample(1);
	}
	
	Serial.print("FRAME on ZERO: ");
	Serial.print(score(WWVB_ZERO));
	Serial.print("\nFRAME on ONE: ");
	Serial.print(score(WWVB_ONE));
	Serial.print("\nFRAME on FRAME: ");
	Serial.print(score(WWVB_FRAME));
	Serial.print("\n");
}




void loop() {
	if (tod_changed) {
		tod_changed = false;
		updateTimeOfDayLocal();
		updateNixies();
	}
}

// Invoked at 60Hz.
void heartbeat() {

	// Set heartbeat pin high
	PORTD |= B00000100;

	// Sample the input - port D bit 7
	//bool input = (PORTD & B00000001) >> 7;
	//uint8_t input = digitalRead(PIN_WWVB);
	uint8_t input = fake_frame.NextBit();
	shiftSample(input);
	//printSamples();

	uint8_t score_ZERO = score(WWVB_ZERO);
	shiftScore(history_ZERO, historyLength, score_ZERO);
	uint8_t score_ONE = score(WWVB_ONE);
	shiftScore(history_ONE, historyLength, score_ZERO);
	uint8_t score_FRAME = score(WWVB_FRAME);
	shiftScore(history_FRAME, historyLength, score_ZERO);



	flashZero(score_ZERO);
	flashOne(score_ONE);
	flashFrame(score_FRAME);

	if (score_ZERO > scoreThreshold || score_ONE > scoreThreshold || score_FRAME > scoreThreshold) {
		Serial.print(score_ZERO);
		if (score_ZERO > scoreThreshold)
			Serial.print("**  ");
		else
			Serial.print("    ");
	
		Serial.print(score_ONE);
		if (score_ONE > scoreThreshold)
			Serial.print("**  ");
		else
			Serial.print("    ");

		Serial.print(score_FRAME);
		if (score_FRAME > scoreThreshold)
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

	tick();
	// Set heartbeat pin low
	PORTD &= B11111011;
}

// Diagnostic to echo sample data on the terminal. Bytes are space-separated; but
// leading zeroes are omitted; remember to mentally fill in enough zeroes on each segment to make 8 bits.
void printSamples() {
	Serial.print(samples[9], BIN);
	Serial.print(' ');
	Serial.print(samples[8], BIN);
	Serial.print(' ');
	Serial.print(samples[7], BIN);
	Serial.print(' ');
	Serial.print(samples[6], BIN);
	Serial.print(' ');
	Serial.print(samples[5], BIN);
	Serial.print(' ');
	Serial.print(samples[4], BIN);
	Serial.print(' ');
	Serial.print(samples[3], BIN);
	Serial.print(' ');
	Serial.print(samples[2], BIN);
	Serial.print(' ');
	Serial.print(samples[1], BIN);
	Serial.print(' ');
	Serial.print(samples[0], BIN);
	Serial.print('\n');
}

void flashZero(int score) {
	static int hold = 0;

	if (score > scoreThreshold) {
		hold = 20;
		ring.setPixelColor(68, FLASH);
		ring.setPixelColor(69, FLASH);
	}
	else {
		if (hold > 0)
			hold--;
		else {
			ring.setPixelColor(68, BLUE);
			ring.setPixelColor(69, PURPLE);
		}
	}
}

void flashOne(int score) {
	static int hold = 0;

	if (score > scoreThreshold) {
		hold = 20;
		ring.setPixelColor(64, FLASH);
		ring.setPixelColor(65, FLASH);
	}
	else {
		if (hold > 0)
			hold--;
		else {
			ring.setPixelColor(64, YELLOW);
			ring.setPixelColor(65, GREEN);
		}
	}
}

void flashFrame(int score) {
	static int hold = 0;

	if (score > scoreThreshold) {
		hold = 20;
		ring.setPixelColor(60, FLASH);
		ring.setPixelColor(61, FLASH);
	}
	else {
		if (hold > 0)
			hold--;
		else {
			ring.setPixelColor(60, RED);
			ring.setPixelColor(61, ORANGE);
		}
	}
}

// Shifts in a new bit sample into the array. The new bit is the LSB of the value
void shiftSample(uint8_t value) {

	// Shift sample array left one bit
	asm volatile(

		// Constraints at end of asm block:
		// Output constraints:
		// "=r" (value)		Contstraint 0: Variable "value" will be written. Use a register for it.
		// Input constraints
		// "e" (samples)	Constraint 1: Set one of X, Y, or Z base register to address of "samples".
		//					Refer to it with %a1. (This code will be inserted at top of the asm block.)
		// "0" (value)		Constraint 2: Varialbe "value" will be input to an operation. It's
		//					location must match that used for constraint 0 (same register). Refer
		//					to it as %2.

		// Shift LSB of value into Carry flag
		"lsr %2 \n\t"
		
		// Unrolled loop to shift 10 bytes.  __tmp_reg__ will be replaced with a register
		// that can be used freely, without saving or restoring its value. %a1 refers to the
		// pointer register selected by the compiler.
		"ld __tmp_reg__, %a1 \n\t"
		"rol __tmp_reg__ \n\t"
		"st %a1+, __tmp_reg__ \n\t"

		"ld __tmp_reg__, %a1 \n\t"
		"rol __tmp_reg__ \n\t"
		"st %a1+, __tmp_reg__ \n\t"

		"ld __tmp_reg__, %a1 \n\t"
		"rol __tmp_reg__ \n\t"
		"st %a1+, __tmp_reg__ \n\t"

		"ld __tmp_reg__, %a1 \n\t"
		"rol __tmp_reg__ \n\t"
		"st %a1+, __tmp_reg__ \n\t"

		"ld __tmp_reg__, %a1 \n\t"
		"rol __tmp_reg__ \n\t"
		"st %a1+, __tmp_reg__ \n\t"

		"ld __tmp_reg__, %a1 \n\t"
		"rol __tmp_reg__ \n\t"
		"st %a1+, __tmp_reg__ \n\t"

		"ld __tmp_reg__, %a1 \n\t"
		"rol __tmp_reg__ \n\t"
		"st %a1+, __tmp_reg__ \n\t"

		"ld __tmp_reg__, %a1 \n\t"
		"rol __tmp_reg__ \n\t"
		"st %a1+, __tmp_reg__ \n\t"

		"ld __tmp_reg__, %a1 \n\t"
		"rol __tmp_reg__ \n\t"
		"st %a1+, __tmp_reg__ \n\t"

		"ld __tmp_reg__, %a1 \n\t"
		"rol __tmp_reg__ \n\t"
		"st %a1+, __tmp_reg__ \n\t"

		// See above for constraint explanation
		: "=r" (value) : "e" (samples), "0" (value)
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
	for (int i=0; i<10; i++) {
		// Compute matching bits: XOR pattern and samples, and complement
		uint8_t bits = ~(samples[i] ^ pattern[i]);
		sum += parity[bits];
	}

	return sum;
}

void shiftScore(unsigned short *buffer, short int length, unsigned short score){
	for (int i=length-1;  i>0;  i--) {
		buffer[i] = buffer[i-1];
	}
	buffer[0] = score;
}

// Locates the peak position in a score buffer. A peak means while scanning over the buffer, scores are first
// increasing (really non-decreasing) to a value above the threshold, and then decreasing (really non-increasing)
// thereafter. If all scores are below the threshold, returns false; otherwise, returns true, and the peak position
// is passed out in pos_out.
bool hasPeak(unsigned short *buffer, short int length, int *pos_out) {
	short int pos = 1;

	// Find max value and position
	short int max_pos = 0;
    short int max_val = buffer[0];
	for (int i=1;  i < length;  i++) {
		if (buffer[i] > max_val) {
			max_val = buffer[i];
			max_pos = i;
		}
	}

	if (max_val < scoreThreshold)
		return false;

	// Non-increasing toward pos 0?
	if (max_pos == 0)
		return false;
	short int prior = max_val;
	short int current;
	for (int i = max_pos - 1;  i>=0;  i--) {
		current = buffer[i];
		if (current > prior)
			return false;
		prior = current;
	}

	// Non-increasing toward pos n?
	if (max_pos == length-1)
		return false;
	prior = max_val;
	for (int i = max_pos + 1;  i>length;  i++) {
		current = buffer[i];
		if (current > prior)
			return false;
		prior = current;
	}
	
	*pos_out = max_pos;
	return true;
}

// Increment the time of day
void tick() {

	tod_ticks++;
	if (tod_ticks < 60)
		return;

	tod_changed = true;
	tod_ticks = 0;
	tod_seconds++;
	
	unsigned short minute_length = 60;
	if (tod_isleapminute)
		minute_length = 61;

	if (tod_seconds < minute_length)
		return;

	tod_isleapminute = false;
	tod_seconds = 0;
	tod_minutes++;
	if (tod_minutes < 60)
		return;

	tod_minutes = 0;
	tod_hours++;
	if (tod_hours < 24)
		return;

	tod_hours = 0;
	tod_day++;

	unsigned int year_length = 365;
	if (tod_isleapyear)
		year_length = 366;
	if (tod_day < year_length)
		return;

	tod_day = 1;
	tod_year++;

}

// Set the current time of day on the hours, minutes, and seconds digits, 
// using UTC.
void updateTimeOfDayUtc() {
	setSeconds(tod_seconds);
	setMinutes(tod_minutes);
	setHours(tod_hours);
}

// Set current local time of day on the hours, minutes and seconds digits.
void updateTimeOfDayLocal() {

	unsigned short local_hours = tod_hours + tod_TZoffset;
	if (local_hours > 23) {
		local_hours -= 23;
	}

	setSeconds(tod_seconds);
	setMinutes(tod_minutes);
	setHours(local_hours);
}

// Send current nixie data
void updateNixies() {
	digitalWrite(PIN_RCK, LOW);
	SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
	for (int i=0; i<6; i++) {
		SPI.transfer(nixieData[i]);
	}
	SPI.endTransaction();
	digitalWrite(PIN_RCK, HIGH);
}

// Clear all nixie digits
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

