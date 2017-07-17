#include <SPI.h>
#include "DataGenerator.h"
#include "ScoreBoard.h"
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
// The WWVB bitstream consists of data symbols which last one second each. A full word,
// containing complete time-of-day (TOD) data, is 60 symbols, taking one minute,
// and is transmitted continuously. Each full frame gives the minute, hour, day, year
// (and some status bits) as of the moment that frame began. The frame does not include
// seconds; they are implcit. Second "0" begins at the start of the first bit of the frame.
//
// There are three possible symbol patterns within the word that can be received: a logic
// zero, a logic one, and a framing symbol that helps to determine begining and end of each
// frame. Each is a pulse of duration 1 second with a varying pulse width.
// ZERO: 0.2s high, 0.8s low
// ONE: 0.5s high, 0.5s low
// FRAME: 0.8s high, 0.2s low
// 
// The incoming bitstream is sampled at 60 Hz, and and samples are transferred to a shift register.
// Symbols are detected by comparing the stored samples with ideal templates of
// the three possible symbols. A scoring function counts up the number of
// matching bits, and on each bit shift compares the sampled data to the three templates. At
// the 60Hz sample rate, a perfect bit pattern for SYMBOL_ZERO would consist of 12 high samples,
// followed by 48 low samples.
//
// The shift register and matching patterns are 80 bits instead of only 60. This allows
// capturing and comparing not just a full pulse, but some of the preceding pulse's tail end,
// and some of the following pulse's head end as well. Including these extra transitions will
// clearly delineate a pulse, and improve noise immunity. The matching patterns include 10 extra
// bits at the front and 10 bits at the back, bracketing the 60 samples for the symbol.
// Since all three symbols begin with at least 12 high samples, and end with at least 12 low
// samples, padding the patterns with only 10 bits prevents any chance of having an extra
// transition at either end of the sample buffer because of small timing inaccuracies. Also,
// a buffer size that is a multiple of 8 bits is simpler to deal with.
//
// While the additional 20 bits do not help to distinguish one symbol from another (since they
// are the same for all symbols), they do help to distinguish a valid symbol from random
// noise.
//
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
// As code symbols are recognized, they are shifted into a symbol buffer of length 60. On each
// shift, the buffer is scored on its resemblance to a full data frame. We check each bit position,
// and score a point for each frame symbol seen in a frame slot, and a point for each non-frame
// symbol seen in a non-frame slot. When the maximum score of 60 is received, we consider that
// a match, and decode the current time of day, and update the displayed time.
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
// "fractional PLL" technique.  Count heartbeat_frac_numerator (out of
// heartbeat_frac_denominator) using heartbeat_cycles+1, and the remainder
// using heartbeat_cycles.
// For 60Hz, count two periods of 33,333 cycles for each 
// period of 33,334 cycles (with prescaler = 8, or 2,000,000Hz clock).

// No. of (whole) cycles in a short period
unsigned int heartbeat_cycles = 33333;

// Total no. of periods in full sequence; i.e., denominator of fractional part.
// A power of 2 to simplify math.
const uint8_t heartbeat_frac_denominator = 16;

// No. of long periods in full sequence, i.e. fractional part, 0/16 to 15/16
uint8_t heartbeat_frac_numerator = 5;

// Communicates to main loop that the heartbeat period changed. Set true during
// interrupt processing; reset in main loop.
volatile bool heartbeat_period_changed = false;

// First 60 pixels are the ring; the final 10 are the on-board backlight pixels
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(70, PIN_PIXEL, NEO_GRB + NEO_KHZ800);

// Colors
const uint32_t OFF = 0L;
const uint32_t SAMPLE_ONE = pixels.Color(6, 1, 0);
const uint32_t SAMPLE_ZERO = pixels.Color(5, 1, 8);
const uint32_t SAMPLE_CURSOR = pixels.Color(3, 50, 2);

const uint32_t RED = pixels.Color(255,0,0);
const uint32_t ORANGE = pixels.Color(255,50,0);
const uint32_t YELLOW = pixels.Color(255,150,0);
const uint32_t GREEN = pixels.Color(0,255,0);
const uint32_t BLUE = pixels.Color(0,0,255);
const uint32_t PURPLE = pixels.Color(128,0,128);
const uint32_t PINK = pixels.Color(60, 0, 10);
const uint32_t FLASH = pixels.Color(255, 128, 128);

// Six bytes of data for nixies
uint8_t nixieData[6];

// 80-bit long shift register for input samples.
// Offset 0, bit 0 has most recent sample bit; offset 9 bit 7
// has oldest sample bit. Shifts left. 
uint8_t volatile samples[10];

// Correlation template for a zero bit, including the trailing 0s of the preceding symbol,
// and the leading 1s of the following symbol. From head end to tail end:
// 10 0s, 12 1s, 48 0s, 10 0s.  Initialzing values start with LSB, which
// is the most recent bit (the tail end of the pulse), and progress to
// the oldest bit (the head end).  (Remember: bytes are written LSB first, but bits
// in a byte are MSB first!)
uint8_t SYMBOL_ZERO[10] = { 0xff, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0x3f, 0x00 };
// Correlaton template for one bit. From head to tail:
// 10 0s, 30 1s, 30 0s, 10 1s
uint8_t SYMBOL_ONE[10] = { 0xff, 0x03, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0x3f, 0x00 };
// Correlation template for frame bit. From head to tail:
// 10 0s, 48 1s, 12 0s, 10 1s
uint8_t SYMBOL_FRAME[10] = { 0xff, 0x03, 0xc0, 0xff, 0xff, 0xff, 0xff, 0xff, 0x3f, 0x00 };

// Pattern matching threshold
uint8_t scoreThreshold = 70;

// Score history buffers
ScoreBoard scoreboard_zero = ScoreBoard();
ScoreBoard scoreboard_one = ScoreBoard();
ScoreBoard scoreboard_frame = ScoreBoard();

// Decoded symbol stream. New symbols are shifted into position 59, and move
// toward 0. This makes the receved word symbol positions match the documentation
// for WWVB, where bit 0 is longest ago, and bit 59 is most recent.
char symbolStream[60];

// Set true by shiftSymbol; watched by main loop;
volatile bool valid_frame = false;

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

// Operating modes
const uint8_t MODE_BITSEEK = 0;
const uint8_t MODE_BITSYNC = 1;

// Current operating mode. Don't directly write it; call setMode().
volatile uint8_t mode;
volatile bool mode_changed = false;

// Prepare some fake data.  This represents 10:35am June 1, 2017.
uint8_t fakedata[] = {
	FRAME,	ZERO,	ONE,	ONE,	ZERO,	ZERO,	ONE,	ZERO,	ONE,	FRAME,  // 0-9
	ZERO,	ZERO,	ZERO,	ONE,	ZERO,	ZERO,	ONE,	ZERO,	ZERO,	FRAME,	// 10-19
	ZERO,	ZERO,	ZERO,	ONE,	ZERO,	ZERO,	ONE,	ZERO,	ONE,	FRAME,	// 20-29
	ZERO,	ZERO,	ONE,	ZERO,	ZERO,	ZERO,	ZERO,	ONE,	ZERO,	FRAME,	// 30-39
	ZERO,	ZERO,	ZERO,	ZERO,	ZERO,	ZERO,	ZERO,	ZERO,	ONE,	FRAME,	// 40-49
	ZERO,	ONE,	ONE,	ONE,	ZERO,	ZERO,	ZERO,	ONE,	ONE,	FRAME	// 50-59
};
DataGenerator fake_frame = DataGenerator(fakedata, sizeof(fakedata), 0);

// One-time setup at start
void setup() {

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

	configureTimers();

	// Configure SPI pins
	SPI.begin();

	// Configure neopixels
	pixels.begin();

	Serial.begin(230400);

	// Backlight rainbow
	pixels.setPixelColor(60, RED);
	pixels.setPixelColor(61, ORANGE);
	pixels.setPixelColor(64, YELLOW);
	pixels.setPixelColor(65, GREEN);
	pixels.setPixelColor(68, BLUE);
	pixels.setPixelColor(69, PURPLE);

	// Colons
	pixels.setPixelColor(62, PINK);
	pixels.setPixelColor(63, PINK);
	pixels.setPixelColor(66, PINK);
	pixels.setPixelColor(67, PINK);

	pixels.show();

	setTubePwm(150);

	//test_showPatterns();
	//test_shifter();

	setMode(MODE_BITSEEK);

	//adjustTickInterval(997L, 1000L);
	//adjustTickInterval(1997L, 2000L);
	//adjustTickInterval(7997L, 8000L);
}



// Main processing loop. Polls various aspects, and updates display / writes messages
// as they change.
// Time of day: Updates numeric display.
// Tick interval period: Stores in EEPROM.
// Operting mode: Write a message.
void loop() {
	if (valid_frame) {
		valid_frame = false;
		
		Serial.print("Valid frame: ");
		printSymbols();
		for (int i=0;  i<60;  i++) {
			pixels.setPixelColor(i, PINK);
		}
		Serial.print('\n');

		Serial.print("Tick interval: ");
		Serial.print(heartbeat_cycles);
		Serial.print(' ');
		Serial.print(heartbeat_frac_numerator);
		Serial.print('/');
		Serial.print(heartbeat_frac_denominator);
		Serial.print('\n');

		// Set time of day from the symbol frame, taking processing time offset.
		decodeTimeOfDay(14);
	}

	if (tod_changed) {
		tod_changed = false;
		updateTimeOfDayLocal();
		updateNixies();
	}

	if (heartbeat_period_changed) {
		heartbeat_period_changed = false;
		Serial.print ("Pretending to save new heartbeat period of ");
		Serial.print(heartbeat_cycles);
		Serial.print(' ');
		Serial.print(heartbeat_frac_numerator);
		Serial.print('/');
		Serial.print(heartbeat_frac_denominator);
		Serial.print('\n');
	}

	if (mode_changed) {
		mode_changed = false;
		Serial.print("Mode changed to ");
		switch(mode) {
			case MODE_BITSEEK:
				Serial.print("MODE_BITSEEK");
				break;
			case MODE_BITSYNC:
				Serial.print("MODE_BITSYNC");
				break;

			default:
				Serial.print("unknown mode: ");
				Serial.print(mode);			
		}
		Serial.print('\n');
	}
}

void decodeTimeOfDay(uint8_t ticksDelta) {

	// Decode the symbol word in the buffer, and set the time. Adjust by the tickDelta value.
	uint8_t minutes = 0;
	uint8_t hours = 0;
	uint8_t daynum = 0;
	uint8_t year = 2000;
	bool leapYear = false;

	// Symbols are stored as '0' and '1' characters; LSB for 0 symbol is 0, and LSB for 1 symbol is 1.
	if (symbolStream[1] & 0x01) minutes += 40;
	if (symbolStream[2] & 0x01) minutes += 20;
	if (symbolStream[3] & 0x01) minutes += 10;
	if (symbolStream[5] & 0x01) minutes += 8;
	if (symbolStream[6] & 0x01) minutes += 4;
	if (symbolStream[7] & 0x01) minutes += 2;
	if (symbolStream[8] & 0x01) minutes += 1;

	if (symbolStream[12] & 0x01) hours += 20;
	if (symbolStream[13] & 0x01) hours += 10;
	if (symbolStream[15] & 0x01) hours += 8;
	if (symbolStream[16] & 0x01) hours += 4;
	if (symbolStream[17] & 0x01) hours += 2;
	if (symbolStream[18] & 0x01) hours += 1;

	if (symbolStream[22] & 0x01) daynum += 200;
	if (symbolStream[23] & 0x01) daynum += 100;
	if (symbolStream[25] & 0x01) daynum += 80;
	if (symbolStream[26] & 0x01) daynum += 40;
	if (symbolStream[27] & 0x01) daynum += 20;
	if (symbolStream[28] & 0x01) daynum += 10;
	if (symbolStream[30] & 0x01) daynum += 8;
	if (symbolStream[31] & 0x01) daynum += 4;
	if (symbolStream[32] & 0x01) daynum += 2;
	if (symbolStream[33] & 0x01) daynum += 1;
			
	if (symbolStream[45] & 0x01) year += 80;
	if (symbolStream[46] & 0x01) year += 40;
	if (symbolStream[47] & 0x01) year += 20;
	if (symbolStream[48] & 0x01) year += 10;
	if (symbolStream[50] & 0x01) year += 8;
	if (symbolStream[51] & 0x01) year += 4;
	if (symbolStream[52] & 0x01) year += 2;
	if (symbolStream[53] & 0x01) year += 1;

	if (symbolStream[55] & 0x01) leapYear = true;

 	// Update time of day. Seconds value is implicitly 0.
	tod_ticks = ticksDelta;
	tod_seconds = 0;
	tod_minutes = minutes;
	tod_hours = hours;
	tod_day = daynum;
	tod_year = year;

	// Adjust for overflow.
	while (tod_ticks > 59) {
		tod_ticks -= 60;
		tod_seconds++;
	}
	while (tod_seconds > 59) {
		tod_seconds -= 60;
		tod_minutes++;
	}
	while (tod_minutes > 59) {
		tod_minutes -= 60;
		tod_hours++;
	}
	while (tod_hours > 23) {
		tod_hours -= 24;
		tod_day++;
	}
	if (leapYear) {
		if (tod_day > 366) {
			tod_day -= 366;
			tod_year++;
		}
		else if (tod_day > 365) {
			tod_day -= 365;
			tod_year++;
		}
	}

}

// Invoked at 60Hz by ISR. Samples incoming data bits, and processes them through the discriminators.
void tick() {

	// Set heartbeat pin high
	PORTD |= B00000100;

	// Sample the input - port D bit 7
	//bool input = (PORTD & B00000001) >> 7;
	uint8_t input = digitalRead(PIN_WWVB);
	//uint8_t input = fake_frame.nextBit();
	shiftSample(input);

	uint8_t score_ZERO = score(SYMBOL_ZERO);
	scoreboard_zero.shiftScore(score_ZERO);
	uint8_t score_ONE = score(SYMBOL_ONE);
	scoreboard_one.shiftScore(score_ONE);
	uint8_t score_FRAME = score(SYMBOL_FRAME);
	scoreboard_frame.shiftScore(score_FRAME);

	sampleToRing(input);

	pixels.show();

	switch (mode) {
		case MODE_BITSEEK:
			bitSeek();
			break;

		case MODE_BITSYNC:
			bitSync();
			break;
	}

	flashZero(score_ZERO);
	flashOne(score_ONE);
	flashFrame(score_FRAME);

	//printScores(score_ZERO, score_ONE, score_FRAME);

	// Update running time
	tickTime();

	// Set heartbeat pin low
	PORTD &= B11111011;
}

// Show incoming data sample in the ring
void sampleToRing(uint8_t sample) {
	// Indicates the active pixel
	static uint8_t pixelIndex = 0;

	if (sample)
		pixels.setPixelColor(pixelIndex, SAMPLE_ONE);
	else
		pixels.setPixelColor(pixelIndex, SAMPLE_ZERO);
	if (++pixelIndex >= 60)
		pixelIndex = 0;
	pixels.setPixelColor(pixelIndex, SAMPLE_CURSOR);
}

// Variables for MODE_BITSEEK
uint8_t bitSeek_matchCount = 0;
int bitSeek_tickCount = 0;

// Variables for MODE_BITSYNC
uint8_t bitSync_ticksRemaining = 60;
long bitSync_ticksSinceSync;

// Changes the operating mode, updating necessary variables.
void setMode(uint8_t newMode) {
	switch (newMode) {
		case MODE_BITSEEK:
			// Reset counters
			bitSeek_matchCount = 0;
			bitSeek_tickCount = 0;
			setColonColor(PINK);
			break;

		case MODE_BITSYNC:
			bitSync_ticksRemaining = 60;
			bitSync_ticksSinceSync = 0;
			setColonColor(BLUE);
			break;
	}

	mode = newMode;
	mode_changed = true;
}

// Invoked on each tick when in MODE_BITSEEK. Look for 6 consecutive successful bits. If a bit
// interval elapses without a bit, reset the match count to 0.
void bitSeek() {
	uint8_t maxScore;
	uint8_t maxIndex;

	static uint8_t timeout = 65;
	bitSeek_tickCount++;

	if (--timeout == 0) {
		// No bit seen. Reset.
		bitSeek_matchCount = 0;
		timeout = 60;
	}

	// A successful bit has peak in slot 4 of 7
	if (scoreboard_zero.maxOverThreshold(scoreThreshold, &maxScore, &maxIndex)) {
		if (maxIndex == 4) {
			bitSeek_matchCount++;
			timeout = 65;
			shiftSymbol('0');
		}
	}
	else if (scoreboard_one.maxOverThreshold(scoreThreshold, &maxScore, &maxIndex)) {
		if (maxIndex == 4) {
			bitSeek_matchCount++;
			timeout = 65;
			shiftSymbol('1');
		}
	}
	else if (scoreboard_frame.maxOverThreshold(scoreThreshold, &maxScore, &maxIndex)) {
		if (maxIndex == 4) {
			bitSeek_matchCount++;
			timeout = 65;
			shiftSymbol('F');
		}
	}

	if (bitSeek_matchCount == 6) {
		Serial.print("Found 6 consecutive symbols.\n");

		setMode(MODE_BITSYNC);
	}
}

// Invoked on each tich when in MODE_BITSYNC. Let 60 ticks elapse, and look for a
// symbol match. If max scores slot index changes, indicating drift, recalibrate the tick
// time interval.
// If we fail to see a symbol for 30 counts, switch back to MODE_BITSEEK.
void bitSync() {
	static uint8_t failCountdown = 30;

	bitSync_ticksSinceSync++;
	if (--bitSync_ticksRemaining > 0)
		return;

	// Look for next symbol.
	uint8_t maxScore = 0;
	uint8_t maxIndex = 4;

	if (scoreboard_zero.maxOverThreshold(scoreThreshold, &maxScore, &maxIndex)) {
		shiftSymbol('0');
		failCountdown = 30;
	}
	else if (scoreboard_one.maxOverThreshold(scoreThreshold, &maxScore, &maxIndex)) {
		shiftSymbol('1');
		failCountdown = 30;
	}
	else if (scoreboard_frame.maxOverThreshold(scoreThreshold, &maxScore, &maxIndex)) {
		shiftSymbol('F');
		failCountdown = 30;
	}
	else {
		// No symbol seen.
		shiftSymbol('X');
		if (--failCountdown) {
			setMode(MODE_BITSEEK);
		}
		return;
	}

	// Are we getting out of sync?  A peak in the middle slot is right on time; a peak
	// in a different slot means the local oscillator is running fast or slow compared
	// to the reference.  When we have accumulated 3 ticks of drift, update the
	// tick interval timer accordingly.  Waiting until 3 ticks prevents updating
	// too often.
	switch (maxIndex) {
		case 0:
			// Symbol arrived 3 slots early. Tick period too long. Shorten.
			Serial.print(" LONG ");

			adjustTickInterval(bitSync_ticksSinceSync, bitSync_ticksSinceSync+3);

			// Re-sync by waiting 3 extra ticks before next check.
			bitSync_ticksRemaining = 63;
			bitSync_ticksSinceSync = 0L;
			break;

		case 7:
			// Symbol arrived 3 slots late. Tick period too short. Lengthen.
			Serial.print(" SHORT ");

			adjustTickInterval(bitSync_ticksSinceSync, bitSync_ticksSinceSync-3);

			// Re-sync by waiting 3 fewer ticks before next check.
			bitSync_ticksRemaining = 57;
			bitSync_ticksSinceSync = 0L;
			break;

		default:
			// Symbol peak found within +- 2 ticks of expected. That's OK.
			bitSync_ticksRemaining = 60;
	}

}




void shiftSymbol(char newSymbol) {
	uint8_t score = 0;

//	Serial.print("Shifting ");
//	Serial.print(newSymbol);
//	Serial.print('\n');
//	printSymbols();

	// Single loop for shifting and scoring. Only check that positions that should have
	// frame symbol have them, and that non-frame positions do not.

	// When framePos = 0, we're shifting into a frame position. It gets reset to 9
	// after processing a frame position.
	uint8_t framePosCountdown = 10;
	for (uint8_t i=0;  i<59;  i++) {
		framePosCountdown--;
		char symbol = symbolStream[i+1];
		symbolStream[i] = symbol;
		if (i == 0) {
			// Should be a frame symbol.
			if (symbol == 'F')
				score++;
		}
		if (framePosCountdown == 0) {
			// Should be a frame symbol.
			if (symbol == 'F')
				score++;
			framePosCountdown = 10;
		}
		else {
			// Should be a 1 or 0.
			if (symbol == '0'  ||  symbol == '1')
				score++;
		}
	}

	symbolStream[59] = newSymbol;
	if (newSymbol == 'F' )
		score++;

	//Serial.print("Symbol score: ");
	//Serial.print(score);
	//Serial.print('\n');

	if (score == 60) {
		valid_frame = true;
	}
}




// Udpate the tick interval to compensate for counting actualTicks while
// expecting to count expectedTicks.  Both parameters should be near each
// other (+-3). When they are above 8000, the math would overflow a 32-bit
// integer, so an alternate technique is needed.d
void adjustTickInterval(unsigned long actualTicks, unsigned long expectedTicks) {

	Serial.print("Adjusting tick interval\n  Actual ticks: ");
	Serial.print(actualTicks);
	Serial.print('\n');
	Serial.print("  Expected ticks: ");
	Serial.print(expectedTicks);
	Serial.print('\n');

	// Combine whole cycles and fraction into scaled integer
	unsigned long scaledCounts = (unsigned long)heartbeat_cycles * heartbeat_frac_denominator + heartbeat_frac_numerator;
	Serial.print("  Scaled counts: ");
	Serial.print(scaledCounts);
	Serial.print('\n');

	unsigned long updatedCounts;

	if (expectedTicks < 8000) {
		updatedCounts = (((unsigned long)scaledCounts) * actualTicks) / expectedTicks;
		Serial.print("  Linear update: ");
		Serial.print(updatedCounts);
		Serial.print('\n');
	}
	else {
		// Need to implement "logarithmic" update here
		Serial.print("  Logarithmic update: Didn't implement this yet!");
	}

	// Convert back to whole cycles and fraction.
	uint8_t rem = updatedCounts % heartbeat_frac_denominator;
	uint16_t quot = updatedCounts / heartbeat_frac_denominator;

	Serial.print("  New period: ");
	Serial.print(quot);
	Serial.print(" rem. ");
	Serial.print(rem);
	Serial.print('\n');

	heartbeat_cycles = quot;
	heartbeat_frac_numerator = rem;

}

void flashZero(int score) {
	static int hold = 0;

	if (score > scoreThreshold) {
		hold = 20;
		pixels.setPixelColor(68, FLASH);
		pixels.setPixelColor(69, FLASH);
	}
	else {
		if (hold > 0)
			hold--;
		else {
			pixels.setPixelColor(68, BLUE);
			pixels.setPixelColor(69, PURPLE);
		}
	}
}

void flashOne(int score) {
	static int hold = 0;

	if (score > scoreThreshold) {
		hold = 20;
		pixels.setPixelColor(64, FLASH);
		pixels.setPixelColor(65, FLASH);
	}
	else {
		if (hold > 0)
			hold--;
		else {
			pixels.setPixelColor(64, YELLOW);
			pixels.setPixelColor(65, GREEN);
		}
	}
}

void flashFrame(int score) {
	static int hold = 0;

	if (score > scoreThreshold) {
		hold = 20;
		pixels.setPixelColor(60, FLASH);
		pixels.setPixelColor(61, FLASH);
	}
	else {
		if (hold > 0)
			hold--;
		else {
			pixels.setPixelColor(60, RED);
			pixels.setPixelColor(61, ORANGE);
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
		// "0" (value)		Constraint 2: Varialbe "value" will be input to an operation. Its
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
void tickTime() {

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

// Print the scores over the serial port.
void printScores(uint8_t zero, uint8_t one, uint8_t frame) {
	static bool separated = false;
	if (zero > scoreThreshold || one > scoreThreshold || frame > scoreThreshold) {
		Serial.print(zero);
		if (zero > scoreThreshold)
			Serial.print("**  ");
		else
			Serial.print("    ");
	
		Serial.print(one);
		if (one > scoreThreshold)
			Serial.print("**  ");
		else
			Serial.print("    ");

		Serial.print(frame);
		if (frame > scoreThreshold)
			Serial.print("**\n");
		else
			Serial.print("\n");

		separated = false;
	}
	else {
		if (!separated) {
			Serial.print("----------\n");
			separated = true;
		}
	}
}

void setColonColor(uint32_t color) {
	pixels.setPixelColor(62, color);
	pixels.setPixelColor(63, color);
	pixels.setPixelColor(66, color);
	pixels.setPixelColor(67, color);
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

void configureTimers() {

	// Configure timer 1 to interrupt at 60Hz
 	OCR1A = heartbeat_cycles;

	TCCR1A = 0;
    // Mode 4, CTC on OCR1A, prescaler 8
    TCCR1B = (1 << WGM12) | (1 << CS11);

    //Set interrupt on compare match
    TIMSK1 |= (1 << OCIE1A);

	// Configure timer 2 for PWM at 244Hz
	// Arduino pin 3 is OC2B
	TCCR2A = _BV(COM2B1) | _BV(COM2B0) | _BV(WGM21) | _BV(WGM20);
 	TCCR2B = _BV(CS22) | _BV(CS21);
	OCR2B = 255;
}

// Heartbeat interrupt.
ISR (TIMER1_COMPA_vect) {
	// Current fractional period index. Counts from 0 to heartbeat_frac_denominator-1.
	// When heartbeat_period < heartbeat_frac_numerator, set counter for a long
	// period of heartbeat_cycles+1 counts; otherwise, a short period of heartbeat_cycles.
	static uint8_t fraction_counter = 0;

	if (++fraction_counter >= heartbeat_frac_denominator)
		fraction_counter = 0;

	// Reset the CTC value. For a short period, count CTC_value cycles;
	// for a long period, count CTC_value+1. 
    // Set compare value for a short or long cycle. Must set OCR1A to n-1.
	if (fraction_counter < heartbeat_frac_numerator) {
		// Long period: n+1-1
		OCR1A = heartbeat_cycles;
	}
	else {
		// Short period: n-1
		OCR1A = heartbeat_cycles-1;
	}

	tick();
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

// Diagnositc to echo the decoded symbols in the symbol buffer.
void printSymbols() {
	for (int i = 0;  i<60;  i++) {
		Serial.print(symbolStream[i]);
	}
	Serial.print('\n');
}

void test_showPatterns() {

	Serial.print("SYMBOL_ZERO: ");
	Serial.print(SYMBOL_ZERO[9], BIN);
	Serial.print(' ');
	Serial.print(SYMBOL_ZERO[8], BIN);
	Serial.print(' ');
	Serial.print(SYMBOL_ZERO[7], BIN);
	Serial.print(' ');
	Serial.print(SYMBOL_ZERO[6], BIN);
	Serial.print(' ');
	Serial.print(SYMBOL_ZERO[5], BIN);
	Serial.print(' ');
	Serial.print(SYMBOL_ZERO[4], BIN);
	Serial.print(' ');
	Serial.print(SYMBOL_ZERO[3], BIN);
	Serial.print(' ');
	Serial.print(SYMBOL_ZERO[2], BIN);
	Serial.print(' ');
	Serial.print(SYMBOL_ZERO[1], BIN);
	Serial.print(' ');
	Serial.print(SYMBOL_ZERO[0], BIN);
	Serial.print('\n');

	Serial.print("SYMBOL_ONE: ");
	Serial.print(SYMBOL_ONE[9], BIN);
	Serial.print(' ');
	Serial.print(SYMBOL_ONE[8], BIN);
	Serial.print(' ');
	Serial.print(SYMBOL_ONE[7], BIN);
	Serial.print(' ');
	Serial.print(SYMBOL_ONE[6], BIN);
	Serial.print(' ');
	Serial.print(SYMBOL_ONE[5], BIN);
	Serial.print(' ');
	Serial.print(SYMBOL_ONE[4], BIN);
	Serial.print(' ');
	Serial.print(SYMBOL_ONE[3], BIN);
	Serial.print(' ');
	Serial.print(SYMBOL_ONE[2], BIN);
	Serial.print(' ');
	Serial.print(SYMBOL_ONE[1], BIN);
	Serial.print(' ');
	Serial.print(SYMBOL_ONE[0], BIN);
	Serial.print('\n');

	Serial.print("SYMBOL_FRAME: ");
	Serial.print(SYMBOL_FRAME[9], BIN);
	Serial.print(' ');
	Serial.print(SYMBOL_FRAME[8], BIN);
	Serial.print(' ');
	Serial.print(SYMBOL_FRAME[7], BIN);
	Serial.print(' ');
	Serial.print(SYMBOL_FRAME[6], BIN);
	Serial.print(' ');
	Serial.print(SYMBOL_FRAME[5], BIN);
	Serial.print(' ');
	Serial.print(SYMBOL_FRAME[4], BIN);
	Serial.print(' ');
	Serial.print(SYMBOL_FRAME[3], BIN);
	Serial.print(' ');
	Serial.print(SYMBOL_FRAME[2], BIN);
	Serial.print(' ');
	Serial.print(SYMBOL_FRAME[1], BIN);
	Serial.print(' ');
	Serial.print(SYMBOL_FRAME[0], BIN);
	Serial.print('\n');
}

void test_shifter() {
	// Shift in a simulated SYMBOL_ZERO, then compare to the other patterns
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
	Serial.print(score(SYMBOL_ZERO));
	Serial.print("\nZERO on ONE: ");
	Serial.print(score(SYMBOL_ONE));
	Serial.print("\nZERO on FRAME: ");
	Serial.print(score(SYMBOL_FRAME));
	Serial.print("\n");

	// Shift in a simulated SYMBOL_ONE, then compare to the other patterns
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
	Serial.print(score(SYMBOL_ZERO));
	Serial.print("\nONE on ONE: ");
	Serial.print(score(SYMBOL_ONE));
	Serial.print("\nONE on FRAME: ");
	Serial.print(score(SYMBOL_FRAME));
	Serial.print("\n");

	// Shift in a simulated SYMBOL_FRAME, then compare to the other patterns
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
	Serial.print(score(SYMBOL_ZERO));
	Serial.print("\nFRAME on ONE: ");
	Serial.print(score(SYMBOL_ONE));
	Serial.print("\nFRAME on FRAME: ");
	Serial.print(score(SYMBOL_FRAME));
	Serial.print("\n");
}

