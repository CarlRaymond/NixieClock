#include <SPI.h>
#include <EEPROM.h>
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
// MARKER: 0.8s high, 0.2s low
// 
// The incoming bitstream is sampled at 60 Hz, and and samples are transferred to a shift register.
// Symbols are detected by comparing the stored samples with ideal templates of
// the three possible symbols. A scoring function counts up the number of
// matching bits, and on each bit shift compares the sampled data to the three templates. At
// the 60Hz sample rate, a perfect bit pattern for PATTERN_ZERO would consist of 12 high samples,
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
// the score history. The history buffer size, n, is an odd number (11), and
// the peak finder looks for the case where the peak value is in the middle slot.
//
// At this point, the data bits can be recognized by the appearance of the peaks in the three
// symbol scoring buffers. Right after receiving a nice clean bit, one of the three buffers should
// show a score above the threshold, with a peak value. The other two buffers should show scores
// below the threshold, and not sharply peaked. (The threshold is chosen so that only one score buffer
// at a time can have a score above the threshold.) After 60 ticks (one second), the following bit
// will have been recieved, and one of the three buffers should show a peak, ideally in the same
// position within the buffer.  If the receiver should miss a bit, no buffer will show a peak.
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

// PWM for tube brightness
const int PIN_PWM = 3;

// HV supply /enable
const int PIN_HV = 5;

// Ring serial output
const int PIN_PIXEL = 6;

// SYMTRIK input pin
const int PIN_WWVB = 7; // PORTD bit 7

// Nixie tube register clock (RCK)
const int PIN_RCK = 8;

// SPI pins for Nixie tube data and SRCK
const int PIN_MOSI = 11;
const int PIN_MISO = 12; // Unused
const int PIN_SCK = 13;

// Version number for parameters structure.
const int parametersVersion = 2;

// Parameters for clock that get saved in EEPROM. Version number helps with sanity checking.
typedef struct {
	uint8_t version;
	uint32_t scaledCounts;
} PersistentParameters;

// Set true to prevent loading stored parameters at startup, and storing adjusted parameters.
bool overrideSavedParameters = false;

// Heartbeat counter. Timer1 will produce interrupts at 60Hz, using
// "fractional PLL" technique.  Count tick_frac_numerator (out of
// tick_frac_denominator) intervals using tick_interval_cycles+1, and the remainder
// using tick_interval_cycles. The denominator of the fraction is 64, effectively
// adding 6 bits of resolution to the counter (from 16 to 22 bits).
// For 60Hz (with prescaler = 8, or 2,000,000Hz clock), count 33,333 1/3
// cycles. The closest frational value is 33,333 21/64.
volatile uint16_t tick_interval_cycles = 33333;  // Whole cycles
volatile uint8_t tick_frac_numerator = 21;  // Numerator of fraction (no. of long cycles)
const uint8_t tick_frac_denominator = 64;  // Denominator of fraction (no. of long + no. of short cycles), always power of 2

// Communicates to main loop that the heartbeat period changed. Set true during
// interrupt processing; reset in main loop after saving parameters.
volatile bool unsaved_parameters = false;

// Set true by adjustTickInterval; monitored and reset by main loop.
volatile bool tick_interval_changed = false;

// First 60 pixels are the ring; the final 10 are the on-board backlight pixels
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(70, PIN_PIXEL, NEO_GRB + NEO_KHZ800);

// Holds input samples
uint8_t sampleBuffer[60];
uint8_t sampleIndex;

// Colors
const uint32_t OFF = 0L;
const uint32_t COLOR_SAMPLE_ONE = pixels.Color(7, 1, 0);
const uint32_t COLOR_SAMPLE_ZERO = pixels.Color(3, 1, 6);
const uint32_t COLOR_SAMPLE_CURSOR = pixels.Color(3, 12, 6);

const uint32_t COLOR_SYMBOL_ZERO = pixels.Color(0, 0, 6);
const uint32_t COLOR_SYMBOL_ONE = pixels.Color(5, 1, 0);
const uint32_t COLOR_SYMBOL_MARKER = pixels.Color(1, 3, 1);

const uint32_t COLOR_HAND_HOUR = pixels.Color(40, 0, 0);
const uint32_t COLOR_HAND_MINUTE = pixels.Color(30, 18, 0);

const uint32_t COLOR_SYNC = pixels.Color(6, 0, 6);

const uint32_t COLOR_BACKGROUND = pixels.Color(1, 4, 1);
const uint32_t COLOR_RED = pixels.Color(255,0,0);
const uint32_t COLOR_ORANGE = pixels.Color(204,51,0);
const uint32_t COLOR_YELLOW = pixels.Color(255,150,0);
const uint32_t COLOR_GREEN = pixels.Color(0,255,0);
const uint32_t COLOR_BLUE = pixels.Color(0,0,255);
const uint32_t COLOR_PURPLE = pixels.Color(96,0,96);
const uint32_t COLOR_PINK = pixels.Color(60, 0, 10);
const uint32_t COLOR_FLASH = pixels.Color(255, 128, 128);

// Six bytes of data for nixies
volatile uint8_t nixieData[6];

// 80-bit long shift register for input samples.
// Offset 0, bit 0 has most recent sample bit; offset 9 bit 7
// has oldest sample bit. Shifts left. 
volatile uint8_t samples[10];

// Correlation template for a zero bit, including the trailing 0s of the preceding symbol,
// and the leading 1s of the following symbol. From head end to tail end:
// 10 zeroes, 12 ones, 48 zeroes, 10 ones.  Initialzing values start with LSB, which
// is the most recent bit (the tail end of the pulse), and progress to
// the oldest bit (the head end).  (Bytes are written LSB first, but bits in a byte are
// MSB first!  If you get confused, print this out and read it in a mirror.)
uint8_t PATTERN_ZERO[10] = { 0xff, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0x3f, 0x00 };
// Correlaton template for one bit. From head to tail:
// 10 zeroes, 30 ones, 30 zeroes, 10 ones
uint8_t PATTERN_ONE[10] = { 0xff, 0x03, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0x3f, 0x00 };
// Correlation template for marker bit. From head to tail:
// 10 zeroes, 48 oness, 12 zeroes, 10 ones
uint8_t PATTERN_MARKER[10] = { 0xff, 0x03, 0xc0, 0xff, 0xff, 0xff, 0xff, 0xff, 0x3f, 0x00 };

// Pattern matching threshold
uint8_t scoreThreshold = 69;

// Score history buffers
ScoreBoard scoreboard_zero = ScoreBoard();
ScoreBoard scoreboard_one = ScoreBoard();
ScoreBoard scoreboard_marker = ScoreBoard();

// Decoded symbol stream. New symbols are shifted into position 59, and move
// toward 0. This makes the receved word symbol positions match the documentation
// for WWVB (i.e., Wikipedia), where bit 0 is longest ago, and bit 59 is most recent.
char symbolStream[60];

// Current time of day, UTC
volatile unsigned short tod_ticks = 0;
volatile unsigned short tod_seconds = 0;
volatile unsigned short tod_minutes = 0;
volatile unsigned short tod_hours = 0;
volatile unsigned int tod_day = 0;
volatile unsigned int tod_year = 0;
volatile bool tod_isdst = true;
volatile bool tod_isleapminute = false;
volatile bool tod_isleapyear = false;
volatile bool tod_changed = false;

// Set true when time has been decoded
volatile bool tod_fix = false;

// Set your time zone here!
int8_t tzOffsetHours = -4;
int8_t tzOffsetMinutes = 0;

// Operating modes
const uint8_t MODE_SEEK = 0;
const uint8_t MODE_SYNC = 1;

// Current operating mode. Don't directly write it; call setMode().
volatile uint8_t mode;
volatile bool mode_changed = false;

// State variables for MODE_SEEK
uint8_t bitSeek_detectedSymbolCount = 0;			// No. of symbols detected since entering state
const uint8_t bitSeek_detectedSymbolThreshold = 10;	// No. of detected symbols needed to change state

// State variables for MODE_SYNC
uint8_t bitSync_peekCountdown = 60;		// No. of ticks to wait before peeking at scoreboard for symbol
uint32_t bitSync_localTicksSinceSync = 0;
int16_t bitSync_accumulatedOffset = 0;
uint8_t bitSync_missedSymbolCount = 0;				// No. of consecutive symbols missed
const uint8_t bitSync_missedSymbolThreshold = 6;	// No. of missed symbols needed to change state
bool bitSync_parametersSaved = false;	// Set true when current parameters saved to EEPROM

// Prepare some fake data.  This represents 10:35am June 1, 2017.
uint8_t fakedata[] = {
	MARKER,	ZERO,	ONE,	ONE,	ZERO,	ZERO,	ONE,	ZERO,	ONE,	MARKER,  // 0-9
	ZERO,	ZERO,	ZERO,	ONE,	ZERO,	ZERO,	ONE,	ZERO,	ZERO,	MARKER,	// 10-19
	ZERO,	ZERO,	ZERO,	ONE,	ZERO,	ZERO,	ONE,	ZERO,	ONE,	MARKER,	// 20-29
	ZERO,	ZERO,	ONE,	ZERO,	ZERO,	ZERO,	ZERO,	ONE,	ZERO,	MARKER,	// 30-39
	ZERO,	ZERO,	ZERO,	ZERO,	ZERO,	ZERO,	ZERO,	ZERO,	ONE,	MARKER,	// 40-49
	ZERO,	ONE,	ONE,	ONE,	ZERO,	ZERO,	ZERO,	ONE,	ONE,	MARKER	// 50-59
};
DataGenerator fake_frame = DataGenerator(fakedata, sizeof(fakedata), 0);

// Set true in tick(); watched and reset by main loop.
volatile bool update_pixels_flag;

// Set true by shiftSymbol(); watched and reset by main loop.
volatile bool valid_frame_flag = false;

// set true by bitSync(); watched and reset by main loop.
volatile bool new_parameters_flag = false;


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

	configureTickTimer();
	configurePwmTimer();

	// Configure SPI pins
	SPI.begin();

	// Configure neopixels
	pixels.begin();
	pixels.show();

	Serial.begin(230400);

	setTubePwm(170);

	if (!overrideSavedParameters) {
		configureFromMemory();
		Serial.print("Using stored parameters: ");
		Serial.print(tick_interval_cycles);
		Serial.print(' ');
		Serial.print(tick_frac_numerator);
		Serial.print('/');
		Serial.print(tick_frac_denominator);
		Serial.print('\n');
	}
	else {
		Serial.print("Overriding parameters: ");
		Serial.print(tick_interval_cycles);
		Serial.print(' ');
		Serial.print(tick_frac_numerator);
		Serial.print('/');
		Serial.print(tick_frac_denominator);
		Serial.print('\n');
	}

	setMode(MODE_SEEK);
}

// Main processing loop. Polls various aspects, and updates display / writes messages
// as they change.
// Time of day: Updates numeric display.
// Tick interval period: Stores in EEPROM.
// Operting mode: Write a message.
void loop() {
	if (valid_frame_flag) {
		valid_frame_flag = false;
		
		Serial.print("Valid frame!\n");
		for (int i=0;  i<60;  i++) {
			pixels.setPixelColor(i, COLOR_BLUE);
		}
		Serial.print('\n');

		Serial.print("Tick interval: ");
		Serial.print(tick_interval_cycles);
		Serial.print(' ');
		Serial.print(tick_frac_numerator);
		Serial.print('/');
		Serial.print(tick_frac_denominator);
		Serial.print('\n');

		// Set time of day from the symbol frame, taking processing time offset into account.
		decodeTimeOfDay(10 + ScoreBoard::centerIndex);
		tod_fix = true;

		printTimeUtc();
	}

	if (tod_changed) {
		if (tod_fix)
			updateTimeOfDayLocal(tzOffsetHours, tzOffsetMinutes, true);
		else
			updateTimeOfDayUtc();

		updateNixies();
		tod_changed = false;
	}

	//if (mode_changed) {
	//	mode_changed = false;
	//	Serial.print("Mode changed to ");
	//	switch(mode) {
	//		case MODE_SEEK:
	//			Serial.print("MODE_SEEK");
	//			break;
	//		case MODE_SYNC:
	//			Serial.print("MODE_SYNC");
	//			break;
//
	//		default:
	//			Serial.print("unknown mode: ");
	//			Serial.print(mode);			
	//	}
	//	Serial.print('\n');
	//}

	if (tick_interval_changed) {
		tick_interval_changed = false;
		Serial.print("New tick interval: ");
		Serial.print(tick_interval_cycles);
		Serial.print(' ');
		Serial.print(tick_frac_numerator);
		Serial.print('/');
		Serial.print(tick_frac_denominator);
		Serial.print('\n');
	}

	if (unsaved_parameters) {
		// Have we gone long enough to save the parameters?
		if (bitSync_localTicksSinceSync > 500000) {
			PersistentParameters params;
			params.version = parametersVersion;
			params.scaledCounts = (uint32_t) tick_interval_cycles * tick_frac_denominator + tick_frac_numerator;
			saveParameters(0, &params);
			bitSync_parametersSaved = true;
			Serial.print("Saved parameters to EEPROM.\n");
			unsaved_parameters = false;
		}
	}

	if (update_pixels_flag) {
		updatePixels();
		update_pixels_flag = false;
	}
}


void updatePixels() {
	uint32_t color;

	switch (mode) {
		case MODE_SEEK:
			// Show incoming samples.
			for (uint8_t i = 0;  i<60;  i++) {
				if (sampleIndex == i)
					color = COLOR_SAMPLE_CURSOR;
				else {
					if (sampleBuffer[i])
						color = COLOR_SAMPLE_ONE;
					else
						color = COLOR_SAMPLE_ZERO;
				}
				pixels.setPixelColor(i, color);
			}
			break;

		case MODE_SYNC:
			// Show received data bits.
			for (uint8_t i = 0;  i<60;  i++) {
				switch(symbolStream[i]) {
					case '0':
						color = COLOR_SYMBOL_ZERO;
						break;
					case '1':
						color = COLOR_SYMBOL_ONE;
						break;
					case 'M':
						color = COLOR_SYMBOL_MARKER;
						break;
					default:
						color = OFF;
				}

				pixels.setPixelColor(i, color);
			}

			break;
	}

	if (tod_fix) {
		// Get local time in AM/PM form
		int8_t localHours = tod_hours + tzOffsetHours;
		int8_t localMinutes = tod_minutes + tzOffsetMinutes;
		if (localMinutes < 0) {
			localMinutes += 60;
			localHours--;
		}
		if (localMinutes > 59) {
			localMinutes -= 60;
			localHours++;
		}
		if (localHours < 0) {
			localHours += 12;
		}
		if (localHours > 11) {
			localHours -= 12;
		}

		// Superimpose hour hand
		uint8_t hourPos = localHours * 5 + (localMinutes / 12) + 21;
		if (hourPos > 59)
			hourPos -= 60;
		pixels.setPixelColor(hourPos, COLOR_HAND_HOUR);
		if (++hourPos > 59)
			hourPos -= 60;
		pixels.setPixelColor(hourPos, COLOR_HAND_HOUR);
		if (++hourPos > 59)
			hourPos -= 60;
		pixels.setPixelColor(hourPos, COLOR_HAND_HOUR);


		// Superimpose minute hand
		uint8_t minutePos = localMinutes + 21;
		if (minutePos < 0)
			minutePos += 60;
		if (minutePos > 59)
			minutePos -= 60;
		pixels.setPixelColor(minutePos, COLOR_HAND_MINUTE);
		if (++minutePos > 59)
			minutePos -= 60;
		pixels.setPixelColor(minutePos, COLOR_HAND_MINUTE);
		if (++minutePos > 59)
			minutePos -= 60;
		pixels.setPixelColor(minutePos, COLOR_HAND_MINUTE);

	}

	// Superimpose sync offset
	if (bitSync_accumulatedOffset < 0) {
		for (uint8_t i = 22+bitSync_accumulatedOffset;  i <= 22;  i++)
			pixels.setPixelColor(i, COLOR_SYNC);
	}
	else if (bitSync_accumulatedOffset > 0) {
		for (uint8_t i = 22;  i <= 22+bitSync_accumulatedOffset;  i++)
			pixels.setPixelColor(i, COLOR_SYNC);
	}
	else {
		pixels.setPixelColor(22, COLOR_SYNC);
	}
}


// Invoked at 60Hz by ISR. Samples incoming data bits, and processes them through the discriminators.
void tick() {
	// Sample the input - port D bit 7
	uint8_t input = (PIND & B10000000) >> 7;
	//uint8_t input = fake_frame.nextBit();
	shiftSample(input);

	uint8_t score_ZERO = score(PATTERN_ZERO);
	scoreboard_zero.shiftScore(score_ZERO);
	uint8_t score_ONE = score(PATTERN_ONE);
	scoreboard_one.shiftScore(score_ONE);
	uint8_t score_MARKER = score(PATTERN_MARKER);
	scoreboard_marker.shiftScore(score_MARKER);

	sampleToBuffer(input);

	pixels.show();

	// Hoisted out of MODE_SYNC to keep sync count across all modes
	bitSync_localTicksSinceSync++;


	switch (mode) {
		case MODE_SEEK:
			bitSeek();
			break;

		case MODE_SYNC:
			bitSync();
			break;
	}

	flashZero(score_ZERO);
	flashOne(score_ONE);
	flashMarker(score_MARKER);

	// Update running time
	tickTime();

	update_pixels_flag = true;
}

// Copy new sample to the buffer, and update the index.
void sampleToBuffer(uint8_t value) {
	sampleBuffer[sampleIndex] = value;
	sampleIndex++;
	if (sampleIndex >= 60)
		sampleIndex = 0;
}

// Change the operating mode, updating necessary variables.
void setMode(uint8_t newMode) {
	switch (newMode) {
		case MODE_SEEK:
			// Reset counters
			bitSeek_detectedSymbolCount = 0;
			setColonColor(COLOR_PINK);
			break;

		case MODE_SYNC:
			bitSync_peekCountdown = 60;
			//bitSync_localTicksSinceSync = 0L;
			//bitSync_accumulatedOffset = 0;
			bitSync_parametersSaved = false;
			bitSync_missedSymbolCount = 0;
			setColonColor(COLOR_BLUE);
			break;
	}

	mode = newMode;
	mode_changed = true;
}

// Invoked on each tick when in MODE_SEEK. Look for multiple successful symbols.
// We see a symbol when one of the scoreboards shows a peak value in the center slot. No
// attempt to detect missing symbols.  When we hit the threshold, switch to MODE_SYNC.
void bitSeek() {
	uint8_t peakScore;
	uint8_t peakIndex;

	char detectedSymbol = 0;

	// A successful bit has peak in the middle slot. When the peak is elsewhere, ignore it.
	if (scoreboard_zero.maxOverThreshold(scoreThreshold, &peakScore, &peakIndex)) {
		if (peakIndex == ScoreBoard::centerIndex) detectedSymbol = '0';
	}
	else if (scoreboard_one.maxOverThreshold(scoreThreshold, &peakScore, &peakIndex)) {
		if (peakIndex == ScoreBoard::centerIndex) detectedSymbol = '1';
	}
	else if (scoreboard_marker.maxOverThreshold(scoreThreshold, &peakScore, &peakIndex)) {
		if (peakIndex == ScoreBoard::centerIndex) detectedSymbol = 'M';
	}

	// Any symbol seen?
	if (detectedSymbol != 0) {
		bitSeek_detectedSymbolCount++;
		shiftSymbol(detectedSymbol);
	}

	// Enough symbos in a row?
	if (bitSeek_detectedSymbolCount == bitSeek_detectedSymbolThreshold) {
		// Commence syncin' proper.
		setMode(MODE_SYNC);
	}
}

// Invoked on each tick when in MODE_SYNC. Let 60 (+- offset) ticks elapse, and look for a
// symbol match. Detect and accumulate drift (when a symbol arrives in an off-center slot)
// and recalibrate the tick timer interval when a drift threshold is exceeded.
// If we hit a threshold of missed symbols, switch back to MODE_SEEK.
void bitSync() {

	if (--bitSync_peekCountdown > 0)
		return;

	// Look for next symbol.
	uint8_t peakScore = 0;
	uint8_t peakIndex = ScoreBoard::centerIndex;

	char detectedSymbol = 0;

	if (scoreboard_zero.maxOverThreshold(scoreThreshold, &peakScore, &peakIndex)) {
		detectedSymbol = '0';
	}
	else if (scoreboard_one.maxOverThreshold(scoreThreshold, &peakScore, &peakIndex)) {
		detectedSymbol = '1';
	}
	else if (scoreboard_marker.maxOverThreshold(scoreThreshold, &peakScore, &peakIndex)) {
		detectedSymbol = 'M';
	}
	else {
		// No symbol seen.
		shiftSymbol('-');
		if (++bitSync_missedSymbolCount == bitSync_missedSymbolThreshold) {
			// Sync lost.
			setMode(MODE_SEEK);
			return;
		}

		//Serial.print("Missed symbol\n");

		// Try again next second.
		bitSync_peekCountdown = 60;
		return;
	}

	// Saw a symbol.
	shiftSymbol(detectedSymbol);
	bitSync_missedSymbolCount = 0;

	// Are we getting out of sync?  A peak in the middle slot is right on time; a peak
	// in a different slot means the local oscillator is running fast or slow compared
	// to the reference.  Accumulate this delta over multiple cycles.  When the delta
	// reaches a limit, adjust the tick counter interval.

	int8_t offset = ScoreBoard::centerIndex - peakIndex;

	bitSync_accumulatedOffset += offset;

	if (offset != 0) {
		Serial.print("Accumulated offset: ");
		Serial.print(bitSync_accumulatedOffset);
		Serial.print(" , ticks since sync: ");
		Serial.print(bitSync_localTicksSinceSync);
		Serial.print('\n');
	}

	// Next time, peek when next symbol should be centered
	bitSync_peekCountdown = 60 + offset;

	//Serial.print(detectedSymbol);
	//Serial.print("    Sync offset: ");
	//if (offset >= 0)
	//	Serial.print(' ');
	//Serial.print(offset);
	//Serial.print("    Accumulated offset: ");
	//if (bitSync_accumulatedOffset >= 0)
	//	Serial.print(' ');
	//Serial.print(bitSync_accumulatedOffset);
	//Serial.print("    Local ticks: ");
	//Serial.print(bitSync_localTicksSinceSync);
	//Serial.print('\n');

	// Have we accumulated enough delta to adjust?
	if (bitSync_accumulatedOffset < -15  ||  bitSync_accumulatedOffset > 15) {
		// Only adjust if local ticks is large enough -- otherwise we overreact to noise
		if (bitSync_localTicksSinceSync > 1000) {
			adjustTickInterval(bitSync_localTicksSinceSync, bitSync_localTicksSinceSync - bitSync_accumulatedOffset);
			bitSync_localTicksSinceSync = 0;
			bitSync_accumulatedOffset = 0;
		}
	}
}


// Udpate the tick interval to compensate for counting localTicks while
// appearing to count apparentTicks.  Both parameters should be near each
// other (+-k, for small k). When they are above 4000, the math would overflow a 32-bit
// integer, so an alternate, approximate, technique is needed.
void adjustTickInterval(unsigned long localTicks, unsigned long apparentTicks) {

	Serial.print("Adjusting tick interval\n  Local ticks: ");
	Serial.print(localTicks);
	Serial.print("\n  Apparent ticks: ");
	Serial.print(apparentTicks);

	// Combine whole cycles and fraction into scaled integer
	unsigned long scaledCounts = (unsigned long)tick_interval_cycles * tick_frac_denominator + tick_frac_numerator;
	Serial.print("\n  Current counts: ");
	Serial.print(scaledCounts);

	unsigned long updatedCounts;
	updatedCounts = muldiv(scaledCounts, localTicks, apparentTicks);

	Serial.print("\nUpdated counts: ");
	Serial.print(updatedCounts);
	Serial.print("\n  Difference: ");

	long difference = updatedCounts - scaledCounts;
	Serial.print(difference);
	Serial.print(" [");
	Serial.print(difference, BIN);
	Serial.print("]\n");

	// Convert back to whole cycles and fraction.
	tick_frac_numerator = updatedCounts % tick_frac_denominator;
	tick_interval_cycles = updatedCounts / tick_frac_denominator;

	tick_interval_changed = true;
	unsaved_parameters = true;
}

// Computes (a*b)/c without overflow. Taken from https://stackoverflow.com/a/4144956. Based on
// ancient Egyptian multiplication, https://en.wikipedia.org/wiki/Ancient_Egyptian_multiplication
uint32_t muldiv(uint32_t a, uint32_t b, uint32_t c) {
    uint32_t q = 0;              // the quotient
    uint32_t r = 0;              // the remainder
    uint32_t qn = b / c;
    uint32_t rn = b % c;
    while(a) {
        if (a & 1) {
            q += qn;
            r += rn;
            if (r >= c) {
                q++;
                r -= c;
            }
        }
        a  >>= 1;
        qn <<= 1;
        rn <<= 1;
        if (rn >= c) {
            qn++; 
            rn -= c;
        }
    }
    return q;
}

void shiftSymbol(char newSymbol) {
	uint8_t score = 0;

	char outgoingSymbol = symbolStream[0];

	// Single loop for shifting and scoring. Only check that positions that should have
	// marker symbol have them, and that non-marker positions do not.

	// When markerPos = 0, we're shifting into a marker position. It gets reset to 9
	// after processing a marker position.
	uint8_t markerPosCountdown = 10;
	
	for (uint8_t i=0;  i<59;  i++) {
		markerPosCountdown--;
		char symbol = symbolStream[i+1];
		symbolStream[i] = symbol;
		if (markerPosCountdown == 0) {
			// Should be a marker symbol.
			if (symbol == 'M')
				score++;
			markerPosCountdown = 10;
		}
		else {
			// Should be a 1 or 0.
			if (symbol == '0'  ||  symbol == '1')
				score++;
		}
	}

	// Position 0 is a marker when aligned
	if (symbolStream[0] == 'M')
		score++;	

	symbolStream[59] = newSymbol;
	if (newSymbol == 'M' )
		score++;

	//printSymbols();

	if (score == 60) {
		valid_frame_flag = true;
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

 	// Update time of day. Decoded time is that at the start of the current frame transmission,
	// so the current minute is one later. Seconds value is implicitly 0.
	tod_ticks = ticksDelta;
	tod_seconds = 0;
	tod_minutes = minutes + 1;
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

// Indicate that we have detected a ZERO symbol.
void flashZero(int score) {
	static int hold = 0;

	if (score > scoreThreshold) {
		hold = 24;
		pixels.setPixelColor(68, COLOR_FLASH);
		pixels.setPixelColor(69, COLOR_FLASH);
	}
	else {
		if (hold > 0)
			hold--;
		else {
			pixels.setPixelColor(68, COLOR_BLUE);
			pixels.setPixelColor(69, COLOR_PURPLE);
		}
	}
}

// Indicate that we have detected a ONE symbol.
void flashOne(int score) {
	static int hold = 0;

	if (score > scoreThreshold) {
		hold = 24;
		pixels.setPixelColor(64, COLOR_FLASH);
		pixels.setPixelColor(65, COLOR_FLASH);
	}
	else {
		if (hold > 0)
			hold--;
		else {
			pixels.setPixelColor(64, COLOR_YELLOW);
			pixels.setPixelColor(65, COLOR_GREEN);
		}
	}
}

// Indicate that we have detected a MARKER symbol.
void flashMarker(int score) {
	static int hold = 0;

	if (score > scoreThreshold) {
		hold = 24;
		pixels.setPixelColor(60, COLOR_FLASH);
		pixels.setPixelColor(61, COLOR_FLASH);
	}
	else {
		if (hold > 0)
			hold--;
		else {
			pixels.setPixelColor(60, COLOR_RED);
			pixels.setPixelColor(61, COLOR_ORANGE);
		}
	}
}

// Shifts in a new bit sample into the array. The new bit is the LSB of the passed value.
void shiftSample(uint8_t value) {

	// Assembly language for fastness.
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

	int score = 0;
	for (int i=0; i<10; i++) {
		// Compute matching bits: XOR pattern and samples, and complement
		uint8_t matchingBits = ~(samples[i] ^ pattern[i]);
		score += parity[matchingBits];
	}

	return score;
}

// Increment the time of day.  Sets tod_changed when tod_seconds changes.
void tickTime() {

	tod_ticks++;

	// Blank time on the half-second when we haven't got a time fix
	if (!tod_fix && tod_ticks > 45) {
		resetNixies();
		updateNixies();
	}

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

void setColonColor(uint32_t color) {
	pixels.setPixelColor(62, color);
	pixels.setPixelColor(63, color);
	pixels.setPixelColor(66, color);
	pixels.setPixelColor(67, color);
}


// Set the current time of day on the hours, minutes, and seconds digits, 
// using UTC.
void updateTimeOfDayUtc() {
	setHours(tod_hours);
	setMinutes(tod_minutes);
	setSeconds(tod_seconds);
}

// Set current local time of day on the hours, minutes and seconds digits.
void updateTimeOfDayLocal(int8_t hoursOffset, int8_t minutesOffset, bool AMPM) {

	int16_t local_day = tod_day;
	int8_t local_hours = tod_hours + hoursOffset;
	int8_t local_minutes = tod_minutes + minutesOffset;
	int16_t local_year = tod_year;

	if (local_minutes > 59) {
		local_minutes -= 60;
		local_hours++;
	}
	else if (local_minutes < 0) {
		local_minutes += 60;
		local_hours--;
	}

	if (local_hours > 23) {
		local_day++;
		local_hours -= 24;
	}
	else if (local_hours < 0) {
		local_hours += 24;
		local_day--;
	}

	int days_in_year = 365;
	if (tod_isleapyear)
		days_in_year = 366;
	if (local_day > days_in_year) {
		local_day -= days_in_year;
		local_year++;
	}
	else if (local_day < 1) {
		local_day += days_in_year;
		local_year--;
	}

	// Display in 12 hour format?
	if (AMPM) {
		if (local_hours > 12)
			local_hours -= 12;
		if (local_hours == 0)
			local_hours = 12;
	}

	setHours(local_hours);
	setMinutes(local_minutes);
	setSeconds(tod_seconds);
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


void configureFromMemory() {
	PersistentParameters params;

	loadParameters(0, &params);

	// Load up with defaults in case loaded params look weird.
	tick_interval_cycles = 33333;
	tick_frac_numerator = 21;  // 21/64

	// Validate.
	if (params.version < 1 || params.version > 2) {
		Serial.print("configureFromMemory: bad version.  Expected 1 or 2; found ");
		Serial.print(params.version);
		return;
	}

	// Bound within 5% of standard
	// if (params.scaledCounts < 506666  ||  params.scaledCounts > 560000) {
	// 	Serial.print("configureFromMemory: scaledCount out of range. Found ");
	// 	Serial.print(params.scaledCounts);
	// 	return;
	// }

	// Use params.
	switch (params.version) {
		case 1:
		tick_frac_numerator = (params.scaledCounts % 16) * 4;  // Adjust from n/16 to m/64
		tick_interval_cycles = params.scaledCounts / 16;
		break;

		case 2:
		tick_frac_numerator = params.scaledCounts % 64;
		tick_interval_cycles = params.scaledCounts / 64;
		break;

	}

}

// Writes a parameters structure to EEPROM.  Returns number of bytes written.
uint16_t saveParameters(uint16_t address, PersistentParameters *params) {
	// Cast to byte pointer
	byte *p = (byte *)(void *)params;

	uint16_t i;
	 for (i = 0; i < sizeof(PersistentParameters); i++) {
        EEPROM.write(address++, *p++);
	 }
    return i;
}

// Reads a parameters sructure from EEPROM.  Returns number of bytes read.
uint16_t loadParameters(uint16_t address, PersistentParameters *params) {
	// Cast to byte pointer
	byte *p = (byte *)(void *)params;

	uint16_t i;
	for (i = 0; i < sizeof(PersistentParameters); i++) {
    	*p = EEPROM.read(address++);
		p++;
	}
    return i;	
}

void configureTickTimer() {

	// Configure timer 1 to interrupt at 60Hz
 	OCR1A = tick_interval_cycles;

	TCCR1A = 0;
    // Mode 4, CTC on OCR1A, prescaler 8
    TCCR1B = (1 << WGM12) | (1 << CS11);

    //Set interrupt on compare match
    TIMSK1 |= (1 << OCIE1A);
}

void configurePwmTimer() {
	// Configure timer 2 for PWM at 244Hz
	// Arduino pin 3 is OC2B
	TCCR2A = _BV(COM2B1) | _BV(COM2B0) | _BV(WGM21) | _BV(WGM20);
 	TCCR2B = _BV(CS22) | _BV(CS21);
	OCR2B = 255;
}

// Tick interrupt.
ISR (TIMER1_COMPA_vect) {

	// Set heartbeat pin high
	PORTD |= B00000100;

	// Current fractional period index. Counts from 0 to tick_frac_denominator-1.
	// When heartbeat_period < tick_frac_numerator, set counter for a long
	// period of tick_interval_cycles+1 counts; otherwise, a short period of tick_interval_cycles.
	static uint8_t fraction_counter = 0;

	if (++fraction_counter >= tick_frac_denominator)
		fraction_counter = 0;

	// Reset the CTC value. For a short period, count tick_interval_cycles;
	// for a long period, count tick_interval_cycles+1. 
    // Set compare value for a short or long cycle. Must set OCR1A to n-1.
	if (fraction_counter < tick_frac_numerator) {
		// Long period: n+1-1
		OCR1A = tick_interval_cycles;
	}
	else {
		// Short period: n-1
		OCR1A = tick_interval_cycles-1;
	}

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

// Diagnositc to echo the decoded symbols in the symbol buffer.
void printSymbols() {
	for (int i = 0;  i<60;  i++) {
		Serial.print(symbolStream[i]);
	}
	Serial.print('\n');
}

// Print the scores over the serial port.
void printScores(uint8_t zero, uint8_t one, uint8_t marker) {
	static bool separated = false;
	if (zero > scoreThreshold || one > scoreThreshold || marker > scoreThreshold) {
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

		Serial.print(marker);
		if (marker > scoreThreshold)
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

void printTimeUtc() {
		Serial.print("Time of day (UTC): ");
		Serial.print(tod_hours);
		Serial.print(':');
		if (tod_minutes < 10)
			Serial.print('0');
		Serial.print(tod_minutes);
		Serial.print(':');
		if (tod_seconds < 10)
			Serial.print('0');
		Serial.print(tod_seconds);
		Serial.print('\n');
}

void test_showPatterns() {
	Serial.print("PATTERN_ZERO: ");
	Serial.print(PATTERN_ZERO[9], BIN);
	Serial.print(' ');
	Serial.print(PATTERN_ZERO[8], BIN);
	Serial.print(' ');
	Serial.print(PATTERN_ZERO[7], BIN);
	Serial.print(' ');
	Serial.print(PATTERN_ZERO[6], BIN);
	Serial.print(' ');
	Serial.print(PATTERN_ZERO[5], BIN);
	Serial.print(' ');
	Serial.print(PATTERN_ZERO[4], BIN);
	Serial.print(' ');
	Serial.print(PATTERN_ZERO[3], BIN);
	Serial.print(' ');
	Serial.print(PATTERN_ZERO[2], BIN);
	Serial.print(' ');
	Serial.print(PATTERN_ZERO[1], BIN);
	Serial.print(' ');
	Serial.print(PATTERN_ZERO[0], BIN);
	Serial.print('\n');

	Serial.print("PATTERN_ONE: ");
	Serial.print(PATTERN_ONE[9], BIN);
	Serial.print(' ');
	Serial.print(PATTERN_ONE[8], BIN);
	Serial.print(' ');
	Serial.print(PATTERN_ONE[7], BIN);
	Serial.print(' ');
	Serial.print(PATTERN_ONE[6], BIN);
	Serial.print(' ');
	Serial.print(PATTERN_ONE[5], BIN);
	Serial.print(' ');
	Serial.print(PATTERN_ONE[4], BIN);
	Serial.print(' ');
	Serial.print(PATTERN_ONE[3], BIN);
	Serial.print(' ');
	Serial.print(PATTERN_ONE[2], BIN);
	Serial.print(' ');
	Serial.print(PATTERN_ONE[1], BIN);
	Serial.print(' ');
	Serial.print(PATTERN_ONE[0], BIN);
	Serial.print('\n');

	Serial.print("PATTERN_MARKER: ");
	Serial.print(PATTERN_MARKER[9], BIN);
	Serial.print(' ');
	Serial.print(PATTERN_MARKER[8], BIN);
	Serial.print(' ');
	Serial.print(PATTERN_MARKER[7], BIN);
	Serial.print(' ');
	Serial.print(PATTERN_MARKER[6], BIN);
	Serial.print(' ');
	Serial.print(PATTERN_MARKER[5], BIN);
	Serial.print(' ');
	Serial.print(PATTERN_MARKER[4], BIN);
	Serial.print(' ');
	Serial.print(PATTERN_MARKER[3], BIN);
	Serial.print(' ');
	Serial.print(PATTERN_MARKER[2], BIN);
	Serial.print(' ');
	Serial.print(PATTERN_MARKER[1], BIN);
	Serial.print(' ');
	Serial.print(PATTERN_MARKER[0], BIN);
	Serial.print('\n');
}

void test_shifter() {
	// Shift in a simulated PATTERN_ZERO, then compare to the other patterns
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
	Serial.print(score(PATTERN_ZERO));
	Serial.print("\nZERO on ONE: ");
	Serial.print(score(PATTERN_ONE));
	Serial.print("\nZERO on MARKER: ");
	Serial.print(score(PATTERN_MARKER));
	Serial.print("\n");

	// Shift in a simulated PATTERN_ONE, then compare to the other patterns
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
	Serial.print(score(PATTERN_ZERO));
	Serial.print("\nONE on ONE: ");
	Serial.print(score(PATTERN_ONE));
	Serial.print("\nONE on MARKER: ");
	Serial.print(score(PATTERN_MARKER));
	Serial.print("\n");

	// Shift in a simulated PATTERN_MARKER, then compare to the other patterns
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
	
	Serial.print("MARKER on ZERO: ");
	Serial.print(score(PATTERN_ZERO));
	Serial.print("\nMARKER on ONE: ");
	Serial.print(score(PATTERN_ONE));
	Serial.print("\nMARKER on MARKER: ");
	Serial.print(score(PATTERN_MARKER));
	Serial.print("\n");
}
