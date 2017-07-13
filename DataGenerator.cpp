#include "DataGenerator.h"

DataGenerator::DataGenerator(uint8_t *pattern, int length, int noiselevel) {
	this->pattern = pattern;
	this->length = length;
	this->noiselevel = noiselevel;
	this->position = 0;
	SetCounts(pattern[0]);
}

uint8_t DataGenerator::NextBit() {

	// Any more high bits for current symbol?
	if (high_count > 0) {
		high_count--;
		return Noisy(1);
	}

	// Any more low bits for current symbol?
	if (low_count > 0) {
		low_count--;
		return Noisy(0);
	}

	// Advance to next symbol. Loop back to beginning on end.
	position++;
	if (position >= length)
		position = 0;

	SetCounts(pattern[position]);

	// Send leading bit
	high_count--;
	return Noisy(1);
}

// Randomly flip a bit, based on the noise level.
uint8_t DataGenerator::Noisy(uint8_t val) {
	if (noiselevel == 0)
		return false;

	if (random(1000) > noiselevel) {
		// Unflipped.
		return val;
	}

	// Flipped.
	return 1-val;
}

void DataGenerator::SetCounts(uint8_t symbol) {
	// Set bit counters.
	switch (pattern[position]) {
		case ZERO:
			high_count = 12;
			low_count = 48;
			break;

		case ONE:
			high_count = 30;
			low_count = 30;
			break;

		case FRAME:
		default:
			high_count = 48;
			low_count = 12;
	}
}

