#include "DataGenerator.h"

DataGenerator::DataGenerator(uint8_t *pattern, int length) {
	this->pattern = pattern;
	this->length = length;
	this->position = 0;
	SetCounts(pattern[0]);
}

uint8_t DataGenerator::NextBit() {

	// Issue a high bit for current symbol?
	if (high_count > 0) {
		high_count--;
		return 1;
	}

	// Issue a low bit for current symbol?
	if (low_count > 0) {
		low_count--;
		return 0;
	}

	// Advance to next symbol. Loop to beginning on end.
	position++;
	if (position >= length)
		position = 0;

	SetCounts(pattern[position]);

	// Send leading bit
	high_count--;
	return 1;
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

