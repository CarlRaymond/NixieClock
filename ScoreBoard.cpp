#include "ScoreBoard.h"

ScoreBoard::ScoreBoard() {
	for (uint8_t i=0;  i<size;  i++)
		slots[i] = 0;
 }

void ScoreBoard::shiftScore(uint8_t score) {

	for (uint8_t i = size-1;  i>0;  i--) {
		slots[i] = slots[i-1];
	}
	slots[0] = score;
}

uint8_t ScoreBoard::getSlotValue(uint8_t slot) {
	return slots[slot];
}


// Returns true when the scoreboard has a maximum value above the threshold. Passes
// out the max value and the slot index containing it.
bool ScoreBoard::maxOverThreshold(uint8_t threshold, uint8_t *maxValue, uint8_t *maxSlotIndex) {

	uint8_t max = slots[0];
	uint8_t index = 0;

	for (uint8_t i=1;  i<size;  i++) {
		if (slots[i] > max) {
			max = slots[i];
			index = i;
		}
	}

	*maxValue = max;
	*maxSlotIndex = index;

	return (max > threshold);
}
