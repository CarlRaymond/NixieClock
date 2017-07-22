#include "ScoreBoard.h"

ScoreBoard::ScoreBoard() {
	for (uint8_t i=0;  i<size;  i++)
		slots[i] = 0;
 }

void ScoreBoard::shiftScore(uint8_t score) {

	// Shift in new score
	for (uint8_t i = size-1;  i>0;  i--) {
		slots[i] = slots[i-1];
	}
	slots[0] = score;

	// Find new max value and slot
	peakValue = slots[0];
	peakIndex = 0;

	for (uint8_t i=1;  i<size;  i++) {
		if (slots[i] > peakValue) {
			peakValue = slots[i];
			peakIndex = i;
		}
	}

}

uint8_t ScoreBoard::getSlotValue(uint8_t slot) {
	return slots[slot];
}


// Returns true when the scoreboard has a maximum value above the threshold. Passes
// out the max value and the slot index containing it.
bool ScoreBoard::maxOverThreshold(uint8_t threshold, uint8_t *peakValue_out, uint8_t *peakIndexIndex_out) {


	*peakValue_out = peakValue;
	*peakIndexIndex_out = peakIndex;

	return (peakValue > threshold);
}
