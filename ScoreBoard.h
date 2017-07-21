#include <Arduino.h>

// A scoreboard keeps a history of the last n scores, and
// can indicate which slot has the maximum value
class ScoreBoard {

	public:
		static const int size = 11;
		ScoreBoard();

		void shiftScore(uint8_t score);
		bool maxOverThreshold(uint8_t threshold, uint8_t *maxValue, uint8_t *maxSlotIndex);

		uint8_t getSlotValue(uint8_t slot);
		
	private:
		uint8_t slots[size];
};