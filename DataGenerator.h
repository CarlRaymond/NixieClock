#define ZERO 0
#define ONE 1
#define FRAME 2

#include <Arduino.h>

// Returns synthetic data bits for testing purposes.
class DataGenerator {

	public:
		DataGenerator(uint8_t *pattern, int length);

		uint8_t NextBit();

	private:
		// Symbol pattern supplied in constructor
		uint8_t *pattern;

		// Position within the supplied pattern
		int position;
		int length;

		// Number of high and low bits remaining to emit for current symbol
		uint8_t high_count;
		uint8_t low_count;

		void SetCounts(uint8_t symbol);

};