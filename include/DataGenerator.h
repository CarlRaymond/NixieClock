#define ZERO 0
#define ONE 1
#define MARKER 2

#include <Arduino.h>

// Returns synthetic data bits for testing purposes.
class DataGenerator {

	public:
		DataGenerator(uint8_t *pattern, size_t length, int noiselevel);

		uint8_t nextBit();

	private:
		// Symbol pattern supplied in constructor
		uint8_t *pattern;
		// Length of pattern
		size_t length;

		// Current position within the supplied pattern
		int position;

		// Amount of noise. 0 for no noise; 1000 for all noise (complete inversion)
		int noiselevel;

		// Number of high and low bits remaining to emit for current symbol
		uint8_t high_count;
		uint8_t low_count;

		void setCounts(uint8_t symbol);

		uint8_t noisy(uint8_t);

};