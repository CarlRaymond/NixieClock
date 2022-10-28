// Parameters for clock that get saved in EEPROM. Version number helps with sanity checking.
typedef struct {
	uint8_t version;
	uint32_t scaledCounts;
} PersistentParameters;


uint16_t loadParameters(uint16_t address, PersistentParameters *params);
void configureTickTimer(void);
void configurePwmTimer(void);
void setHours(unsigned short);
void setMinutes(unsigned short);
void setSeconds(unsigned short);
void setBacklightRainbow(void);
void setBacklightColor(uint32_t color);
void setColonColor(uint32_t color);
void setTubePwm(uint8_t);
void configureFromMemory();
uint8_t scale480(uint16_t val);
void setMode(uint8_t);
void decodeTimeOfDay(uint8_t ticksDelta);
void printTimeUtc(void);
void updateTimeOfDayLocal(int8_t hoursOffset, int8_t minutesOffset, bool AMPM);
void updateTimeOfDayUtc(void);
void resetNixies(void);
void updateNixies(void);
void updatePixels(void);
void flashZero(int score);
void tickTime(void);
uint16_t saveParameters(uint16_t address, PersistentParameters *params);
void bitSeek(void);
void bitSync(void);
void shiftSymbol(char newSymbol);
void shiftSample(uint8_t value);
void sampleToBuffer(uint8_t value);
void adjustTickInterval(unsigned long localTicks, unsigned long apparentTicks);
int score(uint8_t *pattern);
uint32_t muldiv(uint32_t a, uint32_t b, uint32_t c);