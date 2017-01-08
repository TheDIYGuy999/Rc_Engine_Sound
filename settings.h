
// Choose the motor sound (uncomment the one you want)
//#include "diesel.h"
//#include "v8.h"
#include "chevyNovaV8.h"
//#include "Mustang68.h"
//#include "LaFerrari.h"

// Mode settings - These could easily be 4 jumpers connected to spare pins, checked at startup to determine mode
boolean managedThrottle = true;     // Managed mode looks after the digipot if fitted for volume, and adds some mass to the engine
boolean potThrottle = false;        // A pot connected to A1, 0-1023 sets speed
boolean pwmThrottle = true;         // Takes a standard servo signal on pin 2 (UNO)
boolean spiThrottle = false;        // SPI mode, is an SPI slave, expects 1-255 for throttle position, with 0 being engine off

// Pins
#define POT_PIN A1     // Pot wiper when using pot mode
#define POT_CS  4      // MCP4131 CS // If using a digi pot to control volume these are the pins
#define POT_SCK 5      // MCP4131 Clock 
#define POT_SDO 6      // MCP4131 Data

// Volume, max. speed
#define DEFAULT_VOLUME 127      // Volume when in non managed mode
#define VOL_MIN 20              // Min volume in managed mode 0 - 127
#define VOL_MAX 127             // Max volume in managed mode 0 - 127
#define TOP_SPEED_MULTIPLIER 30 // RPM multiplier in managed mode, bigger the number the larger the rev range, 10 - 15 is a good place to start

