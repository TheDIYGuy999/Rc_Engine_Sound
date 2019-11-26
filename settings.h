/* Certain combinations are not possible, because there is not enough flash memory available on the ATmega328. Example: The Scania V8 start file
   is rather long and can not be used in combination with all motor sounds.

 ******* RECOMMENDED Audacity Settings: 16'000 KHz, 8 bit PCM **********
   
*/

// Choose the start sound (uncomment the one you want)
//#include "ScaniaV8Start.h" // Scania V8 Start
#include "UralV8Start.h" // Ural 4320 V8 Start
//#include "DefenderV8Start.h" // Land Rover Defender V8 Start

// Choose the motor sound (uncomment the one you want)
//#include "diesel.h" // Generic old diesel truck
//#include "ScaniaV8Idle.h" // Scania V8
//#include "UralV8Idle.h" // Ural 4320 V8
//#include "DefenderV8Idle.h" // Land Rover Defender V8
//#include "v8.h" // Generic V8
//#include "chevyNovaV8.h" // Chevy Nova Coupe 1975 <------- The best sounding!
//#include "Mustang68.h" // Ford Mustang 1968
//#include "MgBGtV8.h" // MG B GT V8
//#include "LaFerrari.h" // Ferrari "LaFerrari"
#include "TrophyTruckIdle.h" // V8 Trophy Truck (select TOP_SPEED_MULTIPLIER 50 for best effect!!)

// Choose the horn sound
//#include "horn.h" // The horn (not yet implemented)

// PWM Throttle range calibration -----------------------------------------------------------------------------------
int16_t pulseZero = 1500; // Usually 1500 (range 1000 - 2000us) Autocalibration active, if "engineManualOnOff" = "false"
int16_t pulseNeutral = 20; // pulseZero +/- this value
int16_t pulseSpan = 450; // pulseZero +/- this value (150 for JMT 10A ESC, otherwise around 450)
int16_t pulseLimit = 700; // pulseZero +/- this value (700)

// Engine parameters ------------------------------------------------------------------------------------------------
//Activate for "engine on off" functionality in combination with "Micro RC" Receiver from TheDIYGuy999. No Pulse Zero auto calibration in this case!!
boolean engineManualOnOff = false; 

// Engine RPM range
#define TOP_SPEED_MULTIPLIER 50 // RPM multiplier: the bigger the number the larger the rev range, 20 - 50 is a good place to start (for most sounds = 30)
const int16_t maxRpm = 1023; // max. 1023
const int16_t minRpm = 0; // always 0

// Engine mass simulation
const int8_t acc = 9; // Acceleration step per 5ms (9)
const int8_t dec = 6; // Deceleration step per 5ms (6)
