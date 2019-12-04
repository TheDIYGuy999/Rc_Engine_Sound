
/*
      This code was quick and dirty, based on a PCM audio example in the
      arduino playground: http://playground.arduino.cc/Code/PCMAudio

      It's been heavely modified for use with RC to generate something that's
      a bit like an engine sound. I've started work on making the program
      readable, still some to do though.
      https://github.com/BeigeMatchbox/mojoEngineSim/blob/master/README.md

      Changes, done by TheDIYGUY999 2017 - 2019: https://github.com/TheDIYGuy999/Rc_Engine_Sound
        - more engine sounds added
        - removed SPI throttle mode, digipot support (I had no use for it in an RC vehicle)
        - added engine start sound
        - added engine switch off (fader)
        - added throttle zero autocalibration
        - added simulated engine shifting points
*/

// All the required vehicle specific settings are done in settings.h!
#include "settings.h" // <<------- SETTINGS

const float codeVersion = 1.33; // Software revision

//
// =======================================================================================================
// LIRBARIES & TABS
// =======================================================================================================
//

#include "curves.h" // load nonlinear throttle curve arrays

//
// =======================================================================================================
// PIN ASSIGNMENTS & GLOBAL VARIABLES (Do not play around here)
// =======================================================================================================
//

#define SPEAKER 3                               // Amplifier PAM8403 connected to pin 3 (via 10kOhm potentiometer)
#define THROTTLE_INPUT 2                         // RC Signal connected to pin 2

#define FREQ 16000000L                          // 16MHz clock frequency

// Define global variables
volatile uint16_t currentSmpleRate = BASE_RATE; // Current playback rate, this is adjusted depending on engine RPM
volatile uint16_t fixedSmpleRate = FREQ / BASE_RATE; // Current playback rate, this is adjusted depending on engine RPM
volatile uint8_t engineState = 0; // 0 = off, 1 = starting, 2 = running, 3 = stopping

volatile boolean engineOn = false;              // Signal for engine on / off

volatile uint16_t curEngineSample;              // Index of currently loaded engine sample
volatile uint16_t curStartSample;               // Index of currently loaded start sample
volatile uint16_t curHornSample;                // Index of currently loaded horn sample

uint16_t  currentThrottle = 0;                  // 0 - 1000, a top value of 1023 is acceptable
volatile int16_t pulseWidth = 0;                // Current RC signal pulse width
volatile boolean pulseAvailable;                // RC signal pulses are coming in

int16_t pulseMaxNeutral; // PWM throttle configuration storage variables
int16_t pulseMinNeutral;
int16_t pulseMax;
int16_t pulseMin;
int16_t pulseMaxLimit;
int16_t pulseMinLimit;

//
// =======================================================================================================
// MAIN ARDUINO SETUP (1x during startup)
// =======================================================================================================
//

void setup() {

  attachInterrupt(0, getPulsewidth, CHANGE);

  // wait for RC receiver to initialize
  delay(1000);

  // then compute the RC channel offset (only, if "engineManualOnOff" inactive)
  if (!engineManualOnOff) pulseZero = pulseWidth;

  // Calculate throttle range
  pulseMaxNeutral = pulseZero + pulseNeutral;
  pulseMinNeutral = pulseZero - pulseNeutral;
  pulseMax = pulseZero + pulseSpan;
  pulseMin = pulseZero - pulseSpan;
  pulseMaxLimit = pulseZero + pulseLimit;
  pulseMinLimit = pulseZero - pulseLimit;

  // setup complete, so start making sounds
  setupPcm();
}

//
// =======================================================================================================
// PCM setup
// =======================================================================================================
//

void setupPcm() {

  pinMode(SPEAKER, OUTPUT);

  // Set up Timer 2 to do pulse width modulation on the speaker pin.
  ASSR &= ~(_BV(EXCLK) | _BV(AS2));                         // Use internal clock (datasheet p.160)

  TCCR2A |= _BV(WGM21) | _BV(WGM20);                        // Set fast PWM mode  (p.157)
  TCCR2B &= ~_BV(WGM22);

  TCCR2A = (TCCR2A | _BV(COM2B1)) & ~_BV(COM2B0);           // Do non-inverting PWM on pin OC2B (p.155)
  TCCR2A &= ~(_BV(COM2A1) | _BV(COM2A0));                   // On the Arduino this is pin 3.
  TCCR2B = (TCCR2B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10); // No prescaler (p.158)

  OCR2B = pgm_read_byte(&idle_data[0]);                     // Set initial pulse width to the first sample.

  // Set up Timer 1 to send a sample every interrupt.
  cli();

  TCCR1B = (TCCR1B & ~_BV(WGM13)) | _BV(WGM12);             // Set CTC mode (Clear Timer on Compare Match) (p.133)
  TCCR1A = TCCR1A & ~(_BV(WGM11) | _BV(WGM10));             // Have to set OCR1A *after*, otherwise it gets reset to 0!

  TCCR1B = (TCCR1B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10); // No prescaler (p.134)

  OCR1A = FREQ / BASE_RATE;                                // Set the compare register (OCR1A).
  // OCR1A is a 16-bit register, so we have to do this with
  // interrupts disabled to be safe.

  TIMSK1 |= _BV(OCIE1A);                                   // Enable interrupt when TCNT1 == OCR1A (p.136)


  curEngineSample = 0;
  curStartSample = 0;
  //curHornSample = 0;

  sei();
}

//
// =======================================================================================================
// MAP PULSEWIDTH TO THROTTLE
// =======================================================================================================
//

void mapThrottle() {

  // Input is around 1000 - 2000us, output 0-500 for forward and backwards

  // check if the pulsewidth looks like a servo pulse
  if (pulseWidth > pulseMinLimit && pulseWidth < pulseMaxLimit) {
    if (pulseWidth < pulseMin) pulseWidth = pulseMin; // Constrain the value
    if (pulseWidth > pulseMax) pulseWidth = pulseMax;

    // calculate a throttle value from the pulsewidth signal
    if (pulseWidth > pulseMaxNeutral) currentThrottle = map(pulseWidth, pulseMaxNeutral, pulseMax, 0, 500);
    else if (pulseWidth < pulseMinNeutral) currentThrottle = map(pulseWidth, pulseMinNeutral, pulseMin, 0, 500);
    else currentThrottle = 0;
  }
}

//
// =======================================================================================================
// ENGINE MASS SIMULATION
// =======================================================================================================
//

void engineMassSimulation() {

  static int16_t  mappedThrottle = 0;
  static int16_t currentRpm = 0;
  static unsigned long throtMillis;

  if (millis() - throtMillis > 5) { // Every 5ms
    throtMillis = millis();

    // compute unlinear throttle curve
    mappedThrottle = reMap(curveShifting, currentThrottle);

    // Accelerate engine
    if (mappedThrottle + acc > currentRpm && engineState == 2) {
      currentRpm += acc;
      if (currentRpm > maxRpm) currentRpm = maxRpm;
    }

    // Decelerate engine
    else if (mappedThrottle - dec < currentRpm) {
      currentRpm -= dec;
      if (currentRpm < minRpm) currentRpm = minRpm;
    }

    // Speed (sample rate) output
    currentSmpleRate = FREQ / (BASE_RATE + long(currentRpm * TOP_SPEED_MULTIPLIER) );
  }
}

//
// =======================================================================================================
// SWITCH ENGINE ON OR OFF
// =======================================================================================================
//

void engineOnOff() {

  static unsigned long pulseDelayMillis;
  static unsigned long idleDelayMillis;

  if (engineManualOnOff) { // Engine manually switched on or off depending on presence of servo pulses
    if (pulseAvailable) pulseDelayMillis = millis(); // reset delay timer, if pulses are available

    if (millis() - pulseDelayMillis > 100) {
      engineOn = false; // after delay, switch engine off
    }
    else engineOn = true;
  }
  else { // Engine automatically switched on or off depending on throttle position and 15s delay timne
    if (currentThrottle > 80) idleDelayMillis = millis(); // reset delay timer, if throttle not in neutral

    if (millis() - idleDelayMillis > 15000) {
      engineOn = false; // after delay, switch engine off
    }
    else {
      if (currentThrottle > 100) engineOn = true;
    }
  }
}



//
// =======================================================================================================
// MAIN LOOP
// =======================================================================================================
//

void loop() {

  // Map pulsewith to throttle
  mapThrottle();

  // Simulate engine mass, generate RPM signal
  engineMassSimulation();

  // Switch engine on or off
  engineOnOff();
}

//
// =======================================================================================================
// INTERRUPTS
// =======================================================================================================
//

// Uses a pin change interrupt and micros() to get the RC signal pulsewidth at pin 2 ----------------
void getPulsewidth() {

  unsigned long currentMicros = micros();
  boolean currentState = PIND & B00000100; // Pin 2 is PIND Bit 2 ( = digitalRead(2) )
  static unsigned long prevMicros = 0;
  static boolean lastState = LOW;

  if (lastState == LOW && currentState == HIGH) {    // Rising edge
    prevMicros = currentMicros;
    pulseAvailable = true;
    lastState = currentState;
  }
  else if (lastState == HIGH && currentState == LOW) { // Falling edge
    pulseWidth = currentMicros - prevMicros;
    pulseAvailable = false;
    lastState = currentState;
  }
}


// This is the main sound playback interrupt, keep this nice and tight!! ------------------------------
ISR(TIMER1_COMPA_vect) {

  static float attenuator;  // Float required for finer granularity!

  switch (engineState) {

    case 0: // off ----
      OCR1A = fixedSmpleRate; // fixed sample rate (speed)!
      OCR2B = 0;
      if (engineOn) engineState = 1;
      break;

    case 1: // starting ----
      if (curStartSample >= start_length) { // Loop the sample
        curStartSample = 0;
        engineState = 2;
      }
      OCR1A = fixedSmpleRate; // fixed sample rate (speed)!
      OCR2B = pgm_read_byte(&start_data[curStartSample]);
      curStartSample++;
      break;

    case 2: // running ----
      if (curEngineSample >= idle_length) { // Loop the sample
        curEngineSample = 0;
        attenuator = 1;
      }
      OCR1A = currentSmpleRate; // variable sample rate (RPM)!
      OCR2B = pgm_read_byte(&idle_data[curEngineSample]);
      curEngineSample++;
      if (!engineOn) {
        engineState = 3;
      }
      break;

    case 3: // stopping ----
      if (curEngineSample >= idle_length) { // Loop the sample
        curEngineSample = 0;
      }

      OCR1A = fixedSmpleRate;
      //OCR1A = fixedSmpleRate * attenuator; // engine slowing down
      OCR2B = pgm_read_byte(&idle_data[curEngineSample]) / attenuator;
      curEngineSample++;
      attenuator += 0.001; // fade engine sound out 0.002
      if (attenuator >= 20) {  // 3 - 20
        engineOn = false;
        if (!engineOn) engineState = 0; // Important: ensure, that engine is off, before we go back to "starting"!!
      }
      break;

  } // end of switch case
}
