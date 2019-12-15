# This is an Arduino RC engine sound generator

THIS PROJECT IS DEPRECATED! See new ESP32 version: https://github.com/TheDIYGuy999/Rc_Engine_Sound_ESP32

It's based on the mojoEngineSim: https://github.com/BeigeMatchbox/mojoEngineSim

GitHub repo: https://github.com/TheDIYGuy999/Rc_Engine_Sound

Video: https://www.youtube.com/watch?v=EaOJE_GU5pk&t=58s (showing version 1.2)

STL data for 3D printed housing: https://www.thingiverse.com/thing:2814025

## Features:
- Many selectable engine sounds and startup sounds for cars and trucks
- Sound files up to 8bit, 16kHz, mono can be used
- Works best with a PAM8403 amplifier module, connected to pin 3, via a 10kOhm potentiometer
- The engine RPM is calculated according to RC signal input on pin 2
- Gear shifting is simulated in "curves.h"
- Use an Arduino Pro Mini 5V, 16MHz

## New in V 1.0:
- Runs on an ATMega328 with 8 or 16MHz clock (RC PWM throttle mode only on 16MHz)
- More engine sounds added

## New in V 1.1:
- Engine sound is switched off, if there is no PWM signal detected on Pin 2. Works together with my "Micro RC" receiver.

## New in V 1.2:
- PWM throttle range now adjustable in settings.h

## New in V 1.3:
- code cleaned up, removed unused SPI and digipot support, removed 8MHz support (was for SPI mode only)
- Scania V8 and URAL-4320 V8 Diesel sounds added, including start sounds
- fader for engine switch off phase added
- throttle auto zero calibration added
- simulated gearbox shifting points in "curves.h"
- boolean variable "engineManualOnOff" ensures compatibility with older projects (Mustang 68)
- engine is switching on or off depending on the throttle input or the presence of the RC signal (see above)

## New in V 1.31:
- mapThrottle() cleaned up, scaling bug fixed, throttle auto zero calibration now working correctly
- adjust "pulseSpan" in "settings.h" according to your ESC / receiver settings
- schematic and pictures added

## New in V 1.32:
- added the requested V8 trophy truck sound
- optimized the generic "V8" sound to fit the memory in combination with the Ural start sound

## New in V 1.33:
- added the Defender V8 sounds (I forgot to upload them)

## Ho to create new sound arrays:

### Audacity:
- Import the sound file you want in Audacity
- Convert it to mono, if needed
- on the bottom left, select project frequency 16000kHz
- cut the sound to one engine cycle. Zoom in to find the exact zero crossing
- adjust the volume, so that the entire range is used
- select > export audio > WAV > 8-bit-PCM
- note, that the files should be as short as possible: around 0.3s for idle and 1s for start

### Loading and compiling wav2c (this is for OS X):
- download it from: https://github.com/olleolleolle/wav2c
- compile it in terminal with the following steps:
- type "gcc" and space
- drag and drop the files "main.c", wavdata.h", wavdata.c" into the terminal window.
- press enter -> the executable is then compiled and stored as "youruserdirectory/a.out"

### Processing the new header file with your sound:
- copy an existing "enginesound.h" file, rename it with your new engine name
- drag and drop "a.out" into your terminal
- drag and drop the exported WAV file into your terminal
- drag and drop your copied "enginesound.h" file into your terminal
- type "idle" for the idle file or "start" for the start file
- press enter -> the "enginesound.h" file is now overwritten with the new sound data.
- uncomment the line "sampleRate"
- change "signed char" to "unsigned char"
- include this file in "settings.h"

### Compiling the new sketch:
- compile and upload the sketch in Arduino IDE
- the new engine should now run...

## Schematic:
![](https://github.com/TheDIYGuy999/Rc_Engine_Sound/blob/master/wiring.jpg)

## Pictures (including optional additional filtering):
![](https://github.com/TheDIYGuy999/Rc_Engine_Sound/blob/master/top.jpg)

![](https://github.com/TheDIYGuy999/Rc_Engine_Sound/blob/master/bottom.jpg)


2017 - 2019, TheDIYGuy999
