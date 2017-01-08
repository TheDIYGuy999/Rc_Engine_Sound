#This is an Arduino engine sound generator.
It's based on the mojoEngineSim: https://github.com/BeigeMatchbox/mojoEngineSim

GitHub repo: https://github.com/TheDIYGuy999/Rc_Engine_Sound

## Features:
- Selectable engine sound
- Sound files up to 8bit, 16kHz, mono can be used
- works best with a PAM8403 amplifier module

New in V 1.0:
- Runs on an ATMega328 with 8 or 16MHz clock (RC PWM throttle mode only on 16MHz)
- More engine sounds added

## Ho to create new sound arrays:

### Audacity:
- Import the sound file you want in Audacy
- Convert it to mono, if needed
- on the bottom left, select project frequency 16000kHz
- cut the sound to one engine cycle. Zoom in to find the exact zero crossing
- adjust the volume, so that the entire range is used
- select > export audio > WAV > 8-bit-PCM

### Load and compile wav2c (this is for OS X):
- download it from: https://github.com/olleolleolle/wav2c
- compile it in terminal:
* type "gcc" and space
* drag and drop the files "main.c", wavdata.h", wavdata.c" into the terminal window.
* press enter -> the executable is then compiled and stored as "youruserdirectory/a.out

### processing the new header file with your sound:
- copy an existing "enginesound.h" file, rename it with your new engine name
- drag and drop "a.out" into your terminal
- drag and drop the exported WAV file into your terminal
- drag and drop your copied "enginesound.h" file into your terminal
- type "idle"
- press enter -> the "enginesound.h" file is now overwritten with the new sound data.
- uncomment the line "sampleRate"
- change "signed char" to "unsigned char"
- include this file in "settings.h"

### compiling the new sketch:
- compile and upload the sketch in Arduino IDE
- the new engine should now run...


2017, TheDIYGuy999
