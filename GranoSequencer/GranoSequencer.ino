/*
 Auduino, the Lo-Fi granular synthesiser https://code.google.com/archive/p/tinkerit/downloads
 added a 4 step sequencer inspired by https://learn.sparkfun.com/tutorials/build-an-auduino-step-sequencer
 Tom Tobback Sep 2016
 BuffaloLabs www.cassiopeia.hk/kids

 LICENSE:
 Copyright (c) 2016 Tom Tobback 

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

/* analog inputs:
 * A0 pitch step 1 (syncPhaseInc)
 * A1 pitch step 2
 * A2 pitch step 3
 * A3 pitch step 4
 * A4 tempo
 * A5 effect (grainPhaseInc)
 * plus volume potentiometer
 *
 * audio out via 220 ohm resistor to 3.5mm jack
 * audio out via 10K/10K voltage divider and 10K volume pot and 10uF cap to internal mono amp with speaker
 * power switch on amp
 * built with Arduino Nano on perf board powered by USB cable
 */

#include <avr/io.h>
 #include <avr/interrupt.h>

uint16_t syncPhaseAcc;
uint16_t syncPhaseInc;
uint16_t grainPhaseAcc;
uint16_t grainPhaseInc;
uint16_t grainAmp;
uint8_t grainDecay;

// Map Analogue channels
#define GRAIN_FREQ_CONTROL   (0)
#define GRAIN_DECAY_CONTROL  (1)
#define OCTAVE_CONTROL       (2)

// Map Digital channels
#define C         13//PB5
#define Csharp    12//PB4
#define D         11//PB3
#define Dsharp    10//PB2
#define E         9 //PB1
#define F         8 //PB0
#define Fsharp    7 //PD7
#define G         6 //PD6
#define Gsharp    5 //PD5
#define A         4 //PD4
#define Asharp    2 //PD2
#define B         1 //PD1
#define Cupper    0 //PD0

// Changing these will also requires rewriting audioOn()
// For modern ATmega168 and ATmega328 boards
//    Output is on pin 3
//
#define PWM_PIN       3
#define PWM_VALUE     OCR2B
#define PWM_INTERRUPT TIMER2_OVF_vect

uint16_t freqMap, octaveMap;

uint16_t input, inputOct;
uint8_t note;


// Smooth logarithmic mapping
 //
 uint16_t antilogTable[] = {
 64830, 64132, 63441, 62757, 62081, 61413, 60751, 60097, 59449, 58809, 58176, 57549, 56929, 56316, 55709, 55109,
 54515, 53928, 53347, 52773, 52204, 51642, 51085, 50535, 49991, 49452, 48920, 48393, 47871, 47356, 46846, 46341,
 45842, 45348, 44859, 44376, 43898, 43425, 42958, 42495, 42037, 41584, 41136, 40693, 40255, 39821, 39392, 38968,
 38548, 38133, 37722, 37316, 36914, 36516, 36123, 35734, 35349, 34968, 34591, 34219, 33850, 33486, 33125, 32768
 };
 uint16_t mapPhaseInc(uint16_t input) {
 return (antilogTable[input & 0x3f]) >> (input >> 6);
 }

// Vibiss mapping OCTAVE 2
//
uint16_t Octave2Table[] = {
  65,69,73,77,82,86,92,97,103,109,115,122,129
};

// Vibiss mapping OCTAVE 3
//
uint16_t Octave3Table[] = {
  131,139,147,156,165,175,185,196,208,220,233,247
};

// Vibiss mapping OCTAVE 4
//
uint16_t Octave4Table[] = {
  262,277,294,311,330,349,370,392,415,440,466,494
};

// Vibiss mapping OCTAVE 5
//
uint16_t Octave5Table[] = {
  523,554,587,622,659,698,740,784,831,880,932,988
};


// Vibiss mapping OCTAVE 6
//
uint16_t Octave6Table[] = {
  1047,1109,1175,1245,1319,1397,1480,1568,1661,1760,1865,1976
};

uint16_t mapOctave() {
  if (digitalRead(C)==0){
    return selectOctave(12);
  }
  else if (digitalRead(Csharp)==0){
    return selectOctave(11);
  }
  else if (digitalRead(D)==0){
    return selectOctave(10);
  }
  else if (digitalRead(Dsharp)==0){
    return selectOctave(9);
  }
  else if (digitalRead(E)==0){
    return selectOctave(8);
  }
  else if (digitalRead(F)==0){
    return selectOctave(7);
  }
  else if (digitalRead(Fsharp)==0){
    return selectOctave(6);
  }
  else if (digitalRead(G)==0){
    return selectOctave(5);
  }
  else if (digitalRead(Gsharp)==0){
    return selectOctave(4);
  }
  else if (digitalRead(A)==0){
    return selectOctave(3);
  }
  else if (digitalRead(Asharp)==0){
    return selectOctave(2);
  }
  else if (digitalRead(B)==0){
    return selectOctave(1);
  }
  else if (digitalRead(Cupper)==0){
    return selectOctave(0);
    }
}


uint16_t selectOctave(uint8_t note){

  inputOct = analogRead(OCTAVE_CONTROL);
  octaveMap = (inputOct*5)/1024;
  /*
  Serial.print("  Note:");
  Serial.print(note);
  Serial.print("  Octave:");
  Serial.print(octaveMap);
  Serial.print("\n");
  */
  if (octaveMap == 0){
    // Serial.print("Freq:");
    // Serial.print(Octave2Table[11-note]);
    return Octave2Table[12-note];
  }
  if (octaveMap == 1){
    // Serial.print("Freq:");
    // Serial.print(Octave3Table[11-note]);
    return Octave3Table[12-note];
  }
  if (octaveMap== 2){
    // Serial.print("Freq:");
    // Serial.print(Octave4Table[11-note]);
    return Octave4Table[12-note];
  }
  if (octaveMap == 3){
    // Serial.print("Freq:");
    // Serial.print(Octave5Table[11-note]);
    return Octave5Table[12-note];
  }
  if (octaveMap == 4){
    // Serial.print("Freq:");
    // Serial.print(Octave6Table[11-note]);
    return Octave6Table[12-note];
  }

}

void audioOn() {
  // Set up PWM to 31.25kHz, phase accurate
  TCCR2A = _BV(COM2B1) | _BV(WGM20);
  TCCR2B = _BV(CS20);
  TIMSK2 = _BV(TOIE2);
}

uint16_t step1, step2, step3, step4;

bool selectNote1 = false;
bool selectNote2 = false;
bool selectNote3 = false;
bool selectNote4 = false;


void setSequence(void){
  while (!selectNote1){
    step1 = mapOctave();
    delay(300);
    selectNote1 = true;
  }
  while (!selectNote2){
    step2 = mapOctave();
    delay(300);
    selectNote2 = true;
  }
  while (!selectNote3){
    step3 = mapOctave();
    delay(300);
    selectNote3 = true;
  }
  while (!selectNote4){
    step4 = mapOctave();
    delay(300);
    selectNote4 = true;
  }
}

long counter = 0;
long tempo;
uint8_t pattern = 0;


void setup() {
  
  pinMode(PWM_PIN,OUTPUT);
  audioOn();
  
  pinMode(C,INPUT_PULLUP);
  pinMode(Csharp,INPUT_PULLUP);
  pinMode(D,INPUT_PULLUP);
  pinMode(Dsharp,INPUT_PULLUP);
  pinMode(E,INPUT_PULLUP);
  pinMode(F,INPUT_PULLUP);
  pinMode(Fsharp,INPUT_PULLUP);
  pinMode(G,INPUT_PULLUP);
  pinMode(Gsharp,INPUT_PULLUP);
  pinMode(A,INPUT_PULLUP);
  pinMode(Asharp,INPUT_PULLUP);
  pinMode(B,INPUT_PULLUP);
  pinMode(Cupper,INPUT_PULLUP);

// presets for 3 less important pots
 grainDecay = 200 / 8;

 setSequence();
 }

void loop() {

tempo = map(analogRead(A4), 0, 1023, 100, 4000);
 counter++;
 if (counter > tempo) {
 counter = 0;
 if (pattern == 4) {
 pattern = 0;
 }
 switch (pattern) {
  case 0:
  syncPhaseInc = step1;
  break;
  case 1:
  syncPhaseInc = step2;
  break;
  case 2:
  syncPhaseInc = step3;
  break;
  case 3:
  syncPhaseInc = step4;
  break;
 }

grainPhaseInc = mapPhaseInc(analogRead(A5)) / 2;
 pattern++;
 }
 }

SIGNAL(PWM_INTERRUPT)
 {
 uint8_t value;
 uint16_t output;

syncPhaseAcc += syncPhaseInc;
 if (syncPhaseAcc < syncPhaseInc) {
 // Time to start the next grain
 grainPhaseAcc = 0;
 grainAmp = 0x7fff;
 if (grainPhaseAcc & 0x8000) value = ~value;
 // Multiply by current grain amplitude to get sample
 output = value * (grainAmp >> 8);

// Make the grain amplitudes decay by a factor every sample (exponential decay)
 grainAmp -= (grainAmp >> 8) * grainDecay;

// Scale output to the available range, clipping if necessary
 output >>= 9;
 if (output > 255) output = 255;

// Output to PWM (this is faster than using analogWrite)
 PWM_VALUE = output;
 }
} 

