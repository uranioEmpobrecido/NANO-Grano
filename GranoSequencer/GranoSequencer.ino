
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
#define GRAIN_FREQ_CONTROL   A0
#define GRAIN_DECAY_CONTROL  A1
#define OCTAVE_CONTROL       A2
#define TEMPO_CONTROL        A3

//MASK
#define PORT_B  0
#define PORT_D  1

//Capacitive Threshold
#define Threshold 6

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
// Output is on pin 3

#define PWM_PIN       3
#define PWM_VALUE     OCR2B
#define PWM_INTERRUPT TIMER2_OVF_vect

uint16_t freqMap, octaveMap;

uint16_t input, inputOct;
uint8_t note;


// Smooth logarithmic mapping
 //
uint16_t antilogTable[64] = {
  64830,64132,63441,62757,62081,61413,60751,60097,59449,58809,58176,57549,56929,56316,55709,55109,
  54515,53928,53347,52773,52204,51642,51085,50535,49991,49452,48920,48393,47871,47356,46846,46341,
  45842,45348,44859,44376,43898,43425,42958,42495,42037,41584,41136,40693,40255,39821,39392,38968,
  38548,38133,37722,37316,36914,36516,36123,35734,35349,34968,34591,34219,33850,33486,33125,32768
};
uint16_t mapPhaseInc(uint16_t input) {
  if (analogRead(GRAIN_DECAY_CONTROL) > 75){
    if (((antilogTable[input & 0x3f]) >> (input >> 6)) < 5000){
      return (5000);
      }
    else {
      return ((antilogTable[input & 0x3f]) >> (input >> 6)); 
      }
    } else {
  return ((antilogTable[input & 0x3f]) >> (input >> 6));
  }
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

uint8_t readCapacitivePin(int pinToMeasure) {

  // Variables used to translate from Arduino to AVR pin naming
  volatile uint8_t* port;
  volatile uint8_t* ddr;
  volatile uint8_t* pin;
  // Here we translate the input pin number from
  //  Arduino pin number to the AVR PORT, PIN, DDR,
  //  and which bit of those registers we care about.
  byte bitmask;
  port = portOutputRegister(digitalPinToPort(pinToMeasure));
  ddr = portModeRegister(digitalPinToPort(pinToMeasure));
  bitmask = digitalPinToBitMask(pinToMeasure);
  pin = portInputRegister(digitalPinToPort(pinToMeasure));
  // Discharge the pin first by setting it low and output
  *port &= ~(bitmask);
  *ddr  |= bitmask;
  //delay(1);
  uint8_t SREG_old = SREG; //back up the AVR Status Register
  // Prevent the timer IRQ from disturbing our measurement
  noInterrupts();
  // Make the pin an input with the internal pull-up on
  *ddr &= ~(bitmask);
  *port |= bitmask;

  // Now see how long the pin to get pulled up. This manual unrolling of the loop
  // decreases the number of hardware cycles between each read of the pin,
  // thus increasing sensitivity.

  uint8_t cycles = 17;

  if (*pin & bitmask) { cycles =  0;}
  else if (*pin & bitmask) { cycles =  1;}
  else if (*pin & bitmask) { cycles =  2;}
  else if (*pin & bitmask) { cycles =  3;}
  else if (*pin & bitmask) { cycles =  4;}
  else if (*pin & bitmask) { cycles =  5;}
  else if (*pin & bitmask) { cycles =  6;}
  else if (*pin & bitmask) { cycles =  7;}
  else if (*pin & bitmask) { cycles =  8;}
  else if (*pin & bitmask) { cycles =  9;}
  else if (*pin & bitmask) { cycles = 10;}
  else if (*pin & bitmask) { cycles = 11;}
  else if (*pin & bitmask) { cycles = 12;}
  else if (*pin & bitmask) { cycles = 13;}
  else if (*pin & bitmask) { cycles = 14;}
  else if (*pin & bitmask) { cycles = 15;}
  else if (*pin & bitmask) { cycles = 16;}

  // End of timing-critical section; turn interrupts back on if they were on before, or leave them off if they were off before

  SREG = SREG_old;
  
  // Discharge the pin again by setting it low and output
  //  It's important to leave the pins low if you want to 
  //  be able to touch more than 1 sensor at a time - if
  //  the sensor is left pulled high, when you touch
  //  two sensors, your body will transfer the charge between
  //  sensors.
  *port &= ~(bitmask);
  *ddr  |= bitmask;
  return cycles;
}

uint16_t mapOctave() {
  bool noteOk = false;
  while (!noteOk){
  if (readCapacitivePin(C)>Threshold){
    Serial.println("LOL1");
    noteOk = true;
    return selectOctave(12);
  }
  else if (readCapacitivePin(Csharp)>Threshold){
    noteOk = true;
    return selectOctave(11);
  }
  else if (readCapacitivePin(D)>Threshold){
    noteOk = true;
    return selectOctave(10);
  }
  else if (readCapacitivePin(Dsharp)>Threshold){
    noteOk = true;
    return selectOctave(9);
  }
  else if (readCapacitivePin(E)>Threshold){
    noteOk = true;
    return selectOctave(8);
  }
  else if (readCapacitivePin(F)>Threshold){
    noteOk = true;
    return selectOctave(7);
  }
  else if (readCapacitivePin(Fsharp)>Threshold){
    noteOk = true;
    return selectOctave(6);
  }
  else if (readCapacitivePin(G)>Threshold){
    noteOk = true;
    return selectOctave(5);
  }
  else if (readCapacitivePin(Gsharp)>Threshold){
    noteOk = true;
    return selectOctave(4);
  }
  else if (readCapacitivePin(A)>Threshold){
    noteOk = true;
    return selectOctave(3);
  }
  else if (readCapacitivePin(Asharp)>Threshold){
    noteOk = true;
    return selectOctave(2);
  }/*
  else if (readCapacitivePin(B)>Threshold){
    noteOk = true;
    return selectOctave(1);
  }
  else if (readCapacitivePin(Cupper)>Threshold){
    noteOk = true;
    return selectOctave(0);
  }*/
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
    return Octave2Table[12-note];
  }
  if (octaveMap == 1){
    return Octave3Table[12-note];
  }
  if (octaveMap == 2){
    return Octave4Table[12-note];
  }
  if (octaveMap == 3){
    return Octave5Table[12-note];
  }
  if (octaveMap == 4){
    return Octave6Table[12-note];
  }

}

void audioOn() {
  // Set up PWM to 31.25kHz, phase accurate
  TCCR2A = _BV(COM2B1) | _BV(WGM20);
  TCCR2B = _BV(CS20);
  TIMSK2 = _BV(TOIE2);
}

uint16_t step1, step2, step3, step4, step5, step6, step7, step8;

bool selectNote1 = false;
bool selectNote2 = false;
bool selectNote3 = false;
bool selectNote4 = false;
bool selectNote5 = false;
bool selectNote6 = false;
bool selectNote7 = false;
bool selectNote8 = false;


void setSequence(void){
  while (!selectNote1){
    step1 = mapOctave();
    delay(500);
    selectNote1 = true;
  }
  while (!selectNote2){
    step2 = mapOctave();
    delay(500);
    selectNote2 = true;
  }
  while (!selectNote3){
    step3 = mapOctave();
    delay(500);
    selectNote3 = true;
  }
  while (!selectNote4){
    step4 = mapOctave();
    delay(500);
    selectNote4 = true;
  }
  while (!selectNote5){
    step5 = mapOctave();
    delay(500);
    selectNote5 = true;
  }
  while (!selectNote6){
    step6 = mapOctave();
    delay(500);
    selectNote6 = true;
  }
  while (!selectNote7){
    step7 = mapOctave();
    delay(500);
    selectNote7 = true;
  }
  while (!selectNote8){
    step8 = mapOctave();
    delay(500);
    selectNote8 = true;
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

  Serial.begin(9600);
  
  setSequence();
 
 }

void loop() {
  
  tempo = map(analogRead(TEMPO_CONTROL), 0, 1023, 10, 1200);
  counter++;
  if (counter > tempo) {
    counter = 0;
    if (pattern == 8) {
      pattern = 0;
      }
      grainPhaseInc  =  mapPhaseInc(analogRead(GRAIN_FREQ_CONTROL))/2;
      grainDecay     =  analogRead(GRAIN_DECAY_CONTROL)/8;
 
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
  case 4:
  syncPhaseInc = step5;
  break;
  case 5:
  syncPhaseInc = step6;
  break;
  case 6:
  syncPhaseInc = step7;
  break;
  case 7:
  syncPhaseInc = step8;
  break;
 }
 delay(tempo);
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
  }

  // Increment the phase of the grain oscillators
  grainPhaseAcc += grainPhaseInc;

  // Convert phase into a triangle wave
  value = (grainPhaseAcc >> 7) & 0xff;
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
