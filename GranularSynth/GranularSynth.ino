// Analog in 0: Grain Frecuency Control
// Analog in 1: Octave Selector
// Analog in 2: Grain Decay Control
// Analog in 3: Attack Control (Not implemented)
// Analog in 4: Decay Control (Not implemented)
// Analog in 5: Tri-State Volume Control (Not implemented)

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
#define GRAIN_DECAY_CONTROL  (2)
#define OCTAVE_CONTROL       (1)

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
  64830,64132,63441,62757,62081,61413,60751,60097,59449,58809,58176,57549,56929,56316,55709,55109,
  54515,53928,53347,52773,52204,51642,51085,50535,49991,49452,48920,48393,47871,47356,46846,46341,
  45842,45348,44859,44376,43898,43425,42958,42495,42037,41584,41136,40693,40255,39821,39392,38968,
  38548,38133,37722,37316,36914,36516,36123,35734,35349,34968,34591,34219,33850,33486,33125,32768
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
  /*else if (digitalRead(B)==0){
    return selectOctave(1);
  }*/
  else if (digitalRead(Cupper)==0){
    return selectOctave(0);
  }
  else return 0;
}


uint16_t selectOctave(uint8_t note){

  inputOct = analogRead(OCTAVE_CONTROL);
  octaveMap = (inputOct*5)/1024;
  
  Serial.print("  Note:");
  Serial.print(note);
  Serial.print("  Octave:");
  Serial.print(octaveMap);
  Serial.print("\n");
  
  if (octaveMap == 0){
     Serial.print("Freq:");
     Serial.print(Octave2Table[11-note]);
    return Octave2Table[12-note];
  }
  if (octaveMap == 1){
     Serial.print("Freq:");
     Serial.print(Octave3Table[11-note]);
    return Octave3Table[12-note];
  }
  if (octaveMap== 2){
     Serial.print("Freq:");
     Serial.print(Octave4Table[11-note]);
    return Octave4Table[12-note];
  }
  if (octaveMap == 3){
     Serial.print("Freq:");
     Serial.print(Octave5Table[11-note]);
    return Octave5Table[12-note];
  }
  if (octaveMap == 4){
     Serial.print("Freq:");
     Serial.print(Octave6Table[11-note]);
    return Octave6Table[12-note];
  }

}

void audioOn() {
  // Set up PWM to 31.25kHz, phase accurate
  TCCR2A = _BV(COM2B1) | _BV(WGM20);
  TCCR2B = _BV(CS20);
  TIMSK2 = _BV(TOIE2);
}

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
}

void loop() {

  // Stepped Fotomapping to MIDI notes: C, Db, D, Eb, E, F...
  syncPhaseInc = mapOctave();

  // Stepped pentatonic mapping: D, E, G, A, B
  // syncPhaseInc = mapPentatonic(analogRead(SYNC_CONTROL));

  grainPhaseInc  = mapPhaseInc(analogRead(GRAIN_FREQ_CONTROL)) / 2;
  grainDecay     = analogRead(GRAIN_DECAY_CONTROL) / 8;
  /*Serial.print("  Grain:");
  Serial.print(analogRead(GRAIN_FREQ_CONTROL));
  Serial.print("  Decay:");
  Serial.print(analogRead(GRAIN_DECAY_CONTROL));*/
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
