
//
// Analog in 0: Grain 1 pitch
// Analog in 1: Octave Selector
// Analog in 2: Grain 1 decay
// Analog in 4: Grain repetition frequency

#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>

uint16_t syncPhaseAcc;
uint16_t syncPhaseInc;
uint16_t grainPhaseAcc;
uint16_t grainPhaseInc;
uint16_t grainAmp;
uint8_t grainDecay;

// Map Analogue channels
//#define SYNC_CONTROL         (4)
#define GRAIN_FREQ_CONTROL   (0)
#define GRAIN_DECAY_CONTROL  (2)
#define OCTAVE_CONTROL       (1)

// Map Digital channels

/*
#define C         13
#define Csharp    12
#define D         11
#define Dsharp    10
#define E         9
#define F         8
#define Fsharp    7
#define G         6
#define Gsharp    5
#define A         4
#define Asharp    3
#define B         2
#define Cupper    1
*/

// Changing these will also requires rewriting audioOn()

#if defined(__AVR_ATmega8__)
//
// On old ATmega8 boards.
//    Output is on pin 11
//
#define LED_PIN       2
#define LED_PORT      PORTD
#define LED_BIT       5
#define PWM_PIN       11
#define PWM_VALUE     OCR2
#define PWM_INTERRUPT TIMER2_OVF_vect
#elif defined(__AVR_ATmega1280__)
//
// On the Arduino Mega
//    Output is on pin 3
//
//#define LED_PIN       2
//#define LED_PORT      PORTD
#define LED_PIN       13
#define LED_PORT      PORTB
//#define LED_BIT       7
#define PWM_PIN       3
#define PWM_VALUE     OCR3C
#define PWM_INTERRUPT TIMER3_OVF_vect
#else
//
// For modern ATmega168 and ATmega328 boards
//    Output is on pin 3
//
#define PWM_PIN       3
#define PWM_VALUE     OCR2B
#define LED_PIN       2
#define LED_PORT      PORTD
#define LED_BIT       5
#define PWM_INTERRUPT TIMER2_OVF_vect
#endif

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

// Stepped chromatic mapping
//
uint16_t midiTable[] = {
  17,18,19,20,22,23,24,26,27,29,31,32,34,36,38,41,43,46,48,51,54,58,61,65,69,73,
  77,82,86,92,97,103,109,115,122,129,137,145,154,163,173,183,194,206,218,231,
  244,259,274,291,308,326,346,366,388,411,435,461,489,518,549,581,616,652,691,
  732,776,822,871,923,978,1036,1097,1163,1232,1305,1383,1465,1552,1644,1742,
  1845,1955,2071,2195,2325,2463,2610,2765,2930,3104,3288,3484,3691,3910,4143,
  4389,4650,4927,5220,5530,5859,6207,6577,6968,7382,7821,8286,8785,9301,9854,
  10440,11060,11718,12415,13153,13935,14764,15642,16572,17557,18601,19708,20885,
  22121,23436,24830,26306
};
uint16_t mapMidi(uint16_t input) {
  return (midiTable[(1023-input) >> 3]);
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

  /*input = analogRead(SYNC_CONTROL);
  // note = ((-0.00002*input*input)+(0.0435*input)-9.15);*/
  if (digitalRead(8)==0){
    selectOctave(0);
  }
  else if (digitalRead(9)==0){
    selectOctave(1);
  }
  else if (digitalRead(10)==0){
    selectOctave(2);
  }
  else if (digitalRead(11)==0){
    selectOctave(3);
  }
  else if (digitalRead(12)==0){
    selectOctave(4);
  }
  else if (digitalRead(13)==0){
    selectOctave(5);
  }
  
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
    return Octave2Table[11-note];
  }
  if (octaveMap == 1){
     Serial.print("Freq:");
     Serial.print(Octave3Table[11-note]);
    return Octave3Table[11-note];
  }
  if (octaveMap== 2){
       Serial.print("Freq:");
     Serial.print(Octave4Table[11-note]);
    return Octave4Table[11-note];
  }
  if (octaveMap == 3){
       Serial.print("Freq:");
     Serial.print(Octave5Table[11-note]);
    return Octave5Table[11-note];
  }
  if (octaveMap == 4){
       Serial.print("Freq:");
     Serial.print(Octave6Table[11-note]);
    return Octave6Table[11-note];
  }
}

void audioOn() {
#if defined(__AVR_ATmega8__)
  // ATmega8 has different registers
  TCCR2 = _BV(WGM20) | _BV(COM21) | _BV(CS20);
  TIMSK = _BV(TOIE2);
#elif defined(__AVR_ATmega1280__)
  TCCR3A = _BV(COM3C1) | _BV(WGM30);
  TCCR3B = _BV(CS30);
  TIMSK3 = _BV(TOIE3);
#else
  // Set up PWM to 31.25kHz, phase accurate
  TCCR2A = _BV(COM2B1) | _BV(WGM20);
  TCCR2B = _BV(CS20);
  TIMSK2 = _BV(TOIE2);
#endif
}

void setup() {
  
  pinMode(PWM_PIN,OUTPUT);
  
  pinMode(13,INPUT_PULLUP);
  pinMode(12,INPUT_PULLUP);
  pinMode(11,INPUT_PULLUP);
  pinMode(10,INPUT_PULLUP);
  pinMode(9,INPUT_PULLUP);
  pinMode(8,INPUT_PULLUP);
  
  
  /*
  pinMode(C,INPUT);
  pinMode(Csharp,INPUT);
  pinMode(D,INPUT);
  pinMode(Dsharp,INPUT);
  pinMode(E,INPUT);
  pinMode(F,INPUT);
  pinMode(Fsharp,INPUT);
  pinMode(G,INPUT);
  pinMode(Gsharp,INPUT);
  pinMode(A,INPUT);
  pinMode(Asharp,INPUT);
  pinMode(B,INPUT);
  pinMode(Cupper,INPUT);
  */
/*
  DDRB = 0b00000000;    
  PORTB = 0b11111111;  
  */
  
  audioOn();
  
  pinMode(LED_PIN,OUTPUT);
  Serial.begin(9600);
}

void loop() {
  // The loop is pretty simple - it just updates the parameters for the oscillators.
  //
  // Avoid using any functions that make extensive use of interrupts, or turn interrupts off.
  // They will cause clicks and poops in the audio.
  
  // Smooth frequency mapping
  //syncPhaseInc = mapPhaseInc(analogRead(SYNC_CONTROL)) / 4;
  
  // Stepped mapping to MIDI notes: C, Db, D, Eb, E, F...
  syncPhaseInc = mapOctave();
  
  // Stepped pentatonic mapping: D, E, G, A, B
  // syncPhaseInc = mapPentatonic(analogRead(SYNC_CONTROL));

  grainPhaseInc  = mapPhaseInc(analogRead(GRAIN_FREQ_CONTROL)) / 2;
  grainDecay     = analogRead(GRAIN_DECAY_CONTROL) / 8;
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
    LED_PORT ^= 1 << LED_BIT; // Faster than using digitalWrite
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
