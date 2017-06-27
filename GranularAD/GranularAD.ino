// Analog in 0: Grain Frecuency Control
// Analog in 1: Octave Selector
// Analog in 2: Grain Decay Control
// Analog in 3: Attack Control
// Analog in 4: Decay Control
// Analog in 5: Tri-State Volume Control

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
#define GRAIN_FREQ_CONTROL   (0)
#define OCTAVE_CONTROL       (1)
#define GRAIN_DECAY_CONTROL  (2)
#define ATTACK_CONTROL       (3)
#define DECAY_CONTROL        (4)

// Map TriState channel
#define pinTriState A5

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

bool keyPressed  = false;
bool keyReleased = false;

bool attackEnable  = false;
bool decayEnable = false;

bool decayState = false;

int8_t dB = -20;
int8_t maxdB = 3;
int8_t mindB = -20;

uint16_t attack, decay;

uint16_t freqMap, octaveMap;

uint16_t input, inputOct;
uint8_t note, state;

uint16_t decayNote;

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

void setLow(int pin){

  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);

  }

void setHighZ(int pin){

  pinMode(pinTriState, INPUT);

  }

void setHigh(int pin){

  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);

  }

void pulseLow(int pulses){

  decay  = (analogRead(DECAY_CONTROL)/100)+1;
  for (int i = 0; i <= pulses; i++){
    setLow(pinTriState);
    delay(decay);//?
    setHighZ(pinTriState);
    delay(8); //Fixed OK
  }
  //decay = 0;
}

void pulseHigh(int pulses){

  attack = (analogRead(ATTACK_CONTROL)+200);
  for (int i = 0; i <= pulses; i++){
    setHigh(pinTriState);
    delayMicroseconds(attack); //200 to 1600 OK
    setHighZ(pinTriState);
    delayMicroseconds(1000); //Fixed OK
  }
  //attack = 0;
}


void ADProcess(){
  if (analogRead(ATTACK_CONTROL) > 200 && keyPressed){
    Serial.println("Attack");
    pulseHigh(20);
  }
  else if (analogRead(DECAY_CONTROL) > 200 && keyReleased){
    Serial.println("Decay");
    pulseLow(20);
    keyPressed  = false;
    keyReleased = false;
  }
}

uint16_t mapOctave() {
  if (digitalRead(C)==0){
    keyPressed = true;
    ADProcess();
    return selectOctave(9);
  }
  /*else if (digitalRead(Csharp)==0){
    keyPressed = true;
    ADProcess();
    return selectOctave(1);
  }*/
  else if (digitalRead(D)==0){
    keyPressed = true;
    ADProcess();
    return selectOctave(7);
  }
 /* else if (digitalRead(Dsharp)==0){
    keyPressed = true;
    ADProcess();
    return selectOctave(3);
  }*/
  else if (digitalRead(E)==0){
    keyPressed = true;
    ADProcess();
    return selectOctave(5);
  }
  else if (digitalRead(F)==0){
    keyPressed = true;
    ADProcess();
    return selectOctave(4);
  }
 /* else if (digitalRead(Fsharp)==0){
    keyPressed = true;
    ADProcess();
    return selectOctave(6);
  }*/
  else if (digitalRead(G)==0){
    keyPressed = true;
    ADProcess();
    return selectOctave(2);
  }
  /*else if (digitalRead(Gsharp)==0){
    keyPressed = true;
    ADProcess();
    return selectOctave(8);
  }*/
  else if (digitalRead(A)==0){
    keyPressed = true;
    ADProcess();
    return selectOctave(0);
  }
 /* else if (digitalRead(Asharp)==0){
    keyPressed = true;
    ADProcess();
    return selectOctave(10);
  }
  else if (digitalRead(B)==0){
    keyPressed = true;
    ADProcess();
    return selectOctave(11);
  }
  else if (digitalRead(Cupper)==0){
    keyPressed = true;
    ADProcess();
    return selectOctave(12);
  }*/
  else {
    if (keyPressed) {
      keyReleased = true;
      ADProcess();
    }
    return 0;
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
    /* Serial.print("Freq:");
     Serial.print(Octave2Table[11-note]);*/
    return Octave2Table[11-note];
  }
  if (octaveMap == 1){
    // Serial.print("Freq:");
     //Serial.print(Octave3Table[11-note]);
    return Octave3Table[11-note];
  }
  if (octaveMap== 2){
     //  Serial.print("Freq:");
    // Serial.print(Octave4Table[11-note]);
    return Octave4Table[11-note];
  }
  if (octaveMap == 3){
      // Serial.print("Freq:");
    // Serial.print(Octave5Table[11-note]);
    return Octave5Table[11-note];
  }
  if (octaveMap == 4){
      // Serial.print("Freq:");
     //Serial.print(Octave6Table[11-note]);
    return Octave6Table[11-note];
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

  pulseHigh(8); //Set gain to -10 dB

  pinMode(13,INPUT_PULLUP);
  pinMode(12,INPUT_PULLUP);
  pinMode(11,INPUT_PULLUP);
  pinMode(10,INPUT_PULLUP);
  pinMode(9,INPUT_PULLUP);
  pinMode(8,INPUT_PULLUP);
  /*
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
  */
/*
  DDRB = 0b00000000;
  PORTB = 0b11111111;
  */
  Serial.begin(9600);
}

void AudioProcess(){
  
  state = syncPhaseInc;
  syncPhaseInc = mapOctave();
  grainPhaseInc  = mapPhaseInc(analogRead(GRAIN_FREQ_CONTROL)) / 2;
  grainDecay     = analogRead(GRAIN_DECAY_CONTROL) / 8;
  
}

void loop() {
  
  AudioProcess();
  
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
