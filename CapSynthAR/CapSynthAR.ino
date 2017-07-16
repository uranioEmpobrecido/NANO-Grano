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
#define GRAIN_FREQ_CONTROL   A0
#define GRAIN_DECAY_CONTROL  A1
#define OCTAVE_CONTROL       A2
#define ATTACK_CONTROL       A3
#define RELEASE_CONTROL      A4

//VCA Control
#define VCA_CONTROL          A5

//MASK
#define PORT_B  0
#define PORT_D  1

//Capacitive Threshold
#define Threshold 6

// Map Digital channels
#define C         13  //PB5 - OK
#define Csharp    12  //PB4 - OK
#define D         11  //PB3 - OK 
#define Dsharp    10  //PB2 - OK 
#define E         9   //PB1 - OK
#define F         8   //PB0 - OK
#define Fsharp    7   //PD7 - OK 
#define G         6   //PD6 - OK
#define Gsharp    5   //PD5 - OK
#define A         4   //PD4 - OK 
#define Asharp    2   //PD2 - OK
#define B         1   //PD1 - OK 
#define Cupper    0   //PD0 - OK

#define PWM_PIN       3
#define PWM_VALUE     OCR2B
#define PWM_INTERRUPT TIMER2_OVF_vect

uint16_t freqMap, octaveMap;

uint16_t input, inputOct;

uint8_t note;

bool attackNote = false;
bool releaseNote = false;

bool noAttack = false;
bool noRelease = false;

bool dbAdjust = false;
bool dbLowAdj = true;

uint8_t attackCount, releaseCount, pulseCount;

// Smooth logarithmic mapping
//
uint16_t antilogTable[64] = {
  64830,64132,63441,62757,62081,61413,60751,60097,59449,58809,58176,57549,56929,56316,55709,55109,
  54515,53928,53347,52773,52204,51642,51085,50535,49991,49452,48920,48393,47871,47356,46846,46341,
  45842,45348,44859,44376,43898,43425,42958,42495,42037,41584,41136,40693,40255,39821,39392,38968,
  38548,38133,37722,37316,36914,36516,36123,35734,35349,34968,34591,34219,33850,33486,33125,32768
};
uint16_t mapPhaseInc(uint16_t input) {
  //Serial.println(antilogTable[input & 0x3f] >> ((input >> 6)));
  return (antilogTable[input & 0x3f]) >> (input >> 6);
}

//---------- Thresholdacitive Touch sensing -----------------------------
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

// Vibiss mapping OCTAVE 2
//
uint16_t Octave2Table[13] = {65,69,73,78,82,87,93,98,104,110,117,123,131};

// Vibiss mapping OCTAVE 3
//
uint16_t Octave3Table[13] = {131,139,147,156,165,175,185,196,208,220,233,247,262};

// Vibiss mapping OCTAVE 4
//
uint16_t Octave4Table[13] = {262,277,294,311,330,349,370,392,415,440,466,494,523};

// Vibiss mapping OCTAVE 5
//
uint16_t Octave5Table[13] = {523,554,587,622,659,698,740,784,831,880,932,988,1047};

// Vibiss mapping OCTAVE 6
//
uint16_t Octave6Table[13] = {1047,1109,1175,1245,1319,1397,1480,1568,1661,1760,1865,1976,2093};

void setLow(int pin){

  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);

  }

void setHighZ(int pin){

  pinMode(pin, INPUT);

  }

void setHigh(int pin){

  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);

  }

void pulseLow(void){

    setLow(VCA_CONTROL);
    delayMicroseconds(500);
    setHighZ(VCA_CONTROL);
    delayMicroseconds(500); //Fixed OK
}

void pulseHigh(void){

    setHigh(VCA_CONTROL);
    delayMicroseconds(500);
    setHighZ(VCA_CONTROL);
    delayMicroseconds(500); //Fixed OK
}

void attackProcess(void){

  if (analogRead(ATTACK_CONTROL)>125){
    noAttack = false;
    if (32 > attackCount && pulseCount < 32){ 
      attackCount++;
      pulseCount++;
      pulseHigh();
      }
      delay((analogRead(ATTACK_CONTROL)-125)/25);
      } else { 
        noAttack = true;
        if (32 > attackCount && pulseCount < 32){ 
          attackCount++;
          pulseCount++;
          pulseHigh();
          }
      }
  releaseCount = 0;
  releaseNote = true;
  attackNote = false;
}

void releaseProcess(void){
  
  uint8_t i = 0;
  if (attackCount >= 31){ i = 32;}
  else i = attackCount;
  if (analogRead(RELEASE_CONTROL)>125){
    noRelease = false;
    while (releaseCount < i && releaseNote){ 
      while (releaseCount < i){
    pulseLow();
    releaseCount++;
    delay((analogRead(RELEASE_CONTROL)-125)/25);
    }
    dbAdjust = false;
  }
 } else { 
    noRelease = false;
    while (releaseCount < i && releaseNote){ 
      while (releaseCount < i){
        pulseLow();
        releaseCount++;
        }
    }
    dbAdjust = false;
  noRelease = true;
  }
  pulseCount = 0;
  attackCount = 0;
  attackNote = true;
}

uint16_t mapOctave() {
  if (readCapacitivePin(C)>Threshold){
    attackProcess();
    return selectOctave(12);
  }
  else if (readCapacitivePin(Csharp)>Threshold){
    attackProcess();
    return selectOctave(11);
  }
  else if (readCapacitivePin(D)>Threshold){
    attackProcess();
    return selectOctave(10);
  }
  else if (readCapacitivePin(Dsharp)>Threshold){
    attackProcess();
    return selectOctave(9);
  }
  else if (readCapacitivePin(E)>Threshold){
    attackProcess();
    return selectOctave(8);
  }
  else if (readCapacitivePin(F)>Threshold){
    attackProcess();
    return selectOctave(7);
  }
  else if (readCapacitivePin(Fsharp)>Threshold){
    attackProcess();
    return selectOctave(6);
  }
  else if (readCapacitivePin(G)>Threshold){
    attackProcess();
    return selectOctave(5);
  }
  else if (readCapacitivePin(Gsharp)>Threshold){
    attackProcess();
    return selectOctave(4);
  }
  else if (readCapacitivePin(A)>Threshold){
    attackProcess();
    return selectOctave(3);
  }
  else if (readCapacitivePin(Asharp)>Threshold){
    attackProcess();
    return selectOctave(2);
  }
  else if (readCapacitivePin(B)>Threshold){
    attackProcess();
    return selectOctave(1);
  }
  else if (readCapacitivePin(Cupper)>Threshold){
    attackProcess();
    return selectOctave(0);
  }
  else releaseProcess();
  return 0;
}


uint16_t selectOctave(uint8_t note){

  octaveMap = (analogRead(OCTAVE_CONTROL)*5)/1024;
  /*
  Serial.print("  Note:");
  Serial.print(note);
  Serial.print("  Octave:");
  Serial.print(octaveMap);
  Serial.print("\n");
  */
  if (octaveMap == 0){
     //Serial.print("Freq:");
     //Serial.print(Octave2Table[12-note]);
    return Octave2Table[12-note];
  }
  if (octaveMap == 1){
     //Serial.print("Freq:");
     //Serial.print(Octave3Table[12-note]);
    return Octave3Table[12-note];
  }
  if (octaveMap== 2){
     //Serial.print("Freq:");
     //Serial.print(Octave4Table[12-note]);
    return Octave4Table[12-note];
  }
  if (octaveMap == 3){
     //Serial.print("Freq:");
     //Serial.print(Octave5Table[12-note]);
    return Octave5Table[12-note];
  }
  if (octaveMap == 4){
     //Serial.print("Freq:");
     //Serial.print(Octave6Table[12-note]);
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
  
  syncPhaseInc   =  mapOctave();
  grainPhaseInc  =  mapPhaseInc(analogRead(GRAIN_FREQ_CONTROL)) / 2;
  grainDecay     =  analogRead(GRAIN_DECAY_CONTROL) / 8;
  
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
