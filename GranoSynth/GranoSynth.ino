// GRANO, the granular synthesiser
//
// All in the spirit of open-source and open-hardware
// by Jorge Gutiérrez
//
// NANO.es
//
// 2017 Spain
//
// Analog in 0: Grain Frecuency Control
// Analog in 1: Octave Selector
// Analog in 2: Grain Decay Control
// Analog in 3: Effect Control 
// Analog in 4: Effect Selector
// Analog in 5: Tri-State Volume Contro

#include <avr/io.h>
#include <avr/interrupt.h>

uint16_t syncPhaseAcc;
uint16_t syncPhaseInc;
uint16_t grainPhaseAcc;
uint16_t grainPhaseInc;
uint16_t grainAmp;
uint16_t grainDecay;

// Map Analogue channels
#define GRAIN_FREQ_CONTROL   A1
#define GRAIN_DECAY_CONTROL  A0
#define OCTAVE_CONTROL       A2
#define ATTACK_CONTROL       A4
#define RELEASE_CONTROL      A3

//VCA Control
#define VCA_CONTROL          A5
#define VCA_NORMAL_AMPLITUDE  0
#define VCA_MAX_AMPLITUDE    20

//Fixed DECAY value
#define DECAY                64  //48
//Fixed DECAY effect value
#define DECAY_EFF             1

//Fixed FREQ value
#define FREQ                255

//MASK
#define PORT_B                0
#define PORT_D                1

//Envelope Selector
#define ENVELOPE              1
#define NO_ENVELOPE           0

//Capacitive Threshold
#define Threshold             5

// Map Digital channels
#define C                     0 
#define Csharp                1 
#define D                     2 
#define Dsharp                4  
#define E                     5   
#define F                     6 
#define Fsharp                7  
#define G                    13  
#define Gsharp               12  
#define A                    11   
#define Asharp               10   
#define B                     9  
#define Cupper                8  

#define PWM_PIN               3
#define PWM_VALUE     OCR2B
#define PWM_INTERRUPT TIMER2_OVF_vect

uint16_t freqMap, octaveMap, arpeggioNote;

uint16_t input, inputOct;

uint8_t note, releasePulse, arpeggioState, arpeggioOctave, volume;

bool tremoloON = false;
bool volumeAdjust = false;

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
  
  if (input > 512){ 
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

//---------- Thresholdacitive Touch sensing -----------------------------
uint8_t readCapacitivePin(int pinToMeasure) {

  // Variables used to translate from Arduino to AVR pin naming
  volatile uint8_t* port;
  volatile uint8_t* ddr;
  volatile uint8_t* pin;
  //  Here we translate the input pin number from
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

  if      (*pin & bitmask) { cycles =  0;}
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

  //  End of timing-critical section; turn interrupts back on if they were on before,
  //  or leave them off if they were off before

  SREG = SREG_old;
  
  //  Discharge the pin again by setting it low and output
  //  It's important to leave the pins low if you want to 
  //  be able to touch more than 1 sensor at a time - if
  //  the sensor is left pulled high, when you touch
  //  two sensors, your body will transfer the charge between
  //  sensors.
  
  *port &= ~(bitmask);
  *ddr  |= bitmask;
  return cycles;
}

//OCTAVE 2
//
uint16_t Octave2Table[13] = {68,73,76,78,85,92,96,102,108,114,121,129,136};

//OCTAVE 3
//
uint16_t Octave3Table[13] = {135,144,152,162,171,183,193,204,216,230,243,257,274};

//OCTAVE 4
//
uint16_t Octave4Table[13] = {274,290,308,325,344,365,387,409,434,460,487,515,546};

//OCTAVE 5
//
uint16_t Octave5Table[13] = {546,579,615,650,689,728,772,818,867,918,974,1032,1095};

//OCTAVE 6
//
uint16_t Octave6Table[13] = {1093,1159,1227,1300,1378,1459,1546,1638,1742,1838,1947,2063,2186};

//ARPEGGIATOR TABLE
//
uint16_t ArpeggioTable[65] = {65,69,73,78,82,87,93,98,104,110,117,123,131,                  //OCTAVE2

                              139,147,156,165,175,185,196,208,220,233,247,262,              //OCTAVE3
                              
                              277,294,311,330,349,370,392,415,440,466,494,523,              //OCTAVE4
                              
                              554,587,622,659,698,740,784,831,880,932,988,1047,             //OCTAVE5
                              
                              1109,1175,1245,1319,1397,1480,1568,1661,1760,1865,1976,2093}; //OCTAVE6

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
    delayMicroseconds(500); 
    volume--;
}

void pulseHigh(void){

    setHigh(VCA_CONTROL);
    delayMicroseconds(500);
    setHighZ(VCA_CONTROL);
    delayMicroseconds(500);
    volume++;
}

void noEnvelopeAdj(void){
  
  if ((analogRead(ATTACK_CONTROL)<25)&&(analogRead(RELEASE_CONTROL)<25)){
    if (volume < VCA_MAX_AMPLITUDE){
      pulseHigh();
      delay(1);      
    }
  }
  
}

void attackProcess(void){

  if ((analogRead(ATTACK_CONTROL)>25)||(analogRead(RELEASE_CONTROL)>25)){
    if (VCA_MAX_AMPLITUDE > volume){ 
    pulseHigh();
    delay((analogRead(ATTACK_CONTROL)+25)/25);
    }
  }
}

void releaseProcess(void){
  
  if ((analogRead(ATTACK_CONTROL)>25)||(analogRead(RELEASE_CONTROL)>25)){
    while (volume > VCA_NORMAL_AMPLITUDE){
    pulseLow();
    delay((analogRead(RELEASE_CONTROL)+25)/25);
    }
  }
}


uint16_t mapArpeggio(){

  arpeggioOctave = (1+((analogRead(OCTAVE_CONTROL)*5)/1024));
  
  if (readCapacitivePin(C)>Threshold){
    arpeggioNote = 1;
  }
  else if (readCapacitivePin(Csharp)>Threshold){
    arpeggioNote = 2;
  }
  else if (readCapacitivePin(D)>Threshold){
    arpeggioNote = 3;
  }
  else if (readCapacitivePin(Dsharp)>Threshold){
    arpeggioNote = 4;
  }
  else if (readCapacitivePin(E)>Threshold){
    arpeggioNote = 5;
  }
  else if (readCapacitivePin(F)>Threshold){
    arpeggioNote = 6;
  }
  else if (readCapacitivePin(Fsharp)>Threshold){
    arpeggioNote = 7;
  }
  else if (readCapacitivePin(G)>Threshold){
    arpeggioNote = 8;
  }
  else if (readCapacitivePin(Gsharp)>Threshold){
    arpeggioNote = 9;
  }
  else if (readCapacitivePin(A)>Threshold){
    arpeggioNote = 10;
  }
  else if (readCapacitivePin(Asharp)>Threshold){
    arpeggioNote = 11;
  }
  else if (readCapacitivePin(B)>Threshold){
    arpeggioNote = 12;
  }
  else if (readCapacitivePin(Cupper)>Threshold){
    arpeggioNote = 13;
  }
  else return 0;
  
  if      (arpeggioOctave == 1){ return arpeggioNote;    }
  else if (arpeggioOctave == 2){ return arpeggioNote+13; }
  else if (arpeggioOctave == 3){ return arpeggioNote+26; }
  else if (arpeggioOctave == 4){ return arpeggioNote+39; }
  else if (arpeggioOctave == 5){ return arpeggioNote+52; }
}

uint16_t mapOctave(bool Envelope) {

  if (Envelope){
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
  
  } else {
  
    if (readCapacitivePin(C)>Threshold){
    return selectOctave(12);
    }
    else if (readCapacitivePin(Csharp)>Threshold){
    return selectOctave(11);
    }
    else if (readCapacitivePin(D)>Threshold){
    return selectOctave(10);
    }
    else if (readCapacitivePin(Dsharp)>Threshold){
    return selectOctave(9);
    }
    else if (readCapacitivePin(E)>Threshold){
    return selectOctave(8);
    }
    else if (readCapacitivePin(F)>Threshold){
    return selectOctave(7);
    }
    else if (readCapacitivePin(Fsharp)>Threshold){
    return selectOctave(6);
    }
    else if (readCapacitivePin(G)>Threshold){
    return selectOctave(5);
    }
    else if (readCapacitivePin(Gsharp)>Threshold){
    return selectOctave(4);
    }
    else if (readCapacitivePin(A)>Threshold){
    return selectOctave(3);
    }
    else if (readCapacitivePin(Asharp)>Threshold){
    return selectOctave(2);
    }
    else if (readCapacitivePin(B)>Threshold){
    return selectOctave(1);
    }
    else if (readCapacitivePin(Cupper)>Threshold){
    return selectOctave(0);
    }
    else return 0;    
  }
}


uint16_t selectOctave(uint8_t note){

  octaveMap = (analogRead(OCTAVE_CONTROL)*5)/1024;

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

void GPIOSetup(void){
  
  pinMode(PWM_PIN,OUTPUT);
  
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
  
}

void granularEffect(void){
  //GRAIN EFFECT
  syncPhaseInc   =  mapOctave(ENVELOPE);
  grainPhaseInc  =  mapPhaseInc(analogRead(FREQ))/2;  
  if (analogRead(FREQ) < 512){
    grainDecay   =  DECAY_EFF;  
  }
  else {
    grainDecay   = ((analogRead(GRAIN_FREQ_CONTROL)-512)/2);
  }
}

void tremoloEffect(void){
  //TREMOLO EFFECT
  if (!tremoloON){
    syncPhaseInc   =  mapOctave(NO_ENVELOPE);
    grainPhaseInc  =  mapPhaseInc(FREQ);
    grainDecay     =  DECAY;  //analogRead(GRAIN_DECAY_CONTROL) / 8;
    delay(analogRead(GRAIN_FREQ_CONTROL)/4);
    tremoloON = true;
  } else {
    syncPhaseInc   =  0;
    grainPhaseInc  =  mapPhaseInc(FREQ);
    grainDecay     =  DECAY;  //analogRead(GRAIN_DECAY_CONTROL) / 8;
    delay(analogRead(GRAIN_FREQ_CONTROL)/8);
    tremoloON = false;
  }  
}

void arpeggiatorEffect(void){
  //ARPEGIATTOR EFFECT
  if (mapArpeggio() != 0){
    if (arpeggioState == 0){
      syncPhaseInc   =  ArpeggioTable[mapArpeggio()];
      grainPhaseInc  =  mapPhaseInc(FREQ);
      grainDecay     =  DECAY; 
      delay(analogRead(GRAIN_FREQ_CONTROL)/2);
      arpeggioState++;
      }
    else if (arpeggioState == 1){
      syncPhaseInc   =  ArpeggioTable[mapArpeggio()+5];
      grainPhaseInc  =  mapPhaseInc(FREQ);
      grainDecay     =  DECAY; 
      delay(analogRead(GRAIN_FREQ_CONTROL)/2);
      arpeggioState++;
      }
    else if (arpeggioState == 2){
      syncPhaseInc   =  ArpeggioTable[mapArpeggio()+7];
      grainPhaseInc  =  mapPhaseInc(FREQ);
      grainDecay     =  DECAY; 
      delay(analogRead(GRAIN_FREQ_CONTROL)/2);
      arpeggioState++;
      }
    else if (arpeggioState == 3){
      syncPhaseInc   =  ArpeggioTable[mapArpeggio()+10];
      grainPhaseInc  =  mapPhaseInc(FREQ);
      grainDecay     =  DECAY; 
      delay(analogRead(GRAIN_FREQ_CONTROL)/2);
      arpeggioState  =  0;
      }
  }
  else syncPhaseInc  =  0;
    
}

void volumeAdjG(void){

  noEnvelopeAdj();
  if (volumeAdjust){
    while (volume > VCA_NORMAL_AMPLITUDE){
      pulseLow();
      delay(1);
    }
    volumeAdjust = false;
  }
}


void volumeAdjTA(void){

  if (!volumeAdjust){
    while (volume < VCA_MAX_AMPLITUDE){
      pulseHigh();
      delay(1);
      }
    volumeAdjust = true;
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

void setup() {
  
  GPIOSetup();
  audioOn();
  //Serial.begin(9600);
}


void loop() {

  if (analogRead(GRAIN_DECAY_CONTROL) < 340){
    volumeAdjG();
    granularEffect();
  }
  else if (analogRead(GRAIN_DECAY_CONTROL) < 680){
    volumeAdjTA();
    tremoloEffect();
  }
  else if (analogRead(GRAIN_DECAY_CONTROL) < 1030){
    volumeAdjTA();
    arpeggiatorEffect();
  } 
}
