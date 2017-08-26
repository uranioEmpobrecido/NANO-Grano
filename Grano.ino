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
#include <avr/pgmspace.h>
#include "GranoVariables.h"

#define PWM_VALUE             OCR2B
#define PWM_INTERRUPT         TIMER2_OVF_vect

float noteDuty(void){
  return ((analogRead(CONTROL_2)*(analogRead(CONTROL_1)/1023.0))+0.15); 
}

void stopPlayback()
{
  // Disable playback per-sample interrupt.
  TIMSK1 &= ~_BV(OCIE1A);
  
  // Disable the per-sample timer completely.
  //TCCR1B &= ~_BV(CS10);
  
  // Disable the PWM timer.
  //TCCR2B &= ~_BV(CS10);

}

void startPlayback(unsigned char const *data, int arraylength)
{
  sounddata_data = data;
  sounddata_length = arraylength;
  
  // Set up Timer 2 to do pulse width modulation on the speaker
  // pin.
  
  // Use internal clock (datasheet p.160)
  ASSR &= ~(_BV(EXCLK) | _BV(AS2));
  
  // Set fast PWM mode  (p.157)
  TCCR2A |= _BV(WGM21) | _BV(WGM20);
  TCCR2B &= ~_BV(WGM22);
  
  // Do non-inverting PWM on pin OC2B (p.155)
  // On the Arduino this is pin 3.
  TCCR2A = (TCCR2A | _BV(COM2B1)) & ~_BV(COM2B0);
  TCCR2A &= ~(_BV(COM2A1) | _BV(COM2A0));
  
  // No prescaler (p.158)
  TCCR2B = (TCCR2B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);
  
  // Set initial pulse width to the first sample.
  OCR2B = pgm_read_byte(&sounddata_data[0]);
  
  
  // Set up Timer 1 to send a sample every interrupt.
  
  cli();
  
  // Set CTC mode (Clear Timer on Compare Match) (p.133)
  // Have to set OCR1A *after*, otherwise it gets reset to 0!
  TCCR1B = (TCCR1B & ~_BV(WGM13)) | _BV(WGM12);
  TCCR1A = TCCR1A & ~(_BV(WGM11) | _BV(WGM10));
  
  // No prescaler (p.134)
  TCCR1B = (TCCR1B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);
  
  // Set the compare register (OCR1A).
  // OCR1A is a 16-bit register, so we have to do this with
  // interrupts disabled to be safe.
  OCR1A = F_CPU / SAMPLE_RATE;    // 16e6 / 8000 = 2000
  
  // Enable interrupt when TCNT1 == OCR1A (p.136)
  TIMSK1 |= _BV(OCIE1A);
  
  lastSample = pgm_read_byte(&sounddata_data[sounddata_length-1]);
  sample = 0;
  sei();
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
  
  if ((analogRead(CONTROL_1)<25)&&(analogRead(CONTROL_2)<25)){
    if (volume < VCA_MAX){
      pulseHigh();
      delay(1);      
    }
  }
  
}

void attackProcess(void){

  if ((analogRead(CONTROL_1)>25)||(analogRead(CONTROL_2)>25)){
    if (VCA_MAX > volume){ 
    pulseHigh();
    delay((analogRead(CONTROL_1)+25)/25);
    }
  }
}

void releaseProcess(void){
  
  if ((analogRead(CONTROL_1)>25)||(analogRead(CONTROL_2)>25)){
    while (volume > VCA_NORMAL){
    pulseLow();
    delay((analogRead(CONTROL_2)+25)/25);
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
  beatISR = false;
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
    grainDecay   = ((analogRead(EFFECT_AMT)-512)/2);
  }
}

void tremoloEffect(void){
  //TREMOLO EFFECT
  if (!tremoloON){
    syncPhaseInc   =  mapOctave(NO_ENVELOPE);
    grainPhaseInc  =  mapPhaseInc(FREQ);
    grainDecay     =  DECAY;  //analogRead(EFFECT_SELECTOR) / 8;
    delay(analogRead(CONTROL_2)/4 * (analogRead(CONTROL_1)/1025.0));
    tremoloON = true;
  } else if (tremoloON){
    syncPhaseInc   =  0;
    grainPhaseInc  =  mapPhaseInc(FREQ);
    grainDecay     =  DECAY;  //analogRead(EFFECT_SELECTOR) / 8;
    delay(analogRead(CONTROL_2)/4 * (1-(analogRead(CONTROL_1)/1025.0)));
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
      delay(analogRead(EFFECT_AMT)/2);
      arpeggioState++;
      }
    else if (arpeggioState == 1){
      syncPhaseInc   =  ArpeggioTable[mapArpeggio()+5];
      grainPhaseInc  =  mapPhaseInc(FREQ);
      grainDecay     =  DECAY; 
      delay(analogRead(EFFECT_AMT)/2);
      arpeggioState++;
      }
    else if (arpeggioState == 2){
      syncPhaseInc   =  ArpeggioTable[mapArpeggio()+7];
      grainPhaseInc  =  mapPhaseInc(FREQ);
      grainDecay     =  DECAY; 
      delay(analogRead(EFFECT_AMT)/2);
      arpeggioState++;
      }
    else if (arpeggioState == 3){
      syncPhaseInc   =  ArpeggioTable[mapArpeggio()+10];
      grainPhaseInc  =  mapPhaseInc(FREQ);
      grainDecay     =  DECAY; 
      delay(analogRead(EFFECT_AMT)/2);
      arpeggioState  =  0;
      }
  }
  else syncPhaseInc  =  0;
    
}

void volumeAdjG(void){

  noEnvelopeAdj();
  if (volumeAdjust){
    while (volume > VCA_NORMAL){
      pulseLow();
      delay(1);
    }
    volumeAdjust = false;
  }
}


void volumeAdjTA(void){

  if (!volumeAdjust){
    while (volume < VCA_MAX){
      pulseHigh();
      delay(1);
      }
    volumeAdjust = true;
  }
}

uint16_t mapSeq() {
  bool noteOk = false;
  while (!noteOk){
  if (readCapacitivePin(C)>Threshold){
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
  }
  else if (readCapacitivePin(B)>Threshold){
    noteOk = true;
    return selectOctave(1);
  }
  else if (readCapacitivePin(Cupper)>Threshold){
    noteOk = true;
    return 0;
  }
  }
}


void BeatEffect(void){

  if (readCapacitivePin(Cupper)>Threshold || beatOn){
    if (!setBeat){
      setBeatSequence();
      setBeat = true;
      beatOn  = true;
    }
    beatPlay();
  }

  if (readCapacitivePin(D)>Threshold){
    delay(150);
    while (readCapacitivePin(D)>Threshold){
      //Nothing
    }
    startPlayback(kick,sizeof(kick));
  }
  if (readCapacitivePin(E)>Threshold){
    delay(150);
    while (readCapacitivePin(E)>Threshold){
      //Nothing
    }
    startPlayback(snare,sizeof(snare));
  }
  if (readCapacitivePin(F)>Threshold){
    delay(150);
    while (readCapacitivePin(F)>Threshold){
      //Nothing
    }
    startPlayback(tick,sizeof(tick));
  }
  if (readCapacitivePin(G)>Threshold){
    delay(150);
    while (readCapacitivePin(G)>Threshold){
      //Nothing
    }
    startPlayback(bass,sizeof(bass));
  }
}

void setBeatSequence(void){

  delay(500);

  if (analogRead(EFFECT_SELECTOR) > 800){

  startPlayback(snare,1000);
  
  while (!selectBeat1){
    beat1 = mapBeat();
    beatLength1 = mapLength(beat1);
    selectBeat1 = true;}
    
  beatOK = false;
  delay(500);
  
  startPlayback(snare,sizeof(1000));
  
  while (!selectBeat2){
    beat2 = mapBeat();
    beatLength2 = mapLength(beat2);
    selectBeat2 = true;}
    
  beatOK = false;
  delay(500);
  
  startPlayback(snare,sizeof(1000));
  
  while (!selectBeat3){
    beat3 = mapBeat();
    beatLength3 = mapLength(beat3);
    selectBeat3 = true;}
    
  beatOK = false;
  delay(500);
  
  startPlayback(snare,sizeof(1000));
  
  while (!selectBeat4){
    beat4 = mapBeat();
    beatLength4 = mapLength(beat4);
    selectBeat4 = true;}
    
  beatOK = false;
  delay(500);

  startPlayback(snare,sizeof(1000));

  while (!selectBeat5){
    beat5 = mapBeat();
    beatLength5 = mapLength(beat5);
    selectBeat5 = true;}
    
  beatOK = false;
  delay(500);

  startPlayback(snare,sizeof(1000));

  while (!selectBeat6){
    beat6 = mapBeat();
    beatLength6 = mapLength(beat6);
    selectBeat6 = true;}
    
  beatOK = false;
  delay(500);

  startPlayback(snare,sizeof(1000));

  while (!selectBeat7){
    beat7 = mapBeat();
    beatLength7 = mapLength(beat7);
    selectBeat7 = true;}
    
  beatOK = false;
  delay(500);

  startPlayback(snare,sizeof(1000));

  while (!selectBeat8){
    beat8 = mapBeat();
    beatLength8 = mapLength(beat8);
    selectBeat8 = true;}
    
  beatOK = false;
  delay(500);

  startPlayback(snare,sizeof(1000));
  }
}

uint16_t mapLength(unsigned char * sound){

  if (sound == kick) { return kick_length ;}
  if (sound == snare){ return snare_length;}
  if (sound == tick) { return tick_length ;}
  if (sound == bass) { return bass_length ;}
  
}

unsigned char * mapBeat(void){

  while (!beatOK){
  if (readCapacitivePin(D)>Threshold){
    startPlayback(kick,sizeof(kick));
    beatOK = true;
    return kick;
    }
  else if (readCapacitivePin(E)>Threshold){
    startPlayback(snare,sizeof(snare));
    beatOK = true;
    return snare;
    }
  else if (readCapacitivePin(F)>Threshold){
    startPlayback(tick,sizeof(tick));
    beatOK = true;
    return tick;
    }
  else if (readCapacitivePin(G)>Threshold){
    startPlayback(bass,sizeof(bass));
    beatOK = true;
    return bass;
    }
    }
}

void beatPlay(void){

  if (readCapacitivePin(C)>Threshold){
    stateBeat = true;
    if (prevStateBeat != stateBeat){
      beatRun = !beatRun;
    }
  } else { stateBeat = false; }

  if (readCapacitivePin(Cupper)>Threshold){
    startPlayback(snare,sizeof(1000));
    deleteBeat();
    delay(1000);
  }

  if (beatRun){

  if (countBeat >= (analogRead(CONTROL_2)+25)){
    patternBeat++;
    countBeat = 0;
  }

  if (patternBeat == 8){ 
    patternBeat = 0; 
    beatstep1 = false;
    beatstep2 = false;
    beatstep3 = false;
    beatstep4 = false;
    beatstep5 = false;
    beatstep6 = false;
    beatstep7 = false;
    beatstep8 = false;
    }
  
  if (patternBeat == 0){
    if (!beatstep1) { startPlayback(beat1,beatLength1); beatstep1 = true; }
    countBeat++;
  }
  else if (patternBeat == 1){
    if (!beatstep2) { startPlayback(beat2,beatLength2); beatstep2 = true; }
    countBeat++;
  }
  else if (patternBeat == 2){
    if (!beatstep3) { startPlayback(beat3,beatLength3); beatstep3 = true; }
    countBeat++;
  }
  else if (patternBeat == 3){
    if (!beatstep4) { startPlayback(beat4,beatLength4); beatstep4 = true; }
    countBeat++;
  }
  else if (patternBeat == 4){
    if (!beatstep5) { startPlayback(beat5,beatLength5); beatstep5 = true; }
    countBeat++;
  }
  else if (patternBeat == 5){
    if (!beatstep6) { startPlayback(beat6,beatLength6); beatstep6 = true; }
    countBeat++;
  }
  else if (patternBeat == 6){
    if (!beatstep7) { startPlayback(beat7,beatLength7); beatstep7 = true; }
    countBeat++;
  }
  else if (patternBeat == 7){
    if (!beatstep8) { startPlayback(beat8,beatLength8); beatstep8 = true; }
    countBeat++;
  }
  countBeat++;
  delay(1);
  }
  prevStateBeat = stateBeat;
}

void deleteBeat(void){
  
  setBeat = false;
  selectBeat1 = false;
  selectBeat2 = false;
  selectBeat3 = false;
  selectBeat4 = false;
  selectBeat5 = false;
  selectBeat6 = false;
  selectBeat7 = false;
  selectBeat8 = false;
}



void setSequence(void){

  delay(500);

  if (analogRead(EFFECT_SELECTOR) > 600 && analogRead(EFFECT_SELECTOR) < 800){
  
  while (!selectNote1){
    step1 = mapSeq();
    syncPhaseInc = step1;
    selectNote1 = true;
  }
  delay(500);
  while (!selectNote2){
    step2 = mapSeq();
    syncPhaseInc = step2;
    selectNote2 = true;
  }
  delay(500);
  while (!selectNote3){
    step3 = mapSeq();
    syncPhaseInc = step3;
    selectNote3 = true;
  }
  delay(500);
  while (!selectNote4){
    step4 = mapSeq();
    syncPhaseInc = step4;
    selectNote4 = true;
  }
  delay(500);
  setSeq = true;
  }
}

void sequencePlay(void){
  
  grainPhaseInc  =  mapPhaseInc(FREQ);
  grainDecay     =  DECAY;   

  if (readCapacitivePin(C)>Threshold){
    state = true;
    if (prevState != state){
      seqPlay = !seqPlay;
    }
  } else { state = false; }

  if (readCapacitivePin(Cupper)>Threshold){
    deleteSeq();
    delay(1000);
  }

  if (seqPlay){
  
  if (counter > analogRead(CONTROL_2)){
    pattern++;
    counter = 0;
  }

  if (pattern == 4){ pattern = 0; }

  
  if (pattern == 0){
    if (counter < noteDuty()){
    syncPhaseInc = step1; }
    else {
    syncPhaseInc = 0;
    }
    counter++;
  }
  else if (pattern == 1){
    if (counter < noteDuty()){
    syncPhaseInc = step2; }
    else {
    syncPhaseInc = 0;
    }
    counter++;
  }
  else if (pattern == 2){
    if (counter < noteDuty()){
    syncPhaseInc = step3; }
    else {
    syncPhaseInc = 0; 
    }
    counter++;
  }
  else if (pattern == 3){
    if (counter < noteDuty()){
    syncPhaseInc = step4; }
    else {
    syncPhaseInc = 0; 
    }
    counter++;
  }
  else { counter = 0; }

  counter++;
  delay(1);
  }
  else {
    syncPhaseInc = 0;
  }
  
  prevState = state;
}


void sequenceEffect(void){

  grainPhaseInc  =  mapPhaseInc(FREQ);
  grainDecay     =  DECAY; 

  if (!seqSet){
    setSequence();
    seqSet = true;
  }
  sequencePlay();
}


void deleteSeq(void){
  
  seqSet = false;
  selectNote1 = false;
  selectNote2 = false;
  selectNote3 = false;
  selectNote4 = false;
  seqPlay = true;
}

void stateProcess(void){

  if (analogRead(EFFECT_SELECTOR) > 0 && analogRead(EFFECT_SELECTOR) < 200){
    beatISR = false; 
    deleteSeq();
  }
  if (analogRead(EFFECT_SELECTOR) > 200 && analogRead(EFFECT_SELECTOR) < 400){
    beatISR = false;
    deleteSeq();
  }
  if (analogRead(EFFECT_SELECTOR) > 400 && analogRead(EFFECT_SELECTOR) < 600){
    beatISR = false;
    deleteSeq();
  }
  if (analogRead(EFFECT_SELECTOR) > 600 && analogRead(EFFECT_SELECTOR) < 800){
    beatISR = false;
  }
  if (analogRead(EFFECT_SELECTOR) > 800 && analogRead(EFFECT_SELECTOR) < 1100){     
    beatISR = true;
    deleteSeq();
  }
  
}

SIGNAL(PWM_INTERRUPT){
  
  if (!beatISR){
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
}

// This is called at 8000 Hz to load the next sample.
ISR(TIMER1_COMPA_vect) {
  
  if (beatISR){
  if (sample >= sounddata_length) {
    if (sample == sounddata_length + lastSample) {
      stopPlayback();
    }
    else {
      // Ramp down to zero to reduce the click at the end of playback.
      OCR2B = sounddata_length + lastSample - sample;
    }
  }
  else {
    OCR2B = pgm_read_byte(&sounddata_data[sample]);
  }
  ++sample;
  } 
}



void setup() {
  
  GPIOSetup();
  
}

void loop() {

  stateProcess();

  if (analogRead(EFFECT_SELECTOR) < 200){
    audioOn();
    volumeAdjG();
    granularEffect();
  }
  else if (analogRead(EFFECT_SELECTOR) > 200 && analogRead(EFFECT_SELECTOR) < 400){
    audioOn();
    volumeAdjTA();
    tremoloEffect();
  }
  else if (analogRead(EFFECT_SELECTOR) > 400 && analogRead(EFFECT_SELECTOR) < 600){
    audioOn();
    volumeAdjTA();
    arpeggiatorEffect();
  } 
  else if (analogRead(EFFECT_SELECTOR) > 600 && analogRead(EFFECT_SELECTOR) < 800){
    audioOn();
    volumeAdjTA();
    sequenceEffect();
  } 
  else if (analogRead(EFFECT_SELECTOR) > 800 && analogRead(EFFECT_SELECTOR) < 1100){
    volumeAdjTA();
    BeatEffect();
  } 
}
