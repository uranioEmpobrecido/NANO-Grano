/**************************************************************************/
/*! 
    @file     GranoGPIO.h
    @author   Jorge Gutierrez (NANO.es)
	@license 

    v1.0  - First release
*/
/**************************************************************************/

#ifndef GRANOGPIO_H
#define GRANOGPIO_H


#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include "GranoVariables.h"
#include "GranoProcess.h"
#include "GranoSequence.h"
#include "GranoBeat.h"

//---------- Thresholdacitive Touch sensing -----------------------------
uint8_t readCapacitivePin(uint8_t pinToMeasure) {

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

void softReset(){
  asm volatile ("  jmp 0"); 
  delay(500);
} 


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

void pulseHigh(void){

    setHigh(VCA_CONTROL);
    delayMicroseconds(500);
    setHighZ(VCA_CONTROL);
    delayMicroseconds(500);
    volume++;
}

void pulseLow(void){

    setLow(VCA_CONTROL);
    delayMicroseconds(500);
    setHighZ(VCA_CONTROL);
    delayMicroseconds(500); 
    volume--;
}

float noteDuty(void){
  return ((analogRead(CONTROL_2)*(analogRead(CONTROL_1)/1023.0))+0.15); 
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

#endif
