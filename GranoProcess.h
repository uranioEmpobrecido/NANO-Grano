/**************************************************************************/
/*! 
    @file     GranoFunctions.h
    @author   Jorge Gutierrez (NANO.es)
	@license 

    v1.0  - First release
*/
/**************************************************************************/

#ifndef GRANOPROCESS_H
#define GRANOPROCESS_H


#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include "GranoVariables.h"
#include "GranoGPIO.h"
#include "GranoSequence.h"
#include "GranoBeat.h"

void noEnvelopeAdj(void);
void pulseLow(void);
void pulseHigh(void);

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


#endif
