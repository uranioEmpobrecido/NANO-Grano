/**************************************************************************/
/*! 
    @file     GranoFunctions.h
    @author   Jorge Gutierrez (NANO.es)
	@license 

    v1.0  - First release
*/
/**************************************************************************/

#ifndef GRANOSEQUENCE_H
#define GRANOSEQUENCE_H


#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include "GranoVariables.h"
#include "GranoProcess.h"
#include "GranoBeat.h"
#include "GranoGPIO.h"

uint8_t readCapacitivePin(uint8_t pin);

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


void deleteSeq(void){
  
  seqSet = false;
  selectNote1 = false;
  selectNote2 = false;
  selectNote3 = false;
  selectNote4 = false;
  seqPlay = true;
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


#endif
