/**************************************************************************/
/*! 
    @file     GranoSequence.h
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
#include "GranoMap.h"
#include "GranoTremolo.h"
#include "GranoArpeggio.h"

uint8_t readCapacitivePin(uint8_t pin);
float noteDuty(void);

void setSequence(void){
  
  if (analogRead(EFFECT_SELECTOR) > 600 && analogRead(EFFECT_SELECTOR) < 800){

    syncPhaseInc = 0;
  
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
  while (!selectNote5){
    step5 = mapSeq();
    syncPhaseInc = step5;
    selectNote5 = true;
  }
  delay(500);
  while (!selectNote6){
    step6 = mapSeq();
    syncPhaseInc = step6;
    selectNote6 = true;
  }
  delay(500);
  while (!selectNote7){
    step7 = mapSeq();
    syncPhaseInc = step7;
    selectNote7 = true;
  }
  delay(500);  
  while (!selectNote8){
    step8 = mapSeq();
    syncPhaseInc = step8;
    selectNote8 = true;
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
  selectNote5 = false;
  selectNote6 = false;
  selectNote7 = false;
  selectNote8 = false;
  seqPlay = true;
}

void sequencePlay(void){
  
  grainPhaseInc  =  mapPhaseInc(FREQ);
  grainDecay     =  (analogRead(EFFECT_AMT)/8)+2; 

  if (readCapacitivePin(C)>Threshold){
    state = true;
    if (prevState != state){
      seqPlay = !seqPlay;
      pattern = 8;
    }
  } else { state = false; }

  if (readCapacitivePin(Cupper)>Threshold){
    syncPhaseInc = 0;
    deleteSeq();
    delay(1000);
  }

  if (seqPlay){
  
  if (counter > analogRead(CONTROL_2)){
    pattern++;
    counter = 0;
  }

  if (pattern == 8){ pattern = 0; }

  
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
  else if (pattern == 4){
    if (counter < noteDuty()){
    syncPhaseInc = step5; }
    else {
    syncPhaseInc = 0; 
    }
    counter++;
  }
    else if (pattern == 5){
    if (counter < noteDuty()){
    syncPhaseInc = step6; }
    else {
    syncPhaseInc = 0; 
    }
    counter++;
  }
  else if (pattern == 6){
    if (counter < noteDuty()){
    syncPhaseInc = step7; }
    else {
    syncPhaseInc = 0; 
    }
    counter++;
  }
  else if (pattern == 7){
    if (counter < noteDuty()){
    syncPhaseInc = step8; }
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

  if (!seqSet){
    setSequence();
    seqSet = true;
  }
  sequencePlay();
}


#endif
