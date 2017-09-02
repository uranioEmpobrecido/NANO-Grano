/**************************************************************************/
/*! 
    @file     GranoTremolo.h
    @author   Jorge Gutierrez (NANO.es)
	@license 

    v1.0  - First release
*/
/**************************************************************************/

#ifndef GRANOTREMOLO_H
#define GRANOTREMOLO_H


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

uint8_t readCapacitivePin(uint8_t pin);

void tremoloEffect(void){
  //TREMOLO EFFECT
  if (!tremoloON){
    syncPhaseInc   =  mapOctave(NO_ENVELOPE);
    grainPhaseInc  =  mapPhaseInc(FREQ);
    grainDecay     =  (analogRead(EFFECT_AMT)/8)+2;//DECAY;  //analogRead(EFFECT_AMT) / 8;
    delay(analogRead(CONTROL_2)/4 * (analogRead(CONTROL_1)/1025.0));
    tremoloON = true;
  } else if (tremoloON){
    syncPhaseInc   =  0;
    //grainPhaseInc  =  mapPhaseInc(FREQ);
    grainDecay     =  DECAY;  //analogRead(EFFECT_SELECTOR) / 8;
    delay(analogRead(CONTROL_2)/4 * (1-(analogRead(CONTROL_1)/1025.0)));
    tremoloON = false;
  }  
}

#endif
