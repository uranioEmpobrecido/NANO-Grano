/**************************************************************************/
/*! 
    @file     GranoFunctions.h
    @author   Jorge Gutierrez (NANO.es)
	@license 

    v1.0  - First release
*/
/**************************************************************************/

#ifndef GRANOARPEGGIO_H
#define GRANOARPEGGIO_H


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
#include "GranoSequence.h"

uint8_t readCapacitivePin(uint8_t pin);
uint16_t mapArpeggio();

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

#endif
