/**************************************************************************/
/*! 
    @file     GranoArpeggio.h
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
#include "GranoTremolo.h"
#include "GranoSequence.h"

uint8_t readCapacitivePin(uint8_t pin);
uint16_t mapArpeggio(void);
float noteDuty(void);

void arpeggiatorEffect(void){
  //ARPEGIATTOR EFFECT
  grainPhaseInc  =  mapPhaseInc(FREQ);
  grainDecay     =  DECAY;  
  
  if (counterArp > analogRead(CONTROL_2)){
    arpeggioState++;
    counterArp = 0;
  }

  if (arpeggioState == 4) arpeggioState = 0;
  
  if (mapArpeggio() != 0){
    if (arpeggioState == 0){
      if (counterArp < noteDuty()){
      syncPhaseInc   =  ArpeggioTable[mapArpeggio()];
      } else {
        syncPhaseInc = 0;
      }
      counterArp++;
      }
    else if (arpeggioState == 1){
      if (counterArp < noteDuty()){
        if (analogRead(EFFECT_AMT) < 200) syncPhaseInc =  ArpeggioTable[mapArpeggio()+5];
        else if (analogRead(EFFECT_AMT) < 400 && analogRead(EFFECT_AMT) > 200) syncPhaseInc =  ArpeggioTable[mapArpeggio()+3];
        else if (analogRead(EFFECT_AMT) < 600 && analogRead(EFFECT_AMT) > 400) syncPhaseInc =  ArpeggioTable[mapArpeggio()+3];
        else if (analogRead(EFFECT_AMT) < 800 && analogRead(EFFECT_AMT) > 600) syncPhaseInc =  ArpeggioTable[mapArpeggio()-3];
        else if (analogRead(EFFECT_AMT) < 1100 && analogRead(EFFECT_AMT) > 800)syncPhaseInc =  ArpeggioTable[mapArpeggio()+7];
      } else {
        syncPhaseInc = 0;
      }
      counterArp++;
      }
    else if (arpeggioState == 2){
      if (counterArp < noteDuty()){
        if (analogRead(EFFECT_AMT) < 200) syncPhaseInc =  ArpeggioTable[mapArpeggio()+7];
        else if (analogRead(EFFECT_AMT) < 400 && analogRead(EFFECT_AMT) > 200) syncPhaseInc =  ArpeggioTable[mapArpeggio()+5];
        else if (analogRead(EFFECT_AMT) < 600 && analogRead(EFFECT_AMT) > 400) syncPhaseInc =  ArpeggioTable[mapArpeggio()+2];
        else if (analogRead(EFFECT_AMT) < 800 && analogRead(EFFECT_AMT) > 600)syncPhaseInc =  ArpeggioTable[mapArpeggio()-5];
        else if (analogRead(EFFECT_AMT) < 1100 && analogRead(EFFECT_AMT) > 800)syncPhaseInc =  ArpeggioTable[mapArpeggio()+12];
      } else {
        syncPhaseInc = 0;
      }
      counterArp++;
      }
    else if (arpeggioState == 3){
      if (counterArp < noteDuty()){
        if (analogRead(EFFECT_AMT) < 200) syncPhaseInc =  ArpeggioTable[mapArpeggio()+10];
        else if (analogRead(EFFECT_AMT) < 400 && analogRead(EFFECT_AMT) > 200) syncPhaseInc =  ArpeggioTable[mapArpeggio()+9];
        else if (analogRead(EFFECT_AMT) < 600 && analogRead(EFFECT_AMT) > 400) syncPhaseInc =  ArpeggioTable[mapArpeggio()+1];
        else if (analogRead(EFFECT_AMT) < 800 && analogRead(EFFECT_AMT) > 600)syncPhaseInc =  ArpeggioTable[mapArpeggio()-9];
        else if (analogRead(EFFECT_AMT) < 1100 && analogRead(EFFECT_AMT) >800)syncPhaseInc =  ArpeggioTable[mapArpeggio()+7];
      } else {
        syncPhaseInc = 0;
      }
      counterArp++;
      }
  }
  else syncPhaseInc  =  0;
  counterArp++;
  delay(1);
}

#endif
