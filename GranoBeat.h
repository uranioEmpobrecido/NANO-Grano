/**************************************************************************/
/*! 
    @file     GranoBeat.h
    @author   Jorge Gutierrez (NANO.es)
	@license 

    v1.0  - First release
*/
/**************************************************************************/

#ifndef GRANOBEAT_H
#define GRANOBEAT_H


#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include "GranoVariables.h"
#include "GranoProcess.h"
#include "GranoSequence.h"
#include "GranoTremolo.h"
#include "GranoGPIO.h"

uint8_t readCapacitivePin(uint8_t pin);

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


void beatPlay(void){

  if (readCapacitivePin(C)>Threshold){
    stateBeat = true;
    if (prevStateBeat != stateBeat){
      beatRun = !beatRun;
      patternBeat = 8;
    }
  } else { stateBeat = false; }

  if (readCapacitivePin(Cupper)>Threshold){
    startPlayback(blip,blip_length);
    deleteBeat();
    delay(1000);
  }

  if (beatRun){

  if (countBeat >= ((analogRead(EFFECT_AMT)/2)+50)){
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

uint16_t mapLength(unsigned char * sound){

  if (sound == kick) { return kick_length   ;}
  if (sound == snare){ return snare_length  ;}
  if (sound == tick) { return tick_length   ;}
  if (sound == bass) { return bass_length   ;}
  if (sound == tom)  { return tom_length    ;}
  if (sound ==cowbell){return cowbell_length;}
  if (sound == blip) { return blip_length   ;}
  if (sound == crash){ return crash_length  ;}
  if (sound == clap) { return clap_length   ;}
  if (sound == hat)  { return hat_length    ;}
  
}

unsigned char * mapBeat(void){

  while (!beatOK){
  if (readCapacitivePin(D)>Threshold){
    startPlayback(kick,kick_length);
    beatOK = true;
    return kick;
    }
  else if (readCapacitivePin(E)>Threshold){
    startPlayback(cowbell,cowbell_length);
    beatOK = true;
    return cowbell;
    }
  else if (readCapacitivePin(F)>Threshold){
    startPlayback(snare,snare_length);
    beatOK = true;
    return snare;
    }
  else if (readCapacitivePin(G)>Threshold){
    startPlayback(clap,clap_length);
    beatOK = true;
    return clap;
    }
  else if (readCapacitivePin(A)>Threshold){
    startPlayback(tom,tom_length);
    beatOK = true;
    return tom;
    }
  else if (readCapacitivePin(B)>Threshold){
    startPlayback(hat,hat_length);
    beatOK = true;
    return hat;
    }
    }
}

void setBeatSequence(void){

  delay(500);

  if (analogRead(EFFECT_SELECTOR) > 800){

  startPlayback(blip,blip_length);
  
  while (!selectBeat1){
    beat1 = mapBeat();
    beatLength1 = mapLength(beat1);
    selectBeat1 = true;}
    
  beatOK = false;
  delay(500);
  
  startPlayback(blip,blip_length);
  
  while (!selectBeat2){
    beat2 = mapBeat();
    beatLength2 = mapLength(beat2);
    selectBeat2 = true;}
    
  beatOK = false;
  delay(500);
  
  startPlayback(blip,blip_length);
  
  while (!selectBeat3){
    beat3 = mapBeat();
    beatLength3 = mapLength(beat3);
    selectBeat3 = true;}
    
  beatOK = false;
  delay(500);
  
  startPlayback(blip,blip_length);
  
  while (!selectBeat4){
    beat4 = mapBeat();
    beatLength4 = mapLength(beat4);
    selectBeat4 = true;}
    
  beatOK = false;
  delay(500);

  startPlayback(blip,blip_length);

  while (!selectBeat5){
    beat5 = mapBeat();
    beatLength5 = mapLength(beat5);
    selectBeat5 = true;}
    
  beatOK = false;
  delay(500);

  startPlayback(blip,blip_length);

  while (!selectBeat6){
    beat6 = mapBeat();
    beatLength6 = mapLength(beat6);
    selectBeat6 = true;}
    
  beatOK = false;
  delay(500);

  startPlayback(blip,blip_length);

  while (!selectBeat7){
    beat7 = mapBeat();
    beatLength7 = mapLength(beat7);
    selectBeat7 = true;}
    
  beatOK = false;
  delay(500);

  startPlayback(blip,blip_length);

  while (!selectBeat8){
    beat8 = mapBeat();
    beatLength8 = mapLength(beat8);
    selectBeat8 = true;}
    
  beatOK = false;
  delay(500);

  startPlayback(blip,blip_length);
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
    while (readCapacitivePin(D)>Threshold){
      //Nothing
    }
    startPlayback(kick,kick_length);
  }
  if (readCapacitivePin(E)>Threshold){
    while (readCapacitivePin(E)>Threshold){
      //Nothing
    }
    startPlayback(cowbell,cowbell_length);
  }
  if (readCapacitivePin(F)>Threshold){
    while (readCapacitivePin(F)>Threshold){
      //Nothing
    }
    startPlayback(snare,snare_length);
  }
  if (readCapacitivePin(G)>Threshold){
    while (readCapacitivePin(G)>Threshold){
      //Nothing
    }
    startPlayback(clap,clap_length);
  }
    if (readCapacitivePin(A)>Threshold){
    while (readCapacitivePin(A)>Threshold){
      //Nothing
    }
    startPlayback(tom,tom_length);
  }
    if (readCapacitivePin(B)>Threshold){
    while (readCapacitivePin(B)>Threshold){
      //Nothing
    }
    startPlayback(hat,hat_length);
  }
}


#endif
