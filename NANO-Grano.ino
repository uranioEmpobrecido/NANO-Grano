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
#include "GranoGPIO.h"
#include "GranoProcess.h"
#include "GranoSequence.h"
#include "GranoBeat.h"
#include "GranoArpeggio.h"
#include "GranoTremolo.h"
#include "GranoMap.h"

#define PWM_VALUE             OCR2B
#define PWM_INTERRUPT         TIMER2_OVF_vect

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
  //Sound that says that Grano is ready to work
  //startPlayback(blip,blip_length);
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
