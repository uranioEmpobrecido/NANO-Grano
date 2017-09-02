/**************************************************************************/
/*! 
    @file     GranoMap.h
    @author   Jorge Gutierrez (NANO.es)
	@license 

    v1.0  - First release
*/
/**************************************************************************/

#ifndef GRANOMAP_H
#define GRANOMAP_H


#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include "GranoVariables.h"
#include "GranoProcess.h"
#include "GranoBeat.h"
#include "GranoGPIO.h"
#include "GranoArpeggio.h"
#include "GranoTremolo.h"
#include "GranoSequence.h"

uint8_t readCapacitivePin(uint8_t pin);
void attackProcess(void);
void releaseProcess(void);

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


#endif
