// Analog in 0: Grain 1 pitch
// Analog in 1: Octave Selector
// Analog in 2: Grain 1 decay
// Analog in 4: Grain repetition frequency

// Map Analogue channels
#define GRAIN_FREQ_CONTROL   (0)
#define GRAIN_DECAY_CONTROL  (2)
#define OCTAVE_CONTROL       (1)

// Map Digital channels
#define C         13//PB5
#define Csharp    12//PB4
#define D         11//PB3
#define Dsharp    10//PB2
#define E         9 //PB1
#define F         8 //PB0
#define Fsharp    7 //PD7
#define G         6 //PD6
#define Gsharp    5 //PD5
#define A         4 //PD4
#define Asharp    2 //PD2
#define B         A4//PC4
#define Cupper    A5//PC5

#define LED       3 //PD3

boolean toggle = false;

void setup() {
  // put your setup code here, to run once:
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

  pinMode(LED,OUTPUT);

  pinMode(GRAIN_FREQ_CONTROL,INPUT);
  pinMode(GRAIN_DECAY_CONTROL,INPUT);
  pinMode(OCTAVE_CONTROL,INPUT);

  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("Octave: ");
  Serial.print(analogRead(OCTAVE_CONTROL));
  Serial.print("  Decay: ");
  Serial.print(analogRead(GRAIN_DECAY_CONTROL));
  Serial.print("  Freq: ");
  Serial.print(analogRead(GRAIN_FREQ_CONTROL));
  Serial.print("\n");
  Serial.print(digitalRead(C));
  Serial.print("  ");
  Serial.print(digitalRead(Csharp));
  Serial.print("  ");
  Serial.print(digitalRead(D));
  Serial.print("  ");
  Serial.print(digitalRead(Dsharp));
  Serial.print("  ");
  Serial.print(digitalRead(E));
  Serial.print("  ");
  Serial.print(digitalRead(F));
  Serial.print("  ");
  Serial.print(digitalRead(Fsharp));
  Serial.print("  ");
  Serial.print(digitalRead(G));
  Serial.print("  ");
  Serial.print(digitalRead(Gsharp));
  Serial.print("  ");
  Serial.print(digitalRead(A));
  Serial.print("  ");
  Serial.print(digitalRead(Asharp));
  Serial.print("  ");
  Serial.print(digitalRead(B));
  Serial.print("  ");
  Serial.print(digitalRead(Cupper));
  Serial.print("\n");
  toggle = !toggle;
  digitalWrite(13,toggle);
  delay(200);
}
