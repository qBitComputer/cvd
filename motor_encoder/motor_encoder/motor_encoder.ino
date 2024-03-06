// N20 Gelijkstroom motor met encoder en L298N motor driver module. De encoder geeft 7 pulsen per rotatie van de motor.
//
// pins:
// encoder: 2 & 3, deze pinnen ondersteunen interrupts
// motor: pin 5 & 6

#define PIN_ENC1 2
#define PIN_ENC2 3
#define PIN_MOT1 5
#define PIN_MOT2 6

volatile long encoderCount = 0;
volatile boolean aSet = false;
volatile boolean bSet = false;

int motorStop = 1000;

void setup() {
  // Start seriele verbinding met pc
  Serial.begin(115200);
  
  // Stel pinnen van arduino in
  pinMode(PIN_MOT1, OUTPUT);  
  pinMode(PIN_ENC1, INPUT_PULLUP);
  pinMode(PIN_ENC2, INPUT_PULLUP);

  // Start interrupt voor encoder pulsen
  attachInterrupt(digitalPinToInterrupt(PIN_ENC1), updateEncoder, CHANGE);

  // Laat motor op 50% vermogen draaien (0 = uit, 255 = 100%)
  analogWrite(PIN_MOT1, 64);  
}

void loop() {
  // if (Serial.available()) {
  //   Serial.print("aSet: ");
  //   Serial.print(aSet);
  //   Serial.print(", bSet: ");
  //   Serial.print(bSet);
  //   Serial.println(".");
  // }
}

void updateEncoder() {
  aSet = digitalRead(PIN_ENC1);
  bSet = digitalRead(PIN_ENC2);

  if (aSet != bSet) {
    encoderCount++;
  } else {
    encoderCount--;
  }

  Serial.print("encoder count: ");
  Serial.println(encoderCount);


  // if (encoderCount > motorStop || encoderCount < -motorStop) {
    // analogWrite(PIN_MOT1, 0);
  // }
}