// GA12-N20-E Gelijkstroom motor met encoder en mini L298N motor driver module. De encoder geeft 7 pulsen per rotatie van de motor.
//
// pins:
// encoder: 2 & 3, deze pinnen ondersteunen interrupts
// motor: pin 5 & 6
#include <TimerOne.h>

// Definities
#define PIN_ENC1 2
#define PIN_ENC2 3
#define PIN_MOT1 5
#define PIN_MOT2 6

#define SAMPLE_US 10000   // 10ms per iteratie van de regellus, dus 100Hz

// Encoder variabelen
volatile long encoderCount = 0;
volatile boolean aSet = false;
volatile boolean bSet = false;

// PID controller constanten
double Kp = 8.0;    // Proportional gain
double Ki = 2.0;    // Integral gain
double Kd = 0.1;    // Derivative gain

// PID variables
double setPoint = 100.0;    // Gewenste positie
double input = 0;           // 
double output = 0;          // Output van PID controller, dus input naar PWM signaal welke naar de motor controller gaat
double error = 0;           // Berekende fout (verschil tussen gemeten waarde en setpoint)
double previousError = 0;   // Fout tijdens vorige berekening
double integral = 0;        // Om waarde integraal op te slaan  
double derivative = 0;      // Om waarde differentiaal op te slaan
unsigned long lastTime = 0; // Laatste tijdstip waarop PID is berekend

void setup() {
  // Start seriele verbinding met pc
  Serial.begin(115200);
  
  // Stel pinnen van arduino in
  pinMode(PIN_MOT1, OUTPUT);  
  pinMode(PIN_MOT2, OUTPUT);
  pinMode(PIN_ENC1, INPUT_PULLUP);
  pinMode(PIN_ENC2, INPUT_PULLUP);

  // Start interrupt voor encoder pulsen
  attachInterrupt(digitalPinToInterrupt(PIN_ENC1), updateEncoder, CHANGE);

  // Initialiseer timer voor het doen van de PID berekeningen
  Timer1.initialize(SAMPLE_US);
  Timer1.attachInterrupt(calculatePID);
}

void loop() {
  // Check of je een nieuw signaal ontvangt vanaf de computer
  if (Serial.available() > 0) {
    // Lees deze data in het String formaat
    String data = Serial.readStringUntil('\n');

    // Converteer de string naar integer zodat we 'm kunnen gebruiken voor de PID controller
    int newSetPoint = data.toInt();

    // Check of de conversie gelukt is en er geen onzin uit komt
    if (newSetPoint != 0 || data == "0") {
      setPoint = newSetPoint;
      Serial.print("New setpoint: ");
      Serial.println(setPoint);
    }
  }
}

//
// Functies
//

// Elke keer als de encoder een puls maakt wordt deze functie uitgevoerd
void updateEncoder() {    
  aSet = digitalRead(PIN_ENC1);
  bSet = digitalRead(PIN_ENC2);

  if (aSet != bSet) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

// Elke keer als timer1 een signaal geeft worden de PID waardes berekend en naar de motor controller gestuurd
void calculatePID() {
  // Sturen gemeten positie naar computer
  Serial.print(0);    // Prachtige manier om serial plotter y-as schaling mooi te houden
  Serial.print('\t');
  Serial.println(encoderCount);

  // input van de PID controller is de gemeten encoder positie
  input = encoderCount; 
  unsigned long currentTime = millis();
  double elapsedTime = (double)(currentTime - lastTime) / 1000; // Convert to seconds

  error = setPoint - input;                             // Berekenen error
  integral += error * elapsedTime;                      // Berekenen integral
  derivative = (error - previousError) / elapsedTime;   // Berekenen derivative

  // Bereken PID output
  output = (Kp * error) + (Ki * integral) + (Kd * derivative);

  // Draairichting van motor veranderen als output negatief wordt
  if (output > 0) {
    digitalWrite(PIN_MOT1, HIGH);
    digitalWrite(PIN_MOT2, LOW);
    analogWrite(PIN_MOT1, min(abs(output), 255)); // Ensure output doesn't exceed PWM limits
  } else {
    digitalWrite(PIN_MOT1, LOW);
    digitalWrite(PIN_MOT2, HIGH);
    analogWrite(PIN_MOT2, min(abs(output), 255)); // Ensure output doesn't exceed PWM limits
  }

  // Opslaan error en tijd voor volgende berekening
  previousError = error;
  lastTime = currentTime;
}
