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
long encoderCount = 0;
boolean aSet = false;
boolean bSet = false;

// PID controller constanten
double Kp = 20.0;                                   // Proportional gain
double Ki = 0.0;                                    // Integral gain
double Kd = 0.3;                                    // Derivative gain
double sampleTimeS = (double)SAMPLE_US / 1000000;   // Converteren sample tijd naar secondes

// PID variabelen
double setpoint = 100;      // Gewenste positie
double output = 0;          // Output van PID controller, dus input naar PWM signaal welke naar de motor controller gaat
double error = 0;           // Berekende fout (verschil tussen gemeten waarde en setpoint)
double previousError = 0;   // Fout van vorige berekening, nodig voor berekenen derative
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
    int newSetpoint = data.toInt();

    // Check of de conversie gelukt is en er geen onzin uit komt, zet hem daarna door als setpoint
    if (newSetpoint != 0 || data == "0") setpoint = newSetpoint;      
  }
}

//
// Functies
//

// Elke keer als de encoder een puls maakt wordt deze functie uitgevoerd
void updateEncoder() {    
  aSet = digitalRead(PIN_ENC1);
  bSet = digitalRead(PIN_ENC2);

  if (aSet != bSet) encoderCount++;
  else encoderCount--;
}

// Elke keer als timer1 een signaal geeft worden de PID waardes berekend en naar de motor controller gestuurd
void calculatePID() {
  // Sturen gemeten positie naar computer
  Serial.print(0);    // Prachtige manier om serial plotter y-as schaling mooi te houden
  Serial.print('\t');
  Serial.println(encoderCount);
  
  // Berekeningen PID controller
  error = setpoint - encoderCount;                              // Berekenen error
  integral += error * sampleTimeS;                              // Berekenen integral
  derivative = (error - previousError) / sampleTimeS;           // Berekenen derivative  
  output = (Kp * error) + (Ki * integral) + (Kd * derivative);  // Bereken PID output

  // Draairichting van motor veranderen als output negatief wordt
  if (output > 0) {
    digitalWrite(PIN_MOT1, HIGH);
    digitalWrite(PIN_MOT2, LOW);
    analogWrite(PIN_MOT1, min(abs(output), 255)); // Beveiligen dat de output hoger wordt dan de maximale waarde van 255
  } else {
    digitalWrite(PIN_MOT1, LOW);
    digitalWrite(PIN_MOT2, HIGH);
    analogWrite(PIN_MOT2, min(abs(output), 255)); 
  }

  // Opslaan error en tijd voor volgende berekening
  previousError = error;
}
