#include <math.h>
#include <Servo.h>
// setup servo
Servo AlphaServo;
Servo BetaServo;
// parameters
const int l1 = 3.0;
const int l2 = 5.0;
const int r1 = 3.0;
const int r2 = 5.0;

// coordinates
const int x4 = 1.0;
const int y4 = 0.0;
const int x5 = 4.0;
const int y5 = 0.0;

// servo angles
const int alpha = 135.0;
const int beta = 45.0;

// define computed angles
float compAngleAlpha;
float compAngleBeta;

void setup() {
  Serial.begin(9600);
  AlphaServo.attach(9);
  BetaServo.attach(8);
  // L(x1,y1)
  float x1 = l1 * cos((PI * alpha) / 180.0) + x4;
  float y1 = l1 * sin((PI * alpha) / 180.0) + y4;
  // R(x2,y2)
  float x2 = r1 * cos((PI * beta) / 180.0) + x5;
  float y2 = r1 * sin((PI * beta) / 180.0) + y5;

  // Calculate S(x3,y3)
  float distLR = sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
  float angleLR = atan2(y2 - y1, x2 - x1);
  float anglePhi = acos((distLR * distLR + l2 * l2 - r2 * r2) / (2.0 * distLR * l2));

  // S(x3,y3)
  float x3 = l2 * cos(angleLR + anglePhi) + x1;
  float y3 = l2 * sin(angleLR + anglePhi) + y1;

  // compute l3 and r3
  float l3 = sqrt((x3 - x4) * (x3 - x4) + (y3 - y4) * (y3 - y4));
  float r3 = sqrt((x3 - x5) * (x3 - x5) + (y3 - y5) * (y3 - y5));
  // compute L(x1,y1) and R(x2,y2) from S(x3,y3)
  float angleL3 = atan2(y3 - y4, x3 - x4);
  float angleR3 = atan2(y3 - y5, x3 - x5);

  float angleL = acos((l1 * l1 + l3 * l3 - l2 * l2) / (2.0 * l1 * l3));
  float angleR = acos((r1 * r1 + r3 * r3 - r2 * r2) / (2.0 * r1 * r3));

  // compute alpha
  float compAngleAlpha = angleL + angleL3;

  Serial.println("");
  Serial.println("Alpha: ");
  Serial.println(compAngleAlpha * 180.0 / PI);

  // compute beta
  float compAngleBeta = angleR3 - angleR;
  Serial.println("Beta: ");
  Serial.println(compAngleBeta * 180.0 / PI);

  // print S(x3,y3)
  Serial.print("S: (");
  Serial.print(x3);
  Serial.print(", ");
  Serial.print(y3);
  Serial.print(")");
}

void loop() {

  AlphaServo.write(compAngleAlpha * 180.0 / PI);
  BetaServo.write(compAngleBeta * 180.0 / PI);
  delay(10000);
}
