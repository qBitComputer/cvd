#include <math.h>
#include <Servo.h>
#include <string.h>

#define PIN_ALPHA 3
#define PIN_BETA 4
#define PI 3.14

void computeAngles(double x3, double y3, double *compAngleAlpha, double *compAngleBeta);

// setup servo
Servo AlphaServo;
Servo BetaServo;
// parameters
const int l1 = 3.5;
const int l2 = 4.5;
const int r1 = 3.5;
const int r2 = 4.5;

// coordinates
const int x4 = 3.0;
const int y4 = 0.0;
const int x5 = 5.5;
const int y5 = 0.0;

// servo angles
const int alpha = 135.0;
const int beta = 45.0;

// define computed angles
double compAngleAlpha;
double compAngleBeta;

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(10);
  AlphaServo.attach(PIN_ALPHA);
  BetaServo.attach(PIN_BETA);

}


void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()){
      String line = Serial.readString();
      if(line.equals("square")){
        // todo
        
      }
  }


}


void computeAngles(double x3, double y3, double *compAngleAlpha, double *compAngleBeta){
  // double angleZ[2];
  // angleZ[0] = 0.0;
  // angleZ[1] = 0.0; 
  // L(x1,y1)
  double x1 = l1 * cos((PI * alpha) / 180.0) + x4;
  double y1 = l1 * sin((PI * alpha) / 180.0) + y4;
  // R(x2,y2)
  double x2 = r1 * cos((PI * beta) / 180.0) + x5;
  double y2 = r1 * sin((PI * beta) / 180.0) + y5;

  // Calculate S(x3,y3)
  double distLR = sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
  double angleLR = atan2(y2 - y1, x2 - x1);
  double anglePhi = acos((distLR * distLR + l2 * l2 - r2 * r2) / (2.0 * distLR * l2));

  // S(x3,y3)
  // double x3 = l2 * cos(angleLR + anglePhi) + x1;
  // double y3 = l2 * sin(angleLR + anglePhi) + y1;

  //double x3 = 1.0;
  //double y3 = 1.0;

  // compute l3 and r3
  double l3 = sqrt((x3 - x4) * (x3 - x4) + (y3 - y4) * (y3 - y4));
  double r3 = sqrt((x3 - x5) * (x3 - x5) + (y3 - y5) * (y3 - y5));
  // compute L(x1,y1) and R(x2,y2) from S(x3,y3)
  double angleL3 = atan2(y3 - y4, x3 - x4);
  double angleR3 = atan2(y3 - y5, x3 - x5);

  double angleL = acos((l1 * l1 + l3 * l3 - l2 * l2) / (2.0 * l1 * l3));
  double angleR = acos((r1 * r1 + r3 * r3 - r2 * r2) / (2.0 * r1 * r3));

  // compute alpha
  *compAngleAlpha = angleL + angleL3;
  //angleZ[0] = compAngleAlpha * 180.0 / PI;
  //alpha = compAngleAlpha * 180.0 / PI;
  // compute beta
  *compAngleBeta = angleR3 - angleR;
  //angleZ[1] = compAngleBeta * 180.0 / PI;
  //beta = compAngleBeta * 180.0 / PI;
  //return angleZ;
}
