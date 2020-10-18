
#include <Encoder.h>
// Create relationship between motor input voltagePin
int voltagePin = 9;
int VoltageR = 125;
int VoltageL = -125;
int enablePin = 4;
int DirectionR  = 7;
int DirectionL = 8;
Encoder encoderR(3, 6);
Encoder encoderL(4, 7);

void setup() {
  Serial.begin(9600);
  Serial.println("Time:    Voltage:  Velocity:   ");
  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, HIGH);
  pinMode(voltagePin, OUTPUT);
  pinMode(DirectionR, OUTPUT);
  digitalWrite(DirectionR, LOW);
  pinMode(DirectionL, OUTPUT);
  digitalWrite(DirectionL, LOW);
 
}

int motorStepResponseR(int x) {
  long lowerTime = 1000;
  long upperTime = 2000;
  long oldLocationR;
  long newLocationR;
  long oldLocationL;
  long newLocationL;
 
  double sampleRate = 0.05;
  double Delay = sampleRate*1000;
  double angularVelocityR;
  double angularVelocityL;
  int wheelRadius = 4; 
  double instantaneousVelocity;

  double currentTime = millis();
  double actualVoltage = (7.5 / 255)* x;
 
  if (millis() >= lowerTime && millis() <= upperTime){
    analogWrite(voltagePin, x);
   
    oldLocationR = encoderR.read();
    delay(Delay);
    newLocationR = encoderR.read();

    
    oldLocationL = encoderL.read();
    delay(Delay);
    newLocationL = encoderL.read();
   
    double deltaPositionR = (newLocationR - oldLocationR)*(6.2832/3200);
    double deltaPositionL = (newLocationL - oldLocationL)*(6.2832/3200);

    
    angularVelocityR = (deltaPositionR) / (sampleRate);
    angularVelocityL = (deltaPositionL) / (sampleRate);
    instantaneousVelocity = wheelRadius*(angularVelocityR + angularVelocityL)/2;
   
    Serial.print(currentTime);
    Serial.print("    ");
    Serial.print(actualVoltage);
    Serial.print("    ");
    Serial.print(instantaneousVelocity);
    Serial.print("\n");
  }
  if (millis() > upperTime){
    digitalWrite(enablePin, LOW);
  }
}
int motorStepResponseL(int x) {
  
}

void loop(){
  motorStepResponse(VoltageR);
  motorStepRes
}
