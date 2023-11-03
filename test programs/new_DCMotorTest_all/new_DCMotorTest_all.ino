/*
This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
It won't work with v1.x motor shields! Only for the v2's with built in PWM
control

For use with the Adafruit Motor Shield v2
---->	http://www.adafruit.com/products/1438
*/

#include <BTS7960_MotorShield.h>

// Create the motor shield object with the default I2C address
BTS7960_MotorShield AFMS = BTS7960_MotorShield();
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);

// Select which 'port' M1, M2, M3 or M4. In this case, M1
BTS7960_DCMotor *myMotor = AFMS.getMotor(1);
BTS7960_DCMotor *myMotor2 = AFMS.getMotor(2);
BTS7960_DCMotor *myMotor3 = AFMS.getMotor(3);
BTS7960_DCMotor *myMotor4 = AFMS.getMotor(4);
// You can also make another motor on port M2
//Adafruit_DCMotor *myOtherMotor = AFMS.getMotor(2);

void setup() {
  Serial.begin(115200);           // set up Serial library at 9600 bps
  delay(3000);
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");

  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
  // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");

  // Set the speed to start, from 0 (off) to 255 (max speed)
  myMotor->setSpeed(150);
  myMotor->run(FORWARD);
  // turn on motor
  myMotor->run(RELEASE);
    // Set the speed to start, from 0 (off) to 255 (max speed)
  myMotor2->setSpeed(150);
  myMotor2->run(FORWARD);
  // turn on motor
  myMotor2->run(RELEASE);
    // Set the speed to start, from 0 (off) to 255 (max speed)
  myMotor3->setSpeed(150);
  myMotor3->run(FORWARD);
  // turn on motor
  myMotor3->run(RELEASE);
    // Set the speed to start, from 0 (off) to 255 (max speed)
  myMotor4->setSpeed(150);
  myMotor4->run(FORWARD);
  // turn on motor
  myMotor4->run(RELEASE);
}

void loop() {
  uint8_t i;

  Serial.print("tick1");

  myMotor->run(FORWARD);
  for (i=0; i<255; i++) {
    myMotor->setSpeed(i);
    delay(10);
  }
  for (i=255; i!=0; i--) {
    myMotor->setSpeed(i);
    delay(10);
  }

  Serial.print("tock1");

  myMotor->run(BACKWARD);
  for (i=0; i<255; i++) {
    myMotor->setSpeed(i);
    delay(10);
  }
  for (i=255; i!=0; i--) {
    myMotor->setSpeed(i);
    delay(10);
  }

  Serial.print("tech1");
  myMotor->run(RELEASE);
  delay(1000);
  Serial.print("tick2");

  myMotor2->run(FORWARD);
  for (i=0; i<255; i++) {
    myMotor2->setSpeed(i);
    delay(10);
  }
  for (i=255; i!=0; i--) {
    myMotor2->setSpeed(i);
    delay(10);
  }

  Serial.print("tock2");

  myMotor2->run(BACKWARD);
  for (i=0; i<255; i++) {
    myMotor2->setSpeed(i);
    delay(10);
  }
  for (i=255; i!=0; i--) {
    myMotor2->setSpeed(i);
    delay(10);
  }

  Serial.print("tech2");
  myMotor2->run(RELEASE);
  delay(1000);
  Serial.print("tick3");

  myMotor3->run(FORWARD);
  for (i=0; i<255; i++) {
    myMotor3->setSpeed(i);
    delay(10);
  }
  for (i=255; i!=0; i--) {
    myMotor3->setSpeed(i);
    delay(10);
  }

  Serial.print("tock3");

  myMotor3->run(BACKWARD);
  for (i=0; i<255; i++) {
    myMotor3->setSpeed(i);
    delay(10);
  }
  for (i=255; i!=0; i--) {
    myMotor3->setSpeed(i);
    delay(10);
  }

  Serial.print("tech3");
  myMotor3->run(RELEASE);
  delay(1000);
  Serial.print("tick4");

  myMotor4->run(FORWARD);
  for (i=0; i<255; i++) {
    myMotor4->setSpeed(i);
    delay(10);
  }
  for (i=255; i!=0; i--) {
    myMotor4->setSpeed(i);
    delay(10);
  }

  Serial.print("tock4");

  myMotor4->run(BACKWARD);
  for (i=0; i<255; i++) {
    myMotor4->setSpeed(i);
    delay(10);
  }
  for (i=255; i!=0; i--) {
    myMotor4->setSpeed(i);
    delay(10);
  }

  Serial.print("tech4");
  myMotor4->run(RELEASE);
  delay(1000);
  
}
