/*
This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
It won't work with v1.x motor shields! Only for the v2's with built in PWM
control

For use with the Adafruit Motor Shield v2
---->	http://www.adafruit.com/products/1438
*/

#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield MC1 = Adafruit_MotorShield(0x60);

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *auger1 = MC1.getMotor(1);
Adafruit_DCMotor *auger2 = MC1.getMotor(2);
Adafruit_DCMotor *auger3 = MC1.getMotor(3);
Adafruit_DCMotor *auger4 = MC1.getMotor(4);

void setup() {
  Serial.begin(115200);           // set up Serial library at 9600 bps
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");

  if (!MC1.begin(100)) {         // create with the default frequency 1.6KHz
    Serial.println("Could not find Motor Shield 1. Check wiring.");
  }

  // Set the speed to start, from 0 (off) to 255 (max speed)
  auger1->setSpeed(150);
  auger1->run(FORWARD);
  // turn on motor
  auger1->run(RELEASE);
}

void loop() {
  uint16_t i;

  Serial.print("tick");

  auger1->run(FORWARD);
  for (i=1000; i<4095; i++) {
    auger1->setSpeedFine(i);
    Serial.println(i);
    delay(100);
  }
  for (i=4095; i!=1000; i--) {
    auger1->setSpeedFine(i);
    Serial.println(i);
    delay(100);
  }

  Serial.print("tock");

  auger1->run(BACKWARD);
  for (i=0; i<4095; i++) {
    auger1->setSpeedFine(i);
    Serial.println(i);
    delay(10);
  }
  for (i=4095; i!=0; i--) {
    auger1->setSpeedFine(i);
    Serial.println(i);
    delay(10);
  }

  Serial.print("tech");
  auger1->run(RELEASE);
  delay(1000);
}
