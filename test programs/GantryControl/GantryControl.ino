#include <Adafruit_PCF8575.h>

/* Example for 16 input buttons that are connected from the GPIO expander pins to ground.
   Note the buttons must be connected with the other side of the switch to GROUND. There is
   a built in pull-up 'resistor' on each input, but no pull-down resistor capability.
*/

#define xNegStop 0
#define xPosStop 1
#define yNegStop 2
#define yPosStop 3

enum axis_t {
  XAXIS,
  YAXIS
};

enum direction_t {
  FORWARD,
  BACKWARD
};

Adafruit_PCF8575 pcf;

void setup() {
  while (!Serial) {
    delay(10);
  }
  Serial.begin(115200);
  Serial.println("Adafruit PCF8575 button read test");

  if (!pcf.begin(0x20, &Wire)) {
    while (1) {
      Serial.println("Couldn't find PCF8575");
      delay(1000);
    }
  }
  for (uint8_t p = 0; p < 4; p++) {
    pcf.pinMode(p, INPUT_PULLUP);
  }
}

void loop() {
  readEndstops();
  Serial.println(readEndstop(YAXIS, negative));
  delay(100); // a short debounce delay
}

void readEndstops() {
  for (uint8_t p = 0; p < 4; p++) {
    if (! pcf.digitalRead(p)) {
      switch (p) {
        case 0:  // your hand is on the sensor
          Serial.println("x negative end reached");
          break;
        case 1:  // your hand is close to the sensor
          Serial.println("x positive end reached");
          break;
        case 2:  // your hand is a few inches from the sensor
          Serial.println("y negative end reached");
          break;
        case 3:  // your hand is nowhere near the sensor
          Serial.println("y positive end reached");
          break;
      }
    }
  }
}

//returns a 1 for the endstop being reached, a 0 for not reached
int readEndstop(axis_t axis, direction_t direct) {
  if (axis == XAXIS) {
    if (direct == FORWARD) {
      return !pcf.digitalRead(xPosStop);
    } else if (direct == BACKWARD) {

      return !pcf.digitalRead(xNegStop);
    } else {
      return -1;
    }
  } else if (axis == YAXIS) {
    if (direct == FORWARD) {

      return !pcf.digitalRead(yPosStop);
    } else if (direct == BACKWARD) {

      return !pcf.digitalRead(yNegStop);
    } else {
      return -1;
    }
  } else {
    return -1;
  }
}
