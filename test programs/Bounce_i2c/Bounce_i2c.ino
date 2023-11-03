
//outputs
#define Xstep 3//direct IO
#define Xerror 0
#define Xdirection 1
#define Xenable 2
#define Xmoving 3
#define Ystep 4//direct IO
#define Yerror 4
#define Ydirection 5
#define Yenable 6
#define Ymoving 7

//inputs
#define Xalarm 0
#define XposStop 1
#define XnegStop 2

#define Yalarm 4
#define YposStop 5
#define YnegStop 6

// These defs cause trouble on some versions of Arduino
#undef round

// Use the system yield() whenever possoible, since some platforms require it for housekeeping, especially
// ESP8266
#if (defined(ARDUINO) && ARDUINO >= 155) || defined(ESP8266)
#define YIELD yield();
#else
#define YIELD
#endif

/// Symbolic names for the motor being used
typedef enum
{
  Xmotor = 0,  //just the bowl magnet
  Ymotor  = 1   //entire sled
} StepMotor;

typedef enum
{
  NEGATIVE = 0,  //just the bowl magnet
  POSITIVE  = 1   //entire sled
} direction_t;


#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_PCF8575.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
Adafruit_PCF8575 pcf;


float homeSpeed = 500;
float moveSpeed = 1500;
float userAccel = 2000;


const int serialDirection = 5;
#include <SerialCommand485.h>
SerialCommand485 sCmd;

//todo
//axis alarm (low for alarm)
//axis moving (low for moving)

//commands
//move to position


void setup()
{
  Serial.begin(9600);
  //while (!Serial) {
  //  ; // wait for serial port to connect. Needed for native USB port only
  //}
  if (!pcf.begin(0x20, &Wire)) {
    while (1) {
      Serial.println("Couldn't find PCF8575");
      delay(1000);
    }
  }
  for (uint8_t p = 0; p < 16; p++) {
    pcf.pinMode(p, INPUT_PULLUP);
  }
  pwm.begin();
  pwm.setPWMFreq(1000);  // Set to whatever you like, we don't use it in this demo!
  pwm.setOutputMode(1);
  Wire.setClock(400000);

  delay(1000);
  Serial.println("\nPT Gantry");
  delay(1000);

  pinMode(Xstep, OUTPUT);
  pinMode(Ystep, OUTPUT);
  // Define a stepper and the pins it will use
  AccelStepper(Xmotor, Xstep, Xdirection);
  AccelStepper(Ymotor, Ystep, Ydirection);

  initAxes();

  pinMode(serialDirection, OUTPUT);  //sets the direction of the rs485 converter, 0 to recieve, 1 to transmit
  digitalWrite(serialDirection, 0);  //recieve
  Serial1.setRX(1);
  Serial1.setTX(0);
  Serial1.begin(9600);
  sCmd.addCommand("HOME", homeAxes);        //home both axes
  sCmd.addCommand("EXTREME", gotoExtremes);          //goes to max endstops
  sCmd.addCommand("STOP", stopAll);  //stops motor in motion
  sCmd.addCommand("GETPOS", getPosition);            //returns current position of a given motor (XMOTOR or YMOTOR)
  sCmd.addCommand("MOVESPEED", setMoveSpeed);        //defines movement speed, takes values up to around 1500 is good
  sCmd.addCommand("HOMESPEED", setHomeSpeed);        //defines homing speed, takes values up to around 500 is good
  sCmd.addCommand("ACCEL", setAccel);        //defines acceleration, takes values up to around 2000 is good
  sCmd.setDefaultHandler(unrecognized);             // Handler for command that isn't matched  (says "What?")

}

void loop()
{
  sCmd.readSerial();
  checkAlarms();

}

void stopAll()
{
  eStop(Xmotor);
  eStop(Ymotor);
}
void initAxes()
{
  //re-init the speed fand acceleration for homing
  homeSpeed = 500;
  moveSpeed = 1500;
  userAccel = 2000;
  initAxis(Xmotor);
  initAxis(Ymotor);
}

void initAxis(StepMotor motor)
{
  eStop(motor);  //this resets the local variables that are tracked for speed and acceleration
  setMaxSpeed(motor, moveSpeed);
  setAcceleration(motor, userAccel);
  enableAxis(motor);  //this may want a delay as we just disabled it above in estop
  //stop(motor);  //this may require further hacking to reset the position regardless of acceleration
}

void enableAxis(StepMotor motor)
{
  if (motor == Xmotor) {
    digitalWrite24v(Xenable, 0);
  } else if (motor == Ymotor) {
    digitalWrite24v(Yenable, 0);
  }
}

void disableAxis(StepMotor motor)
{
  if (motor == Xmotor) {
    digitalWrite24v(Xenable, 1);
  } else if (motor == Ymotor) {
    digitalWrite24v(Yenable, 1);
  }
}
void gotoExtremes()
{
  sCmd.clearBuffer();//lets me interrupt this command with another one
  Serial.println("Moving Y");
  gotoExtreme(Ymotor);
  Serial.println("Stop Y");
  Serial.println("Moving X");
  gotoExtreme(Xmotor);
  Serial.println("Stop X");
}


void homeAxes()
{
  sCmd.clearBuffer();//lets me interrupt this command with another one
  Serial.println("Homing Y");
  homeAxis(Ymotor);
  Serial.println("Homed Y");
  Serial.println("Homing X");
  homeAxis(Xmotor);
  Serial.println("Homed X");
}

void homeAxis(StepMotor motor)
{
  float tempSpeed = maxSpeed(motor);
  setMaxSpeed(motor, homeSpeed);
  moveTo(motor, -20000);
  while (!readEndstop(motor, NEGATIVE))//need to have an out if it doesn't reach that
  {
    sCmd.readSerial();
    run(motor);
  }
  eStop(motor);
  setCurrentPosition(motor, 0);
  setMaxSpeed(motor, tempSpeed);
}

void gotoExtreme(StepMotor motor)
{
  moveTo(motor, 20000);
  while (!readEndstop(motor, POSITIVE))//need to have an out if it doesn't reach that
  {
    sCmd.readSerial();
    run(motor);
  }
  eStop(motor);
  Serial.println(currentPosition(motor));

}

void digitalWrite24v(int pinNumber, bool pinState)
{
  if (pinState)//high
  {
    pwm.setPin(pinNumber, 4095);       // turns pin fully on
  }
  else//low
  {
    pwm.setPin(pinNumber, 0);       // turns pin fully off
  }
}

void motorMoving(StepMotor motor)
{
  if (motor == Xmotor)
  {
    digitalWrite24v(Xmoving, 1);
  } else if (motor == Ymotor)
  {
    digitalWrite24v(Ymoving, 1);
  }
}

void motorStoping(StepMotor motor)
{
  if (motor == Xmotor)
  {
    digitalWrite24v(Xmoving, 0);
  } else if (motor == Ymotor)
  {
    digitalWrite24v(Ymoving, 0);
  }
}

void readEndstops() {
  if (! pcf.digitalRead(XposStop)) {
    Serial.println("x positive end reached");
  }
  if (! pcf.digitalRead(XnegStop)) {
    Serial.println("x negative end reached");
  }
  if (! pcf.digitalRead(YposStop)) {
    Serial.println("y positive end reached");
  }
  if (! pcf.digitalRead(YnegStop)) {
    Serial.println("y negative end reached");
  }
}

//returns a 1 for the endstop being reached, a 0 for not reached
int readEndstop(StepMotor motor, direction_t direct) {
  if (motor == Xmotor) {
    if (direct == POSITIVE) {
      return !pcf.digitalRead(XposStop);
    } else if (direct == NEGATIVE) {

      return !pcf.digitalRead(XnegStop);
    } else {
      return -1;
    }
  } else if (motor == Ymotor) {
    if (direct == POSITIVE) {

      return !pcf.digitalRead(YposStop);
    } else if (direct == NEGATIVE) {

      return !pcf.digitalRead(YnegStop);
    } else {
      return -1;
    }
  } else {
    return -1;
  }
}
void setAccel() {

  //take time and set global variable for empty hopper timer
  char *arg;
  arg = sCmd.next();                            // Get the next argument from the SerialCommand object buffer
  if (arg != NULL) {                            // As long as it existed, take it
    userAccel = (atoi(arg));  //set acceleration, something around 2000 is fine
    setAcceleration(Xmotor, userAccel);
    setAcceleration(Ymotor, userAccel);
    Serial.print("Acceleration: ");
    Serial.println(userAccel);
  } else {
    Serial.println("No Accel Args");
  }
}
void setHomeSpeed() {

  //take time and set global variable for empty hopper timer
  char *arg;
  arg = sCmd.next();                            // Get the next argument from the SerialCommand object buffer
  if (arg != NULL) {                            // As long as it existed, take it
    homeSpeed = (atoi(arg));  //set acceleration, something around 2000 is fine
    Serial.print("Homing Speed: ");
    Serial.println(homeSpeed);
  } else {
    Serial.println("No Homing Speed Args");
  }
}
void setMoveSpeed() {

  //take time and set global variable for empty hopper timer
  char *arg;
  arg = sCmd.next();                            // Get the next argument from the SerialCommand object buffer
  if (arg != NULL) {                            // As long as it existed, take it
    moveSpeed = (atoi(arg));  //set acceleration, something around 2000 is fine
    setMaxSpeed(Xmotor, moveSpeed);
    setMaxSpeed(Ymotor, moveSpeed);
    Serial.print("Moving Speed: ");
    Serial.println(moveSpeed);
  } else {
    Serial.println("No Move Speed Args");
  }
}


void getPosition() {
  //take time and set global variable for empty hopper timer
  char *arg;
  arg = sCmd.next();                            // Get the next argument from the SerialCommand object buffer
  if (arg != NULL) {                            // As long as it existed, take it
    //Serial.println(arg);
    String tempString = arg;
    if (tempString == "XMOTOR")
    {
      Serial.println(currentPosition(Xmotor));
      digitalWrite(serialDirection, 1);  //transmit
      delay(1);
      Serial1.println(currentPosition(Xmotor));
      Serial1.flush();
      delay(1);
      digitalWrite(serialDirection, 0);  //recieve
    } else if (tempString == "YMOTOR")
    {
      Serial.println(currentPosition(Ymotor));
      digitalWrite(serialDirection, 1);  //transmit
      delay(1);
      Serial1.println(currentPosition(Ymotor));
      Serial1.flush();
      delay(1);
      digitalWrite(serialDirection, 0);  //recieve
    }
  }
}




//This gets set as the default handler, and gets called when no other command matches.
void unrecognized(const char *command) {

  //digitalWrite(Direction, 1);//transmit
  Serial.print("unknown command: ");
  Serial.println(command);
  //Serial.flush();
  //delay(1);
  //digitalWrite(Direction, 0);//recieve
}

void checkAlarms()//perhaps set a bit to inhibit movement if there's an alarm
{
  if (!pcf.digitalRead(Xalarm)) {
    Serial.println("x alarm");
    digitalWrite24v(Xerror, 1);
  } else
  {
    digitalWrite24v(Xerror, 0);
  }
  
  if (!pcf.digitalRead(Yalarm)) {
    Serial.println("y alarm");
    digitalWrite24v(Yerror, 1);
  } else
  {
    digitalWrite24v(Yerror, 0);
  }
}
