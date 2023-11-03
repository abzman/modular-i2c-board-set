
//algorithm updated to V6 on 12/14

enum pinState {
  LOW0v,
  HIGH24v,
};

const int serialDirection = 5;
bool currentlyEmpty = 0;

int dispenseHopper = -1;
unsigned long scaleTarget = 0;
unsigned long emptyTarget = 60000;  //60 seconds by default
unsigned long emptyStart = 0;

#include <SerialCommand485.h>
SerialCommand485 sCmd;

#include <Wire.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h>  //Click here to get the library: http://librarymanager/All#SparkFun_I2C_Mux
QWIICMUX myMux;

#include <EEPROM.h>  //Needed to record user settings

#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver Dout = Adafruit_PWMServoDriver();


#include <BTS7960_MotorShield.h>
BTS7960_MotorShield MotorShield1 = BTS7960_MotorShield(0x60);
BTS7960_MotorShield MotorShield2 = BTS7960_MotorShield(0x61);
BTS7960_MotorShield MotorShield3 = BTS7960_MotorShield(0x62);

// Select which 'port' M1, M2, M3 or M4. In this case, M1
BTS7960_DCMotor *auger1 = MotorShield1.getMotor(1);
BTS7960_DCMotor *auger2 = MotorShield1.getMotor(2);
BTS7960_DCMotor *auger3 = MotorShield1.getMotor(3);
BTS7960_DCMotor *auger4 = MotorShield1.getMotor(4);
BTS7960_DCMotor *auger5 = MotorShield2.getMotor(1);
BTS7960_DCMotor *auger6 = MotorShield2.getMotor(2);
BTS7960_DCMotor *auger7 = MotorShield2.getMotor(3);
BTS7960_DCMotor *auger8 = MotorShield2.getMotor(4);
BTS7960_DCMotor *auger9 = MotorShield3.getMotor(1);
BTS7960_DCMotor *auger10 = MotorShield3.getMotor(2);
BTS7960_DCMotor *auger11 = MotorShield3.getMotor(3);
BTS7960_DCMotor *auger12 = MotorShield3.getMotor(4);


#include "SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h"  // Click here to get the library: http://librarymanager/All#SparkFun_NAU7802
NAU7802 scale1;  //Create instance of the NAU7802 class
NAU7802 scale2;  //Create instance of the NAU7802 class
//EEPROM locations to store 4-byte variables
#define LOCATION_CALIBRATION_FACTOR 0  //Float, requires 4 bytes of EEPROM
#define LOCATION_ZERO_OFFSET 10        //Must be more than 4 away from previous spot. Long, requires 4 bytes of EEPROM

bool settingsDetected = false;  //Used to prompt user to calibrate their scale

//Create an array to take average of weights. This helps smooth out jitter.
#define AVG_SIZE 4
float avgWeights[AVG_SIZE];
byte avgWeightSpot = 0;

float boxcar[3][20];
float tempArray[20];
float peakArray[20];
float valleyArray[20];
int boxcarindex[3];
int arraySize = 20;
//int arraySize = (sizeof(boxcar) / sizeof(boxcar[0]));
int newArraySize = arraySize;

void setup() {
  Serial.begin(115200);
  //while (!Serial) {
  //  ; // wait for serial port to connect. Needed for native USB port only
  // }
  delay(5000);
  // prints title with ending line break
  Serial.println("integrated scale controller");

  Serial1.setRX(1);
  Serial1.setTX(0);
  Serial1.begin(9600);

  sCmd.addCommand("WEIGHT", dispenseWeight);        //number of grams, which auger (zero indexed
  sCmd.addCommand("RESET", resetDispense);          //resets all dispense parameters back to defaults
  sCmd.addCommand("EMPTYRESET", emptyHopperReset);  //resets empty hopper bit
  sCmd.addCommand("EMPTY", emptyHopper);            //defines time for the empty hopper parameter
  sCmd.addCommand("PERCENTAGE", setPercent);        //defines percent for pulsing, takes values from 0 to 100
  sCmd.setDefaultHandler(unrecognized);             // Handler for command that isn't matched  (says "What?")

  EEPROM.begin(512);
  pinMode(serialDirection, OUTPUT);  //sets the direction of the rs485 converter, 0 to recieve, 1 to transmit
  digitalWrite(serialDirection, 0);  //recieve

  Wire.begin();

  if (myMux.begin() == false) {
    Serial.println("Mux not detected. Freezing...");
    while (1)
      ;
  }
  Serial.println("Mux detected");

  byte currentPortNumber = myMux.getPort();
  Serial.print("CurrentPort: ");
  Serial.println(currentPortNumber);

  myMux.setPort(1);
  if (scale1.begin() == false) {
    Serial.println("Scale1 not detected. Please check wiring. Freezing...");
    while (1)
      ;
  }
  Serial.println("Scale detected!");
  readSystemSettings(1);  //Load zeroOffset and calibrationFactor from EEPROM

  scale1.setGain(NAU7802_GAIN_64); //Gain can be set to 1, 2, 4, 8, 16, 32, 64, or 128.
  scale1.setSampleRate(NAU7802_SPS_40); //Sample rate can be set to 10, 20, 40, 80, or 320Hz
  scale1.calibrateAFE();                  //Re-cal analog front end when we change gain, sample rate, or channel

  Serial.print("Zero offset: ");
  Serial.println(scale1.getZeroOffset());
  Serial.print("Calibration factor: ");
  Serial.println(scale1.getCalibrationFactor());


  myMux.setPort(2);
  if (scale2.begin() == false) {
    Serial.println("Scale2 not detected. Please check wiring. Freezing...");
    while (1)
      ;
  }
  Serial.println("Scale detected!");
  readSystemSettings(2);  //Load zeroOffset and calibrationFactor from EEPROM

  scale2.setGain(NAU7802_GAIN_64); //Gain can be set to 1, 2, 4, 8, 16, 32, 64, or 128.
  scale2.setSampleRate(NAU7802_SPS_40); //Sample rate can be set to 10, 20, 40, 80, or 320Hz
  scale2.calibrateAFE();                  //Re-cal analog front end when we change gain, sample rate, or channel

  Serial.print("Zero offset: ");
  Serial.println(scale2.getZeroOffset());
  Serial.print("Calibration factor: ");
  Serial.println(scale2.getCalibrationFactor());

  myMux.setPort(0);
  Dout.begin();
  Dout.setOscillatorFrequency(23550000);  //this is deceptive, the steps are pretty big
  Dout.setPWMFreq(1600);                  // This is the maximum PWM frequency

  if (!MotorShield1.begin(100)) {         // create with the default frequency 1.6KHz
    // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield 1. Check wiring.");
    //while (1);
  }
  Serial.println("Motor Shield 1 found.");
  if (!MotorShield2.begin(100)) {         // create with the default frequency 1.6KHz
    // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield 2. Check wiring.");
    //while (1);
  }
  Serial.println("Motor Shield 2 found.");
  if (!MotorShield3.begin(100)) {         // create with the default frequency 1.6KHz
    // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield 3. Check wiring.");
    //while (1);
  }
  Serial.println("Motor Shield 3 found.");

  stopAll();
  //cycleAll();
  dispenseHopper = -1;
}
void loop() {
  //setCurrentMotor(FORWARD, 0);
  //delay(1000);
  sCmd.readSerial();
  /*
    //if our empty hopper timer expired, set output pin
    if ((millis() - emptyStart) > emptyTarget) {
    hopperEmpty();
    }

  */
  //float g = readScale();//remove for more responsive code
  if (Serial.available())
  {
    byte incoming = Serial.read();

    if (incoming == 't') //Tare the scale
      doTare();
    else if (incoming == 'c') //Calibrate
    {
      calibrateScale();
    }
    else if (incoming == 'r') //run cycle (51g test)
    {
      scaleTarget = 51;
      dispenseHopper = 1;
      doTare();
      for (int i = 0; i < 30; i++)//clear out averages
      {
        readScale();
      }
      runCycle();
      //cycleAll();
    }
    else if (incoming == 's') //stop hoppers
    {
      stopAll();
    }
    else if (incoming == 'i') //instantaneous read
    {
      for (int i = 0; i < 30; i++)
      {
        readScale();
      }
    }
  }
}

void runCyclenew() {

  float targetWeight = scaleTarget * 1.0;
  long startTime = millis();
  Serial.println("cycle start");
  Serial.print("target: ");
  Serial.println(targetWeight);
  Serial.println(readScale());



  while (targetWeight - readScale() > 4)
  {

    setCurrentMotor(FORWARD, map(readScale(), 0, targetWeight, 4095, 0));
  }
  setCurrentMotor(FORWARD, 0);

  // final auger reverse
  //Serial.println("final reverse");
  setCurrentMotor(BACKWARD, 4000);
  delay(1500);
  setCurrentMotor(FORWARD, 0);
  delay(1000);

  // end of cycle weight report
  delay(5000);
  for (int i = 0; i < 20; i++)
  {
    readScale();
  }
  /*
    Serial.print("Final Weight: ");
    Serial.print(readScale());
    Serial.println("g");

    // end of cycle and dispense time readoug
    Serial.println("cycle complete");
    Serial.println("****************");
    Serial.println(" ");
    Serial.print("dispense time (seconds):*******************  ");
    Serial.println((millis() - startTime) / 1000.0);
    Serial.println(" ");
    Serial.println("****************");
  */
  dispenseComplete();
  sCmd.clearBuffer();
  rs485Flush();
  stopAll();
  dispenseHopper = -1;
  return;
}
// cycle with reversing compensation
void runCycle() {
  float targetWeight = scaleTarget * 1.0;
  long startTime = millis();
  Serial.println("cycle start");
  Serial.print("target: ");
  Serial.println(targetWeight);
  Serial.println(readScale());

  // initial dispense - one
  while (readScale() <= (targetWeight - 30)) {
    //Serial.println("initial dispense - one");
    //Serial.println(readScale());
    setCurrentMotor(FORWARD, 3000);
  }

  // initial dispense - two
  setCurrentMotor(FORWARD, 0);
  delay(1000);
  while (readScale() <= (targetWeight - 15)) {
    //Serial.println("initial dispense - two");
    setCurrentMotor(FORWARD, 1000);
  }

  // initial dispense - three
  setCurrentMotor(FORWARD, 0);
  delay(1000);
  while (readScale() <= (targetWeight - 15)) {
    //Serial.println("initial dispense - three");
    setCurrentMotor(FORWARD, 1000);
  }

  // first auger reverse
  Serial.println("first reverse");
  setCurrentMotor(FORWARD, 0);
  delay(500);
  setCurrentMotor(BACKWARD, 4000);
  delay(2500);
  setCurrentMotor(FORWARD, 0);
  delay(1000);

  // if weight > targetWeight-6 grams then the cycle is complete, else dispense more
  if (readScale() < (targetWeight - 6)) {

    // forward to re-engage auger drive
    //Serial.println("auger re-engage");
    setCurrentMotor(FORWARD, 3000);
    delay(500);

    // second forward - one
    setCurrentMotor(FORWARD, 0);
    delay(1000);
    while (readScale() < (targetWeight - 6)) {
      //Serial.println("second forward - one");
      setCurrentMotor(FORWARD, 1000);
    }

    // scale settle and check
    setCurrentMotor(FORWARD, 0);
    //delay(1000);

    for (int i = 0; i < 30; i++)
    {
      readScale();
    }

    while (readScale() < (targetWeight - 6)) {
      //Serial.println("second forward - two");
      setCurrentMotor(FORWARD, 1000);
    }

    // scale settle and check
    setCurrentMotor(FORWARD, 0);
    //delay(1000);

    for (int i = 0; i < 30; i++)
    {
      readScale();
    }

    while (readScale() < (targetWeight - 6)) {
      //Serial.println("second forward - three");
      setCurrentMotor(FORWARD, 1000);
    }

    // final auger reverse
    //Serial.println("final reverse");
    setCurrentMotor(BACKWARD, 4000);
    delay(1500);
    setCurrentMotor(FORWARD, 0);
    delay(1000);
  }
  // end of cycle weight report
  delay(2000);
  for (int i = 0; i < 30; i++)
  {
    readScale();
  }
  
    Serial.print("Final Weight: ");
    Serial.print(readScale());
    Serial.println("g");

    // end of cycle and dispense time readoug
    Serial.println("cycle complete");
    Serial.println("****************");
    Serial.println(" ");
    Serial.print("dispense time (seconds):*******************  ");
    Serial.println((millis() - startTime) / 1000.0);
    Serial.println(" ");
    Serial.println("****************");
  
  dispenseComplete();
  sCmd.clearBuffer();
  rs485Flush();
  stopAll();
  dispenseHopper = -1;
  return;
}


//Gives user the ability to set a known weight on the scale and calculate a calibration factor
void calibrateScale(void) {
  myMux.setPort(1);
  Serial.println();
  Serial.println();
  Serial.println(F("Scale1 calibration"));

  Serial.println(F("Setup scale1 with no weight on it. Press a key when ready."));
  while (Serial.available()) Serial.read();   //Clear anything in RX buffer
  while (Serial.available() == 0) delay(10);  //Wait for user to press key

  scale1.calculateZeroOffset(64);  //Zero or Tare the scale. Average over 64 readings.
  Serial.print(F("New zero offset: "));
  Serial.println(scale1.getZeroOffset());

  Serial.println(F("Place known weight on scale1. Press a key when weight is in place and stable."));
  while (Serial.available()) Serial.read();   //Clear anything in RX buffer
  while (Serial.available() == 0) delay(10);  //Wait for user to press key

  Serial.print(F("Please enter the weight, without units, currently sitting on the scale1 (for example '4.25'): "));
  while (Serial.available()) Serial.read();   //Clear anything in RX buffer
  while (Serial.available() == 0) delay(10);  //Wait for user to press key

  //Read user input
  float weightOnScale = Serial.parseFloat();
  Serial.println();

  scale1.calculateCalibrationFactor(weightOnScale, 64);  //Tell the library how much weight is currently on it
  Serial.print(F("New cal factor: "));
  Serial.println(scale1.getCalibrationFactor(), 2);

  Serial.print(F("New Scale Reading: "));
  Serial.println(scale1.getWeight(), 2);

  recordSystemSettings(1);  //Commit these values to EEPROM


  myMux.setPort(2);
  Serial.println();
  Serial.println();
  Serial.println(F("Scale2 calibration"));

  Serial.println(F("Setup scale2 with no weight on it. Press a key when ready."));
  while (Serial.available()) Serial.read();   //Clear anything in RX buffer
  while (Serial.available() == 0) delay(10);  //Wait for user to press key

  scale2.calculateZeroOffset(64);  //Zero or Tare the scale. Average over 64 readings.
  Serial.print(F("New zero offset: "));
  Serial.println(scale2.getZeroOffset());

  Serial.println(F("Place known weight on scale2. Press a key when weight is in place and stable."));
  while (Serial.available()) Serial.read();   //Clear anything in RX buffer
  while (Serial.available() == 0) delay(10);  //Wait for user to press key

  Serial.print(F("Please enter the weight, without units, currently sitting on the scale2 (for example '4.25'): "));
  while (Serial.available()) Serial.read();   //Clear anything in RX buffer
  while (Serial.available() == 0) delay(10);  //Wait for user to press key

  //Read user input
  weightOnScale = Serial.parseFloat();
  Serial.println();

  scale2.calculateCalibrationFactor(weightOnScale, 64);  //Tell the library how much weight is currently on it
  Serial.print(F("New cal factor: "));
  Serial.println(scale2.getCalibrationFactor(), 2);

  Serial.print(F("New Scale Reading: "));
  Serial.println(scale2.getWeight(), 2);

  recordSystemSettings(2);  //Commit these values to EEPROM
}

//Record the current system settings to EEPROM
void recordSystemSettings(int scaleNumber) {
  myMux.setPort(scaleNumber);
  //Get various values from the library and commit them to NVM
  Serial.println(LOCATION_CALIBRATION_FACTOR + ((scaleNumber - 1) * 20));
  Serial.println(LOCATION_ZERO_OFFSET + ((scaleNumber - 1) * 20));
  switch (scaleNumber) {
    case 1:
      EEPROM.put(LOCATION_CALIBRATION_FACTOR + ((scaleNumber - 1) * 20), scale1.getCalibrationFactor());
      EEPROM.put(LOCATION_ZERO_OFFSET + ((scaleNumber - 1) * 20), scale1.getZeroOffset());
      break;
    case 2:
      EEPROM.put(LOCATION_CALIBRATION_FACTOR + ((scaleNumber - 1) * 20), scale2.getCalibrationFactor());
      EEPROM.put(LOCATION_ZERO_OFFSET + ((scaleNumber - 1) * 20), scale2.getZeroOffset());
      break;
    default:
      // statements
      break;
  }
  if (EEPROM.commit()) {
    //Serial.println("EEPROM successfully committed");
  } else {
    //Serial.println("ERROR! EEPROM commit failed");
  }
}

//Reads the current system settings from EEPROM
//If anything looks weird, reset setting to default value
void readSystemSettings(int scaleNumber) {
  myMux.setPort(scaleNumber);
  float settingCalibrationFactor;  //Value used to convert the load cell reading to lbs or kg
  long settingZeroOffset;          //Zero value that is found when scale is tared

  //Look up the calibration factor
  EEPROM.get(LOCATION_CALIBRATION_FACTOR + ((scaleNumber - 1) * 20), settingCalibrationFactor);
  if (settingCalibrationFactor == 0xFFFFFFFF) {
    settingCalibrationFactor = 0;  //Default to 0
    EEPROM.put(LOCATION_CALIBRATION_FACTOR + ((scaleNumber - 1) * 20), settingCalibrationFactor);
    if (EEPROM.commit()) {
      //Serial.println("EEPROM successfully committed");
    } else {
      //Serial.println("ERROR! EEPROM commit failed");
    }
  }

  //Look up the zero tare point
  EEPROM.get(LOCATION_ZERO_OFFSET + ((scaleNumber - 1) * 20), settingZeroOffset);
  if (settingZeroOffset == 0xFFFFFFFF) {
    settingZeroOffset = 1000L;  //Default to 1000 so we don't get inf
    EEPROM.put(LOCATION_ZERO_OFFSET + ((scaleNumber - 1) * 20), settingZeroOffset);
    if (EEPROM.commit()) {
      //Serial.println("EEPROM successfully committed");
    } else {
      //Serial.println("ERROR! EEPROM commit failed");
    }
  }
  switch (scaleNumber) {
    case 1:
      //Pass these values to the library
      scale1.setCalibrationFactor(settingCalibrationFactor);
      scale1.setZeroOffset(settingZeroOffset);
      break;
    case 2:
      //Pass these values to the library
      //hack to allow 2 channels on one scale
      scale2.setCalibrationFactor(settingCalibrationFactor);
      scale2.setZeroOffset(settingZeroOffset);
      break;
    default:
      // statements
      break;
  }
  settingsDetected = true;  //Assume for the moment that there are good cal values
  if (settingCalibrationFactor < 0.1 || settingZeroOffset == 1000)
    settingsDetected = false;  //Defaults detected. Prompt user to cal scale.
}

float readLoadCell(int whichOne) {
  float newReading = 0;
  switch (whichOne) {
    case 1:
      myMux.setPort(1);
      newReading = scale1.getWeight(true, 1);
      Serial.print("first:");
      break;
    case 2:
      myMux.setPort(2);
      newReading = scale2.getWeight(true, 1);
      Serial.print("second:");
      break;
    default:
      Serial.println("error");
      return 0;
      break;
  }
  Serial.print(newReading); //print reading
  Serial.print(",");


  boxcar[whichOne][boxcarindex[whichOne]] = newReading; //insert into boxcar
  boxcarindex[whichOne] += 1; //increment boxcar index
  if (boxcarindex[whichOne] > (arraySize - 1)) //boxcar rollover
    boxcarindex[whichOne] = 0;
  float average = 0;
  for (int i = 0; i < arraySize; i++)
  {
    //Serial.println(i);
    average += boxcar[whichOne][i]; //sum together boxcar data
  }
  average = average / arraySize; //generate average

  switch (whichOne) {
    case 1:
      Serial.print("firstav:");
      break;
    case 2:
      Serial.print("secondav:");
      break;
  }
  Serial.print(average); //print reading
  Serial.print(",");

  float numerator = 0;
  for (int i = 0; i < arraySize; i++)
  {
    numerator += (boxcar[whichOne][i] - average) * (boxcar[whichOne][i] - average);
  }
  float sigma = sqrt(numerator / arraySize);
  /*
    Serial.print(",");
    Serial.print("s:");
    Serial.print(sigma); //print reading
  */
  float upperBound = average + sigma;
  float lowerBound = average - sigma;

  newArraySize = 0;

  for (int i = 0; i < arraySize; i++)//populate new array
  {
    if ((boxcar[whichOne][i] < upperBound) && (boxcar[whichOne][i] > lowerBound))
    {
      tempArray[newArraySize] = boxcar[whichOne][i];
      newArraySize += 1;
    }
  }
  /*
    Serial.print(",");
    Serial.print("ars:");
    Serial.print(newArraySize); //print reading
  */
  float newAverage = 0;
  for (int i = 0; i < newArraySize; i++)
  {
    //Serial.println(i);
    newAverage += tempArray[i]; //sum together boxcar data
  }
  newAverage = newAverage / newArraySize;

  return newAverage;
}

float readScale()
{
  boolean goodReading = false;
  while (goodReading == false)
  {
    myMux.setPort(1);
    if (scale1.available() == true)
    {
      myMux.setPort(2);
      if (scale2.available() == true)
      {
        float currentReading = readLoadCell(1) + readLoadCell(2);
        //float currentReading = readLoadCell(1);
        Serial.print("a:");
        Serial.println(currentReading);
        return currentReading;
        goodReading = true;
      }
    }
  }


  return 0;
}


//steals dir defines from motor shield library
//release is freespin, leave in a direction and speed 0 to brake
void setCurrentMotor(int dir, int rawSpeed)  //4095 is full speed, 0 is stopped
{
  if (dispenseHopper == -1)
  {
    Serial.println("no dispense hopper set");
    return;
  }
  myMux.setPort(0);
  switch (dispenseHopper) {
    case 1:
      auger1->run(dir);
      if (rawSpeed == 0)
      {
        auger1->run(RELEASE);
      }
      auger1->setSpeedFine(rawSpeed);
      break;
    case 2:
      auger2->run(dir);
      if (rawSpeed == 0)
      {
        auger2->run(RELEASE);
      }
      auger2->setSpeedFine(rawSpeed);
      break;
    case 3:
      auger3->run(dir);
      if (rawSpeed == 0)
      {
        auger3->run(RELEASE);
      }
      auger3->setSpeedFine(rawSpeed);
      break;
    case 4:
      auger4->run(dir);
      if (rawSpeed == 0)
      {
        auger4->run(RELEASE);
      }
      auger4->setSpeedFine(rawSpeed);
      break;
    case 5:
      auger5->run(dir);
      if (rawSpeed == 0)
      {
        auger5->run(RELEASE);
      }
      auger5->setSpeedFine(rawSpeed);
      break;
    case 6:
      auger6->run(dir);
      if (rawSpeed == 0)
      {
        auger6->run(RELEASE);
      }
      auger6->setSpeedFine(rawSpeed);
      break;
    case 7:
      auger7->run(dir);
      if (rawSpeed == 0)
      {
        auger7->run(RELEASE);
      }
      auger7->setSpeedFine(rawSpeed);
      break;
    case 8:
      auger8->run(dir);
      if (rawSpeed == 0)
      {
        auger8->run(RELEASE);
      }
      auger8->setSpeedFine(rawSpeed);
      break;
    case 9:
      auger9->run(dir);
      if (rawSpeed == 0)
      {
        auger9->run(RELEASE);
      }
      auger9->setSpeedFine(rawSpeed);
      break;
    case 10:
      auger10->run(dir);
      if (rawSpeed == 0)
      {
        auger10->run(RELEASE);
      }
      auger10->setSpeedFine(rawSpeed);
      break;
    case 11:
      auger11->run(dir);
      if (rawSpeed == 0)
      {
        auger11->run(RELEASE);
      }
      auger11->setSpeedFine(rawSpeed);
      break;
    default:
      Serial.print(dispenseHopper);
      Serial.println(" is not a valid auger number");
      return;
      break;
  }
}

void setDout(enum pinState i, int pin) {
  if (i) {
    myMux.setPort(0);
    Dout.setPWM(pin, 4096, 0);  // turns pin fully on
  } else {
    myMux.setPort(0);
    Dout.setPWM(pin, 0, 4096);  // turns pin fully off
  }
}

void dispenseIncomplete() {
  setDout(LOW0v, 0);
}

void dispenseComplete() {
  setDout(HIGH24v, 0);
}

void runContinuous() {
  setDout(LOW0v, 1);
}

void pulseMode() {
  setDout(HIGH24v, 1);
}

void hopperNotEmpty() {
  setDout(LOW0v, 2);
  currentlyEmpty = 0;
}

void hopperEmpty() {
  setDout(HIGH24v, 2);
  currentlyEmpty = 1;
}

void lessThanHalf() {
  setDout(LOW0v, 3);
}

void moreThanHalf() {
  setDout(HIGH24v, 3);
}

void emptyHopperReset() {

  //reset variable for empty hopper
  hopperNotEmpty();  //set machine not in empty hopper state
  emptyStart = millis();
  Serial.println("empty hopper reset");
}

void dispenseWeight() {

  Serial.println("in dispense weight command");
  char *arg;
  arg = sCmd.next();          // Get the next argument from the SerialCommand object buffer
  if (arg != NULL) {          // As long as it existed, take it
    scaleTarget = atoi(arg);  //set target in g
    //bodge code here
    //161g is blue scoop ham
    //133g for blue scoop italian sausage
    //51g for white scoop
    //91g for blue scoop green pepper
    //37g for white scoop
    //136g for red scoop mushroom
    //159g for 2x red scoops of mushroom
    if (scaleTarget == 133) //italian sausage make single topping pizza white scoop
    {
      scaleTarget = 51;
    }
    else if (scaleTarget == 161) //turn blue scoop ham to italian sausage
    {
      scaleTarget = 133;
    }
    else if (scaleTarget == 144) //italian sausage make single topping pizza white scoop
    {
      scaleTarget = 91;
    }
    else if (scaleTarget == 91) //green pepper make single topping pizza white scoop
    {
      scaleTarget = 37;
    }
    else if (scaleTarget == 136)//makes red scoop canned mush
    {
      scaleTarget = 159;
    }

    arg = sCmd.next();        // Get the next argument from the SerialCommand object buffer
    if (arg != NULL) {        // As long as it existed, take it
      dispenseHopper = atoi(arg) + 1;  //set hopper, it comes in zero indexed
      Serial.println("reading in cycle command");
      Serial.print("target in g: ");
      Serial.println(scaleTarget);
      Serial.print("hopper #");
      Serial.println(dispenseHopper);

      doTare();
      for (int i = 0; i < 30; i++)//clear out averages
      {
        readScale();
      }
      dispenseIncomplete();
      runContinuous();
      lessThanHalf();
      hopperNotEmpty();

      digitalWrite(serialDirection, 1);  //transmit
      delay(1);
      Serial1.println("RECEIVED");
      Serial1.flush();
      delay(1);
      digitalWrite(serialDirection, 0);  //recieve

      Serial.println("run cycle");
      runCycle();
    }
  }
}

void resetDispense() {

  unsigned long beamTime = 0;
  scaleTarget = 0;
  dispenseHopper = -1;
  boolean firstRun = 0;
  dispenseComplete();
  pulseMode();
  hopperEmpty();
  moreThanHalf();
  digitalWrite(serialDirection, 0);  //recieve
  Serial.println("reset dispense");
}

void emptyHopper() {

  //take time and set global variable for empty hopper timer
  char *arg;
  arg = sCmd.next();          // Get the next argument from the SerialCommand object buffer
  if (arg != NULL) {          // As long as it existed, take it
    emptyTarget = atoi(arg);  //set empty time in ms
    Serial.println(arg);
  } else {
  }
}

//do nothing
void setPercent() {

  //take time and set global variable for empty hopper timer
  char *arg;
  arg = sCmd.next();                            // Get the next argument from the SerialCommand object buffer
  if (arg != NULL) {                            // As long as it existed, take it
    float pulsePercentage = (atoi(arg) / 100);  //set empty time from 0 to 1
    Serial.println(pulsePercentage);
  } else {
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

void doTare() {
  Serial.println("tare scale 1");

  myMux.setPort(1);
  float newReading = scale1.getWeight(true, 1);
  Serial.println(newReading);
  while ((newReading > 0.30) || (newReading < -0.30)) {
    scale1.calculateZeroOffset(32);
    newReading = scale1.getWeight(true, 1);
    Serial.print("tare reading scale 1");
    Serial.println(newReading);
  }

  myMux.setPort(2);
  newReading = scale2.getWeight(true, 1);
  Serial.println(newReading);
  while ((newReading > 0.30) || (newReading < -0.30)) {
    scale2.calculateZeroOffset(32);
    newReading = scale2.getWeight(true, 1);
    Serial.print("tare reading scale 2");
    Serial.println(newReading);
  }
}

void rs485Flush() {
  while (Serial1.available() > 0) {
    char t = Serial1.read();
  }
}

void stopAll() {
  for (dispenseHopper = 1; dispenseHopper < 12; dispenseHopper++)
  {
    setCurrentMotor(FORWARD, 0);
  }
  delay(1000);
}

void cycleAll() {
  Serial.println("cycle all");
  for (dispenseHopper = 1; dispenseHopper < 12; dispenseHopper++)
  {
    //initial test of both directions of both motor controllers
    Serial.println(dispenseHopper);
    setCurrentMotor(FORWARD, 4095);
    delay(1000);
    setCurrentMotor(FORWARD, 0);
    delay(1000);
    setCurrentMotor(BACKWARD, 4095);
    delay(1000);
    setCurrentMotor(BACKWARD, 0);
    delay(1000);
  }
}
