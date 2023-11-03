const int serialDirection = 5;

void setup() { // the setup routine runs once:
    //Initialize serial and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  // prints title with ending line break
  Serial.println("rs485 converter");

  pinMode(serialDirection, OUTPUT);  //sets the direction of the rs485 converter, 0 to recieve, 1 to transmit
  digitalWrite(serialDirection, 0);  //recieve
  Serial1.setRX(1);
  Serial1.setTX(0);
  Serial1.begin(9600);
}
void loop() { // the loop routine runs over and over again forever:

  if (Serial1.available() > 0) {
    // get incoming byte:
    byte inByte = Serial1.read();
    Serial.write(inByte);
  }
  if (Serial.available() > 0) {
    // get outgoing byte:
    byte outByte = Serial.read();
    digitalWrite(serialDirection, 1);  //transmit
    delay(1);
    Serial1.write(outByte);
    Serial1.flush();
    delay(1);
    digitalWrite(serialDirection, 0);  //recieve
  }

}
