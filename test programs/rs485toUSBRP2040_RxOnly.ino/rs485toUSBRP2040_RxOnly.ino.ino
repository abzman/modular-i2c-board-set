const int serialDirection = 5;

void setup() { // the setup routine runs once:
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  // prints title with ending line break
  Serial.println("rs485 converter Rx Only");

  pinMode(serialDirection, OUTPUT);  //sets the direction of the rs485 converter, 0 to recieve, 1 to transmit
  digitalWrite(serialDirection, 0);  //rx
  Serial1.setRX(1);
  Serial1.setTX(0);
  Serial1.begin(9600);
}
void loop() { // the loop routine runs over and over again forever:

  if (Serial1.available() > 0) {
    // get outgoing byte:
    byte outByte = Serial1.read();
    Serial.write(outByte);
    //Serial1.flush();
  }

}
