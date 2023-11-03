const int Direction = 4;

void setup() { // the setup routine runs once:
  pinMode(Direction, OUTPUT); //sets the direction of the rs485 converter, 0 to recieve, 1 to transmit
  
  Serial1.begin(9600);  //initialize serial communication at 115200 bits per second

  // set the data rate for the SoftwareSerial port
  Serial.begin(9600);
  Serial.println("Drop Sensor Time connection");
  digitalWrite(Direction, 0);//recieve
}
void loop() { // the loop routine runs over and over again forever:
 
  if (Serial1.available() > 0) {
    // get incoming byte:
    byte inByte = Serial1.read();
    Serial.write(inByte);
  }
  
}
