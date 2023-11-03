const int Direction = 4;

void setup() {
  pinMode(Direction, OUTPUT);

  Serial1.begin(9600);
  Serial.begin(9600);
    while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("rs485 converter");
  digitalWrite(Direction, 0);//Rx

}

void loop() {
  if(Serial1.available() > 0){
    byte inByte = Serial1.read();
    Serial.write(inByte);
  }

}
