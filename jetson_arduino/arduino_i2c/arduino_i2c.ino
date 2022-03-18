#include <Wire.h>

const uint8_t LED_PIN = 12;
volatile uint8_t test_data[5] = {0};

void setup() {
  Serial.begin(9600);
  Wire.begin(0x08);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // function that executes whenever data is received from writer
  // put your setup code here, to run once:
  pinMode(LED_PIN, OUTPUT);  
}

void loop() {

  delay(100);

  /*
  Serial.print("Enter numbers ");
  readSerial(test_data);
  Serial.println(test_data);
  Wire.beginTransmission(8); // transmit to device #8
  Wire.write(test_data);        // sends the given value
  Wire.endTransmission();    // stop transmitting
  delay(500);
  */
}

void receiveEvent(int howMany) {
   Serial.println("Hello");
   char c = Wire.read(); // receive a character
   Serial.println(c);
   
   if(c == '0'){
     digitalWrite(LED_PIN, LOW);   // turn the LED off by making the voltage LOW
   }
   if(c == '1'){
     digitalWrite(LED_PIN, HIGH);  // turn the LED on (HIGH is the voltage level)
   }
}

/* Read input serial */
int readSerial(uint8_t data[]) {
  int i = 0;
  while (1) {
    while (Serial.available() > 0) {
      uint8_t readByte = Serial.read();
      if (readByte == '\n') {
        test_data[i] = '\0';
        Serial.flush();
        return 0;
      }
      if (readByte != '\r') {
        test_data[i] = readByte;
        i++;
      }
    }
  }
}
