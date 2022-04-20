#include <Wire.h>

const uint8_t SLAVE_ADDR = 0x02;

struct msg_format {
    uint8_t ex1;
    uint8_t ex2;
    uint8_t ex3;
    uint8_t ex4;
};

const uint8_t MSG_LEN = 4;

volatile msg_format test_msg = {'n', 'a', 'n', 'o'};
volatile msg_format rx_msg = {0,0,0,0};

void setup() {
  Serial.begin(9600);
  Wire.begin(); // enter the bus as master

}

void loop() {
  Wire.beginTransmission(SLAVE_ADDR);
    Wire.write(test_msg.ex1);
    Wire.write(test_msg.ex2);
    Wire.write(test_msg.ex3);
    Wire.write(test_msg.ex4);
  Wire.endTransmission();
  delay(1000);
  Wire.requestFrom(SLAVE_ADDR, MSG_LEN);
  rx_msg.ex1 = Wire.read();
  rx_msg.ex2 = Wire.read();
  rx_msg.ex3 = Wire.read();
  rx_msg.ex4 = Wire.read();
  Serial.print("RX: " + String((char) rx_msg.ex1));
  Serial.print((char) rx_msg.ex2);
  Serial.print((char) rx_msg.ex3); 
  Serial.println((char) rx_msg.ex4);
  delay(1000);
  
}
