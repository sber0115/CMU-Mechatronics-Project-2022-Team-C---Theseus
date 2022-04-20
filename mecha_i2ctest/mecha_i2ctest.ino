#include <Wire.h>

const uint8_t SELF_ADDR = 0x08;

const uint8_t MSG_LEN = 4;

struct msg_format {
    uint8_t ex1;
    uint8_t ex2;
    uint8_t ex3;
    uint8_t ex4;
};

volatile msg_format tx_message = {'t', 'e', 's', 't'};
volatile msg_format rx_msg = {0,0,0,0};


void setup() {
    Serial.begin(9600);
    Wire.begin(SELF_ADDR);
    Wire.onRequest(event_tx_msg);
    Wire.onReceive(event_rx_msg);
}

void loop() {
    // put your main code here, to run repeatedly:

}

// Event handler for requests to send messages to master
void event_tx_msg() {
    Wire.write((byte *) &tx_message, MSG_LEN);
    return;
}

// Event handler for when data is ready to be received from master
void event_rx_msg(int num_bytes) {
  rx_msg.ex1 = Wire.read();
  rx_msg.ex2 = Wire.read();
  rx_msg.ex3 = Wire.read();
  rx_msg.ex4 = Wire.read();
  Serial.print("RX: " + String((char) rx_msg.ex1));
  Serial.print((char) rx_msg.ex2);
  Serial.print((char) rx_msg.ex3); 
  Serial.println((char) rx_msg.ex4);  
  return;
}
