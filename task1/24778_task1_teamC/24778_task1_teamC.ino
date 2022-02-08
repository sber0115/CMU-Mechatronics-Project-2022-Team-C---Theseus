/*
 * 24778 MECHATRONICS SPRING 2022
 * TEAM C - PROJECT THESEUS 
 * ETHAN RICH, VALMIKI, SEBASTIAN BERNAL, SAM YIN, DAJUN TAO
 * 
 * FSR DATASHEET:  http://cdn.sparkfun.com/datasheets/Sensors/ForceFlex/2010-10-26-DataSheet-FSR400-Layout2.pdf
 * IR DATASHEET:   https://www.sparkfun.com/datasheets/Components/GP2Y0A21YK.pdf
 * PING DATASHEET: https://www.parallax.com/product/ping-ultrasonic-distance-sensor/
 * ADDT'L SOURCES:
 *   https://learn.adafruit.com/force-sensitive-resistor-fsr/using-an-fsr
 *   https://www.makerguides.com/sharp-gp2y0a21yk0f-ir-distance-sensor-arduino-tutorial/
 */


// Arduino pin mappings
const uint16_t FSR_PIN = A0;
const uint16_t IR_PIN = A1;
const uint16_t PING_ECHO_PIN = 2;
const uint16_t PING_TRIG_PIN = 3;

// inverse speed of sound in ms/cm
const uint16_t SPEED_SOUND = 34; // [cm/ms]

// measured VCC value
const uint16_t VCC_MEAS = 1020;  // [V SCALED: 0-5V -> 0-1024] 
// measured FSR pull down resistance
const uint16_t R_DIV = 3230;     // [OHM]


void setup() {
  pinMode(PING_ECHO_PIN, INPUT);
  pinMode(PING_TRIG_PIN, OUTPUT);
  // FSR_PIN / IR_PIN Analog inputs, 
  // no call to pinMode() required

  Serial.begin(115200);
}

void loop() {
  Serial.println("=================[POLLING SENSORS]=================");
  double ir_distance_cm = read_ir_sensor();
  uint32_t ping_distance_cm = read_ping_sensor();
  uint32_t fsr_force_N = read_fsr();
  Serial.println("[" + String(millis()) + "] IR DISTANCE   = " + String(ir_distance_cm, 1) + " cm");
  Serial.println("[" + String(millis()) + "] PING DISTANCE = " + String(ping_distance_cm) + " cm");
  Serial.println("[" + String(millis()) + "] FSR FORCE:    = " + String(fsr_force_N) + " N");
  Serial.println("===================================================");
  delay(200); // polling period = 200ms
}

double read_ir_sensor() {
  uint16_t ir_in = analogRead(IR_PIN);

  // calculate distance based on curve in datasheet
  double distance_cm = 29.988 * pow(ir_in, -1.173);

  return distance_cm;
  
}

uint32_t read_ping_sensor() {     
  uint32_t time_ms = 0;
  uint32_t distance_cm = 0;

  // generates 40kHz ultrasonic pulse for 10ms
  digitalWrite(PING_TRIG_PIN, HIGH);  
  delayMicroseconds(10);    
  digitalWrite(PING_TRIG_PIN, LOW);
  
  // reads sound wave travel time (ms/bit) from echoPin
  time_ms = pulseIn(PING_ECHO_PIN, HIGH); 
   
  // calculates the distance (div. by 2 for time of flight)
  distance_cm = time_ms * SPEED_SOUND / 2;    
  return distance_cm;
}

// 30K -> 0.3K
uint32_t read_fsr() {
    int fsrADC = analogRead(FSR_PIN);
  // If the FSR has no pressure, the resistance will be
  // near infinite. So the voltage should be near 0.
  float force_N = 0;
  if (fsrADC != 0) {
    // Use ADC reading to calculate voltage:
    float fsrV = fsrADC * VCC_MEAS / 1023.0;
    // Use voltage and static resistor value to 
    // calculate FSR resistance:
    float fsrR = R_DIV * (VCC_MEAS / fsrV - 1.0);
    Serial.println("Resistance: " + String(fsrR) + " ohms");
    // Guesstimate force based on slopes in figure 3 of
    // FSR datasheet:
    
    float fsrG = 1.0 / fsrR; // Calculate conductance
    // Break parabolic curve down into two linear slopes:
    if (fsrR <= 600) 
      force_N = (fsrG - 0.00075) / 0.00000032639;
    else
      force_N =  fsrG / 0.000000642857; 
  }
  return force_N;
}
