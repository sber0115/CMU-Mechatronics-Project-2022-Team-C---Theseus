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
const uint16_t POT_PIN = A2;
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
  uint16_t pot_deg = read_potentiometer();
  uint32_t ping_distance_cm = read_ping_sensor();
  uint32_t fsr_force_N = read_fsr();
  Serial.println("[" + String(millis()) + "] POTENTIOMETER = " + String(pot_deg) + ((pot_deg == 0) ? "-" : "") + ((pot_deg == 180) ? "+" : "") + " degrees");
  Serial.println("[" + String(millis()) + "] PING DISTANCE = " + String(ping_distance_cm) + " cm");
  Serial.println("[" + String(millis()) + "] FSR FORCE:    = " + String(fsr_force_N) + " N");
  Serial.println("===================================================");
  delay(200); // polling period = 200ms
}

uint16_t read_potentiometer() {
  uint16_t pot_in = analogRead(POT_PIN);
  uint16_t pot_deg = map(pot_in, 0, 1023, 0, 180);

  return pot_deg;
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
  distance_cm = time_ms * SPEED_SOUND / 2 / 1000;    
  return distance_cm;
}

uint32_t read_fsr() {

  uint16_t fsr_in = analogRead(FSR_PIN);
  uint32_t fsr_f = 0;
 
  // analog voltage reading ranges from about 0 to 1023 which maps to 0V to 5V (= 5000mV)
  uint32_t fsr_v = map(fsr_in, 0, 1023, 0, 5000); 
 
  if (fsr_v != 0) {
    // The voltage = Vcc * R / (R + FSR) where R = 10K and Vcc = 5V
    // so FSR = ((Vcc - V) * R) / V        yay math!
    uint32_t fsr_r = 5000 - fsr_v;     // fsrVoltage is in millivolts so 5V = 5000mV
    fsr_r *= 10000;                // 10K resistor
    fsr_r /= fsr_v;
    uint32_t fsr_g = 1000000;           // we measure in micromhos so 
    fsr_g /= fsr_r;
 
    // Use the two FSR guide graphs to approximate the force
    if (fsr_g <= 1000) {
      fsr_f = fsr_g / 80;   
    } else {
      fsr_f = fsr_g - 1000;
      fsr_f /= 30;          
    }
  }
  return fsr_f;
}
