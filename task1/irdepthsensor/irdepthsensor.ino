const int IR_PIN = A1;
const float VCC = 4.98;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(IR_PIN, INPUT);
}

void loop() {
  int irADC = analogRead(IR_PIN);
  float irV = irADC * VCC / 1023.0;
  float distance = 29.988 * pow(irV, -1.173);
  Serial.println("Distance: " + String(distance) + " cm");
  Serial.println();

  delay(500);
}
