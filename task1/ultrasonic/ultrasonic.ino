#define echoPin 2 // pin D2 Arduino to pin Echo
#define trigPin 3 // pin D3 Arduino to pin Trig

long time;      // time (ms) of sound wave traveled
int distance;   // measured distance

void setup(){
  pinMode(trigPin, OUTPUT); // sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT);  // sets the echoPin as an INPUT
  Serial.begin(9600);       // sets the serial communication with 9600 baudrate
  Serial.println("Ultrasonic Sensor Test");
}

void loop(){
  digitalWrite(trigPin, LOW);   // clears the trigPin
  delayMicroseconds(2);       
  digitalWrite(trigPin, HIGH);  // generates ultrasonic pulse for 10 microseconds from trigPin
  delayMicroseconds(10);    
  digitalWrite(trigPin, LOW);
  time = pulseIn(echoPin, HIGH);  // reads sound wave travel time (ms) from echoPin
  distance = time * 0.034 / 2;    // calculates the distance with time and the speed of sound
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println("cm");
}
