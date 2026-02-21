// Distance sensors - Ultrasonic Rangefinders
const int trigPinL = 6;
const int echoPinL = 7;

const int trigPinR = 10;
const int echoPinR = 11;


void setup() {
  Serial.begin(9600);
  pinMode(trigPinL, OUTPUT);
  pinMode(echoPinL, INPUT);
  pinMode(trigPinR, OUTPUT);
  pinMode(echoPinR, INPUT);
}

void loop() {
  digitalWrite(trigPinL, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinL, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinL, LOW);
//----
  digitalWrite(trigPinR, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinR, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinR, LOW);
//=================

  long durationL = pulseIn(echoPinL, HIGH);
  int distanceL = (durationL * 0.0345 * 0.5) - 3;

  long durationR = pulseIn(echoPinR, HIGH);
  int distanceR = (durationR * 0.0345 *0.5); 

  Serial.print("DistanceL: ");
  Serial.print(distanceL);
  Serial.println(" cm");
  Serial.println();
  Serial.print("DistanceR: ");
  Serial.print(distanceR);
  Serial.println(" cm");

  delay(100);
}
