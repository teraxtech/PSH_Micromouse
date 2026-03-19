#include "motor.h"

struct Sensor{
  int trigPin;
  int echoPin;
};

long ping(Sensor sensor){
  digitalWrite(sensor.trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(sensor.trigPin, HIGH);
  delayMicroseconds(11);
  digitalWrite(sensor.trigPin, LOW);

  long duration = pulseIn(sensor.echoPin, HIGH);
  return duration * 69/4; //34.5 mm/s divided by 2
  // return (duration * 0.0345 * 0.5);
}

Sensor newSensor(int trigPin, int echoPin){
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Sensor s = {trigPin, echoPin};
  return s;
}

Sensor left;
Sensor right;
Sensor front;

Motor left_motor = *(new Motor(14,15,2,4));
Motor right_motor = *(new Motor(16,17,3,5));

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

  left = newSensor(6, 7);
  right = newSensor(8, 9);
  front = newSensor(10, 11);

  attachInterrupt(digitalPinToInterrupt(right_motor.get_pin_a()), [right_motor]{ right_motor.readHall(); }, RISING);
  // attachInterrupt(digitalPinToInterrupt(right_motor.get_pin_b()), [right_motor]{ right_motor.readHall(); }, RISING);
  attachInterrupt(digitalPinToInterrupt(left_motor.get_pin_a()), [left_motor]{ left_motor.readHall(); }, RISING);
  // attachInterrupt(digitalPinToInterrupt(left_motor.get_pin_b()), [left_motor]{ left_motor.readHall(); }, RISING);

  left_motor.turn(-4000);
  right_motor.turn(4000);
  // left_motor.turn(-1000);
  // right_motor.turn(1330);
}


//some variables to help with printing debug info
long int timeOfLastPrint = 0;
char output_buf[130];
int state = 0;
// the loop routine runs over and over again forever:
void loop() {

  // long distanceL = ping(left);
  // delay(10);
  // long distanceR = ping(right);
  // delay(10);
  // long distanceF = ping(front);

  if(millis() - timeOfLastPrint > 100){
    // Serial.print("DistanceL: ");
    // Serial.print(distanceL);
    // Serial.print(" cm, DistanceR: ");
    // Serial.print(distanceR);
    // Serial.print(" cm, DistanceF: ");
    // Serial.print(distanceF);
    // Serial.println(" cm");

    sprintf(output_buf, "s: %2d, right_rev: %4d, target: %4d, dir: %d, left_rev: %4d, target: %4d, dir: %2d \n", state, right_motor.getRevolutions(), right_motor.getTarget(), right_motor.getDirection(), left_motor.getRevolutions(), left_motor.getTarget(), left_motor.getDirection());
    // sprintf(output_buf, "right_rev: %4d, target: %4d, dir: %d, left_rev: %4d, target: %4d, dir: %2d, distL: %4d, distR: %4d \n", right_motor.getRevolutions(), right_motor.getTarget(), right_motor.getDirection(), left_motor.getRevolutions(), left_motor.getTarget(), left_motor.getDirection());
    Serial.println(output_buf);
    // sprintf(output_buf, "distL: %6dmm, distF: %6dmm, distR: %6dmm \n", distanceL, distanceF, distanceR);
    // Serial.println(output_buf);
    timeOfLastPrint = millis();
  }
  switch(state){
    case 0: 
      if(millis() > 8000){
        state++;
        right_motor.turn(-1030);
      }
      break;
    case 1:
      if(millis() > 10000){
        state++;
        left_motor.turn(-1030);
      }
      break;
    case 2:
      if(millis() > 12000){
        state++;
        left_motor.turn(-4000);
        right_motor.turn(4000);
        // left_motor.turn(-1330);
      }
      break;
    // default:
  }

}
