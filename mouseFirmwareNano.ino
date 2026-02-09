/*
  ReadAnalogVoltage

  Reads an analog input on pin 0, converts it to voltage, and prints the result to the Serial Monitor.
  Graphical representation is available using Serial Plotter (Tools > Serial Plotter menu).
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/ReadAnalogVoltage
*/

volatile int direction = 1; //1=clockwise, -1 = counterClockwise
int rotationThreshhold = 1;
struct motor_state {
  short pin_cw;
  short pin_ccw;
  short hall_pin_a;
  short hall_pin_b;
  bool direction;
  long revolutions;
};

volatile motor_state motor1 = {7,6,2,4, true, 0};
volatile motor_state motor2 = {9,8,3,5, true, 0};
volatile int count = 0;


long int programCounter = 0;

long getRevolution(volatile motor_state &state);

void handleHallTrigger_motor1(){
  motor1.revolutions += getRevolution(motor1);
}
void handleHallTrigger_motor2(){
  motor2.revolutions += getRevolution(motor2);
}

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  // initialize serial communication at 115200 bits per second:
  // Serial.begin(115200);

  //set the resolution to 12 bits (0-4095)
  // analogReadResolution(12);;
  pinMode(motor1.pin_cw , OUTPUT);
  pinMode(motor1.pin_ccw , OUTPUT);
  pinMode(motor2.pin_cw , OUTPUT);
  pinMode(motor2.pin_ccw , OUTPUT);
  //
  //
  pinMode(motor1.hall_pin_a  , INPUT);
  pinMode(motor1.hall_pin_b , INPUT);
  pinMode(motor2.hall_pin_a  , INPUT);
  pinMode(motor2.hall_pin_b , INPUT);

  attachInterrupt(digitalPinToInterrupt(motor1.hall_pin_a), handleHallTrigger_motor1, HIGH);
  attachInterrupt(digitalPinToInterrupt(motor2.hall_pin_a), handleHallTrigger_motor2, HIGH);
}


long getRevolution(volatile motor_state &state){
  int sensorValue1 = digitalRead(state.hall_pin_a);
  int sensorValue2 = digitalRead(state.hall_pin_b);
  bool hall_a = sensorValue1 == HIGH;
  bool hall_b = sensorValue2 == HIGH;

  
  // if(past_hall_a == hall_a && past_hall_b == hall_b) return previous_count;
  // if(hall_a == 0) return 0;
  bool clockwise = hall_b;

  motor_state.hall_a = hall_a
  motor_state.hall_b = hall_b
  Serial.println(clockwise);


  return clockwise?-1:1;
}

// the loop routine runs over and over again forever:
void loop() {
  programCounter++;


  int timestep = 4000;
  if(programCounter%timestep == 0){
    if(programCounter%(2*timestep) == 0*timestep)direction = -1;
    if(programCounter%(2*timestep) == 1*timestep)direction = 1;
  }

  // read the input on analog pin 0:
  int sensorValue1 = digitalRead(motor1.hall_pin_a);
  int sensorValue2 = digitalRead(motor1.hall_pin_b);

  // count = countRotations(previous_state, count, sensorValue1, sensorValue2);

  // previous_state.motor1a = false;

  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  // float voltage1 = sensorValue1 * (5.0 / 1023.0);
  // float voltage2 = sensorValue2 * (5.0 / 1023.0);
  // print out the value you read:

  char output_buf[50];
  if (programCounter%100==0){
    // sprintf(output_buf, "dir: %2d, val1: %4d, val2: %4d", direction, sensorValue1, sensorValue2);
    // Serial.print(output_buf);
    // Serial.print(", count = ");
    // Serial.print(motor1.revolutions);
    // Serial.print(", count = ");
    // Serial.print(count);
    // Serial.print(", clock = ");
    // Serial.println(programCounter);
  }

  // if((sensorValue1 - 512) * rotationThreshhold > 400){
  //   count+=direction;
  //   rotationThreshhold*=-1;
  // }


  switch(direction){
    case 1:
      digitalWrite(motor2.pin_ccw, LOW);
      if(motor2.revolutions<3000){
        digitalWrite(motor2.pin_cw, HIGH);
      }else{
        digitalWrite(motor2.pin_cw, LOW);
      }
      break;
    case 0:
      digitalWrite(motor2.pin_cw, LOW);
      digitalWrite(motor2.pin_ccw, LOW);
      break;
    case -1:
      digitalWrite(motor2.pin_cw, LOW);
      if(motor2.revolutions>0){
        digitalWrite(motor2.pin_ccw, HIGH);
      }else{
        digitalWrite(motor2.pin_ccw, LOW);
      }
      break;
  }
}
