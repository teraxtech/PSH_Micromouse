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
class Motor {
  
  static void interrupt1(){
  }
  private:
    short pin_cw;
    short pin_ccw;
    short hall_pin_a;
    short hall_pin_b;
    int hall_a;
    int hall_b;
    bool direction;
    long revolutions;
    long target = 0;

  public:
    Motor(short cw, short ccw, short a, short b){
        pin_cw = cw;
        pin_ccw = ccw;
        hall_pin_a = a;
        hall_pin_b = b;

        pinMode(cw , OUTPUT);
        pinMode(ccw , OUTPUT);
        pinMode(a  , INPUT);
        pinMode(b , INPUT);
    }

    void readHall(){
      bool hall_a = digitalRead(this->hall_pin_a);
      bool hall_b = digitalRead(this->hall_pin_b);
      
      this->hall_a = hall_a;
      this->hall_b = hall_b;
      short direction = hall_b?-1:1;


      this->revolutions += direction;
      if((this->target-this->revolutions)*direction>0)return;

      if(direction == 1){
        digitalWrite(this->pin_cw, LOW);
      }else{
        digitalWrite(this->pin_ccw, LOW);
      }
    }

    void turn(int amount){
      this->target+= amount;
      if(amount > 0){
        digitalWrite(this->pin_cw, HIGH);
      }else{
        digitalWrite(this->pin_ccw, HIGH);
      }
    }

    int get_pin_a(){ return this->hall_pin_a; }

    int getRevolutions(){ return this->revolutions; }
    
    // void addInterrupt(void* callback()){
    //   attachInterrupt(digitalPinToInterrupt(this->hall_a), [callback]{ callback(); }, RISING);
    // }
};

volatile Motor left_motor = Motor(16,17,3,5);

long int programCounter = 0;

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  // initialize serial communication at 115200 bits per second:
  // Serial.begin(115200);

  //set the resolution to 12 bits (0-4095)
  // analogReadResolution(12);;
  const Motor right_motor = Motor(14,15,2,4);

  // right_motor.attachInterrupt(right_motor.readHall);
  attachInterrupt(digitalPinToInterrupt(right_motor.get_pin_a()), [right_motor]{ right_motor.readHall(); }, RISING);
  // attachInterrupt(digitalPinToInterrupt(left_motor.get_pin_a()), []{ left_motor.readHall(); }, RISING);

  right_motor.turn(1000);
}

long int timeOfLastPrint = 0;
char output_buf[50];
// the loop routine runs over and over again forever:
void loop() {
  programCounter++;

  // if(millis() - timeOfLastPrint > 100){
  //   sprintf(output_buf, "dir: %2d, val1: %4d, val2: %4d \n", 0, right_motor.getRevolutions(), left_motor.getRevolutions());
  //   Serial.println(output_buf);
  //   timeOfLastPrint = millis();
  // }

}
