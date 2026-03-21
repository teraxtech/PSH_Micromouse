class Motor {
  // static bool interrupt1Free = true;
  // static bool interrupt2Free = true;
  // static void interrupt1(){
  // }
  private:
    // short pin_cw;
    // short pin_ccw;
    short hall_pin_a;
    short hall_pin_b;
    int hall_a;
    int hall_b;

  public:
    short pin_cw;
    short pin_ccw;
    long revolutions;
    int direction;
    long target;
    Motor(short cw, short ccw, short a, short b);
		Motor();

    void readHall();
    // functino to turn the motor by the given number of rotations
    void turn(int amount);
		void init();

    // return the state of hall sensor A
    int get_pin_a(){ return this->hall_pin_a; }
    int get_pin_b(){ return this->hall_pin_b; }

    // return the number of revolutions the motor has completed
    int getRevolutions(){ return this->revolutions; }
    int getTarget(){ return this->target; }
    int getDirection(){ return this->direction; }
    
    // void addInterrupt(void* callback()){
    //   attachInterrupt(digitalPinToInterrupt(this->hall_a), [callback]{ callback(); }, RISING);
    // }
};
