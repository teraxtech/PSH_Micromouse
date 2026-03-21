#include <Arduino.h>
#include "motor.h"

#define CW 1
#define CCW -1

Motor::Motor(){}

Motor::Motor(short cw, short ccw, short a, short b){
	pin_cw = cw;
	pin_ccw = ccw;
	hall_pin_a = a;
	hall_pin_b = b;
	revolutions = 0;
	target = 0;
	direction = 0;
	hall_a = 0;
	hall_b = 0;
}

void Motor::init(){
	// motor outputs go high to spin the motor that direction
	pinMode(this->pin_cw , OUTPUT);
	pinMode(this->pin_ccw , OUTPUT);
	// hall sensor inputs
	pinMode(this->hall_pin_a  , INPUT);
	pinMode(this->hall_pin_b , INPUT);

	hall_a = digitalRead(this->hall_pin_a);
	hall_b = digitalRead(this->hall_pin_b);
}

void Motor::readHall(){
	int new_hall_a = digitalRead(this->hall_pin_a);
	int new_hall_b = digitalRead(this->hall_pin_b);

	// use the state of hall b to determine the direction the motor is spinning
	if(new_hall_b ^ hall_a == 1){
		// stop the motor if it has reached the target posotion
		if(this->revolutions < this->target){
			analogWrite(this->pin_ccw, 0);
			analogWrite(this->pin_cw, 0);
			// this->target++;
		}
		// set current direction to counter clockwise
		// this->direction = CCW;
		// decrement turn counter
		this->revolutions--;

	}else{
		// stop the motor if it has reached the target posotion
		if(this->revolutions > this->target){
			analogWrite(this->pin_cw, 0);
			analogWrite(this->pin_ccw, 0);
			// this->target--;
		}

		// set current direction to clockwise
		// this->direction = CW;
		// increment turn counter
		this->revolutions++;

	}
	hall_a = new_hall_a;
	hall_b = new_hall_b;
}

void Motor::turn(int amount){
	this->target+= amount;
	this->direction = amount>0?1:-1;
	// enable the right pin depending on the motor dirction
	if(this->direction == 1){
		// positive turns go clockwise
		analogWrite(this->pin_cw, 128);
	}else{
		// negative turns go counter clockwise
		analogWrite(this->pin_ccw, 128);
	}
}
