#include <Arduino.h>
#include "motor.h"

#define CW 1
#define CCW -1

Motor::Motor(short cw, short ccw, short a, short b){
	pin_cw = cw;
	pin_ccw = ccw;
	hall_pin_a = a;
	hall_pin_b = b;
	revolutions = 0;
	target = 0;
	direction = 0;
	hall_a = digitalRead(this->hall_pin_a);
	hall_b = digitalRead(this->hall_pin_b);

	// motor outputs go high to spin the motor that direction
	pinMode(cw , OUTPUT);
	pinMode(ccw , OUTPUT);
	// hall sensor inputs
	pinMode(a  , INPUT);
	pinMode(b , INPUT);
}

void Motor::readHall(){
	int new_hall_a = digitalRead(this->hall_pin_a);
	int new_hall_b = digitalRead(this->hall_pin_b);

	// use the state of hall b to determine the direction the motor is spinning
	if(new_hall_b ^ hall_a == 1){
		// stop the motor if it has reached the target posotion
		if(this->revolutions > this->target){
			digitalWrite(this->pin_cw, LOW);
			digitalWrite(this->pin_ccw, LOW);
			// this->target--;
		}

		// set current direction to clockwise
		this->direction = CW;
		// increment turn counter
		this->revolutions++;

	}else{
		// stop the motor if it has reached the target posotion
		if(this->revolutions < this->target){
			digitalWrite(this->pin_ccw, LOW);
			digitalWrite(this->pin_cw, LOW);
			// this->target++;
		}
		// set current direction to counter clockwise
		this->direction = CCW;
		// decrement turn counter
		this->revolutions--;

	}
	hall_a = new_hall_a;
	hall_b = new_hall_b;
}

void Motor::turn(int amount){
	this->target+= amount;
	// enable the right pin depending on the motor dirction
	if(amount > 0){
		// positive turns go clockwise
		digitalWrite(this->pin_cw, HIGH);
	}else{
		// negative turns go counter clockwise
		digitalWrite(this->pin_ccw, HIGH);
	}
}
