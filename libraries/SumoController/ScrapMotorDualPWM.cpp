#include "SumoController.h"

ScrapMotorDualPWM::ScrapMotorDualPWM(int PinPWM1, int PinPWM2, int dirMultip) {
	PIN_PWM1 = PinPWM1;
	PIN_PWM2 = PinPWM2;
	setDirectionMultiplier(dirMultip);
	initMotor();
}

// initMotor: initialize pins and reset direction/power
void ScrapMotorDualPWM::initMotor() {
	pinMode(PIN_PWM1,OUTPUT);
	pinMode(PIN_PWM2,OUTPUT);
	setDirection(0);
	setPower(0);
}

// setMotor: set direction + power based on value given
void ScrapMotorDualPWM::setMotor(int pwm) {
	setDirection(pwm);
	setPower(abs(pwm));
}

// setDirectionMultiplier: set direction multiplier (lets you flip direction)
void ScrapMotorDualPWM::setDirectionMultiplier(int multi) {
	if (multi > 0) {
		powerMultiplier = 1; 
	}
	else if (multi < 0) {
		powerMultiplier = -1;
	}
	// do nothing if zero
}

// setDirection: set direction based on sign of number
void ScrapMotorDualPWM::setDirection(int pwm) {
	pwm *= powerMultiplier;
	if (pwm > 0) {
		analogWrite(PIN_PWM1,currPower);
		digitalWrite(PIN_PWM2,LOW);
		currDir = 1;
	}
	else if (pwm < 0) {
		digitalWrite(PIN_PWM1,LOW);
		analogWrite(PIN_PWM2,currPower);
		currDir = -1;
	}
	else {
		digitalWrite(PIN_PWM1,LOW);
		digitalWrite(PIN_PWM2,LOW);
		currDir = 0;
	}
}

// getDirection: return direction: 1, 0, or -1
int ScrapMotorDualPWM::getDirection() {
	return currDir;
}

// setPower: set power (PWM) - make sure is in valid range
void ScrapMotorDualPWM::setPower(int pwm) {
	pwm = max(0,min(255,pwm));
	currPower = pwm;
	if (currDir > 0) {
		analogWrite(PIN_PWM1,currPower);
		digitalWrite(PIN_PWM2,LOW);
	}
	else if (currDir < 0) {
		digitalWrite(PIN_PWM1,LOW);
		analogWrite(PIN_PWM2,currPower);
	}
	else {
		digitalWrite(PIN_PWM1,LOW);
		digitalWrite(PIN_PWM2,LOW);
	}
}

// getPower: return current power (PWM)
int ScrapMotorDualPWM::getPower() {
	return currPower;
}

// stop: stop the motor
void ScrapMotorDualPWM::stop() {
	setMotor(0);
}
