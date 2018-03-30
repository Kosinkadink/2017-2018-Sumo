#include "ScrapController.h"
#include "SumoController.h"


#define SENSOR_PIN A7

#define FRONT_LEFT_PIN_INTERRUPT 3
#define FRONT_LEFT_PIN_CHECKER 4

#define FRONT_LEFT_MOTOR_PWM1 9
#define FRONT_LEFT_MOTOR_PWM2 10

ScrapEncoder encoderFL = ScrapEncoder(FRONT_LEFT_PIN_INTERRUPT, FRONT_LEFT_PIN_CHECKER);
ScrapMotorDualPWM motorFL = ScrapMotorDualPWM(FRONT_LEFT_MOTOR_PWM1, FRONT_LEFT_MOTOR_PWM2);
ScrapMotorControl speedFL = ScrapMotorControl(motorFL, encoderFL);


unsigned long startTime = micros();
unsigned long printTime = micros();
unsigned long switchTime = micros();

float rotation = 230;
long previousCount = 0;


void setupOLD() {
	pinMode(13,OUTPUT);
}


void loopOLD() {
	digitalWrite(13,HIGH);
	delay(500);
	digitalWrite(13,LOW);
	delay(500);
}



void setup() {
	initEncoders();
	Serial.begin(9600);	
	speedFL.setMinPower(30);
	speedFL.setMinSpeed(30);
	speedFL.setMaxSpeed(10000);
	//speedFL.setControl(800);
	//speedFL.setControl(0);
	motorFL.setMotor(-100);
}

void loop() {
	unsigned long currentTime = micros();
	if (currentTime - startTime > 2000) {
		//speedFL.performMovement();
		startTime = currentTime;
	}	
	/*if (currentTime - printTime > 20000) {
		Serial.println(analogRead(SENSOR_PIN));
		printTime = currentTime;
	}*/
	if (currentTime - printTime > 500000) {
		long currentCount = encoderFL.getCount();
		Serial.println(encoderFL.getCount());
		printTime = currentTime;
		/*float rps = (((float)currentCount-previousCount)/rotation)/((currentTime - printTime)/1000000.0);
		float rpm = rps*60;
		
		Serial.print(rps);
		Serial.print("\t");
		Serial.println(rpm);
		
		previousCount = currentCount;*/
	}
	/*if (currentTime - switchTime > 2000000) {
		speedFL.setControl(speedFL.getSpeedGoal()*speedFL.getDirection()*-1);
		switchTime = currentTime;
	}*/
	/*delay(1000);
	Serial.println(encoderFL.getCount());*/
}


void initEncoders() {
	attachInterrupt(digitalPinToInterrupt(FRONT_LEFT_PIN_INTERRUPT),checkEncoderFL,CHANGE);
}

void checkEncoderFL() {
	encoderFL.checkEncoder();
}