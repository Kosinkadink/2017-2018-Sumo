#ifndef SUMOCONTROLLER_H
#define SUMOCONTROLLER_H
#include "Arduino.h"
#include "ScrapDefinitions.h"
#include "ScrapController.h"

class ScrapMotorDualPWM: public ScrapMotorInterface {
	protected:
		int PIN_PWM1;
		int PIN_PWM2;
		virtual void initMotor();
	public:
		ScrapMotorDualPWM(int PinPWM1, int PinPWM2, int dirMultip = 1);
		virtual void setMotor(int pwm);
		virtual void setDirection(int pwm);
		virtual void setDirectionMultiplier(int multi);
		virtual void setPower(int pwm);
		virtual void incrementPower(int val = 1) { setPower(currDir+val); };
		virtual void decrementPower(int val = 1) { setPower(currDir-val); };
		virtual int getDirection();
		virtual int getPower();
		virtual void stop();
};

#endif