#include "Motor.h"
#include <Arduino.h>
#include <esp32-hal-ledc.h>
#include <Adafruit_PWMServoDriver.h>

void Motor::initialize(Adafruit_PWMServoDriver pwm, int pwm_A, int pwm_channel_A, int pwm_channel_B)
{
	this->pwm_channel_A = pwm_channel_A;
	this->pwm_channel_B = pwm_channel_B;
	this->pwm_A = pwm_A;
	this->pwm = pwm;

	// ledcSetup(pwm_channel_A, frequency, resolution_bits);
	// ledcAttachPin(pin_A, pwm_channel_A);
	// ledcSetup(pwm_channel_B, frequency, resolution_bits);
	// ledcAttachPin(pin_B, pwm_channel_B);
}

void Motor::setPwmDuty(float duty)
{
	duty = constrain(inverse * duty, -1, 1);
	pwm.setPWM(this->pwm_A, 0, 0.5 * abs(4000.f * duty));
	if (this->pwm_A == 11)
	{
		if (duty > 0)
		{
			digitalWrite(this->pwm_channel_A, HIGH);
			digitalWrite(this->pwm_channel_B, LOW);
		}
		else
		{
			digitalWrite(this->pwm_channel_B, HIGH);
			digitalWrite(this->pwm_channel_A, LOW);
		}
		return;
	}

	if (duty > 0)
	{
		pwm.setPWM(this->pwm_channel_A, 0, (int)(4000.f * duty));
		pwm.setPWM(this->pwm_channel_B, 0, 0);
	}
	else
	{
		pwm.setPWM(this->pwm_channel_B, 0, (int)(4000.f * -duty));
		pwm.setPWM(this->pwm_channel_A, 0, 0);
	}
}

Motor::Motor(int inv) : inverse(inv)
{
}

Motor::~Motor()
{
}