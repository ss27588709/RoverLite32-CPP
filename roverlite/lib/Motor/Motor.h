#ifndef MOTOR_H_
#define MOTOR_H_

#include <stdint.h>
#include <Adafruit_PWMServoDriver.h>

class Motor
{
public:
	Motor(int inv);
	~Motor();

	void initialize(Adafruit_PWMServoDriver pwm, int pwm_A, int pwm_channel_A, int pwm_channel_B);
	void setPwmDuty(float duty);

private:

	double frequency = 5000;
	uint8_t resolution_bits = 8;
	Adafruit_PWMServoDriver pwm;
	int pwm_A;

	int pwm_channel_A;
	int pwm_channel_B;

	int inverse;

};

#endif