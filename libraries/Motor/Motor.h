#ifndef MOTOR_H
#define MOTOR_H

#include "Arduino.h"
#include "attachInterruptEx.h"

class Motor {
public:
    Motor(unsigned aPin, unsigned bPin, unsigned dirPin, unsigned speedPin);
    void begin(int maxIError, int skipError, int maxSpeed, int offset);
    long getCounter();
    void setMotorSpeed(int val);
    void computeAndSetMotorSpeed(int soft=1);
    void smooth_speed(float input_speed, int soft = 1); 
    float estimate_speed(long currentPosition, long currentTime);
    void goto_position(long sp);
    void setPID(float p, float i, float d);
    bool is_small();
    void reset();
    float Kp, Ki, Kd;
    long setpoint;
    bool is_stop = false;

private:
    void ISR();
    float computePID(long setpoint, long currentPosition, long currentTime);

    long counter;
    unsigned encoderAPin, encoderBPin, dirPin_, speedPin_;
    int lastEncoded;
    float integral, previousError;
    long previousTime;

    int max_i_error, skip_error;
    int max_speed, offset;
	int error_count = 0;
    int max_count = 0;
};

#endif // MOTOR_H
