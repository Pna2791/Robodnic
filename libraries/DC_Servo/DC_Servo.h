#ifndef DC_SERVO_H
#define DC_SERVO_H

#include "Arduino.h"
#include "attachInterruptEx.h"

class Motor {
public:
    Motor(unsigned aPin, unsigned bPin, unsigned dirPin, unsigned speedPin, unsigned resetPin);
    void begin(int maxIError, int skipError, int maxSpeed, int offset, int reset_value);
    long getCounter();
    void setMotorSpeed(int val);
    void computeAndSetMotorSpeed(int soft=1);
    void smooth_speed(float input_speed, int soft = 1); 
    float estimate_speed(long currentPosition, long currentTime);
    void goto_position(long sp);
    void setPID(float p, float i, float d);
    bool is_small();
    void reset();
    void calibrate(int val);
    float Kp, Ki, Kd;
    long setpoint;
    bool is_stop = false;

private:
    void ISR();
    float computePID(long setpoint, long currentPosition, long currentTime);

    long counter;
    unsigned encoderAPin, encoderBPin, dirPin_, speedPin_, resetPin_;
    int lastEncoded;
    float integral, previousError;
    long previousTime;

    int max_i_error, skip_error;
    int max_speed, offset, reset_val;
	int error_count = 0;
    int max_count = 0,;
    int reset_count = 0;
};

#endif // MOTOR_H
