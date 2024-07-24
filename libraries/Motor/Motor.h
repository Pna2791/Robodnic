#ifndef MOTOR_H
#define MOTOR_H

class Motor {
public:
    Motor(int pin);
    void start();
    void stop();
private:
    int _pin;
};

#endif
