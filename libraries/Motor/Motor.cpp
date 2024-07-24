#include "Arduino.h"
#include "Motor.h"

Motor::Motor(int pin) {
    _pin = pin;
    pinMode(_pin, OUTPUT);
}

void Motor::start() {
    digitalWrite(_pin, HIGH);
}

void Motor::stop() {
    digitalWrite(_pin, LOW);
}
