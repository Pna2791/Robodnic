#include <Motor.h>

Motor myMotor(11);

void setup() {
    // Initialize motor
}

void loop() {
    myMotor.start();
    delay(500);
    myMotor.stop();
    delay(500);
}
