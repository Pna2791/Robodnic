#include <Motor.h>

Motor myMotor(9);

void setup() {
    // Initialize motor
}

void loop() {
    myMotor.start();
    delay(1000);
    myMotor.stop();
    delay(1000);
}
