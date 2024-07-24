#include <Motor.h>

Motor myMotor(10);

void setup() {
    // Initialize motor
}

void loop() {
    myMotor.start();
    delay(2000);
    myMotor.stop();
    delay(2000);
}
