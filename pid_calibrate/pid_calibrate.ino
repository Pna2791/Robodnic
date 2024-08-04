#include <Motor.h>
#include "BluetoothSerial.h"
BluetoothSerial SerialBT;

//  Encoder A / B, dir_pin, speed_pin
Motor motor_left(36, 39, 18, 23);  // ID 2
Motor motor_right(33, 32, 19, 22); // ID 3

// Motor motor_left(34, 35, 13, 15);   // ID 1
// Motor motor_right(26, 25, 5, 17);   // ID 4

int gcode2dir[8] = {0, 4, 6, 2, 7, 1, 5, 3};

// Define loop interval in milliseconds for 25Hz frequency
const long loopInterval = 40; // 1000 ms / 25 Hz = 40 ms

float rotary_P = 1;
float wheel_P = 0.5;
float I = 0.1;
float D = 0.07;
// float D = 0.02;
// RESOLUTION OF SERVO 333 IS 5276/ROUND

void setup() {
    Serial.begin(115200);
    SerialBT.begin("4DC_Servo"); // Set the Bluetooth device name
    // Serial.println("The device started, now you can pair it with Bluetooth!");

    // max_i_error, skip_error, max_speed, ofset
    motor_left.begin(32, 10, 255, 1);
    motor_left.setPID(rotary_P, I, D);

    motor_right.begin(32, 10, 255, 1);
    motor_right.setPID(rotary_P, I, D);

    // motor_left.begin(32, 3, 100, 1);
    // motor_left.setPID(wheel_P, I, D);

    // motor_right.begin(32, 3, 100, 1);
    // motor_right.setPID(wheel_P, I, D);
}


#define soft_start_value 50   //2s
void motor_update() {
    static long next_time = millis();
    if(millis() > next_time) {
        // long current = millis();
        // long left_pos = motor_left.getCounter();
        // long right_pos = motor_right.getCounter();
        // float left_speed = motor_left.estimate_speed(left_pos, current);
        // float right_speed = motor_right.estimate_speed(right_pos, current);
        
        // Serial.print(left_speed);
        // Serial.print("\t");
        // Serial.print(right_speed);
        // Serial.print("\t");

        // motor_left.smooth_speed(left_speed, soft_start_value);
        // motor_right.smooth_speed(right_speed, soft_start_value);

        // Serial.print(left_pos);
        // Serial.print("\t");
        // Serial.println(right_pos);

        motor_left.computeAndSetMotorSpeed();
        motor_right.computeAndSetMotorSpeed();
        Serial.print(motor_left.getCounter());
        Serial.print("\t");
        Serial.println(motor_right.getCounter());

        next_time += loopInterval;
    }
}

void loop() {
    // Check for serial commands
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        processSerialCommand(command);
    }

    // Check for serial commands
    if (SerialBT.available()) {
        String command = SerialBT.readStringUntil('\n');
        processSerialCommand(command);
    }

    motor_update();
}


void processSerialCommand(String command) {
    // Serial.println(command);
    command.trim();  // Remove any leading/trailing whitespace

    if (command.startsWith("T")) {
        int target = command.substring(1).toInt();
        motor_left.goto_position(target);
        motor_right.goto_position(target);
    } else if (command.startsWith("P")) {
        // Extract the proportional gain from the command
        float newKp = command.substring(1).toFloat();
        motor_left.Kp = newKp;
        motor_right.Kp = newKp;
    } else if (command.startsWith("I")) {
        // Extract the proportional gain from the command
        float newKp = command.substring(1).toFloat();
        motor_left.Ki = newKp;
        motor_right.Ki = newKp;
    } else if (command.startsWith("D")) {
        // Extract the proportional gain from the command
        float newKp = command.substring(1).toFloat();
        motor_left.Kd = newKp;
        motor_right.Kd = newKp;
    }
}
