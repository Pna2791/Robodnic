#include "Motor.h"
#include "BluetoothSerial.h"
BluetoothSerial SerialBT;

//  Encoder A / B, dir_pin, speed_pin
Motor rotary_left(36, 39, 18, 23);  // ID 2
Motor rotary_right(33, 32, 19, 22); // ID 3

Motor wheel_left(34, 35, 13, 15);   // ID 1
Motor wheel_right(26, 25, 5, 17);   // ID 4

int gcode2dir[8] = {0, 4, 6, 2, 7, 1, 5, 3};

// Define loop interval in milliseconds for 25Hz frequency
const long loopInterval = 40; // 1000 ms / 25 Hz = 40 ms

int rotary_resolution = 400;
int motor_resolution = 2;

void rotate(int direction) {
    int value = -direction * rotary_resolution;
    rotary_left.goto_position(value);
    rotary_right.goto_position(value);
}

void left_move(int distance) {
    static long pre_left = 0;
    int value = distance * motor_resolution;
    pre_left += value;
    wheel_left.goto_position(pre_left);
}

void right_move(int distance) {
    static long pre_right = 0;
    int value = distance * motor_resolution;
    pre_right += value;
    wheel_right.goto_position(pre_right);
}

void move(int dir, int distance) {
    if(dir == 0)    return;
    int dir_ = gcode2dir[dir-1];
    rotate(dir_);
    long t_out = millis() + 1000;
    while(millis() < t_out){
        motor_update();
    }
    left_move(distance);
    right_move(distance);
    Serial.println(dir_);
    Serial.println(distance);
    Serial.println();
}


float rotary_P = 5;
float wheel_P = 0.5;
float I = 0.1;
float D = 0.001;

void setup() {
    Serial.begin(115200);
    SerialBT.begin("ESP32_Bluetooth"); // Set the Bluetooth device name
    Serial.println("The device started, now you can pair it with Bluetooth!");

    // max_i_error, skip_error, max_speed, ofset
    rotary_left.begin(32, 5, 255, 1);
    rotary_left.setPID(rotary_P, I, D);

    rotary_right.begin(32, 5, 255, 1);
    rotary_right.setPID(rotary_P, I, D);

    wheel_left.begin(32, 5, 100, 1);
    wheel_left.setPID(wheel_P, I, D);

    wheel_right.begin(32, 5, 100, 1);
    wheel_right.setPID(wheel_P, I, D);
}

void motor_update() {
    static long next_time = millis();
    if(millis() > next_time) {
        rotary_left.computeAndSetMotorSpeed();
        rotary_right.computeAndSetMotorSpeed();
        wheel_left.computeAndSetMotorSpeed();
        wheel_right.computeAndSetMotorSpeed();

        // Print debug information
        Serial.print(wheel_left.getCounter());
        Serial.print("/");
        Serial.print(wheel_left.setpoint);
        Serial.print("\t");
        Serial.print(wheel_right.getCounter());
        Serial.print("/");
        Serial.print(wheel_right.setpoint);
        Serial.print("\t\t");
        Serial.print(rotary_left.getCounter());
        Serial.print("/");
        Serial.print(rotary_left.setpoint);
        Serial.print("\t");
        Serial.print(rotary_right.getCounter());
        Serial.print("/");
        Serial.println(rotary_right.setpoint);

        next_time += loopInterval;
    }
}

void loop() {
    // Check for serial commands
    if (Serial.available()) {
        processSerialCommand();
    }

    motor_update();
}

void processSerialCommand() {
    String command = Serial.readStringUntil('\n');
    command.trim();  // Remove any leading/trailing whitespace

    if (command.startsWith("T")) {
        long targetPosition = command.substring(1).toInt();
        // rotary_left.goto_position(targetPosition);
        // rotary_right.goto_position(targetPosition);
        // Serial.print("Setpoint updated to: ");
        // Serial.println(targetPosition);
    } else if (command.startsWith("M")) {
        int direction = command.substring(1).toInt();
        // Serial.println(direction);
        move(direction, 200);
    } else if (command == "GET_POSITION") {
        Serial.print("Motor 1 Position: ");
        Serial.println(rotary_left.getCounter());
        Serial.print("Motor 2 Position: ");
        Serial.println(rotary_right.getCounter());
    } else {
        Serial.println("Unknown command");
    }
}
