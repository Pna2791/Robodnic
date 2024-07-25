#include "Motor.h"
#include "BluetoothSerial.h"
BluetoothSerial SerialBT;

//  Encoder A / B, dir_pin, speed_pin
Motor rotary_left(4, 16, 5, 17);  // ID 1
Motor rotary_right(39, 36, 25, 26); // ID 2

Motor wheel_left(13, 15, 23, 18);   // ID 3
Motor wheel_right(34, 35, 32, 33);   // ID 4

int gcode2dir[8] = {0, 4, 6, 2, 7, 1, 5, 3};

// Define loop interval in milliseconds for 25Hz frequency
const long loopInterval = 40; // 1000 ms / 25 Hz = 40 ms

int rotary_resolution = 250;
int motor_resolution = 10;
int abs_rotation = 0;

#define math_ofset 200
void rotate(int direction) {
    abs_rotation += direction;
    rotary_left.goto_position(abs_rotation * rotary_resolution);
    rotary_right.goto_position(abs_rotation * rotary_resolution);
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

int get_rotation(int current, int target) {
    int current_new = current + math_ofset;
    target += math_ofset;
    int values[] = {0, -1, 1};

    for (int i = 0; i < 3; i++) {
        int value = values[i];
        if ((current_new + value) % 4 == target % 4)
            return value;
    }

    return abs(current + 2) < abs(current - 2) ? 2 : -2;
}

void move(int code, int distance) {
    if(code == 0)    return;
    int target = gcode2dir[code-1];
    int rotation = get_rotation(abs_rotation, target);
    int dir = 1;
    if((rotation+abs_rotation-target+math_ofset)% 8)
        distance = -distance;
    rotate(rotation);
    long t_out = millis() + 100;
    while(millis() < t_out){
        motor_update();
    }
    t_out += 900;
    while(millis() < t_out){
        motor_update();
        if(rotary_left.is_stop && rotary_right.is_stop)
            break;
    }
    left_move(distance);
    right_move(distance);
    Serial.println(target);
    Serial.println(distance);
    Serial.println();
}


float rotary_P = 0.7;
float wheel_P = 0.1;
float I = 0.2;
float D = 0.001;
int rotary_ofset = 10;

void setup() {
    Serial.begin(115200);
    SerialBT.begin("4BLDC"); // Set the Bluetooth device name
    delay(100);
    Serial.println("The device started, now you can pair it with Bluetooth!");

    // max_i_error, skip_error, max_speed, ofset
    rotary_left.begin(100, 5, 150, rotary_ofset);
    rotary_left.setPID(rotary_P, I*5, D);

    rotary_right.begin(100, 5, 150, rotary_ofset);
    rotary_right.setPID(rotary_P, I*5, D);

    wheel_left.begin(32, 10, 255, 1);
    wheel_left.setPID(wheel_P, I, D);

    wheel_right.begin(32, 10, 255, 1);
    wheel_right.setPID(wheel_P, I, D);
}

void motor_update() {
    static long next_time = millis();
    if(millis() > next_time) {
        rotary_left.computeAndSetMotorSpeed();
        rotary_right.computeAndSetMotorSpeed();
        wheel_left.computeAndSetMotorSpeed(30);
        wheel_right.computeAndSetMotorSpeed(30);

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
    Serial.println(command);
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
        move(direction, 500);
    } else if (command.startsWith("W")) {
        // Extract the proportional gain from the command
        float newKp = command.substring(1).toFloat();
        wheel_left.Kp = newKp;
        wheel_right.Kp = newKp;
        Serial.print("Kp updated to: ");
        Serial.println(newKp);
    } else if (command.startsWith("R")) {
        // Extract the proportional gain from the command
        float newKp = command.substring(1).toFloat();
        rotary_left.Kp = newKp;
        rotary_right.Kp = newKp;
        Serial.print("Kp updated to: ");
        Serial.println(newKp);
    } else if (command == "GET_POSITION") {
        Serial.print("Motor 1 Position: ");
        Serial.println(rotary_left.getCounter());
        Serial.print("Motor 2 Position: ");
        Serial.println(rotary_right.getCounter());
    } else {
        Serial.println("Unknown command");
    }
}
