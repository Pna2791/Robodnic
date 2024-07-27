#include <Motor.h>
#include "BluetoothSerial.h"
BluetoothSerial SerialBT;

//  Encoder A / B, dir_pin, speed_pin
Motor rotary_left(36, 39, 18, 23);  // ID 2
Motor rotary_right(33, 32, 19, 22); // ID 3

Motor motor_left(34, 35, 13, 15);   // ID 1
Motor motor_right(26, 25, 5, 17);   // ID 4

int gcode2dir[8] = {0, 4, 6, 2, 7, 1, 5, 3};

// Define loop interval in milliseconds for 25Hz frequency
const long loopInterval = 40; // 1000 ms / 25 Hz = 40 ms

int rotary_resolution = 400;
int motor_resolution = 2;
int distance_step = 0;
int abs_rotation = 0;

#define math_ofset 200
void rotate(int direction) {
    abs_rotation += direction;
    rotary_left.setpoint += direction * rotary_resolution;
    rotary_right.setpoint += direction * rotary_resolution;
    // rotary_left.goto_position(abs_rotation * rotary_resolution);
    // rotary_right.goto_position(abs_rotation * rotary_resolution);
}

void left_move(int distance) {
    static long pre_left = 0;
    int value = distance * motor_resolution;
    pre_left += value;
    motor_left.goto_position(pre_left);
}

void right_move(int distance) {
    static long pre_right = 0;
    int value = distance * motor_resolution;
    pre_right += value;
    motor_right.goto_position(pre_right);
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



float rotary_P = 5;
float wheel_P = 0.5;
float I = 0.1;
float rotary_D = 0.15;
float wheel_D = 0.02;

void setup() {
    Serial.begin(115200);
    SerialBT.begin("4DC_Servo"); // Set the Bluetooth device name
    Serial.println("The device started, now you can pair it with Bluetooth!");

    // max_i_error, skip_error, max_speed, ofset
    rotary_left.begin(32, 3, 255, 1);
    rotary_left.setPID(rotary_P, I, rotary_D);

    rotary_right.begin(32, 3, 255, 1);
    rotary_right.setPID(rotary_P, I, rotary_D);

    motor_left.begin(32, 3, 100, 1);
    motor_left.setPID(wheel_P, I, wheel_D);

    motor_right.begin(32, 3, 100, 1);
    motor_right.setPID(wheel_P, I, wheel_D);
}


#define soft_start_value 50   //2s
void motor_update() {
    static long next_time = millis();
    if(millis() > next_time) {
        rotary_left.computeAndSetMotorSpeed();
        rotary_right.computeAndSetMotorSpeed();
        motor_left.computeAndSetMotorSpeed(soft_start_value);
        motor_right.computeAndSetMotorSpeed(soft_start_value);

        // Print debug information
        Serial.print(motor_left.getCounter());
        Serial.print("/");
        Serial.print(motor_left.setpoint);
        Serial.print("\t");
        Serial.print(motor_right.getCounter());
        Serial.print("/");
        Serial.print(motor_right.setpoint);
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


void tuning(String command) {
    if (command.length() != 3) {
        // Handle invalid command length
        Serial.println("Invalid command length");
        return;
    }

    char direction = command.charAt(0);
    char sign = command.charAt(1);
    char FS = command.charAt(2);
    int k = 20;
    if(FS == 'F')   k=2;
    int value = rotary_resolution / k;

    // Compute the value based on the sign
    if (sign == '-') {
        value = -value;
    } else if (sign != '+') {
        // Handle invalid sign
        Serial.println("Invalid sign");
        return;
    }

    // Apply the value to the appropriate rotary
    if (direction == 'L') {
        rotary_left.setpoint += value;
    } else if (direction == 'R') {
        rotary_right.setpoint += value;
    } else {
        // Handle invalid direction
        Serial.println("Invalid direction");
    }
}


void processSerialCommand(String command) {
    Serial.println(command);
    command.trim();  // Remove any leading/trailing whitespace

    if (command.startsWith("T")) {
        tuning(command.substring(1));
    } else if (command.startsWith("M")) {
        int direction = command.substring(1).toInt();
        move(direction, distance_step);
    } else if (command.startsWith("D")) {
        distance_step = command.substring(1).toInt();
    } else if (command.startsWith("W")) {
        // Extract the proportional gain from the command
        float newKp = command.substring(1).toFloat();
        motor_left.Kp = newKp;
        motor_right.Kp = newKp;
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
