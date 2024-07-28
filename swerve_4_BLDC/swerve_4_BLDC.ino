#include "Motor.h"
#include "BluetoothSerial.h"
BluetoothSerial SerialBT;

//  Encoder A / B, dir_pin, speed_pin
Motor rotary_left(4, 16, 5, 17);  // ID 1
Motor rotary_right(39, 36, 25, 26); // ID 2

Motor motor_left(13, 15, 23, 18);   // ID 3
Motor motor_right(34, 35, 32, 33);   // ID 4

int gcode2dir[8] = {0, 4, 6, 2, 7, 1, 5, 3};
bool auto_mode = true;
int manual_speed = 0;

// Define loop interval in milliseconds for 25Hz frequency
const long loopInterval = 40; // 1000 ms / 25 Hz = 40 ms

int rotary_resolution = 250;
int motor_resolution = 10;
int distance_step = 0;
int abs_rotation = 0;

#define math_ofset 200
void rotate(int direction) {
    long left_target = rotary_left.setpoint + direction * rotary_resolution;
    long right_target = rotary_right.setpoint + direction * rotary_resolution;

    rotary_left.goto_position(left_target);
    rotary_right.goto_position(right_target);
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

void move(int code, int distance_speed) {
    if(code == 0){
        if(!auto_mode){
            motor_left.setMotorSpeed(0);
            motor_right.setMotorSpeed(0);
        }
        return;
    }
    int target = gcode2dir[code-1];
    int rotation = get_rotation(abs_rotation, target);
    int dir = 1;
    if((rotation+abs_rotation-target+math_ofset)% 8)
        distance_speed = -distance_speed;
    rotate(rotation);
    long t_out = millis() + 1000;
    while(millis() < t_out){
        motor_update();
        if(rotary_left.is_stop && rotary_right.is_stop)
            break;
    }
    if(auto_mode){
        left_move(distance_speed);
        right_move(distance_speed);
    }else{
        motor_left.setMotorSpeed(distance_speed);
        motor_right.setMotorSpeed(distance_speed);
    }
    Serial.println(target);
    Serial.println(distance_speed);
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

    motor_left.begin(32, 10, 255, 1);
    motor_left.setPID(wheel_P, I, D);

    motor_right.begin(32, 10, 255, 1);
    motor_right.setPID(wheel_P, I, D);
}

void motor_update() {
    static long next_time = millis();
    if(millis() > next_time) {
        rotary_left.computeAndSetMotorSpeed();
        rotary_right.computeAndSetMotorSpeed();
        motor_left.computeAndSetMotorSpeed(30);
        motor_right.computeAndSetMotorSpeed(30);

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
void speed_set(String command){
    int value = command.substring(1).toInt();
    if(command.charAt(0) == 'S')    manual_speed= value;
    Serial.print("Set speed value: ");
    Serial.println(value);
}


void processSerialCommand(String command) {
    Serial.println(command);
    command.trim();  // Remove any leading/trailing whitespace

    if (command.startsWith("T")) {
        tuning(command.substring(1));
    } else if (command.startsWith("S")) {
        speed_set(command.substring(1));
    } else if (command.startsWith("M")) {
        int direction = command.substring(1).toInt();
        if(auto_mode){
            move(direction, distance_step);
        }else{
            move(direction, manual_speed);
        }
    } else if (command.startsWith("D")) {
        distance_step = command.substring(1).toInt();
    } else if (command.startsWith("W")) {
        // Extract the proportional gain from the command
        float newKp = command.substring(1).toFloat();
        motor_left.Kp = newKp;
        motor_right.Kp = newKp;
        Serial.print("Kp updated to: ");
        Serial.println(newKp);
    } else if (command.startsWith("A")) {
        if(command.charAt(1) == 'E'){
            motor_left.reset();
            motor_right.reset();
            auto_mode = true;
            Serial.println("Auto mode: enable");
        }
        if(command.charAt(1) == 'D'){
            auto_mode = false;
            Serial.println("Auto mode: disable");
        }
    } else if (command.startsWith("R")) {
        if(command.charAt(1) == 'L'){
            move(1, 0);
            motor_left.setMotorSpeed(-manual_speed);
            motor_right.setMotorSpeed(manual_speed);
        }
        if(command.charAt(1) == 'R'){
            move(1, 0);
            motor_left.setMotorSpeed(manual_speed);
            motor_right.setMotorSpeed(-manual_speed);
        }
        if(command.charAt(1) == '0'){
            motor_left.setMotorSpeed(0);
            motor_right.setMotorSpeed(0);
        }
    } else {
        Serial.println("Unknown command");
    }
}

