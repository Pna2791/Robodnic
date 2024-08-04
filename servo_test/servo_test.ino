// #define dir_pin 13
// #define speed_pin 15
// #define reader_pin 34
// #define dir 35


// #define dir_pin 5
// #define speed_pin 17
// #define reader_pin 25
// #define dir 26


// #define dir_pin 33
// #define speed_pin 32
// #define reader_pin 16
// #define dir 18

// 4 BLDC
// BRUSHED LESS ID 4
// #define dir_pin 32
// #define speed_pin 33
// #define reader_pin 34
// #define dir 35

// BRUSHED LESS ID 3
// #define dir_pin 23
// #define speed_pin 18
// #define reader_pin 13
// #define dir 15


// // BRUSHED LESS ID 2
// #define dir_pin 25
// #define speed_pin 26
// #define reader_pin 39
// #define dir 36


// // BRUSHED LESS ID 1
// #define dir_pin 5
// #define speed_pin 17
// #define reader_pin 4
// #define dir 16



// Main board
// BRUSHED LESS ID 1
// #define dir_pin 13
// #define speed_pin 15
// #define reader_pin 5
// #define dir 17


// BRUSHED ID 4
// #define dir_pin 18
// #define speed_pin 23
// #define reader_pin 36
// #define dir 39

#define max_speed 100
#define ofset 1

// PID constants
float Kp = 0.1;  // Proportional gain
float Ki = 0.1;  // Integral gain
float Kd = 0.001;  // Derivative gain

long pulseCount = 0;  // To store pulse count
unsigned long lastMillis = 0;  // To store the last time we printed the count

void ICACHE_RAM_ATTR countPulse() {
    if (digitalRead(dir))
        pulseCount++;
    else
        pulseCount--;
}


// PID variables
float integral = 0;
float previousError = 0;
long previousTime = 0;

// Desired position
long setpoint = 0;  // Initial setpoint

void setMotorSpeed(int val) {
    if(val == 0){
        digitalWrite(dir_pin, 0);
        analogWrite(speed_pin, 0);
    }else if (val < 0) {
        digitalWrite(dir_pin, 0);
        analogWrite(speed_pin, -val);
    } else {
        digitalWrite(dir_pin, 1);
        analogWrite(speed_pin, val);
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(dir_pin, OUTPUT);
    pinMode(speed_pin, OUTPUT);
    pinMode(reader_pin, INPUT_PULLUP);                                              // Set reader_pin as input
    pinMode(dir, INPUT_PULLUP);                                                     // Set dir as input
    attachInterrupt(digitalPinToInterrupt(reader_pin), countPulse, RISING);  // Attach interrupt to reader_pin
}


void loop() {
    // Check for serial commands
    if (Serial.available()) {
        processSerialCommand();
    }

    // Get current time
    long currentTime = millis();

    // Get current motor position
    long currentPosition = getMotorPosition();

    // Compute PID output
    float output = computePID(setpoint, currentPosition, currentTime);

    // Set motor speed
    setMotorSpeed(output);

    // Print debug information
    Serial.print("Position: ");
    Serial.print(currentPosition);
    // Serial.print(currentPosition);
    Serial.print(" Error: ");
    // Serial.print("\t");
    Serial.print(setpoint - currentPosition);
    Serial.print(" Output: ");
    // Serial.print("\t");
    Serial.println(output);

    delay(30);  // Loop delay for stability
}

long getMotorPosition() {
    return pulseCount;
}


float computePID(long setpoint, long currentPosition, long currentTime) {
    // Calculate elapsed time
    float deltaTime = (currentTime - previousTime) / 1000.0;
    previousTime = currentTime;

    // Calculate error
    static int count_error = 0;
    float error = setpoint - currentPosition;
    if (abs(error) < 3)  count_error++;
    else                      count_error = 0;
    if(count_error > 10)  return 0;

    // Calculate integral
    integral += error * deltaTime;
    integral = constrain(integral, -10, 10);

    // Calculate derivative
    float derivative = (error - previousError) / deltaTime;

    // Calculate PID output
    float output = Kp * error + Ki * integral + Kd * derivative;
    previousError = error;
    float speed_output = constrain(output, -max_speed, max_speed);
    if(speed_output > ofset)  speed_output-ofset;
    if(speed_output < -ofset)  speed_output+ofset;
    return speed_output;

}

void processSerialCommand() {
    String command = Serial.readStringUntil('\n');
    command.trim();  // Remove any leading/trailing whitespace

    if (command.startsWith("T")) {
        // Extract the target position from the command
        long targetPosition = command.substring(1).toInt();
        setpoint = targetPosition;
        Serial.print("Setpoint updated to: ");
        Serial.println(setpoint);
    } else if (command.startsWith("P")) {
        // Extract the proportional gain from the command
        float newKp = command.substring(1).toFloat();
        Kp = newKp;
        Serial.print("Kp updated to: ");
        Serial.println(Kp);
    } else if (command.startsWith("I")) {
        // Extract the integral gain from the command
        float newKi = command.substring(1).toFloat();
        Ki = newKi;
        Serial.print("Ki updated to: ");
        Serial.println(Ki);
    } else if (command.startsWith("D")) {
        // Extract the derivative gain from the command
        float newKd = command.substring(1).toFloat();
        Kd = newKd;
        Serial.print("Kd updated to: ");
        Serial.println(Kd);
    } else if (command == "GET_POSITION") {
        long currentPosition = getMotorPosition();
        Serial.print("Current Position: ");
        Serial.println(currentPosition);
    } else {
        Serial.println("Unknown command");
    }
}
