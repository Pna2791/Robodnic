#include "Motor.h"

Motor::Motor(unsigned aPin, unsigned bPin, unsigned dirPin, unsigned speedPin)
    : encoderAPin(aPin), encoderBPin(bPin), dirPin_(dirPin), speedPin_(speedPin), 
      counter(0), integral(0), previousError(0), previousTime(0), setpoint(0), 
      Kp(0.5), Ki(0.1), Kd(0.001), max_i_error(32), skip_error(10), max_speed(255), offset(1) {}

void Motor::begin(int maxIError, int skipError, int maxSpeed, int offset) {
    max_i_error = maxIError;
    skip_error = skipError;
    max_speed = maxSpeed;
    this->offset = offset;

    counter = 0;
    pinMode(encoderAPin, INPUT_PULLUP);
    pinMode(encoderBPin, INPUT_PULLUP);
    pinMode(dirPin_, OUTPUT);
    pinMode(speedPin_, OUTPUT);

    attachInterruptEx(encoderAPin, [this] { ISR(); }, CHANGE);
    // attachInterruptEx(encoderBPin, [this] { ISR(); }, CHANGE);
}

long Motor::getCounter() {
    return counter;
}

bool Motor::is_small(){
    return abs(setpoint - counter) < 2*skip_error;
}

void Motor::setMotorSpeed(int val) {
    if (val == 0) {
        digitalWrite(dirPin_, LOW);
        analogWrite(speedPin_, 0);
    } else if (val < 0) {
        digitalWrite(dirPin_, LOW);
        analogWrite(speedPin_, -val);
    } else {
        digitalWrite(dirPin_, HIGH);
        analogWrite(speedPin_, val);
    }
}

void Motor::computeAndSetMotorSpeed(int soft) {
    long currentTime = millis();
    long currentPosition = getCounter();
    float output = computePID(setpoint, currentPosition, currentTime);
    if(abs(output) == max_speed){
        max_count++;
        if(max_count > soft)    max_count = soft;
        
        output *= 1./3 + 2.0/3*max_count / soft;
    }
    setMotorSpeed(output);
}

void Motor::goto_position(long sp) {
    setpoint = sp;
}

void Motor::setPID(float p, float i, float d) {
    Kp = p;
    Ki = i;
    Kd = d;
}

void Motor::ISR() {
    int MSB = digitalRead(encoderAPin); // Most significant bit
    int LSB = digitalRead(encoderBPin); // Least significant bit
    // if(LSB) counter++;
    // else    counter--;
    // return;

    if(MSB){
        if(LSB) counter++;
        else    counter--;
    }else{
        if(LSB) counter--;
        else    counter++;
    }

    // int encoded = (MSB << 1) | LSB; // Combine the two bits
    // int sum = (lastEncoded << 2) | encoded; // Sum up last and current encoding

    // if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) counter++;
    // if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) counter--;

    // lastEncoded = encoded; // Store this value for the next loop
}

// void Motor::ISR() {
//     int MSB = digitalRead(encoderAPin); // Most significant bit
//     int LSB = digitalRead(encoderBPin); // Least significant bit

//     int encoded = (MSB << 1) | LSB; // Combine the two bits
//     int sum = (lastEncoded << 2) | encoded; // Sum up last and current encoding

//     if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) counter++;
//     if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) counter--;

//     lastEncoded = encoded; // Store this value for the next loop
// }

float Motor::computePID(long setpoint, long currentPosition, long currentTime) {
    float deltaTime = (currentTime - previousTime) / 1000.0;
    previousTime = currentTime;

    float error = setpoint - currentPosition;
    if(abs(error) < skip_error){
		error_count++;
	}else{
		error_count = 0;
	}
    // Stop action
    if(error_count > 10){
        max_count = 0;
        is_stop = true;
        integral = 0;
        return 0;
    }
    is_stop = false;
	// Serial.println(error_count);

    integral += error * deltaTime;
    integral = constrain(integral, -max_i_error, max_i_error);

    float derivative = (error - previousError) / deltaTime;

    float output = Kp * error + Ki * integral + Kd * derivative;
    previousError = error;

    // float speed_output = constrain(output, -max_speed, max_speed);
    float speed_output = output;
    if (speed_output > 0) speed_output += offset;
    if (speed_output < 0) speed_output -= offset;
    return constrain(output, -max_speed, max_speed);
}
