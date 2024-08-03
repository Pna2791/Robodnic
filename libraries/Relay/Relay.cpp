#include "Relay.h"

// Constructor with default pin values
Relay::Relay(uint8_t latchPin, uint8_t clockPin, uint8_t dataPin)
    : latchPin(latchPin), clockPin(clockPin), dataPin(dataPin) {}

// Initialization method
void Relay::init() {
    pinMode(latchPin, OUTPUT);
    pinMode(clockPin, OUTPUT);
    pinMode(dataPin, OUTPUT);
    digitalWrite(latchPin, LOW);
    digitalWrite(clockPin, LOW);
    digitalWrite(dataPin, LOW);
}

// Method to shift out data fast
void Relay::shiftOutFast() {
    for (int i=0; i<9; i++) {
        if (bit_status[i])  digitalWrite(dataPin, HIGH);
        else                digitalWrite(dataPin, LOW);

        digitalWrite(clockPin, HIGH);
        delayMicroseconds(time_shift); // Adjust this to achieve 1000 bps
        digitalWrite(clockPin, LOW);
        delayMicroseconds(time_shift); // Adjust this to achieve 1000 bps
    }
}

// Method to set bit values
void Relay::set(uint8_t pin, uint8_t value) {
    if (pin > 3) return; // Ensure index is within the range 0-3

    int bit_index = bit_index_array[pin];
    bit_status[bit_index] = value;

    // Serial.print("BIT: ");
    // Serial.println(bit_variable, BIN);
    digitalWrite(latchPin, LOW);
    shiftOutFast();
    digitalWrite(latchPin, HIGH);
}
