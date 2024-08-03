#ifndef RELAY_H
#define RELAY_H

#include <Arduino.h>

class Relay {
private:
    uint8_t latchPin;
    uint8_t clockPin;
    uint8_t dataPin;
    bool bit_status[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    int bit_index_array[4] = {6, 1, 0, 2};
    int time_shift = 100;

public:
    // Constructor with default pin values
    Relay(uint8_t latchPin = 19, uint8_t clockPin = 22, uint8_t dataPin = 16);

    // Initialization method
    void init();

    // Method to shift out data fast
    void shiftOutFast();

    // Method to set bit values
    void set(uint8_t index, uint8_t value);
};

#endif // RELAY_H
