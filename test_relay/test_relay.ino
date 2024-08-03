#include <Arduino.h>
#include "Relay.h" // Include the Relay header

// Create a Relay object with default pin values
Relay myRelay;

void setup() {
    // Initialize the relay
    myRelay.init();
    Serial.begin(115200); // Initialize serial communication at 9600 bps
}

void loop() {
    // Example usage of the set method
    myRelay.set(0, 1); // Set bit index 0 (bit 3) to 1
    Serial.println("Set bit 0 to 1");
    delay(1000);
    
    myRelay.set(1, 1); // Set bit index 1 (bit 0) to 1
    Serial.println("Set bit 1 to 1");
    delay(1000);
    
    myRelay.set(2, 1); // Set bit index 2 (bit 2) to 1
    Serial.println("Set bit 2 to 1");
    delay(1000);
    
    myRelay.set(3, 1); // Set bit index 3 (bit 7) to 1
    Serial.println("Set bit 3 to 1");
    delay(1000);
    
    // Turn off all bits one by one
    myRelay.set(0, 0); // Set bit index 0 (bit 3) to 0
    Serial.println("Set bit 0 to 0");
    delay(1000);
    
    myRelay.set(1, 0); // Set bit index 1 (bit 0) to 0
    Serial.println("Set bit 1 to 0");
    delay(1000);
    
    myRelay.set(2, 0); // Set bit index 2 (bit 2) to 0
    Serial.println("Set bit 2 to 0");
    delay(1000);
    
    myRelay.set(3, 0); // Set bit index 3 (bit 7) to 0
    Serial.println("Set bit 3 to 0");
    delay(1000);
}
