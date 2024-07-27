#include "Motor.h"
#include "BluetoothSerial.h"
BluetoothSerial SerialBT;

void setup() {
    Serial.begin(115200);
    SerialBT.begin("TEST"); // Set the Bluetooth device name
    delay(100);
    Serial.println("The device started, now you can pair it with Bluetooth!");

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

}

void processSerialCommand(String command) {
    Serial.println(command);
    command.trim();
}
