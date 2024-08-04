#include <esp_now.h>
#include <WiFi.h>

#define BUTTON_PIN 22

const char *message = "HelloWorld";

// Callback function when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  // Init Serial Monitor
  Serial.begin(115200);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  Serial.println(WiFi.macAddress());

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register for a callback function that will be called when data is sent
  esp_now_register_send_cb(OnDataSent);

  // Set the MAC address of the receiver
  // uint8_t broadcastAddress[] = {0x24, 0x6F, 0x28, 0xED, 0x43, 0x4E};
  uint8_t broadcastAddress[] = {0x08,0x3A,0x8D,0x96,0x38,0x10};

  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  // Set pin mode
  pinMode(BUTTON_PIN, INPUT);
}

void loop() {
    digitalWrite(BUTTON_PIN, HIGH);
    esp_err_t result = esp_now_send(0, (uint8_t *) message, strlen(message) + 1);

    if (result == ESP_OK) {
      Serial.println("Sent with success");
    } else {
      Serial.println("Error sending the data");
    }

    digitalWrite(BUTTON_PIN, LOW);
    // Wait to avoid multiple sends
    delay(1000);
}
