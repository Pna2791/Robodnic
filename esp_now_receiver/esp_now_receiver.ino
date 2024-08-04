  #include <esp_now.h>
#include <WiFi.h>

#define signal_pin 22
long t_start = micros();

// Callback function when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  char message[len + 1];
  memcpy(message, incomingData, len);
  message[len] = '\0';
  long delta = micros() - t_start;
  Serial.print("Received message: ");
  Serial.println(message);
  Serial.println(delta);
}

void setup() {
  // Init Serial Monitor
  Serial.begin(115200);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  pinMode(signal_pin, INPUT);

  Serial.println(WiFi.macAddress());
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);

  // Set the MAC address of the sender
  uint8_t broadcastAddress[] = {0x24, 0x6F, 0x28, 0xED, 0x43, 0x4E};

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
}

void loop() {
  if(digitalRead(signal_pin)){
    t_start = micros();
    delay(10);
  }
  // Nothing to do here
}
