//Slave code
#include <Arduino.h>,
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

//MAC to SET 
uint8_t MastereNewMACAddress[] = {0xAA, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; 

struct struct_message {
    uint8_t id; // must be unique for each sender board
    char uuid_[8];
    uint8_t rrsi_;
};
struct_message msg;

struct_message board1;
struct_message board2;

struct_message boardsStruct[2] = {board1, board2};

void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  char macStr[18];
  Serial.print("Packet received from: ");
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.println(macStr);
  memcpy(&msg, incomingData, sizeof(msg));
  Serial.printf("Board ID %u: %u bytes\n", msg.id, len);
  // Update the structures with the new incoming data
  for (size_t i = 0; i < 8; i++){
    boardsStruct[msg.id-1].uuid_[i]  = msg.uuid_[i];
  }
  boardsStruct[msg.id-1].rrsi_ = msg.rrsi_;
      Serial.print("uuid value: ");
  for (size_t i = 0; i < 8; i++){
    Serial.print(boardsStruct[msg.id-1].uuid_[i]);
  }
  Serial.println();
  Serial.printf("rssi value: %d \n", boardsStruct[msg.id-1].rrsi_);
  Serial.println();
}


void setup() {
  Serial.begin(115200);

  //Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  esp_wifi_set_mac(WIFI_IF_STA, &MastereNewMACAddress[0]);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
  return;
  }

  // Once ESPNow is successfully Init, we will register for recv CB to
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
}