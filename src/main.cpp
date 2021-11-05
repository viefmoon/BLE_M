#include <Arduino.h>,
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

//Coordenadas de los 3 AP
#define x1 0
#define x2 5
#define x3 2.5

#define y1 0
#define y2 0
#define y3 4

//calibration 
#define MEASURE_RSSI 60
#define n 2

uint8_t x;
uint8_t y;

//MAC to SET 
uint8_t MastereNewMACAddress[] = {0xAA, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; 

struct struct_message {
    uint8_t id; // must be unique for each sender board
    char uuid_[8];
    uint8_t rssi;
};
struct_message msg;
struct_message aux;

uint8_t rssi[3][5];

uint8_t cont[3] = {0, 0, 0};


void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  char macStr[18];
  Serial.print("Packet received from: ");
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.println(macStr);
  memcpy(&msg, incomingData, sizeof(msg));
  Serial.printf("Board ID %u: %u bytes\n", msg.id, len);

  if(aux.uuid_[0]==0){
    //Se agrega un uuid para empezar a comprobar este
    for (size_t i = 0; i <= 8; i++){
      aux.uuid_[i]  = msg.uuid_[i];
    }
    Serial.print("Searching for UUID: ");
    for (size_t i = 0; i < 8; i++){
      Serial.print(aux.uuid_[i]);
    }
    Serial.println("");
  }
      for (size_t i = 0; i < 8; i++){
      Serial.print(msg.uuid_[i]);
    }
    Serial.println("");
  if(strcmp(aux.uuid_, msg.uuid_) == 0){
    if(cont[msg.id]==5){
      cont[msg.id]=0;
    }
    rssi[msg.id][cont[msg.id]] = msg.rssi;
    Serial.print("Board ID: ");
    Serial.print(msg.id);
    Serial.print("  RSSI: ");
    Serial.println(rssi[msg.id][cont[msg.id]]);
    cont[msg.id]++;
  }
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