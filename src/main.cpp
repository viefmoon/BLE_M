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

uint64_t TimePassedSlaveReply[3]={0,0,0};
uint64_t CurrentTime = 0;

//MAC to SET 
uint8_t MastereNewMACAddress[] = {0xAA, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; 

struct struct_message {
    uint8_t id; // must be unique for each sender board
    uint8_t rssi;
    char uuid_[9];
};
struct_message msg;
struct_message aux;

uint8_t rssi[3][5];

float prom[3];

uint8_t cont[3] = {0, 0, 0};
bool flag_cont[3] = {0, 0, 0};


float average(size_t id){
    float suma = 0;
    for (int x = 0; x < 5; x++){
        suma = suma + rssi[id][x];
    }
    return suma / 5;
}


void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  char macStr[18];
  //Serial.print("Packet received from: ");
  //snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  //Serial.println(macStr);
  memcpy(&msg, incomingData, sizeof(msg));

  if(aux.uuid_[0]==0){
    //Se agrega un uuid para empezar a comprobar este
    strncpy(aux.uuid_, msg.uuid_, sizeof (msg.uuid_));
    Serial.print("Searching for UUID: ");
    for (size_t i = 0; i < 8; i++){
      Serial.print(aux.uuid_[i]);
    }
    Serial.println("");
  }
  //si es igual el uuid guardado a el uuid detectado
  if(strcmp(aux.uuid_, msg.uuid_) == 0){
    //Si el contador se lleno, empezar desde 0 e ir llenado de forma ascendente
    if(cont[msg.id]==5){
      cont[msg.id]=0;
      flag_cont[msg.id] = 1;
    }
    //Guardar el rrsi medido en la cola rssi del esclavo detectado
    rssi[msg.id][cont[msg.id]] = msg.rssi;
    TimePassedSlaveReply[msg.id] = esp_timer_get_time();
    Serial.print("Board ID: ");
    Serial.print(msg.id);
    Serial.print("  RSSI: ");
    Serial.println(rssi[msg.id][cont[msg.id]]);
    cont[msg.id]++;
    if(flag_cont[0]&&flag_cont[1]&&flag_cont[2]){
      Serial.println("Average RSSI of slaves ");
      for (size_t i = 0; i < 3; i++){
        prom[i] = average(i);
        Serial.print(i);
        Serial.print(" : ");
        Serial.println(prom[i]);
      }
    }
  }
}

void CheckTimePassedSlaveReply(){
  for (size_t i = 0; i < 3; i++){
    CurrentTime = esp_timer_get_time();
    if(CurrentTime > TimePassedSlaveReply[i]+3000000){
      //Si ya pasaron 3 segundos despues de la ultima respuesta de un esclavo, vaciar la cola de rssi y poner el flag en 0
      for (size_t x = 0; x < 5; x++){
        rssi[i][x]= 0;
        flag_cont[i] = 0;
      }
    }
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

  CheckTimePassedSlaveReply();

}