#include <Arduino.h>,
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

#define slavesq 3 //cantidad de esclavos
#define Max_uuid 3 //cantidad de uuid checando
#define MaxQueue 5 //Maxima cola de valores de distancia

//Coordenadas de los 3 AP en metros 
#define x0 0
#define x1 1.5
#define x2 3

#define y0 0
#define y1 2
#define y2 0

//Si se encuetra dentro de este rango cuadrado darle entrada al dispositivo
#define x_max 2
#define x_min -2

#define y_max 2
#define y_min -2

//Coordenadas guardadas aqui
double x;
double y;

//Ambiental factor
float n[slavesq] = {4,4,4};
//RSSI AT 1M
int8_t MEASURE_RSSI[slavesq] = {-60,-60,-60};

//Variables to save microseconds 
uint64_t TimePassedSlaveReply[Max_uuid][slavesq]={{0,0,0},{0,0,0},{0,0,0}};
uint64_t CurrentTime = 0;

//MAC to SET 
uint8_t MastereNewMACAddress[] = {0xAA, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; 

//struct to save msg incoming data
struct struct_message {
    uint8_t id; //identificador del esclavo
    uint8_t rssi; //potencia de transmision
    char uuid_[9];
};
struct_message msg; //
struct_message aux[Max_uuid]; //to save rssi, if it's close enough, it will save your uuid

double dist[Max_uuid][slavesq][MaxQueue]=  {0}; // queue to save diferents distance values from the same slave and identifier

double distAverage[Max_uuid][slavesq]= {{0,0,0},{0,0,0},{0,0,0}}; //to save dist avergae from identifier of each slave

uint8_t cont[Max_uuid][slavesq] = {{0,0,0},{0,0,0},{0,0,0}}; // to save the number of elements in queue of each slave

bool flag_cont[Max_uuid][slavesq] = {{0,0,0},{0,0,0},{0,0,0}}; //flag true if queue is full

bool flag_aux = false;

//function to calculate distance average 
float average(size_t aux, size_t slave_id){
    float suma = 0;
    for (int x = 0; x < MaxQueue; x++){
        suma = suma + dist[aux][slave_id][x];
    }
    return suma / MaxQueue;
}

void CalculatePosition(size_t aux_){
  //to probe desmos.com trilateration
  double a = (-2*x0)+(2*x1);
  double b = (-2*y0)+(2*y1);
  double c = (distAverage[aux_][0]*distAverage[aux_][0])-(distAverage[aux_][1]*distAverage[aux_][1])-(x0*x0)+(x1*x1)-(y0*y0)+(y1*y1);
  double d = (-2*x1)+(2*x2);
  double e = (-2*y1)+(2*y2);
  double f = (distAverage[aux_][1]*distAverage[aux_][1])-(distAverage[aux_][2]*distAverage[aux_][2])-(x1*x1)+(x2*x2)-(y1*y1)+(y2*y2);
  x=((c*e)-(f*b))/((e*a)-(b*d));
  y=((c*d)-(a*f))/((b*d)-(a*e));
  if(x>x_min && x<x_max && y>y_min && y<y_max){
    Serial.print("Acceso a aux: ");
    Serial.println(aux_);
    Serial.print("X: ");
    Serial.print(x);
    Serial.print("   ");
    Serial.print("Y: ");
    Serial.println(y);

    aux[aux_].uuid_[0] = 0;
    for (size_t i = 0; i < slavesq; i++){
      TimePassedSlaveReply[aux_][i] = 18000000000000000000;
      flag_cont[aux_][i] = 0;
      cont[aux_][i] = 0;
      distAverage[aux_][i] = 0;
      for (size_t x = 0; x < MaxQueue; x++){
        dist[aux_][i][x] = 0;
      }
    }
  }
}

void CheckTimePassedSlaveReply(size_t aux_){
  for (size_t z = 0; z < Max_uuid; z++){
    CurrentTime = esp_timer_get_time();
    //Si ya pasaron 3 segundos despues de la ultima respuesta de un esclavo, vaciar el auxiliar, cola de rssi y poner el flag en 0
    if(CurrentTime > TimePassedSlaveReply[aux_][z]+3000000){
      for (size_t i = 0; i < slavesq; i++){
        TimePassedSlaveReply[aux_][i] = 18000000000000000000;
        flag_cont[aux_][i] = 0;
        cont[aux_][i] = 0;
        distAverage[aux_][i] = 0;
        for (size_t x = 0; x < MaxQueue; x++){
          dist[aux_][i][x] = 0;
        }
      }
    } 
  }
}

void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  flag_aux = false;
  memcpy(&msg, incomingData, sizeof(msg));
  //Comprueba si el uuid habia sido  ya guardado 
  for (size_t x = 0; x < Max_uuid; x++){
    if(strcmp(aux[x].uuid_, msg.uuid_) == 0){ 
      //el uuid ya esta guardado en la variable auxiliar x
      //Si el contador se lleno, empezar desde 0 e ir llenado de forma ascendente, comportamiento tipo cola
      if(cont[x][msg.id]==MaxQueue){
        cont[x][msg.id]=0;
        //flag para saber si ya esta llena la cola
        flag_cont[x][msg.id] = true;
      }
      //Convertir el rssi a distancia y guardarlo
      dist[x][msg.id][cont[x][msg.id]] = pow(10, ((double) (MEASURE_RSSI[msg.id] + msg.rssi)) / (10 * n[msg.id]));
      //Obtener el tiempo de en el que el esclavo mando el msj
      TimePassedSlaveReply[x][msg.id] = esp_timer_get_time();
      //Serial.print("Board ID: ");
      //Serial.print(msg.id);
      // Serial.print("  DIST: ");
      // Serial.println(dist[x][msg.id][cont[x][msg.id]]);
      cont[x][msg.id]++;
      flag_aux = true;
    }
  }
  //No esta guardado en ninguna variable auxiliar, guardar en nueva variable auxiliar
  for (size_t i = 0; i < Max_uuid; i++){
    if(!flag_aux){
      //buscar auxiliar vacio
      if(aux[i].uuid_[0]==0){
        flag_aux = true;
        //auxiliar vacio
        //Guardar en una uuid en este 
        strncpy(aux[i].uuid_, msg.uuid_, sizeof (msg.uuid_));
        TimePassedSlaveReply[i][msg.id] = esp_timer_get_time();
      }
    }
  }
}



void setup() {
  Serial.begin(115200);

  //Set device as a Wi-Fi Station for receive msg from slaves
  WiFi.mode(WIFI_STA);
  esp_wifi_set_mac(WIFI_IF_STA, &MastereNewMACAddress[0]);

  // Init ESP-NOW (Msg service)
  if (esp_now_init() != ESP_OK) {
    //Serial.println("Error initializing ESP-NOW");
  return;
  }
  // Once ESPNow is successfully Init, we will register for recv Call back function
  esp_now_register_recv_cb(OnDataRecv);

  //init slave time
  for (size_t x = 0; x < Max_uuid; x++){
    for(size_t i = 0; i < slavesq; i++){
    TimePassedSlaveReply[x][i] = 18000000000000000000;
    }
  }
}

void loop() {
  for (size_t x = 0; x < Max_uuid; x++){ 
    CheckTimePassedSlaveReply(x);   
  }
  for (size_t i = 0; i < Max_uuid; i++){
    if(flag_cont[i][0]&&flag_cont[i][1]&&flag_cont[i][2]){
      // Serial.println("Average distance of slaves ");
      // Serial.print("uuid aux: ");
      // Serial.println(i);
      for (size_t x = 0; x < slavesq; x++){
        distAverage[i][x] = average(i,x);
        // Serial.print(x);
        // Serial.print(" : ");
        // Serial.println(distAverage[i][x]);   
      }
      CalculatePosition(i);
    } 
  }
}