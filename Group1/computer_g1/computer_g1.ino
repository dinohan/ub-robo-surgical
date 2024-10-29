#include <esp_now.h>
#include <WiFi.h>
#include "src/RoboticsUB.h"
#include <ESP32Servo.h>

enum class Command : byte
{
  GET_RPW = 1
};

Command command = Command::GET_RPW;

// Direccion MAC del master (reemplazar con el que habeis obtenido)
uint8_t masterMacAddress[] = {0x0c, 0xb8, 0x15, 0xd7, 0xe1, 0x7c};

// Esta es la estructura de los datos que reciviremos
typedef struct {
    float g_roll;
    float g_pitch;
    float g_yaw;
    int s11Status;
    int s12Status;

    float t_roll;
    float t_pitch;
    float t_yaw;
    int s21Status;
    int s22Status;

    float torque_yaw;
    float torque_pitch;
    float torque_roll1;
    float torque_roll2;
} RxMessage;
// Creamos una varaible con la estructura recien creada
RxMessage dataFromMaster;

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  // Copiamos los datos recibidos a nuestra variable dataFromMaster
  memcpy(&dataFromMaster, incomingData, sizeof(dataFromMaster));
  
}
 void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    //Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Registramos el master
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, masterMacAddress, 6);
  peerInfo.channel = 1;  
  peerInfo.encrypt = false;
  
  // Anadimos el master     
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    //Serial.println("Failed to add peer");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);

}

void loop() {
  Serial.println(dataFromMaster.g_roll,DEC);
  Serial.println(dataFromMaster.g_pitch,DEC);
  Serial.println(dataFromMaster.g_yaw,DEC);
  Serial.println(dataFromMaster.s11Status,DEC);
  Serial.println(dataFromMaster.s12Status,DEC);
  Serial.println(dataFromMaster.t_roll,DEC);
  Serial.println(dataFromMaster.t_pitch,DEC);
  Serial.println(dataFromMaster.t_yaw,DEC);
  Serial.println(dataFromMaster.s21Status,DEC);
  Serial.println(dataFromMaster.s22Status,DEC);
  Serial.println(dataFromMaster.torque_yaw,DEC);
  Serial.println(dataFromMaster.torque_pitch,DEC);
  Serial.println(dataFromMaster.torque_roll1,DEC);
  Serial.println(dataFromMaster.torque_roll2,DEC);

  // put your main code here, to run repeatedly:
  // if (Serial.available() > 0)
  // {
  //   command = (Command)Serial.read();

  //   switch (command)
  //   {
  //   case Command::GET_RPW:
  //     Serial.println(dataFromMaster.g_roll,DEC);
  //     Serial.println(dataFromMaster.g_pitch,DEC);
  //     Serial.println(dataFromMaster.g_yaw,DEC);
  //     Serial.println(dataFromMaster.s11Status,DEC);
  //     Serial.println(dataFromMaster.s12Status,DEC);
  //     Serial.println(dataFromMaster.t_roll,DEC);
  //     Serial.println(dataFromMaster.t_pitch,DEC);
  //     Serial.println(dataFromMaster.t_yaw,DEC);
  //     Serial.println(dataFromMaster.s21Status,DEC);
  //     Serial.println(dataFromMaster.s22Status,DEC);
  //     Serial.println(dataFromMaster.torque_yaw,DEC);
  //     Serial.println(dataFromMaster.torque_pitch,DEC);
  //     Serial.println(dataFromMaster.torque_roll1,DEC);
  //     Serial.println(dataFromMaster.torque_roll2,DEC);
  //     break;
    
  //   }
  // }
  

  delay(10);
}
