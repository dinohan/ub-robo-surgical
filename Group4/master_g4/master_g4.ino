#include "src/RoboticsUB.h"
#include <esp_now.h>
#include <WiFi.h>

const int PIN_IMU_INT = 18;
const int PIN_S1 = 14;
const int PIN_S2 = 27;

float NewValueRoll = 0;         // variable para almcacenar el nuevo valor
float OldValueRoll = 0; 
float roll=0;
float valRoll=0;

float NewValuePitch=0;
float OldValuePitch=0;
float pitch=0;
float valPitch=0;

float NewValueYaw=0;
float OldValueYaw=0;
float yaw=0;
float valYaw=0;
float zero_yaw=90;


enum class Command : byte
{
  GET_RPW = 1
};

Command command = Command::GET_RPW;

IMU imu;

float *rpw;
int s1Status = HIGH;
int s2Status = HIGH;

// Direccion MAC del ESP32 computer (reemplazar con el que habeis obtenido)
uint8_t computerMacAddress[] = {0x94, 0xb9, 0x7e, 0xe5, 0x9c, 0xc8};

//Direccion MAC del ESP32 servos (reemplazar con el que habeis obtenido)
uint8_t servosMacAddress[] = {0x7c, 0x9e, 0xbd, 0x60, 0xdc, 0x54};
//Direccion MAC del Master que se comunica con los servos del UR5e
//uint8_t servosMacAddress[] = {0x7c, 0x9e, 0xbd, 0x61, 0xa1, 0x34};


// Esta es la estructura de los datos que enviaremos del master a los servomotores
typedef struct {
    float roll;
    float pitch;
    float yaw;
    int s1Status;
    int s2Status;
    
} TxMessage;
// Creamos una varaible con la estructura recien creada
TxMessage dataToServos;

// Esta es la estructura de los datos que enviaremos del master al computer
typedef struct {
    float roll;
    float pitch;
    float yaw;
    int s1Status;
    int s2Status;
    float torque_yaw;
    float torque_pitch;
    float torque_roll1;
    float torque_roll2;
} Tx2Message;
// Creamos una varaible con la estructura recien creada
Tx2Message dataToComputer;


// Esta es la estructura de los datos que reciviremos
typedef struct {
    float torque_yaw;
    float torque_pitch;
    float torque_roll1;
    float torque_roll2;
    
} RxMessage;
// Creamos una varaible con la estructura recien creada
RxMessage dataFromServos;

// Funcion que se ejecutara cada vez que se haya recibido un mensaje
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  // Copiamos los datos recibidos a nuestra variable dataFromServos
  memcpy(&dataFromServos, incomingData, sizeof(dataFromServos));
  
}

void setup()
{

  Serial.begin(115200);

  pinMode(PIN_IMU_INT, INPUT_PULLUP);
  pinMode(PIN_S1, INPUT);
  pinMode(PIN_S2, INPUT);

  imu.Install();
  // Habilitamos el WiFi en modo estacion
  WiFi.mode(WIFI_STA);

  // Inicializamos ESP-NOW
  if (esp_now_init() != ESP_OK) {
    //Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Registramos el slave de los servos
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, servosMacAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  //comprobamos que el slave de los servos se ha añadido bien   
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    //Serial.println("Failed to add peer");
    return;
  }
  // Configuramos la funcion a utilizar cada vez que se reciva un mensaje
  esp_now_register_recv_cb(OnDataRecv);

  // Registramos el slave computer
  esp_now_peer_info_t peerInfo_computer = {};
  memcpy(peerInfo_computer.peer_addr, computerMacAddress, 6);
  peerInfo_computer.channel = 1;
  peerInfo_computer.encrypt = false;

  //comprobamos que el slave del ordenador se ha añadido bien
  if (esp_now_add_peer(&peerInfo_computer) != ESP_OK) {
    //Serial.println("Failed to add computer slave");
    return;
  }
}


void loop()
{

  if (digitalRead(PIN_IMU_INT) == HIGH)
  {
    imu.ReadSensor();
    rpw = imu.GetRPW();
  }

  s1Status = digitalRead(PIN_S1);
  s2Status = digitalRead(PIN_S2);

  
  //Angle protection
  NewValueRoll=rpw[0];
  valRoll=abs(NewValueRoll-OldValueRoll);
  if (valRoll>10 && valRoll<350) {
    roll=OldValueRoll;
  } 
  else {
    roll=NewValueRoll;
  }
  OldValueRoll=roll;

  NewValuePitch=rpw[1];
  valPitch=abs(NewValuePitch-OldValuePitch);
  if (valPitch>10 && valPitch<350) {
    pitch=OldValuePitch;
  } 
  else {
    pitch=NewValuePitch;
  }
  OldValuePitch=pitch;

  NewValueYaw=rpw[2];
  valYaw=abs(NewValueYaw-OldValueYaw);
  if (valYaw>10 && valYaw<350) {
    yaw=OldValueYaw;
  } 
  else {
    yaw=NewValueYaw;
  }
  OldValueYaw=yaw;
 
  //Enviar los datos a los servomotores
  dataToServos.roll=roll;
  dataToServos.pitch=pitch;
  dataToServos.yaw=fmod(yaw + zero_yaw, 360.0);//New
  dataToServos.s1Status=s1Status;
  dataToServos.s2Status=s2Status;

  //Recibir datos de servomotor y enviarlos al computer
  dataToComputer.torque_yaw=dataFromServos.torque_yaw;
  dataToComputer.torque_pitch=dataFromServos.torque_pitch;
  dataToComputer.torque_roll1=dataFromServos.torque_roll1;
  dataToComputer.torque_roll2=dataFromServos.torque_roll2;
  
  //Enviar los datos al computer
  dataToComputer.roll=roll;
  dataToComputer.pitch=pitch;
  dataToComputer.yaw=fmod(yaw + zero_yaw, 360.0);//New
  dataToComputer.s1Status=s1Status;
  dataToComputer.s2Status=s2Status;

  Serial.print(s1Status,DEC);
  Serial.print("\t");
  Serial.print(s2Status,DEC);
  Serial.println("\t");
  
  
  esp_err_t result = esp_now_send(servosMacAddress, (uint8_t *) &dataToServos, sizeof(dataToServos));
  delay(10);
  esp_err_t result_computer = esp_now_send(computerMacAddress, (uint8_t *) &dataToComputer, sizeof(dataToComputer));
  delay(10);
}
