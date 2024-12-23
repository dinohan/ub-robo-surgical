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
uint8_t computerMacAddress[] = {0x7c, 0x9e, 0xbd, 0x62, 0x48, 0xec};

//Direccion MAC del ESP32 servos (reemplazar con el que habeis obtenido)
uint8_t servosMacAddress[] = {0x7c, 0x9e, 0xbd, 0x66, 0x7f, 0xdc};
uint8_t masterToolMacAddress[] = {0x7c, 0x9e, 0xbd, 0x66, 0x7e, 0x60};
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
} Tx2Message;
// Creamos una varaible con la estructura recien creada
Tx2Message dataToComputer;


// Esta es la estructura de los datos que reciviremos
typedef struct {
    float torque_yaw;
    float torque_pitch;
    float torque_roll1;
    float torque_roll2;
    
    float t_roll;
    float t_pitch;
    float t_yaw;
    int s21Status;
    int s22Status;
} RxMessage;
// Creamos una varaible con la estructura recien creada
RxMessage dataFromTool;

// Funcion que se ejecutara cada vez que se haya recibido un mensaje
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  // Copiamos los datos recibidos a nuestra variable dataFromTool
  memcpy(&dataFromTool, incomingData, sizeof(dataFromTool));
  
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
  memcpy(peerInfo.peer_addr, masterToolMacAddress, 6);
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
  dataToComputer.torque_yaw=dataFromTool.torque_yaw;
  dataToComputer.torque_pitch=dataFromTool.torque_pitch;
  dataToComputer.torque_roll1=dataFromTool.torque_roll1;
  dataToComputer.torque_roll2=dataFromTool.torque_roll2;
  dataToComputer.t_roll=dataFromTool.t_roll;
  dataToComputer.t_pitch=dataFromTool.t_pitch;
  dataToComputer.t_yaw=dataFromTool.t_yaw;
  dataToComputer.s21Status=dataFromTool.s21Status;
  dataToComputer.s22Status=dataFromTool.s22Status;

  
  //Enviar los datos al computer
  dataToComputer.g_roll=roll;
  dataToComputer.g_pitch=pitch;
  dataToComputer.g_yaw=fmod(yaw + zero_yaw, 360.0);//New
  dataToComputer.s11Status=s1Status;
  dataToComputer.s12Status=s2Status;

  Serial.print(s1Status,DEC);
  Serial.print("\t");
  Serial.print(s2Status,DEC);
  Serial.println("\t");
  
  
  esp_err_t result = esp_now_send(servosMacAddress, (uint8_t *) &dataToServos, sizeof(dataToServos));
  delay(10);
  esp_err_t result_computer = esp_now_send(computerMacAddress, (uint8_t *) &dataToComputer, sizeof(dataToComputer));
  delay(10);
}
