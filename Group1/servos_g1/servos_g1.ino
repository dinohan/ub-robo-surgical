#include <esp_now.h>
#include <WiFi.h>
#include "src/RoboticsUB.h"
#include <ESP32Servo.h>

// Direccion MAC del master (reemplazar con el que habeis obtenido)
uint8_t masterMacAddress[] = {0x0c, 0xb8, 0x15, 0xd7, 0xe1, 0x7c};

//variables de identificacion para los servos
Servo servo_yaw;
Servo servo_pitch;
Servo servo_roll1;
Servo servo_roll2;

//variable para almacenar el torque
float torque_yaw=0;
float torque_pitch=0;
float torque_roll1=0;
float torque_roll2=0;

// variables para el angle protection
float NewValueRoll = 0;         
float OldValueRoll = 0; 
float roll=0;

float NewValuePitch=0;
float OldValuePitch=0;
float pitch=0;

float NewValueYaw=0;
float OldValueYaw=0;
float yaw=0;

int s1Status = HIGH;
int s2Status = HIGH;

//variables para almacenar el dato leido correspondiente al torque de los servos
float derivativeCurrent_yaw=0;
float derivativeCurrent_pitch=0;
float derivativeCurrent_roll1=0;
float derivativeCurrent_roll2=0;

float previousfinal_yaw=0;
float previousfinal_pitch=0;
float previousfinal_roll1=0;
float previousfinal_roll2=0;

float final_torque_yaw=0;
float final_torque_pitch=0;
float final_torque_roll1=0;
float final_torque_roll2=0;

//variables de identificacion de los pines
/*YAW*/
const int PIN_ANALOG_YAW = 36; 
const int PIN_SIGNAL_YAW = 32;
/*PITCH*/
const int PIN_ANALOG_PITCH = 39; 
const int PIN_SIGNAL_PITCH = 33;
/*ROLL*/
const int PIN_ANALOG_ROLL1 = 34; 
const int PIN_SIGNAL_ROLL1 = 25;
/*ROLL*/
const int PIN_ANALOG_ROLL2 = 35;
const int PIN_SIGNAL_ROLL2 = 27;

//constantes para las resistencias shunt (son todas iguales)
const float Rshunt = 1.6;

//estructura de datos que enviara este al master
typedef struct {
  float torque_yaw;
  float torque_pitch;
  float torque_roll1;
  float torque_roll2;
} TxMessage;

//variable del tipo estructura para enviar datos
TxMessage dataToMaster;

//estructura de datos que recibiremos del master
typedef struct {
  float roll;
  float pitch;
  float yaw;
  int s1Status;
  int s2Status;
} RxMessage;

//variable del tipo estructura para recibir datos
RxMessage dataFromMaster;

/*SETUP DEL PROGRAMA*/
void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  //comprobamos que se inicializa bien nuestro slave
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW (slave)");
    return;
  }

  // Registramos el master
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, masterMacAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // comprobamos si se ha podido añadir el master
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add master");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);

  //configuramos los PWM que alimentan los servos
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  //designamos un frecuencia a los servos
  servo_yaw.setPeriodHertz(50);
  servo_pitch.setPeriodHertz(50);
  servo_roll1.setPeriodHertz(50);
  servo_roll2.setPeriodHertz(50);

  //asignamos los pines a cada servo
  servo_yaw.attach(PIN_SIGNAL_YAW);
  servo_pitch.attach(PIN_SIGNAL_PITCH);
  servo_roll1.attach(PIN_SIGNAL_ROLL1);
  servo_roll2.attach(PIN_SIGNAL_ROLL2);

  //configuramos los pines analogicos de los servos como entradas
  pinMode(PIN_ANALOG_YAW, INPUT);
  pinMode(PIN_ANALOG_PITCH, INPUT);
  pinMode(PIN_ANALOG_ROLL1, INPUT);
  pinMode(PIN_ANALOG_ROLL2, INPUT);

  //iniciamos los servos en la posicion central (0º)
  servo_yaw.write(90);
  servo_pitch.write(90);
  servo_roll1.write(90);
  servo_roll2.write(90);
  delay(2000);
  
}

void loop() {
  //empezamos enviando los datos de la corriente de los servos
  esp_err_t result = esp_now_send(masterMacAddress, (uint8_t *) &dataToMaster, sizeof(dataToMaster));

  //comprobamos que se han enviado bien los datos
  if (result == ESP_OK) {
    //Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }

  // Read s1Status and s2Status
  s1Status=dataFromMaster.s1Status;
  s2Status=dataFromMaster.s2Status;
  //Protection angle de cada servomotor
  NewValueRoll=dataFromMaster.roll;
  if (abs(NewValueRoll-OldValueRoll)>10 && abs(NewValueRoll-OldValueRoll)<350) {
    roll=OldValueRoll;
  }else{
    roll=NewValueRoll;
  }
  OldValueRoll=roll;
  if (roll > 180 && roll < 270) {
    servo_roll1.write(180);
    servo_roll2.write(0);
    } else if (roll > 270 && roll < 360) {
    servo_roll1.write(0);
    servo_roll2.write(180);
    } else if (!s1Status && !s2Status){
    servo_roll1.write(roll);
    servo_roll2.write(180 - roll);
    //Serial.println("roll");
    } else if (s1Status == LOW && s2Status == HIGH){
    servo_roll1.write(30);
    //Serial.println("30");
    } else if (s2Status == LOW && s1Status == HIGH){
    servo_roll1.write(0);
    //Serial.println("0");
    }   
    

  NewValuePitch=dataFromMaster.pitch;
  if (abs(NewValuePitch-OldValuePitch)>10 && abs(NewValuePitch-OldValuePitch)<350) {
    pitch=OldValuePitch;
  }else{
    pitch=NewValuePitch;
  }
  OldValuePitch=pitch;

  if (s1Status == LOW && s2Status == LOW){
    if (pitch > 0 && pitch < 90) {
      servo_pitch.write(pitch + 90);
    } else if (pitch > 270 && pitch < 360) {
      servo_pitch.write(pitch - 270);
    }else{
      servo_pitch.write(pitch);
    }
  }

   
  //OldValuePitch=pitch;
  //if (pitch > 0 && pitch < 90) {
  //  servo_pitch.write(pitch + 90);
  //} else if (pitch > 270 && pitch < 360) {
  //  servo_pitch.write(pitch - 270);
  //}else{
  //  servo_pitch.write(pitch);
  //}
  
  NewValueYaw=dataFromMaster.yaw;
  if (abs(NewValueYaw-OldValueYaw)>10 && abs(NewValueYaw-OldValueYaw)<350) {
    yaw=OldValueYaw;
  }else{
    yaw=NewValueYaw;
  }
  OldValueYaw=yaw;
  if (yaw > 180 && yaw <= 270) {
    servo_yaw.write(180);
    
  } else if (yaw > 270 && yaw <= 360) {
    servo_yaw.write(0);
    
  }else{
    if (s1Status == LOW && s2Status == LOW){
      servo_yaw.write(yaw);
      }
  }
  
  /*
   * el corriente se calcula mediante la Resistencia shunt e integramos el corriente
   * para saber si hay un obstaculo o no observando la magnitud
   */
  torque_yaw = getDerivative(final_torque_yaw, PIN_ANALOG_YAW, previousfinal_yaw);
  torque_pitch = getDerivative(final_torque_pitch, PIN_ANALOG_PITCH, previousfinal_pitch);
  torque_roll1 = getDerivative(final_torque_roll1, PIN_ANALOG_ROLL1, previousfinal_roll1);
  torque_roll2 = getDerivative(final_torque_roll2, PIN_ANALOG_ROLL2, previousfinal_roll2);

  Serial.print(torque_yaw,DEC);
  Serial.print("\t");
  Serial.print(torque_pitch,DEC);
  Serial.print("\t");
  Serial.print(torque_roll1,DEC);
  Serial.print("\t");
  Serial.print(torque_roll2,DEC);
  Serial.print("\t");
  Serial.print(0);
  Serial.print("\t");
  Serial.print(10);
  Serial.print("\t");
  Serial.print(s1Status,DEC);
  Serial.print("\t");
  Serial.print(s2Status,DEC);
  Serial.println("\t");

  //Enviamos los valores del torque al Master
  dataToMaster.torque_yaw=torque_yaw;
  dataToMaster.torque_pitch=torque_pitch;
  dataToMaster.torque_roll1=torque_roll1;
  dataToMaster.torque_roll2=torque_roll2;
}

float getDerivative(float final_torque, int analog, float previous){
  float torque;
  float derivative;
  torque = getCurrent(20, analog);
  final_torque = final_torque + torque;
  derivative = abs(final_torque - previous);
  previous = final_torque;

  return derivative;
}

float getCurrent(uint32_t integrationTimeMs, int servo) {
 //leemos el ADC donde esta conectada la shunt para obtener el corriente
 //una vez tenemos la lectura, convertimos el dato del ADC para poder aplicar V/R
  uint32_t startTime = millis();
  float integratedCurrent = 0;

  // Vamos sumando la medicion de corriente durante el tiempo fijado -> integrationTimeMs (en milisegundos)
  while (millis() < startTime + integrationTimeMs) {
    uint16_t adcValue = analogRead(servo);
    integratedCurrent = integratedCurrent + ((float)adcValue / 4095.0 * 3.3) / Rshunt;
  }
  return integratedCurrent;
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  //nos llegan los datos del master y los copiamos en el slave
  memcpy(&dataFromMaster, incomingData, sizeof(dataFromMaster));
  //memcpy(destination, source, size)
}
