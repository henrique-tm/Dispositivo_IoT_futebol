//bibliotecas
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <ArduinoJson.h>

//defines
#define UpperThreshold 1877
#define LowerThreshold 1858
//Limites para transição do pulso
WiFiClient CLIENT;
PubSubClient MQTT(CLIENT);
TinyGPSPlus gps;
HardwareSerial SerialGPS(2);
//structs

//pinos
const int analog = 36; //pino pulso
const int sda_pin = 21, scl_pin = 22; //pinos I2C SDA e SCL
const int RXPin = 16, TXPin = 17; //pinos serial

//variaveis
const char* SSID = "G7 ThinQ_9134"; //SSID WIFI
const char* PASSWORD = "23456789"; //Senha WIFI
const char* MQTT_SERVER = "test.mosquitto.org"; //Broker Mosquitto.org
char msgmqtt[220];
bool led_state = false;
StaticJsonDocument<220> doc;
//geral

bool BPMTiming=false;
bool BeatComplete=false;
float LastTime=0;
float BPM=0;
float BPMmed=0;
float interval=0;
int cont=0;
//batimento

const int MPU_ADDR = 0x68; // definição do endereço do sensor MPU6050 (0x68)
const int WHO_AM_I = 0x75; // registro de identificação do dispositivo
const int PWR_MGMT_1 = 0x6B; // registro de configuração do gerenciamento de energia
const int ACCEL_CONFIG = 0x1C; // registro de configuração do acelerômetro
const int ACCEL_XOUT = 0x3B; // registro de leitura do eixo X do acelerômetro
int16_t AcX, AcY, AcZ;
float AcX2, AcY2, AcZ2;
//acelerometro

static const uint32_t GPSBaud = 9600;
float latitude, longitude, velocidade;
int dia, mes, ano;
String hora;
String minu;
String segu;
String csegu;
String calendario;
String traco = "-";
//GPS

//func. aux
void setupWIFI(){
  WiFi.begin(SSID, PASSWORD);
  Serial.print("Conectando na rede: ");
  Serial.println(SSID);
  while (WiFi.status() != WL_CONNECTED) {
   Serial.print(".");
   delay(500);
  }
}
void reconectar(){
  while (!MQTT.connected()) {
    Serial.println("Conectando ao Broker MQTT.");
    if (MQTT.connect("ESP32")) {
      Serial.println("Conectado com Sucesso ao Broker");
    } else {
      Serial.print("Falha ao Conectador, rc=");
      Serial.print(MQTT.state());
      Serial.println(" tentando se reconectar...");
      delay(3000);
    }
  }
}
void initI2C(){
  Wire.begin(sda_pin, scl_pin);
}

//ACELEROMETRO
//escreve no acl
void writeRegMPU(int reg, int val){ //aceita um registro e um valor como parâmetro
  Wire.beginTransmission(MPU_ADDR); // inicia comunicação com endereço do MPU6050
  Wire.write(reg); // envia o registro com o qual se deseja trabalhar
  Wire.write(val); // escreve o valor no registro
  Wire.endTransmission(true); // termina a transmissão
}
//le no acl
uint8_t readRegMPU(uint8_t reg){ // aceita um registro como parâmetro
  uint8_t data;
  Wire.beginTransmission(MPU_ADDR); // inicia comunicação com endereço do MPU6050
  Wire.write(reg); // envia o registro com o qual se deseja trabalhar
  Wire.endTransmission(false); // termina transmissão mas continua com I2C aberto (envia STOP e START)
  Wire.requestFrom(MPU_ADDR, 1); // configura para receber 1 byte do registro escolhido acima
  data = Wire.read(); // lê o byte e guarda em 'data'
  return data; //retorna 'data'
}
//acha acl
void findMPU(int mpu_addr){
  Wire.beginTransmission(MPU_ADDR);
  int data = Wire.endTransmission(true);
  if(data == 0)
  {
    Serial.print("Dispositivo encontrado no endereço: 0x");
    Serial.println(MPU_ADDR, HEX);
  }
  else
  {
    Serial.println("Dispositivo não encontrado!");
  }
}
//acl ativo?
void checkMPU(int mpu_addr){
  findMPU(MPU_ADDR);
  int data = readRegMPU(WHO_AM_I); // Register 117 – Who Am I - 0x75
  if(data == 104) 
  {
    Serial.println("MPU6050 Dispositivo respondeu OK! (104)");
    data = readRegMPU(PWR_MGMT_1); // Register 107 – Power Management 1-0x6B
    if(data == 64) Serial.println("MPU6050 em modo SLEEP! (64)");
    else Serial.println("MPU6050 em modo ACTIVE!"); 
  }
  else Serial.println("Verifique dispositivo - MPU6050 NÃO disponível!");
}
void setSleepOff(){
  writeRegMPU(PWR_MGMT_1, 0); // escreve 0 no registro de gerenciamento de energia(0x68), colocando o sensor em o modo ACTIVE}
}
void setSleepOn(){
  writeRegMPU(PWR_MGMT_1, 1); // escreve 0 no registro de gerenciamento de energia(0x68), colocando o sensor em o modo ACTIVE}
}
void setAccelScale(){
  writeRegMPU(ACCEL_CONFIG, 0);
}
void initMPU(){
  setSleepOff();
  setAccelScale();
}
void readRawMPU(){  
  Wire.beginTransmission(MPU_ADDR);       // inicia comunicação com endereço do MPU6050
  Wire.write(ACCEL_XOUT);                       // envia o registro com o qual se deseja trabalhar, começando com registro 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);            // termina transmissão mas continua com I2C aberto (envia STOP e START)
  Wire.requestFrom(MPU_ADDR, 14);         // configura para receber 14 bytes começando do registro escolhido acima (0x3B)
  AcX = Wire.read() << 8;                 // lê primeiro o byte mais significativo
  AcX |= Wire.read();                     // depois lê o bit menos significativo
  AcY = Wire.read() << 8;
  AcY |= Wire.read();
  AcZ = Wire.read() << 8;
  led_state = !led_state;
  AcX2 = AcX;
  AcY2 = AcY;
  AcZ2 = AcZ;
  digitalWrite(LED_BUILTIN, led_state);         // pisca LED do NodeMCU a cada leitura do sensor                                       
}
void startMPU(){
  initI2C();
  initMPU();
  checkMPU(MPU_ADDR);
)
//BPM
void Batimento(){
  for(cont=0;cont<10;cont++){
    int value=analogRead(analog);
    if(value>UpperThreshold){
      if(BeatComplete){
        interval=millis()-LastTime;
        BPM=float(60000/(4*interval));
        BPMTiming=false;
        BeatComplete=false;
      }
      if(BPMTiming==false){
        LastTime=millis();
        BPMTiming=true;
      }
    }
    //Registro do intervalo entre os batimentos cardíacos
    if((value<LowerThreshold)&(BPMTiming))
      BeatComplete=true;
    //atualização da variável de estado
    BPMmed=BPM+BPMmed;
    //Atualização da variavel para calculo do BPM a cada 0,1s
    delay(100);
  }
  BPMmed=BPMmed/cont;
  cont=0;
  //Calculo da média de BPM
}

//GPS
void displayInfo(){
  if (gps.location.isValid()){
    latitude = gps.location.lat();
    longitude = gps.location.lng();
  }
  else{
    Serial.println(F("INVALID L"));
  }
  if (gps.date.isValid()){
    dia = gps.date.day();
    mes = gps.date.month();
    ano = gps.date.year();
    calendario = dia + traco + mes + traco + ano;
  }
  else{
    Serial.print(F("INVALID D"));
  }
  Serial.print(F(" "));
  if (gps.speed.isValid()){
    velocidade = gps.speed.kmph();
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.println("Iniciando configuração rede");
  setupWIFI(); 
  Serial.println("configurando sensores");
  SerialGPS.begin(GPSBaud, SERIAL_8N1, TXPin, RXPin);
  delay(200);
}

void loop() {
  while (SerialGPS.available() > 0) {
    if (gps.encode(SerialGPS.read())){
      Serial.print("Valores publicados: ");
      Batimento();
      startMPU();
      readRawMPU();
      setSleepOn();
      displayInfo();
      doc["Dia"] = calendario;
      doc["Tempo"] = millis()/1000;
      doc["Longitude"] = longitude;
      doc["Latitude"] = latitude;
      doc["Velocidade M"] = velocidade;
      doc["Acel. X"] = (AcX2/1961)+0.68;
      doc["Acel. Y"] = (AcY2/1961)+0.86;
      doc["Acel. Z"] = (AcZ2/1961)+1.29;
      doc["BPM"] = BPMmed+0.3;
      serializeJson(doc, Serial);
      Serial.println(" ");
      serializeJson(doc, msgmqtt);
      MQTT.setServer(MQTT_SERVER, 1883);
      if (!MQTT.connected()) {
        reconectar();//conecta o controlador a rede em caso de queda 
      }
      MQTT.loop();
      MQTT.publish("PlayerData", msgmqtt);
    }
  }
}
