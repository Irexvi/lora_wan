/*

 */

// 172.20.10.64

#include <avr/wdt.h>
#include <LoRa.h>
#include <SPI.h>
#include <Ethernet.h>
#include <ModbusTCPMaster.h>
#include <ModbusTCPSlave.h>
#include <EthernetUdp.h>
#include <ArduinoJson.h>
#include <math.h>

//Librerías para time set.
#include <TimeLib.h>
#include <Wire.h>
#include <DS1307RTC.h>  // a basic DS1307 library that returns time as a time_t
#define NOMBRE "NAME"
float lvl = 0;
float site_vol = 0;
float caudal = 0;
uint32_t lastSentTime = 0UL;
uint32_t lastSentTime1 = 0UL;
uint32_t lastSentTime2 = 0UL;
uint32_t lastSentTime8 = 0UL;
const uint32_t baudrate = 19200UL;
// libraries needed for MQTT communication
#include <PubSubClient.h>

IPAddress broker(X, X, X, X);        //define IP broker 
IPAddress namesServer(8, 8, 8, 8);   //define nameServer                       
IPAddress netmask(255, 255, 255, 0); //define Netmask gateway 
IPAddress gateway(X, X, X, X);       //define IP gateway 

// Ethernet configuration values
uint8_t mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xEF }; //define Mac Address
uint8_t ip[] = { X, X, X, X }; // define IP of controller
////////////////////////////////////////////////////////////////////////////////////////////////////
//MQTT
char mqttUsername[] = "USERNAME";                           //Name of MQTT User
char mqttPassword[] = "PASSWORD";                           //Contraseña MQTT
short port_MQTT = 1883;                                     //define MQTT port
#define MQTT_ID ""

// Initialize client MQTT
void callback(char*topic, byte*payload, unsigned int length);
EthernetClient client;
PubSubClient mqtt(broker, 1883, callback, client);

// variables and constants for NTP
unsigned int udpPort = 8888;
const char timeServer[] = "time.nist.gov";
EthernetUDP udp;
const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets

// Modbus registers mapping
// utiliza M-Duino38AR+ mapping
int digitalOutputsPins[] = {
#ifdef defined(PIN_R1_8)
  Q0_0, Q0_1, Q0_2, Q0_3, Q0_4, Q0_5, Q0_6, Q0_7, Q1_0, Q1_1, Q1_2,
  R1_1, R1_2, R1_3, R1_4, R1_5, R1_6, R1_7, R1_8
#endif
};

int digitalInputsPins[] = {
#ifdef defined(PIN_I1_5)
  I0_0, I0_1, I0_2, I0_3, I0_4, I0_5, I0_6, I0_7, I0_8, I0_9, I0_10, I0_11, I0_12,
  I1_0, I1_1, I1_2, I1_3, I1_4, I1_5
#endif
};
int analogOutputsPins[] = {
#ifdef defined(PIN_A0_7)
  A1_0, A1_1, A1_2, A0_5, A0_6, A0_7
#endif
};
int analogInputsPins[] = {
#ifdef defined(PIN_I1_5)
  I0_7, I0_8, I0_9, I0_10, I0_11, I0_12, I1_2, I1_3, I1_4, I1_5
#endif
};

#define numDigitalOutputs int(sizeof(digitalOutputsPins) / sizeof(int))
#define numDigitalInputs int(sizeof(digitalInputsPins) / sizeof(int))
#define numAnalogOutputs int(sizeof(analogOutputsPins) / sizeof(int))
#define numAnalogInputs int(sizeof(analogInputsPins) / sizeof(int))

bool digitalOutputs[numDigitalOutputs];
bool digitalInputs[numDigitalInputs];
uint16_t analogOutputs[numAnalogOutputs];
float analogInputs[numAnalogInputs + 23];

#define PI 3.1415926535897932384626433832795

float tank_volume(float r, float h, float l){
  float vol;
  float left_v;
  float right_v;
  float cos_inv;
  
  left_v = (sqrt(2*h*r-pow(h,2)))*(h-r);
  cos_inv=acos( (h-r)/r );
  right_v =(pow(r,2)*(PI-cos_inv))*l;
  vol = left_v + right_v;
  return vol;
}


void sendRequest(const char * address) {
  memset(packetBuffer, 0, NTP_PACKET_SIZE);

  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;

  Serial.println(F("Send request"));


  if (!udp.beginPacket(address, 123)) {
    Serial.println(F("Begin packet error"));
  } else {
    udp.write(packetBuffer, NTP_PACKET_SIZE);
    if (!udp.endPacket()) {
      Serial.println(F("End packet error"));
    }
  }
}

void callback(char*topic, byte*payload, unsigned int length)
{
}

void setup_MQTT()
{

  mqtt.setServer(broker, port_MQTT);
  mqtt.setCallback(callback); //Definicion de callback
}

void setup_processIMAGE() {

  // Init variables, inputs and outputs
  for (int i = 0; i < numDigitalOutputs; ++i) {
    digitalOutputs[i] = false;
    digitalWrite(digitalOutputsPins[i], digitalOutputs[i]);
  }
  for (int i = 0; i < numDigitalInputs; ++i) {
    digitalInputs[i] = digitalRead(digitalInputsPins[i]);
  }
  for (int i = 0; i < numAnalogOutputs; ++i) {
    analogOutputs[i] = 0;
    analogWrite(analogOutputsPins[i], analogOutputs[i]);
  }
  for (int i = 0; i < numAnalogInputs + 2; ++i) {
    analogInputs[i] = analogRead(analogInputsPins[i]);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void reconnect() {
  if (mqtt.connect(MQTT_ID, mqttUsername, mqttPassword)) {
    Serial.println("Se conectó mqtt.");
    mqtt.subscribe("sensores");
  } else {
    // MQTT connect fail
    Serial.println("Error MQTT");
  }
}

void loop_MQTT() {

  if (millis() - lastSentTime8 > 1000) {


    if (!mqtt.connected()) {
      reconnect();
    } else {
      mqtt.loop();
    }

    // avoid having delays in loop, we'll use the strategy from BlinkWithoutDelay
    // see: File -> Examples -> 02.Digital -> BlinkWithoutDelay for more info

    lastSentTime8 = millis();


  }
}

void loop_processIMAGE() {

  // Update inputs
  for (int i = 0; i < numDigitalInputs; ++i) {
    digitalInputs[i] = digitalRead(digitalInputsPins[i]);
  }
  for (int i = 0; i < numAnalogInputs; ++i) {
    analogInputsPins[i] = analogRead(analogInputsPins[i]);
  }

  // Process modbus requests desde Master Modbus TCP
  //Modbus_update(); //Arreglar este pedazo de código

  // Update outputs
  for (int i = 0; i < numDigitalOutputs; ++i) {
    digitalWrite(digitalOutputsPins[i], digitalOutputs[i]);
  }
  for (int i = 0; i < numAnalogOutputs; ++i) {
    analogWrite(analogOutputsPins[i], analogOutputs[i]);
  }
}

void setup() {
  Serial.begin(9600L);

  // Begin LoRa hardware
  if (!LoRa.begin()) {
    Serial.println("LoRa begin error: is LoRa module connected?");
    while (true);
  }

  // SET FREQUENCY OF COUNTRY
  if (!LoRa.setFrequency(X)) {
    Serial.println("LoRa set frequency error");
    while (true);
  }
  Serial.println("Send started");
  
  // init Ethernet connection.
  Ethernet.begin(mac, ip);
  Serial.println(Ethernet.localIP());
  //setup_RTC();
  setup_processIMAGE();
  setup_MQTT();
  wdt_enable(WDTO_4S);


  
  
  Serial.println("Inicio de programa...");
}

void receive_LoRaWAN_upload_mqtt(){
  if (LoRa.available()) {
    
    uint8_t value[46];
    uint8_t len = sizeof(value);
    
    // Save the received packet payload into value
    if (LoRa.recv((uint8_t *) &value, &len)) {

      char json[46];
      
      String str = (char*)value;

      StaticJsonDocument<256> doc;
      
      DeserializationError error = deserializeJson(doc, str);

      Serial.println(str);

  // Test if parsing succeeds.
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }


     // LoRa.isChannelActive;
      float caudal = doc["CAUD"];
      float lvl = doc["LVL"];
      
      char valtem2[15];
      String payload;
      // Envío MQTT
      dtostrf(caudal, 2, 2, valtem2); //convierto en String caudal
      payload = "Sie,Place="+String(NAME)+",Sensor=CAUD";
      payload = payload + " Valor=" + valtem2;
      mqtt.publish("sensors", (char*) payload.c_str());

      dtostrf(nivel, 2, 2, valtem2); //convierto en String nivel
      payload = "Site,Place="+String(NAME)+",Sensor=LVL";
      payload = payload + " Valor=" + valtem2;
      mqtt.publish("sensors", (char*) payload.c_str());

      
      
      wdt_reset();
      }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  receive_LoRaWAN_upload_mqtt();
  loop_processIMAGE();
  loop_MQTT(); // MQTT
}
