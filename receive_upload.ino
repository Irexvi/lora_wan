/*
   Copyright (c) 2020 Boot&Work Corp., S.L. All rights reserved

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU Lesser General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <LoRa.h>
#include <SPI.h>
#include <Ethernet.h>
#include <ModbusTCPMaster.h>
#include <ModbusTCPSlave.h>
#include <LoRa.h>
uint32_t nivel = 0UL;
#include <EthernetUdp.h>
#include <ArduinoJson.h>
#include <math.h>

//Librerías para time set.
#include <TimeLib.h>
#include <Wire.h>
#include <DS1307RTC.h>  // a basic DS1307 library that returns time as a time_t



// Data constant for the setupRTCini() used only if NTP is not available

const int YEAR = 2020;
const int MONTH = 02;
const int DAY = 22;
const int HOUR = 11;
const int MINUTE = 50;
const int SECOND = 00;
char* valor;
char* location;


// Variable to save the Time for calculation
int savedHour;
int savedSecond;
int savedMinute;
int savedYear;
int savedMonth;
int savedDay;
int savedTime;
int hora_panama;
uint32_t lastSentTime = 0UL;
uint32_t lastSentTime1 = 0UL;
uint32_t lastSentTime2 = 0UL;
uint32_t lastSentTime8 = 0UL;
// libraries needed for MQTT communication
#include <PubSubClient.h>
uint32_t counter = 0UL;

IPAddress broker(172, 20, 10, 127);                      //define IP broker (Cambiar a la IP del servidor)
IPAddress namesServer(8, 8, 8, 8);                          //serverName:admin serverPassword:mxadm
IPAddress netmask(255, 255, 255, 0);
IPAddress gateway(172, 20, 10, 1);                         //define IP gateway 

// Ethernet configuration values
uint8_t mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
uint8_t ip[] = { 172, 20, 10, 301 }; // CAMBIAR A IP DE COSTA DEL ESTE.

//MQTT
char mqttUsername[] = "telegraf";                           //Nombre de usuario MQTT
char mqttPassword[] = "telegraf";                           //Contraseña MQTT
short port_MQTT = 1883;                                     //define el puerto MQTT
#define MQTT_ID ""
// Initialize client MQTT
void callback(char*topic, byte*payload, unsigned int length);
EthernetClient client;
PubSubClient mqtt(broker, 1883, callback, client);

#define LUGAR "NELSON_COLLADO"
#define SUCURSAL "CHITRE"
#define TOPICO "sensores/nelson_collado"


void callback(char*topic, byte*payload, unsigned int length)
{
  StaticJsonDocument<256> doc;

  DeserializationError error = deserializeJson(doc, payload, length);
  // Test if parsing succeeds.
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.c_str());
    return;
  }
}

void setup_MQTT()
{

  mqtt.setServer(broker, port_MQTT);
  mqtt.setCallback(callback); //Definicion de callback

}
////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(9600L);

  // Begin LoRa hardware
  if (!LoRa.begin()) {
    Serial.println("LoRa begin error: is LoRa module connected?");
    while (true);
  }

  // Default LoRa values after begin:
  // Frequency: 434.0MHz
  // Modulation GFSK_Rb250Fd250
  // TX power: +13dBm

  // Set LoRa working frequency
  if (!LoRa.setFrequency(869.0)) {
    Serial.println("LoRa set frequency error");
    while (true);
  }

  Serial.println("Send started");

  // init Ethernet connection.
  Ethernet.begin(mac, ip);
  Serial.println(Ethernet.localIP());
  
  setup_MQTT();
}
////////////////////////////////////////////////////////////////////////////////////////////////////
String digitalClockDisplay() {
  // digital clock display of the time

  /*
    Serial.print(hour());
    printDigits(minute());
    printDigits(second());
    Serial.print(" ");
    Serial.print(day());
    Serial.print(" ");
    Serial.print(month());
    Serial.print(" ");
    Serial.print(year());
    Serial.println();
  */

  return (String(hour()) + printDigits(minute()) + printDigits(second()) + " " + String(day()) + "/" + String(month()) + "/" + String(year()));
}
////////////////////////////////////////////////////////////////////////////////////////////////////
String printDigits(int digits) {
  // utility function for digital clock display: prints preceding colon and leading 0
  //Serial.print(":");
  if (digits < 10)
    //Serial.print('0');
    return ":0" + String(digits);
  //Serial.print(digits);

  return (":" + String(digits));
}
////////////////////////////////////////////////////////////////////////////////////////////////////
void reconnect() {
  if (mqtt.connect(MQTT_ID, mqttUsername, mqttPassword)) {
    mqtt.subscribe("sensores/control"); //Topico de Control
  } else {
    // MQTT connect fail
    Serial.println(digitalClockDisplay() + " - Error MQTT");
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////
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
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  // Wait packets from Send example
  if (LoRa.available()) {
    uint32_t value;
    uint8_t len = sizeof(value);

    // Save the received packet payload into value
    if (LoRa.recv((uint8_t *) &value, &len)) {
      //Serial.print("Received value: ");
      //Serial.println(value);

      char valtem[15];
      String payload;
      //Payload para los 5 registros del primer esclavo.
      dtostrf(value, 2, 2, valtem); //convierto en String tank_h
      payload = String(LUGAR)+String(",")+String("Sucursal=")+String(SUCURSAL)+String(",Sensor=NIVL");
      //Serial.println(payload);
      payload = payload + " Valor=" + valtem;
      Serial.println(payload);
      mqtt.publish(TOPICO, (char*) payload.c_str());
    }
  }

  loop_MQTT(); // MQTT
}
