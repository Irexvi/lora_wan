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
// libraries needed for MQTT communication
#include <PubSubClient.h>
uint32_t counter = 0UL;

IPAddress broker(199, 199, 199, 171);                      //define IP broker (Cambiar a la IP del servidor) Servidor: 192.168.18.10
IPAddress namesServer(8, 8, 8, 8);                          //serverName:admin serverPassword:mxadm
IPAddress netmask(255, 255, 255, 0);
IPAddress gateway(199, 199, 6, 1);                         //define IP gateway 

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
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  // Wait packets from Send example
  if (LoRa.available()) {
    uint32_t value;
    uint8_t len = sizeof(value);

    // Save the received packet payload into value
    if (LoRa.recv((uint8_t *) &value, &len)) {
      Serial.print("Received value: ");
      Serial.println(value);
    }
  }
}
