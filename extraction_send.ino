/*
   Copyright (c) 2018 Boot&Work Corp., S.L. All rights reserved

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

// libraries needed for MQTT communication
#include <PubSubClient.h>

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



float niv_min_oper = 0.37;
float niv_min_stop = 0.12;


uint32_t lastSentTime = 0UL;
uint32_t lastSentTime1 = 0UL;
uint32_t lastSentTime2 = 0UL;
uint32_t lastSentTime8 = 0UL;
const uint32_t baudrate = 19200UL;
const float tank_q = 6.0;  // cantidad de tanques
const float tank_r = 119.0;  // radio de tanque
const float tank_w = 47.0;  // radio de tanque
const float tank_l = 235.0;  // radio de tanque

const float tank_max_vol = 2.0;  // en m3
const float max_lvl = 1.70;  // max columna de agua en cm
const float min_lvl = 0.12;  // separacion de la boya del fondo en cm
const float gal_multiply = 264.172;  // para convertir m3 --> gal

int count = 0;
#define NOMBRE "NC"

//// SETEO DE ESCLAVOS Y MAESTROS
// Ethernet configuration values

/*
  Bahía Motors

  Router 4G: 192.168.18.1

  Servidor:  192.168.18.10
  Puerto Grafana: 3000
*/                     
// Ethernet configuration values
uint8_t mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
uint8_t ip[] = { 172, 20, 10, 78 };

uint8_t slaveIp[] = { 172, 20, 10, 68 }; // Sensor de nivel
uint16_t slavePort = 502; //502


// Define the ModbusTCPMaster object
ModbusTCPMaster master;

// Ethernet client object used to connect to the slave
EthernetClient slave;

EthernetClient client;

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
uint16_t analogInputs[numAnalogInputs + 23];

/////// Combine función Piloto //////////
float combine(unsigned int a, unsigned int b) {
  unsigned int variable;
  unsigned int signo;
  unsigned int exponente;
  unsigned int faccion;
  float resultado;
  variable = b * 65536 + a;
  signo = variable >> 31;
  exponente = (variable >> 23) & 255;
  faccion = variable & 8388607;
  resultado = ((-1) * exp(signo));
  resultado = resultado * (2 * exp(exponente - 127) * (1 + (faccion / 8388608)));
  return resultado;
}

//////////////////////////////////////////////////////////////////////////////////////
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
////////////////////////////////////////////////////////////////////////////////
//////////////////////// Setup RTC /////////////////////////////////////////////
void setup_RTC()
{
  setSyncProvider(RTC.get);   // the function to get the time from the RTC

  if (timeStatus() != timeSet) {
    Serial.println("Unable to sync with the RTC");
  }
  else {
    Serial.println("RTC has set the system time. ");
  }
}

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

String printDigits(int digits) {
  // utility function for digital clock display: prints preceding colon and leading 0
  //Serial.print(":");
  if (digits < 10)
    //Serial.print('0');
    return ":0" + String(digits);
  //Serial.print(digits);

  return (":" + String(digits));
}

/*  code to process time sync messages from the serial port   */
#define TIME_HEADER  "T"   // Header tag for serial time sync message

unsigned long processSyncMessage() {
  unsigned long pctime = 0L;
  const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013

  if (Serial.find(TIME_HEADER)) {
    pctime = Serial.parseInt();
    return pctime;
    if ( pctime < DEFAULT_TIME) { // check the value is a valid time (greater than Jan 1 2013)
      pctime = 0L; // return 0 to indicate that the time is not valid
    }
  }
  return pctime;
}
////////////////////////////// Setup MQTT ///////////////////////////////////////

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
////////////////////////////////////////////////////////////////////////////////////////////////////

void loop_Modbus_TCP_M_G1() {

  
  if (!slave.connected()) {
        slave.stop();
        slave.connect(slaveIp, slavePort);
        Serial.println("Error de conexión.");
      }

      // Send a request every 1000ms if connected to slave
      if (slave.connected()) {
        if (millis() - lastSentTime > 500) {
          // Send a Read Input Registers request to the slave with address 3
          // It requests for 6 registers starting at address 0
          // IMPORTANT: all read and write functions start a Modbus transmission, but they are not
          // blocking, so you can continue the program while the Modbus functions work. To check for
          // available responses, call master.available() function often.
          int slave_id = 1;
          // !master.readInputRegisters(slave, slave_id, starting_address, quantity)
          if (!master.readHoldingRegisters(slave, slave_id, 0, 1)) {
            // Failure treatment

          }

          lastSentTime = millis();
        }

        // Check available responses often
        if (master.isWaitingResponse()) {
          ModbusResponse response = master.available();
          if (response) {
            if (response.hasError()) {
              // Response failure treatment. You can use response.getErrorCode()

              // to get the error code.
            } else {

              

              analogInputs[numAnalogInputs] = response.getRegister(0); //Nivel en cm

              nivel = analogInputs[numAnalogInputs];

              ///////////////////////////////////////////////////////////////////////////////

              //ENVÍO LoRa WAN AQUIII!!!!!
              
              // Send counter as packet payload
              LoRa.send((uint8_t*) &nivel, sizeof(nivel));
              Serial.println("Sending nivel...");
              // Wait packet to be sent
              LoRa.waitPacketSent();
              Serial.println("nivel sent!");
            }
            //comando1 = 1;
          }
        }
        //break;
      }
}
////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////


// Setup de esclavos
void slave_connect() {
  if (!slave.connected()) {
    slave.stop();
    slave.connect(slaveIp, slavePort);
  }
  Serial.println("Slave connected.");
}
////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {

  Serial.begin(9600UL);
  while (!Serial) {
    ;
  }
  Serial.println(F("Arrancando..."));
  delay(2);

  //LORA WAN
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

  //////////////////////////////////
  slave_connect();

  // init Ethernet connection.
  Ethernet.begin(mac, ip);
  Serial.println(Ethernet.localIP());

  setup_RTC();
  setup_processIMAGE();

  // NOTE: it is not necessary to start the modbus master object
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  if (Serial.available()) {
    time_t t = processSyncMessage();
    if (t != 0) {

      RTC.set(t);   // set the RTC and the system time to the received value
      setTime(t);
    }
  }

  loop_Modbus_TCP_M_G1(); // Gateway 1
  loop_processIMAGE();        
}
