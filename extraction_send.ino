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
//#include <avr/wdt.h>

#include <SPI.h>
#include <Ethernet.h>
#include <ModbusTCPMaster.h>
#include <ModbusTCPSlave.h>
#include <LoRa.h>

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
int gatew = 1;
int loraw = 0;
int slave_id_1 = 19;
int slave_id_2 = 20;


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
uint32_t lastSentTime3 = 0UL;
const uint32_t baudrate = 19200UL;

int count = 0;
#define NOMBRE "NAME"

//// SETEO DE ESCLAVOS Y MAESTROS
// Ethernet configuration values

// Ethernet configuration values
uint8_t mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
uint8_t ip[] = { X, X, X, X }; // 192.168.1.1

uint8_t slaveIp[] = { X, X, X, X }; // Sensor de nivel
uint16_t slavePort = X; //Port

uint8_t slave2Ip[] = { X, X, X, X }; // Sensor de caudal
uint16_t slave2Port = X; //Port

// Define the ModbusTCPMaster object
ModbusTCPMaster master;
ModbusTCPMaster master2;

// Ethernet client object used to connect to the slave
EthernetClient slave;
EthernetClient slave2;

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

float combine_real(unsigned int a, unsigned int b){
  unsigned long temp = (unsigned long) a << 16 | b;
  return *(float*)&temp;
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


void loop_LoRaWAN(){
switch (loraw){

case 0:
     lastSentTime2=millis();
     loraw=1; 
     break; 
  
case 1:
      if (millis() - lastSentTime2 > 2000) {
      //float caudal_f = (combine_real(analogInputs[numAnalogInputs+1],analogInputs[numAnalogInputs+2])); // Caudal
      float caudal_f = analogInputs[numAnalogInputs+1];
      float lvl_f = (analogInputs[numAnalogInputs]/100.0); // lvl
      
      char str_caud[6];
      char str_nvl[6];
      
      dtostrf(caudal_f, 4, 2, str_caud);
      dtostrf(nivel_f, 4, 2, str_nvl);
      
      uint8_t buffer_s[45];
      
      String payload;
      
      sprintf(buffer_s, "{\"CAUD\":%s,\"NVL\":%s}",str_caud,str_nvl);
      
      LoRa.send((uint8_t*) &buffer_s, sizeof(buffer_s));
      Serial.println("Sending payload...");
      LoRa.waitPacketSent();
      Serial.println("Package sent.");
      loraw=0;
      break;
  }
  
  } 
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void loop_Modbus_TCP_M() {
switch (gatew){
case 1:
  ////// CAUDAL INICIO ////////
  if (!slave2.connected()) {
        slave2.stop();
        slave2.connect(slave2Ip, slave2Port);
        Serial.println("Error de conexión.");
      }

      // Send a request every 1000ms if connected to slave
      if (slave2.connected()) {
        if (millis() - lastSentTime > 500) {

          //if (!master2.readHoldingRegisters(slave2, slave_id_1, 0, 4)
          if (!master2.readInputRegisters(slave2, slave_id_1, 1, 2)) {
            // Failure treatment

          }
          lastSentTime = millis();
        }
        // Check available responses often
        if (master2.isWaitingResponse()) {
          ModbusResponse response = master2.available();
          if (response) {
            if (response.hasError()) {
              // Response failure treatment. You can use response.getErrorCode()

              // to get the error code.
            } else {
              analogInputs[numAnalogInputs+1]=response.getRegister(0); // getRegister(1)
              analogInputs[numAnalogInputs+2]=response.getRegister(1); // getRegister(2)
 
              gatew=2;
              break;
            }
          }
        }
      }

  ////// CAUDAL FINAL ////////

  ////// NIVEL INICIO ///////
case 2:
  if (!slave.connected()) {
        slave.stop();
        slave.connect(slaveIp, slavePort);
        Serial.println("Error de conexión.");
      }

      // Send a request every 1000ms if connected to slave
      if (slave.connected()) {
        if (millis() - lastSentTime > 500) {

          //if (!master.readHoldingRegisters(slave, slave_id_2, 1, 1)
          if (!master.readInputRegisters(slave, slave_id_2, 1, 1)) {
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
              
              gatew=1;
              break;
              
            }
          }
        }
      }
  
  ///// NIVEL FINAL ////////
}
}
////////////////////////////////////////////////////////////////////////////////////////////////////


// Setup de esclavos
void slave_connect() {
  if (!slave.connected()) {
    slave.stop();
    slave.connect(slaveIp, slavePort);
  }
  Serial.println("Slave 1 connected.");

  if (!slave2.connected()) {
    slave2.stop();
    slave2.connect(slave2Ip, slave2Port);
  }
  Serial.println("Slave 2 connected.");
}
////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {

  Serial.begin(9600UL);
  
  //LORA WAN
  if (!LoRa.begin()) {
    Serial.println("LoRa begin error: is LoRa module connected?");
    while (true);
  }
  if (!LoRa.setFrequency(902.0)) {
    Serial.println("LoRa set frequency error");
    while (true);
  }
  //////////////////////////////////
  slave_connect();

  // init Ethernet connection.
  Ethernet.begin(mac, ip);
  Serial.println(Ethernet.localIP());

  setup_RTC();
  setup_processIMAGE();

  lastSentTime2 = millis();
  lastSentTime3 = millis();
  // NOTE: it is not necessary to start the modbus master object
  //wdt_enable(WDTO_4S);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {

//wdt_reset();

loop_Modbus_TCP_M(); //Extracción TCP/IP Caudal
loop_LoRaWAN(); //Envío LoRaWAN
loop_processIMAGE();
delay(1000);

}
