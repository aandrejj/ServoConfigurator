#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <ST7735.h>
#include <SPI.h>

#include "EasyTransfer.h"
#include "SoftwareSerial.h"
#include "TxRx_dataStructures.h"

//#define HIGHSPEED 
#define SERIAL_OUTPUT_LINE_RX 0  // Bluetooth RX -> Arduino D9
#define SERIAL_OUTPUT_LINE_TX 1 // Bluetooth TX -> Arduino D10

#ifdef HIGHSPEED
  #define Baud 38400   // Serial monitor
  #define BTBaud 38400 // There is only one speed for configuring HC-05, and that is 38400.
#else
  #define Baud 9600    // Serial monitor
  #define BTBaud 9600  // HM-10, HM-19 etc
#endif

uint8_t servoNum = 0;
char servo[]="Servo ";

int16_t mode;
int count;
int noDataCount;

long previousSafetyMillis;

unsigned long previousServoMillis=0;
const long servoInterval = 200;

unsigned long previousMillis_SerialLine = 0;
const long interval_SerialLine = 150;

SoftwareSerial serialOutputLine(SERIAL_OUTPUT_LINE_TX, SERIAL_OUTPUT_LINE_RX);
//create object
EasyTransfer serialLine; // send serial
//EasyTransfer ET1;   // send serial
//EasyTransfer ET2;   // rec serial

RX_DATA_STRUCTURE mydata_received;
TX_DATA_STRUCTURE mydata_remote;

void setup() {
  
  //Serial.begin(9600);
  Serial.begin(Baud);
  delay(200);
  
  Serial.println(" ");
  Serial.print("Sketch:   ");   Serial.println(__FILE__);
  Serial.print("Uploaded: ");   Serial.println(__DATE__);

  serialOutputLine.begin(BTBaud);

    serialLine.begin(details(mydata_received), &serialOutputLine);
    //ET1.begin(details(mydata_send), &serialOutputLine);
    //ET2.begin(details(mydata_remote), &serialOutputLine);

   Serial.println("setup:done. setup END.");
}

//-------------------------loop------------------------------------------------
//-------------------------loop------------------------------------------------
//-------------------------loop------------------------------------------------
//-------------------------loop------------------------------------------------
void loop() {
  
  unsigned long currentMillis = millis();
  loop_ReadFromSerialLine(currentMillis);
}

void loop_ReadFromSerialLine(unsigned long currentMillis) {
  if (currentMillis - previousMillis_SerialLine >= interval_SerialLine) {  // start timed event for read and send
    previousMillis_SerialLine = currentMillis;

    if(serialLine.receiveData()){ //ET2.receiveData())            // main data receive
      previousSafetyMillis = currentMillis; 
      //mydata_send.mode = mode;
      //mydata_send.count = count;
      //ToDo here
      Serial.println("loop_ReadFromSerialLine:mydata_received = 0:" + String(mydata_received.s00) +", 1:" + String(mydata_received.s01) +", 2:" + String(mydata_received.s02) +", 3:" + String(mydata_received.s03));
      count = count + 1;                                              // update count for remote monitoring
    } else if(currentMillis - previousSafetyMillis > 200) {         // safeties
      noDataCount = noDataCount + 1;                                  // update count for remote monitoring
      Serial.println("!"+String(noDataCount)+"! No Data ");
    }
  }  // end of timed event Receive/Send

  if (currentMillis - previousServoMillis >= servoInterval) {  // start timed event for Servos  (200 ms)
	previousServoMillis = currentMillis;
	//ToDo here
  }
	  
}