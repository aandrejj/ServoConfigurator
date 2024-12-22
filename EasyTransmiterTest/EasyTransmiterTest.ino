/*
* Arduino Wireless Communication Tutorial
*     Example 1 - Transmitter Code
*                
* by Dejan Nedelkovski, www.HowToMechatronics.com
* 
* Library: TMRh20/RF24, https://github.com/tmrh20/RF24/
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "RxTx_dataStructures.h"

TX_DATA_STRUCTURE mydata;

bool write_succeeded = false;

RF24 radio(10, 9); // CE, CSN

//const byte address[6] = "00001";
const uint64_t my_radio_pipe = 0x0022;

void setup() {
  Serial.begin(9600);
  //Serial.begin(Baud);
  delay(200);
  
  Serial.println(" ");
  Serial.print("Sketch:   ");   Serial.println(__FILE__);
  Serial.print("Uploaded: ");   Serial.println(__DATE__);

  Serial.println("setup: radio.begin()...");
  radio.begin();
  radio.setAutoAck(true);
  radio.openWritingPipe(my_radio_pipe);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();

  ReadHwData();
  Serial.println("setup: OK");
}

void loop() {
  const char text[] = "Hello World";
  Serial.print("sending.. ");
  ReadHwData();
  Serial.println("s1min: "+String(mydata.s1min)+", s1curr: "+String(mydata.s1curr)+", s1mid: "+String(mydata.s1mid)+", s1max: "+String(mydata.s1max)+", servoSet: "+String(mydata.servoSet)+", devType:"+String(mydata.devType));
  //write_succeeded = radio.write(&mydata, sizeof(TX_DATA_STRUCTURE));
  radio.write(&mydata, sizeof(TX_DATA_STRUCTURE));
  //radio.write(&text, sizeof(text));
  /*
  if(write_succeeded) {
    Serial.println("RF_Line_WriteEvent: Write Succeeded");
  } else {
    Serial.println("RF_Line_WriteEvent: Write failed!");
  }
  */
  Serial.println(" send OK");
  delay(1000);
}




void ReadHwData() {
  if(mydata.s1min >247) {
    mydata.s1min =0;
  } else {
    mydata.s1min = mydata.s1min + 8;
  }

  if(mydata.s1mid >251) {
    mydata.s1mid =0;
  } else {
    mydata.s1mid = mydata.s1mid + 4;
  }

  if(mydata.s1max >253) {
    mydata.s1max =0;
  } else {
    mydata.s1max = mydata.s1max + 2;
  }

  if(mydata.s1curr >254) {
    mydata.s1curr =0;
  } else {
    mydata.s1curr = mydata.s1curr + 1;
  }
}

void initData() {
  mydata.servoSet = 0;

  mydata.s1min = 0;
  mydata.s1mid = 0;

  mydata.s1max = 0;

  mydata.s1curr= 0;

  mydata.devType =  2; // mode:  0 = fourSticksController (8 chanels) ,   1 = ServoConfigurator (16 chanels) , 2 = MinMaxServoConfig (min max for 2 chanels)
  //mydata.flags = 0;

  mydata.switchPos = 0;
  mydata.fireBtn1 = 0;
}
