/*  HW from Nilheim Mechatronics Servo Tester 
*   Check nilheim.co.uk for wiring and 3D printing files
*   
*   Put the TFT18 folder from the download package in Documents > Arduino > Libraries
*   Install Adafruit PWM servo driver library from the library manager (From inside arduino IDE > Tools > Manage Libraries > Search for "Adafruit PWM")
*/

//#define USE_PWM_DRIVER;
//#define USE_RF_REMOTE



#include <Wire.h>

#ifdef USE_PWM_DRIVER
  #include <Adafruit_PWMServoDriver.h>
#endif

#include <ST7735_bw.h>
#include <SPI.h>

#include "EasyTransfer.h"
#include "SoftwareSerial.h"
#include "RxTx_dataStructures.h"


#define OLED_RESET 4
#define DISP_CS    6
#define DISP_RS    7
#define DISP_RST   8
#define DISP_SID  11
#define DISP_SCLK 13

  //           ST7735(uint8_t CS, uint8_t RS, uint8_t SID, uint8_t SCLK, uint8_t RST);
  //ST7735 tft = ST7735(   DISP_CS,    DISP_RS,    DISP_SID,    DISP_SCLK,    DISP_RST); 
ST7735 tft = ST7735(         6,          7,          11,           13,           8); 
  //           ST7735(uint8_t CS, uint8_t RS, uint8_t RST);
//ST7735 tft = ST7735(6, 7, 8);    

#ifdef USE_PWM_DRIVER
  Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#endif

//Pin Definitions
#define pot0  A0
#define pot1  A1
#define pot2  A2
#define pot3  A3
#define upButtonPin  9
#define downButtonPin  5

// Color definitions
//#define BLACK           0x0000
//#define YELLOW          0xFFE0  
//#define WHITE           0xFFFF
const uint16_t BLACK = 0x0000;
const uint16_t WHITE = 0xffff;
const uint16_t BLUE = 0x001f;
const uint16_t RED = 0xf800;
const uint16_t YELLOW = 0xffe0;
const uint16_t GREEN = 0x07e0;

uint8_t spacing = 8;
uint8_t yPos = 2;
uint8_t servoNum = 0;
char servo[]="Servo ";
char colon[]=": ";
uint8_t upButtonState = 0;
uint8_t downButtonState = 0;
uint8_t activeServoSet = 0;
#ifdef USE_PWM_DRIVER
  bool pwmAvailable = false;
#endif
uint16_t servoPulse[] =         {127, 127, 127, 127, 
                                 127, 127, 127, 127, 
                                 127, 127, 127, 127, 
                                 127, 127, 127, 127};
/*
uint16_t previousServoPulse[] = {127, 127, 127, 127, 
                                 127, 127, 127, 127, 
                                 127, 127, 127, 127, 
                                 127, 127, 127, 127};
*/
//#define HIGHSPEED 
#define SERIAL_OUTPUT_LINE_RX 2  // Bluetooth RX -> Arduino D9
#define SERIAL_OUTPUT_LINE_TX 3 // Bluetooth TX -> Arduino D10

#ifdef HIGHSPEED
  #define Baud 38400   // Serial monitor
  #define BTBaud 38400 // There is only one speed for configuring HC-05, and that is 38400.
#else
  #define Baud 9600    // Serial monitor
  #define BTBaud 4800  // HM-10, HM-19 etc
#endif

unsigned long previousMillis_BTN_Select = 0;
const long interval_BTN_Select = 150;

unsigned long previousMillis_SerialLine = 0;
const long interval_SerialLine = 150;

unsigned long previousMillis_writeToDisplay = 0;
const long interval_writeToDisplay = 350;

SoftwareSerial serialOutputLine(SERIAL_OUTPUT_LINE_TX, SERIAL_OUTPUT_LINE_RX);
//create object
EasyTransfer serialLine; // send serial
//EasyTransfer ET1;   // send serial
//EasyTransfer ET2;   // rec serial

TX_DATA_STRUCTURE mydata_send;
RX_DATA_STRUCTURE mydata_remote;


void setup() {
  
  //Serial.begin(9600);
  Serial.begin(Baud);
  delay(200);
  
  Serial.println(" ");
  Serial.print("Sketch:   ");   Serial.println(__FILE__);
  Serial.print("Uploaded: ");   Serial.println(__DATE__);

  Serial.println("setup: tft.initR()...");
  tft.initR();
  //tft.initR(INITR_BLACKTAB); 

  //tft.pushColor(uint16_t color)
  //tft.pushColor(tft.Color565(r,g,b));
  //tft.fillScreen(BLACK);
  Serial.println("setup: done");

  #ifdef USE_PWM_DRIVER
    Serial.println("setup: pwd.begin()");
    pwmAvailable = pwm.begin();
    if(pwmAvailable) {
      Serial.println("setup: OK PCA9685 PWM connected");
    } else {
      Serial.println("setup: Looks like PCA9685 PWM driver is NOT connected!!!");
    }

    if(pwmAvailable) {
      pwm.setPWMFreq(60); 
    }
#endif

  serialOutputLine.begin(BTBaud);

    serialLine.begin(details(mydata_send), &serialOutputLine);
    //ET1.begin(details(mydata_send), &serialOutputLine);
    //ET2.begin(details(mydata_remote), &serialOutputLine);

  pinMode(pot0, INPUT);
  pinMode(pot1, INPUT);
  pinMode(pot2, INPUT);
  pinMode(pot3, INPUT);
  
  pinMode(  upButtonPin, INPUT_PULLUP);
  pinMode(downButtonPin, INPUT_PULLUP);

//Set background colour
  Serial.println("setup: tft.fillScreen(BLACK)");
  tft.fillScreen(BLACK);
  Serial.println("setup: BLACK =done");
  Serial.println("setup: Write servo numbers 1.for {for{}} start");
//Write servo numbers 
  for (uint8_t count = 0; count <= 3; count ++){ 
    for (uint8_t i = 0; i <=3; i ++){
      char numRead[2];
      char combined[32]= {0};
      dtostrf(servoNum, 1, 0, numRead);
      strcat(combined, servo);
      strcat(combined, numRead);
      tft.drawString(0, yPos, combined, WHITE);
      //Serial.println("setup: y:"+String(yPos)+", combined:"+String(combined)+", colon:"+String(colon)+"count:"+String(count)+", i:"+String(i)+".");
      tft.drawString(48, yPos, colon, WHITE);    
      servoNum ++;
      yPos += spacing;    
      }
      yPos += 8;
    }
Serial.println("setup: 1.for {for{}} done");

Serial.println("setup: Write initial servo positions (350 to start with)  2.for {for{}} started");
//Write initial servo positions (350 to start with)  
  servoNum = 0;
  yPos = 2;
  for (uint8_t count = 0; count <= 3; count ++){ 
    for (uint8_t i = 0; i <=3; i ++){
      char numRead[3];
      dtostrf(servoPulse[servoNum], 3, 0, numRead);
      tft.drawString(60, yPos, numRead, YELLOW);
      Serial.println("setup: y:"+String(yPos)+", numRead:"+String(numRead)+", count:"+String(count)+", i:"+String(i)+".");
      servoNum ++;
      yPos += spacing;    
      }
    yPos += 8;
  }
  Serial.println("setup: 2.for {for{}} done");

   tft.drawString(95, 3, "<", WHITE, 4);

   Serial.println("setup:done. setup END.");
}

//-------------------------loop------------------------------------------------
//-------------------------loop------------------------------------------------
//-------------------------loop------------------------------------------------
//-------------------------loop------------------------------------------------
void loop() {
  
  unsigned long currentMillis = millis();
  //Run function to see if buttons have been pressed, and pick a servo set accordingly
  loop_servoSet_BTN_Select(currentMillis);

  //Record the positions of all servos mapped to a pulsewidth of between 0 and 255
  servoPulse[(activeServoSet*4)+0] = map(analogRead(pot0), 0, 1023, 0, 255);
  servoPulse[(activeServoSet*4)+1] = map(analogRead(pot2), 0, 1023, 0, 255);
  servoPulse[(activeServoSet*4)+2] = map(analogRead(pot1), 0, 1023, 0, 255);
  servoPulse[(activeServoSet*4)+3] = map(analogRead(pot3), 0, 1023, 0, 255);

  //Clear the previous number, and write the new pulsewidths for the active servo set to the monitor
  loop_writePulsesToDisplay(currentMillis);
  /*
  previousServoPulse[(activeServoSet*4)+0] = servoPulse[(activeServoSet*4)+0];
  previousServoPulse[(activeServoSet*4)+1] = servoPulse[(activeServoSet*4)+1];
  previousServoPulse[(activeServoSet*4)+2] = servoPulse[(activeServoSet*4)+2];
  previousServoPulse[(activeServoSet*4)+3] = servoPulse[(activeServoSet*4)+3];
  */
 loop_WriteToSerialLine(currentMillis);

#ifdef USE_PWM_DRIVER
  if(pwmAvailable) {
    //Using the servo driver board, set the active servos to the position  specified by the potentiometers
    pwm.setPWM((activeServoSet*4)+0, 0, servoPulse[(activeServoSet*4)+0]);
    pwm.setPWM((activeServoSet*4)+1, 0, servoPulse[(activeServoSet*4)+1]);
    pwm.setPWM((activeServoSet*4)+2, 0, servoPulse[(activeServoSet*4)+2]);
    pwm.setPWM((activeServoSet*4)+3, 0, servoPulse[(activeServoSet*4)+3]);
  }
#endif  
  //delay(150);
}

//-----loop_WriteToSerialLine--------------------------------------
void loop_WriteToSerialLine(unsigned long currentMillis) {
  if (currentMillis - previousMillis_SerialLine >= interval_SerialLine) {  // start timed event for read and send
    previousMillis_SerialLine = currentMillis;
    //ToDoHere;
    ReadHwData();
    SerialLine_WriteEvent(currentMillis);
  } // end of timed event send
}

void ReadHwData() {
  mydata_send.s00 = servoPulse[ 0];
  mydata_send.s01 = servoPulse[ 1];
  mydata_send.s02 = servoPulse[ 2];
  mydata_send.s03 = servoPulse[ 3];
  mydata_send.s04 = servoPulse[ 4];
  mydata_send.s05 = servoPulse[ 5];
  mydata_send.s06 = servoPulse[ 6];
  mydata_send.s07 = servoPulse[ 7];
  mydata_send.s08 = servoPulse[ 8];
  mydata_send.s09 = servoPulse[ 9];
  mydata_send.s10 = servoPulse[10];
  mydata_send.s11 = servoPulse[11];
  mydata_send.s12 = servoPulse[12];
  mydata_send.s13 = servoPulse[13];
  mydata_send.s14 = servoPulse[14];
  mydata_send.s15 = servoPulse[15];
}
//------------------BtWriteEvent-------------------------------------

void SerialLine_WriteEvent(unsigned long currentMillis) {
    //bool dataSent = false; 
    
    //if(bluetooth_On){
      //dataSent = true;
      serialLine.sendData();
    //}
}

void loop_writePulsesToDisplay (unsigned long currentMillis){
  if (currentMillis - previousMillis_writeToDisplay >= interval_writeToDisplay) {  // start timed event for read and send
    previousMillis_writeToDisplay = currentMillis;

    servoNum = 0;
    yPos = 2 + (activeServoSet*40);
  
    //Serial.println("loop_writePulsesToDisplay: for{...} start");
    //if(previousServoPulse[(activeServoSet*4)+servoNum] != servoPulse[(activeServoSet*4)+servoNum]) {
      for (uint8_t i = 0; i <=3; i ++){
        char inChar[3];
        dtostrf(servoPulse[(activeServoSet*4)+servoNum], 3, 0, inChar);
        tft.fillRect(58, yPos, 30, 8, BLACK);
        tft.drawString(60, yPos, inChar, YELLOW);
        //Serial.print(" loop_writePulsesToDisplay: yPos:"+String(yPos)+" , inChar:"+String(inChar)+". ");
        servoNum ++;
        yPos += spacing;    
      }
    //}
    yPos += 8;
    //Serial.println("loop_writePulsesToDisplay: end");
  }
}

void loop_servoSet_BTN_Select(unsigned long currentMillis){
  if (currentMillis - previousMillis_BTN_Select >= interval_BTN_Select) {  // start timed event for read and send
    previousMillis_BTN_Select = currentMillis;

    upButtonState = digitalRead(upButtonPin);
    downButtonState = digitalRead(downButtonPin);

    if (upButtonState == LOW){
       activeServoSet ++;
       if (activeServoSet >3){
        activeServoSet = 0;
       }
        tft.fillRect(95, 0, 30, 160, BLACK);
        tft.drawString(95, ((activeServoSet*40)+3), "<", WHITE, 4);
        delay(150);
    }
    if (downButtonState == LOW){
      activeServoSet --;
      if (activeServoSet >3){
        activeServoSet = 3;
      }
        tft.fillRect(95, 0, 30, 160, BLACK);
        tft.drawString(95, ((activeServoSet*40)+3), "<", WHITE, 4);
        delay(150);
    }
    //ToDoHere;
  }
}
