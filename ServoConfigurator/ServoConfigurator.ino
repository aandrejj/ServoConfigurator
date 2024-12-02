/*  HW from Nilheim Mechatronics Servo Tester 
*   Check nilheim.co.uk for wiring and 3D printing files
*   
*   Put the TFT18 folder from the download package in Documents > Arduino > Libraries
*   Install Adafruit PWM servo driver library from the library manager (From inside arduino IDE > Tools > Manage Libraries > Search for "Adafruit PWM")
*/
#include <Arduino.h>

//#define USE_PWM_DRIVER
#define USE_RF_REMOTE
//#define ANALOG_POTENTIOMENTERS_READ
//#define DIGITAL_ENCODERS_READ 
#define TREE_ENCODERS_ONE_POTENTIOMETER_IN_LINE

#include "Servo_Min_Max.h"

#if defined(DIGITAL_ENCODERS_READ) || defined (TREE_ENCODERS_ONE_POTENTIOMETER_IN_LINE)
  #include <Encoder.h>  //  Encoder Library,  https://github.com/PaulStoffregen/Encoder ,  http://www.pjrc.com/teensy/td_libs_Encoder.html
#endif

#ifdef USE_PWM_DRIVER
  #include <Adafruit_PWMServoDriver.h>
#endif

#include <ST7735.h>
#include <SPI.h>

#ifdef USE_RF_REMOTE
  #include <nRF24L01.h>
  #include <RF24.h>
#endif

#include "RxTx_dataStructures.h"


//#define OLED_RESET 4
#define DISP_CS    6 //CS   -CS
#define DISP_RS    7 //A0   -RS
#define DISP_RST   8 //RESET-RST
#define DISP_SID   4 //SDA  -SDA
#define DISP_SCLK  5 //SCK  -SCK
#define LEFT_ARROW_SIZE  2
#define LEFT_ARROW_STEP  2

  //           ST7735(uint8_t CS, uint8_t RS, uint8_t SID, uint8_t SCLK, uint8_t RST);
  ST7735 tft = ST7735(   DISP_CS,    DISP_RS,    DISP_SID,    DISP_SCLK,    DISP_RST); 
//ST7735 tft = ST7735(         6,          7,          11,           13,           8); 
  //           ST7735(uint8_t CS, uint8_t RS, uint8_t RST);
//ST7735 tft = ST7735(6, 7, 8);    

#ifdef USE_PWM_DRIVER
  Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#endif

  #ifdef ANALOG_POTENTIOMENTERS_READ
    //Pin Definitions
    #define pot0  A0
    #define pot1  A2
    #define pot2  A1
    #define pot3  A3
    #define upButtonPin  A4
    #define downButtonPin  A5
    #define minMidMAX_SwitchPin A6 //flag0, flag1
    #define fireButtonPin A7 //flag2
  #endif

  #ifdef DIGITAL_ENCODERS_READ
    #define ANALOG_BUTTON_UP   A0
    #define ANALOG_BUTTON_DOWN A1

    #define ROTARY_ENCODER1_PIN1 A2
    #define ROTARY_ENCODER1_PIN2 A3
    //#define ROTARY_ENCODER1_KEY  4

    #define ROTARY_ENCODER2_PIN1 3
    #define ROTARY_ENCODER2_PIN2 2
    //#define ROTARY_ENCODER2_KEY  4
    #define ROTARY_ENCODER3_PIN1 A4
    #define ROTARY_ENCODER3_PIN2 A5
    //#define ROTARY_ENCODER3_KEY  4
    #define ROTARY_ENCODER4_PIN1 5
    #define ROTARY_ENCODER4_PIN2 4
    //#define ROTARY_ENCODER4_KEY  4
  #endif

#ifdef TREE_ENCODERS_ONE_POTENTIOMETER_IN_LINE
    #define pot0  A0
    #define pot1  A1

    #define upButtonPin  37
    #define downButtonPin  35

    #define minMidMAX_SwitchPin A2 //flag0, flag1
    #define fireButtonPin 34 //flag2

    #define ROTARY_ENCODER_MIN1_PIN1 30
    #define ROTARY_ENCODER_MIN1_PIN2 31

    #define ROTARY_ENCODER_MID1_PIN1 28
    #define ROTARY_ENCODER_MID1_PIN2 29


    #define ROTARY_ENCODER_MAX1_PIN1 26
    #define ROTARY_ENCODER_MAX1_PIN2 27

    #define ROTARY_ENCODER_MIN2_PIN1 24
    #define ROTARY_ENCODER_MIN2_PIN2 25

    #define ROTARY_ENCODER_MID2_PIN1 22
    #define ROTARY_ENCODER_MID2_PIN2 23

    #define ROTARY_ENCODER_MAX2_PIN1 20
    #define ROTARY_ENCODER_MAX2_PIN2 21

#endif

#ifdef TREE_ENCODERS_ONE_POTENTIOMETER_IN_LINE

  Encoder myEncMin1(ROTARY_ENCODER_MIN1_PIN1, ROTARY_ENCODER_MIN1_PIN2);
  Encoder myEncMin2(ROTARY_ENCODER_MIN2_PIN1, ROTARY_ENCODER_MIN2_PIN2);

  Encoder myEncMid1(ROTARY_ENCODER_MID1_PIN1, ROTARY_ENCODER_MID1_PIN2);
  Encoder myEncMid2(ROTARY_ENCODER_MID2_PIN1, ROTARY_ENCODER_MID2_PIN2);
  
  Encoder myEncMax1(ROTARY_ENCODER_MAX1_PIN1, ROTARY_ENCODER_MAX1_PIN2);
  Encoder myEncMax2(ROTARY_ENCODER_MAX2_PIN1, ROTARY_ENCODER_MAX2_PIN2);

  long newPositionMin1 =0;
  long newPositionMin2 =0;
  long newPositionMid1 =0;
  long newPositionMid2 =0;
  long newPositionMax1 =0;
  long newPositionMax2 =0;
  

  long oldPositionMin1  = -999;
  long oldPositionMin2  = -999;
  long oldPositionMid1  = -999;
  long oldPositionMid2  = -999;
  long oldPositionMax1  = -999;
  long oldPositionMax2  = -999;
  
  long oldEncoderPositionMin1 =-999;
  long newEncoderPositionMin1 = 255;

  long oldEncoderPositionMin2 =-999;
  long newEncoderPositionMin2 = 255;

  long oldEncoderPositionMid1 =-999;
  long newEncoderPositionMid1 = 255;

  long oldEncoderPositionMid2 =-999;
  long newEncoderPositionMid2 = 255;

  long oldEncoderPositionMax1 =-999;
  long newEncoderPositionMax1 = 255;

  long oldEncoderPositionMax2 =-999;
  long newEncoderPositionMax2 = 255;

#endif

#ifdef DIGITAL_ENCODERS_READ
  // Change these two numbers to the pins connected to your encoder.
  //   Best Performance: both pins have interrupt capability
  //   Good Performance: only the first pin has interrupt capability
  //   Low Performance:  neither pin has interrupt capability
  Encoder myEncMin1(ROTARY_ENCODER_MIN1_PIN1, ROTARY_ENCODER_MIN1_PIN2);
  Encoder myEncMin2(ROTARY_ENCODER_MIN2_PIN1, ROTARY_ENCODER_MIN2_PIN2);

  Encoder myEncMid1(ROTARY_ENCODER_MID1_PIN1, ROTARY_ENCODER_MID1_PIN2);
  Encoder myEncMid2(ROTARY_ENCODER_MID2_PIN1, ROTARY_ENCODER_MID2_PIN2);
  
  Encoder myEncMax1(ROTARY_ENCODER_MAX1_PIN1, ROTARY_ENCODER_MAX1_PIN2);
  Encoder myEncMax2(ROTARY_ENCODER_MAX2_PIN1, ROTARY_ENCODER_MAX2_PIN2);
  
  long newPosition1 =0;
  long newPosition2 =0;
  long newPosition3 =0;
  long newPosition4 =0;
  
  long oldPosition1  = -999;
  long oldPosition2  = -999;
  long oldPosition3  = -999;
  long oldPosition4  = -999;
  
  long oldEncoderPosition1 =-999;
  long newEncoderPosition1 = 255;

  long oldEncoderPosition2 =-999;
  long newEncoderPosition2 = 255;

  long oldEncoderPosition3 =-999;
  long newEncoderPosition3 = 255;

  long oldEncoderPosition4 =-999;
  long newEncoderPosition4 = 255;
#endif

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
char servo[]="Srv";//"Servo ";
char colon[]=":";//": ";
uint8_t upButtonState = 0;
uint8_t downButtonState = 0;
uint8_t activeServoSet = 0;
uint8_t fireBtnState = 0;
uint8_t minMidMAXState = 0;
uint8_t previousState = 0;
uint8_t previousFireBtnState = 0;

int prevAnalogValue = 0;
int analogValue=0;

#ifdef USE_PWM_DRIVER
  bool pwmAvailable = false;
#endif

#define SERVOPULSE_CONVERSION_NEEDED
uint16_t servoPulse[32] =    {
                              SERVO_MIN_eyeLeftUD       ,
                              SERVO_MIN_eyeLeftLR       ,
                              SERVO_MIN_eyeRightUD      ,
                              SERVO_MIN_eyeRightLR      ,
                              SERVO_MIN_eyelidLeftUpper ,
                              SERVO_MIN_eyelidLeftLower ,
                              SERVO_MIN_eyelidRightUpper,
                              SERVO_MIN_eyelidRightLower,
                              SERVO_MIN_eyebrowRight    ,
                              SERVO_MIN_eyebrowLeft     ,
                              SERVO_MIN_cheekRight      ,
                              SERVO_MIN_cheekLeft       ,
                              SERVO_MIN_upperLip        ,
                              SERVO_MIN_forheadRight    ,
                              SERVO_MIN_forheadLeft     ,
                              SERVO_MIN_Jaw_UpDown      ,

                              SERVO_MAX_eyeLeftUD       ,
                              SERVO_MAX_eyeLeftLR       ,
                              SERVO_MAX_eyeRightUD      ,
                              SERVO_MAX_eyeRightLR      ,
                              SERVO_MAX_eyelidLeftUpper ,
                              SERVO_MAX_eyelidLeftLower ,
                              SERVO_MAX_eyelidRightUpper,
                              SERVO_MAX_eyelidRightLower,
                              SERVO_MAX_eyebrowRight    ,
                              SERVO_MAX_eyebrowLeft     ,
                              SERVO_MAX_cheekRight      ,
                              SERVO_MAX_cheekLeft       ,
                              SERVO_MAX_upperLip        ,
                              SERVO_MAX_forheadRight    ,
                              SERVO_MAX_forheadLeft     ,
                              SERVO_MAX_Jaw_UpDown      ,
                              };


//#define HIGHSPEED 

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

#ifdef USE_RF_REMOTE
  const uint64_t my_radio_pipe = 0x0022; //tento istý kód musí mať aj prijímač
  /*
Arduion RF NANO   pinout
CE   D10
CSN  D09
SCK  D13
MOSI D11
MISO D12
 from: RF-Nano-Schematic.pdf

Arduion MEGA  NRF24L01 PA/LNA   pinout
CE   D10
CSN  D09
SCK  D52
MOSI D51
MISO D50

 SCK, MOSI, MISO and CS (or SS) pins. Those pins are 52, 51, 50 and 53 (defalut) on a Mega.
from: https://forum.arduino.cc/t/pin-connection/613444/4  

Hardware SPI Pins:
 * Arduino Uno   SCK=13, SDA=11
 * Arduino Nano  SCK=13, SDA=11
 * Arduino Due   SCK=76, SDA=75
 * Arduino Mega  SCK=52, SDA=51

*/


RF24 radio(10, 9);  //zapojenie CE a CSN pinov //RF24(rf24_gpio_pin_t _cepin, rf24_gpio_pin_t _cspin, uint32_t _spi_speed = RF24_SPI_SPEED);
  //maximalne 32 kanalov
#endif

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
  //tft.pushColor(tft.Color565(RED,GREEN,BLUE));
  //tft.fillScreen(BLACK);
  //Set background colour
  Serial.println("setup: tft.fillScreen(BLACK)");
  tft.fillScreen(BLACK);
  Serial.println("setup: BLACK =done");


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

  #ifdef SERVOPULSE_CONVERSION_NEEDED
    servopulse_initial_conversion();
  #endif

  #ifdef TREE_ENCODERS_ONE_POTENTIOMETER_IN_LINE
    pinMode(pot0, INPUT);
    pinMode(pot1, INPUT);
    pinMode(minMidMAX_SwitchPin, INPUT);
    

    pinMode(  upButtonPin, INPUT_PULLUP);
    pinMode(downButtonPin, INPUT_PULLUP);
    
    //pinMode(minMidMAX_SwitchPin, INPUT);
    pinMode(fireButtonPin, INPUT);

  #endif

  #ifdef ANALOG_POTENTIOMENTERS_READ
    pinMode(pot0, INPUT);
    pinMode(pot1, INPUT);
    pinMode(pot2, INPUT);
    

    pinMode(  upButtonPin, INPUT_PULLUP);
    pinMode(downButtonPin, INPUT_PULLUP);
    
    pinMode(minMidMAX_SwitchPin, INPUT);
    pinMode(fireButtonPin, INPUT);

  #endif

  prepareServoForm();
  //------------------
  #ifdef USE_RF_REMOTE
    Serial.println("setup: radio.begin()....");
    radio.begin();
    Serial.println("setup: radio.setAutoAck(f)");
    radio.setAutoAck(false);
    Serial.println("setup: radio.setDataRate(RF24_250KBPS)");
    radio.setDataRate(RF24_250KBPS);
    Serial.println("setup: radio.openWritingPipe(my_radio_pipe)");
    radio.openWritingPipe(my_radio_pipe);
    Serial.println("setup: radio - OK.  radio - end");
  #endif

   Serial.println("setup:done. setup END.");
   Serial.println("starting loop.... ");
}

//-------------------------loop------------------------------------------------
//-------------------------loop------------------------------------------------
//-------------------------loop------------------------------------------------
//-------------------------loop------------------------------------------------
void loop() {
  
  unsigned long currentMillis = millis();
  //Run function to see if buttons have been pressed, and pick a servo set accordingly
  loop_servoSet_BTN_Select(currentMillis);

  #if defined(TREE_ENCODERS_ONE_POTENTIOMETER_IN_LINE)
      servoPulse[(activeServoSet*LEFT_ARROW_STEP)+0] = map(analogRead(pot0), 0, 1023, 255, 0);
      servoPulse[(activeServoSet*LEFT_ARROW_STEP)+2] = map(analogRead(pot1), 0, 1023, 255, 0);

  #elif derined(ANALOG_POTENTIOMENTERS_READ)
    //Record the positions of all servos mapped to a pulsewidth of between 0 and 255
    if(LEFT_ARROW_STEP>2) {
      servoPulse[(activeServoSet*LEFT_ARROW_STEP)+0] = map(analogRead(pot0), 0, 1023, 255, 0);
      servoPulse[(activeServoSet*LEFT_ARROW_STEP)+1] = map(analogRead(pot2), 0, 1023, 255, 0);
      servoPulse[(activeServoSet*LEFT_ARROW_STEP)+2] = map(analogRead(pot1), 0, 1023, 255, 0);
      servoPulse[(activeServoSet*LEFT_ARROW_STEP)+3] = map(analogRead(pot3), 0, 1023, 255, 0);
    } else {
      servoPulse[0] = map(analogRead(pot0), 0, 1023, 255, 0);
      servoPulse[1] = map(analogRead(pot1), 0, 1023, 255, 0);

      servoPulse[2] = map(analogRead(pot2), 0, 1023, 255, 0);
      servoPulse[3] = map(analogRead(pot3), 0, 1023, 255, 0);
    }
  #elif defined(DIGITAL_ENCODERS_READ)
    ////Read endoders and compute values for all servos.
    ////rotary encoder handling
    //--------------------------------------------------------------------------
    newPosition1 = myEnc1.read();
    if (newPosition1 != oldPosition1) {
      oldPosition1 = newPosition1;
      //Serial.println("encoder newPosition1 = "+String(newPosition1));
      newEncoderPosition1 = (newPosition1/4);
      if(oldEncoderPosition1 != newEncoderPosition1) {
        Serial.println("newEncoderPosition1 "+String(newEncoderPosition1));
        if(newEncoderPosition1 < oldEncoderPosition1) {
          if(servoPulse[(activeServoSet*LEFT_ARROW_STEP)+0]<255){
            servoPulse[(activeServoSet*LEFT_ARROW_STEP)+0] =servoPulse[(activeServoSet*LEFT_ARROW_STEP)+0] +1;
          }
        }
        if(newEncoderPosition1 > oldEncoderPosition1) {
          if(servoPulse[(activeServoSet*LEFT_ARROW_STEP)+0]>0){
            servoPulse[(activeServoSet*LEFT_ARROW_STEP)+0] =servoPulse[(activeServoSet*LEFT_ARROW_STEP)+0] -1;
          }
        }
        oldEncoderPosition1 = newEncoderPosition1;
      }//if(oldEncoderPosition != newEncoderPosition)
    }
    //--------------------------------------------------------------------------
    newPosition2 = myEnc2.read();
    if (newPosition2 != oldPosition2) {
      oldPosition2 = newPosition2;
      //Serial.println("encoder newPosition2 = "+String(newPosition2));
      newEncoderPosition2 = (newPosition2/4);
      if(oldEncoderPosition2 != newEncoderPosition2) {
        Serial.println("newEncoderPosition2 "+String(newEncoderPosition2));
        if(newEncoderPosition2 < oldEncoderPosition2) {
          if(servoPulse[(activeServoSet*LEFT_ARROW_STEP)+1]<255){
            servoPulse[(activeServoSet*LEFT_ARROW_STEP)+1] =servoPulse[(activeServoSet*LEFT_ARROW_STEP)+1] +1;
          }
        }
        if(newEncoderPosition2 > oldEncoderPosition2) {
          if(servoPulse[(activeServoSet*LEFT_ARROW_STEP)+1]>0){
            servoPulse[(activeServoSet*LEFT_ARROW_STEP)+1] =servoPulse[(activeServoSet*LEFT_ARROW_STEP)+1] -1;
          }
        }
        oldEncoderPosition2 = newEncoderPosition2;
      }//if(oldEncoderPosition != newEncoderPosition)
    }
    //--------------------------------------------------------------------------
    newPosition3 = myEnc3.read();
    if (newPosition3 != oldPosition3) {
      oldPosition3 = newPosition3;
      //Serial.println("encoder newPosition3 = "+String(newPosition3));
      newEncoderPosition3 = (newPosition3/4);
      if(oldEncoderPosition3 != newEncoderPosition3) {
        Serial.println("newEncoderPosition3 "+String(newEncoderPosition3));
        if(newEncoderPosition3 < oldEncoderPosition3) {
          if(servoPulse[(activeServoSet*LEFT_ARROW_STEP)+2]<255){
            servoPulse[(activeServoSet*LEFT_ARROW_STEP)+2] =servoPulse[(activeServoSet*LEFT_ARROW_STEP)+2] +1;
          }
        }
        if(newEncoderPosition3 > oldEncoderPosition3) {
          if(servoPulse[(activeServoSet*LEFT_ARROW_STEP)+2]>0){
            servoPulse[(activeServoSet*LEFT_ARROW_STEP)+2] =servoPulse[(activeServoSet*LEFT_ARROW_STEP)+2] -1;
          }
        }
        oldEncoderPosition3 = newEncoderPosition3;
      }//if(oldEncoderPosition != newEncoderPosition)
    }
    //--------------------------------------------------------------------------
    newPosition4 = myEnc4.read();
    if (newPosition4 != oldPosition4) {
      oldPosition4 = newPosition4;
      //Serial.println("encoder newPosition4 = "+String(newPosition4));
      newEncoderPosition4 = (newPosition4/4);
      if(oldEncoderPosition4 != newEncoderPosition4) {
        Serial.println("newEncoderPosition4 "+String(newEncoderPosition4));
        if(newEncoderPosition4 < oldEncoderPosition4) {
          if(servoPulse[(activeServoSet*LEFT_ARROW_STEP)+3]<255){
            servoPulse[(activeServoSet*LEFT_ARROW_STEP)+3] =servoPulse[(activeServoSet*LEFT_ARROW_STEP)+3] +1;
          }
        }
        if(newEncoderPosition4 > oldEncoderPosition4) {
          if(servoPulse[(activeServoSet*LEFT_ARROW_STEP)+3]>0){
            servoPulse[(activeServoSet*LEFT_ARROW_STEP)+3] =servoPulse[(activeServoSet*LEFT_ARROW_STEP)+3] -1;
          }
        }
        oldEncoderPosition4 = newEncoderPosition4;
      }//if(oldEncoderPosition != newEncoderPosition)
    }
    //--------------------------------------------------------------------------
  #endif

  //Clear the previous number, and write the new pulsewidths for the active servo set to the monitor
  loop_writePulsesToDisplay(currentMillis);
  
  #ifdef USE_RF_REMOTE
    loop_WriteTo_RF_Line(currentMillis);
  #endif

#ifdef USE_PWM_DRIVER
  if(pwmAvailable) {
    //Using the servo driver board, set the active servos to the position  specified by the potentiometers
    pwm.setPWM((activeServoSet*LEFT_ARROW_STEP)+0, 0, map(servoPulse[(activeServoSet*LEFT_ARROW_STEP)+0], 0, 255, 0, 1023));
    pwm.setPWM((activeServoSet*LEFT_ARROW_STEP)+1, 0, map(servoPulse[(activeServoSet*LEFT_ARROW_STEP)+1], 0, 255, 0, 1023));
    pwm.setPWM((activeServoSet*LEFT_ARROW_STEP)+2, 0, map(servoPulse[(activeServoSet*LEFT_ARROW_STEP)+2], 0, 255, 0, 1023));
    pwm.setPWM((activeServoSet*LEFT_ARROW_STEP)+3, 0, map(servoPulse[(activeServoSet*LEFT_ARROW_STEP)+3], 0, 255, 0, 1023));
  }
#endif  
  //delay(150);
}
//-----loop_WriteTo_RF_Line----------------------------------------
void loop_WriteTo_RF_Line (unsigned long currentMillis) {
  if (currentMillis - previousMillis_SerialLine >= interval_SerialLine) {  // start timed event for read and send
    previousMillis_SerialLine = currentMillis;
    //ToDoHere;
    ReadHwData();
    #ifdef USE_RF_REMOTE
      RF_Line_WriteEvent(currentMillis);
    #endif
  } // end of timed event send
}

void prepareServoForm(){
  Serial.println("setup: Write servo numbers 1.for {for{}} start");
  //Write servo numbers 
  for (uint8_t count = 0; count <= ((16/LEFT_ARROW_STEP) - 1); count ++){ 
    for (uint8_t i = 0; i <=(LEFT_ARROW_STEP - 1); i ++){
      char numRead[2];
      char combined[30]= {0};
      dtostrf(servoNum, 1, 0, numRead);
      strcat(combined, servo);
      strcat(combined, numRead);
      tft.drawString(0, yPos, combined, WHITE);
      //Serial.println("setup: y:"+String(yPos)+", combined:"+String(combined)+", colon:"+String(colon)+"count:"+String(count)+", i:"+String(i)+".");
      tft.drawString((((strlen(servo) + 1)) * 8), yPos, colon, WHITE);    
      servoNum ++;
      yPos += spacing;    
      }
      yPos += (2*LEFT_ARROW_STEP); //8;
    }
  Serial.println("setup: 1.for {for{}} done");

  Serial.println("setup: Write initial servo positions (350 to start with)  2.for {for{}} started");
  //Write initial servo positions (350 to start with)  
  servoNum = 0;
  yPos = 2;
  for (uint8_t count = 0; count <= ((16/LEFT_ARROW_STEP) - 1); count ++){ 
    for (uint8_t i = 0; i <=(LEFT_ARROW_STEP - 1); i ++){
      if(LEFT_ARROW_STEP>2) {
        char numRead[3];
        dtostrf(servoPulse[servoNum], 3, 0, numRead);
        tft.drawString((((strlen(servo) + 2)) * 8), yPos, numRead, YELLOW);
      } else {
        char numRead[3];
        dtostrf(servoPulse[servoNum], 3, 0, numRead);
        tft.drawString((((strlen(servo) + 2)) * 8), yPos, numRead, YELLOW);

        char numRead2[3];
        dtostrf(servoPulse[servoNum + 2], 3, 0, numRead2);
        tft.drawString((((strlen(servo) + 2 + 4)) * 8), yPos, numRead2, YELLOW);

      }
      //Serial.println("setup: y:"+String(yPos)+", numRead:"+String(numRead)+", count:"+String(count)+", i:"+String(i)+".");
      servoNum ++;
      yPos += spacing;    
      }
    yPos += (2*LEFT_ARROW_STEP); //8;
  }
  Serial.println("setup: 2.for {for{}} done");

   tft.drawString((128-(LEFT_ARROW_SIZE*8)), 3, "<", WHITE, LEFT_ARROW_SIZE);
}

void servopulse_initial_conversion() {
  Serial.println("servopulse_initial_conversion. started");
  for (byte i=0; i<3 ; i++) {
    //map(long x, long in_min, long in_max, long out_min, long out_max)
    Serial.print("i="+String(i)+", orig="+String(servoPulse[i])+", ");
    servoPulse[i] = map(servoPulse[i], 0,1024, 0,255);
    Serial.println(" new="+String(servoPulse[i]));
  }
  Serial.println("servopulse_initial_conversion. end");
}


void ReadHwData() {
  mydata_send.servoSet = activeServoSet;

  mydata_send.s1min = servoPulse[ 0];
  mydata_send.s2min = servoPulse[ 1];

  mydata_send.s1max = servoPulse[ 2];
  mydata_send.s2max = servoPulse[ 3];

//mydata_send.s1mid = servoPulse[ 4];
//mydata_send.s2mid = servoPulse[ 5];
  
//mydata_send.s1curr = servoPulse[ 6];
//mydata_send.s2curr = servoPulse[ 7];

  
  mydata_send.devType =  2; // mode:  0 = fourSticksController (8 chanels) ,   1 = ServoConfigurator (16 chanels) , 2 = MinMaxServoConfig (min max for 2 chanels)
  //mydata_send.flags = 0;

  mydata_send.switchPos = minMidMAXState;
  mydata_send.fireBtn1 = fireBtnState;
}
//------------------BtWriteEvent-------------------------------------
void RF_Line_WriteEvent (unsigned long currentMillis) {
  #ifdef USE_RF_REMOTE
    radio.write(&mydata_send, sizeof(TX_DATA_STRUCTURE));
  #endif
}


void loop_writePulsesToDisplay (unsigned long currentMillis){
  if (currentMillis - previousMillis_writeToDisplay >= interval_writeToDisplay) {  // start timed event for read and send
    previousMillis_writeToDisplay = currentMillis;

    servoNum = 0;
    yPos = 2 + (activeServoSet*((8+2) * LEFT_ARROW_STEP));
  
    //Serial.println("loop_writePulsesToDisplay: for{...} start");
    //if(previousServoPulse[(activeServoSet*LEFT_ARROW_STEP)+servoNum] != servoPulse[(activeServoSet*LEFT_ARROW_STEP)+servoNum]) {
      for (uint8_t i = 0; i <=(LEFT_ARROW_STEP - 1); i ++){
        if(LEFT_ARROW_STEP>2) {
          char inChar[3];
          dtostrf(servoPulse[(activeServoSet*LEFT_ARROW_STEP)+i], 3, 0, inChar);
          tft.fillRect(((((strlen(servo) + 2)) * 8)-2), yPos, 30, 8, BLACK);
          tft.drawString((((strlen(servo) + 2)) * 8), yPos, inChar, YELLOW);
        } else {
          char inChar[3];
          dtostrf(servoPulse[(activeServoSet*LEFT_ARROW_STEP)+i], 3, 0, inChar);
          //dtostrf(((10*(activeServoSet*LEFT_ARROW_STEP))+(2*i)), 3, 0, inChar);
          tft.fillRect(((((strlen(servo) + 2)) * 8)-2), yPos, 30, 8, BLACK);
          tft.drawString((((strlen(servo) + 2)) * 8), yPos, inChar, YELLOW);

          char inChar2[3];
          dtostrf(servoPulse[(activeServoSet*LEFT_ARROW_STEP)+i+2], 3, 0, inChar2);
          //dtostrf(((10*(activeServoSet*LEFT_ARROW_STEP))+(2*i)+1), 3, 0, inChar2);
          tft.fillRect(((((strlen(servo) + 2 + 4)) * 8)-2), yPos, 30, 8, BLACK);
          tft.drawString((((strlen(servo) + 2 + 4)) * 8), yPos, inChar2, YELLOW);
        }
        //Serial.print(" loop_writePulsesToDisplay: yPos:"+String(yPos)+" , inChar:"+String(inChar)+". ");
        servoNum ++;
        yPos += spacing;    
      }
    //}
    yPos += (2*LEFT_ARROW_STEP); //8;
    //Serial.println("loop_writePulsesToDisplay: end");
  }
}

void loop_servoSet_BTN_Select(unsigned long currentMillis){
  if (currentMillis - previousMillis_BTN_Select >= interval_BTN_Select) {  // start timed event for read and send
    previousMillis_BTN_Select = currentMillis;
    #if defined(ANALOG_POTENTIOMENTERS_READ) || defined (TREE_ENCODERS_ONE_POTENTIOMETER_IN_LINE)
      upButtonState = digitalRead(upButtonPin);
      downButtonState = digitalRead(downButtonPin);

      analogValue = analogRead(minMidMAX_SwitchPin);

      if(analogValue < 100) minMidMAXState = 0;
      else if(analogValue > 900) minMidMAXState = 2;
      else minMidMAXState = 1;
    
      if(previousState != minMidMAXState) {
    
        previousState = minMidMAXState;
        Serial.print("Button state: ");
        Serial.println(minMidMAXState);
      }

      fireBtnState = (analogRead(fireButtonPin )>127 ? LOW : HIGH);
      if(fireBtnState == HIGH && previousFireBtnState == LOW) {
          Serial.println("Button - fire pressed");
      }
      previousFireBtnState = fireBtnState;

    #endif

    #ifdef DIGITAL_ENCODERS_READ
      //upButtonState = digitalRead(upButtonPin);
      //downButtonState = digitalRead(downButtonPin);
        upButtonState = (analogRead(ANALOG_BUTTON_UP  )>127 ? HIGH : LOW);
      downButtonState = (analogRead(ANALOG_BUTTON_DOWN)>127 ? HIGH : LOW);
    #endif

    if (upButtonState == LOW){
       activeServoSet ++;
       if (activeServoSet >((16/LEFT_ARROW_STEP) - 1)){
        activeServoSet = 0;
       }
        tft.fillRect((128-(LEFT_ARROW_SIZE*8)), 0, (LEFT_ARROW_SIZE*8), 160, BLACK);
        tft.drawString((128-(LEFT_ARROW_SIZE*8)), ((activeServoSet * ((2+8) * LEFT_ARROW_STEP))+3), "<", WHITE, LEFT_ARROW_SIZE);
        delay(150);
    }
    if (downButtonState == LOW){
      activeServoSet --;
      if (activeServoSet >((16/LEFT_ARROW_STEP) - 1)){
        activeServoSet = ((16/LEFT_ARROW_STEP) - 1);
      }
        tft.fillRect((128-(LEFT_ARROW_SIZE*8)), 0, (LEFT_ARROW_SIZE*8), 160, BLACK);
        tft.drawString((128-(LEFT_ARROW_SIZE*8)), ((activeServoSet * ((8+2) * LEFT_ARROW_STEP))+3), "<", WHITE, LEFT_ARROW_SIZE);
        delay(150);
    }
    //ToDoHere;
  }
}
