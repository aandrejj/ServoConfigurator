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
#define LEFT_ARROW_SIZE  1
#define LEFT_ARROW_STEP  1

  //           ST7735(uint8_t CS, uint8_t RS, uint8_t SID, uint8_t SCLK, uint8_t RST);
  ST7735 tft = ST7735(   DISP_CS,    DISP_RS,    DISP_SID,    DISP_SCLK,    DISP_RST); 
//ST7735 tft = ST7735(         6,          7,          11,           13,           8); 
  //           ST7735(uint8_t CS, uint8_t RS, uint8_t RST);
//ST7735 tft = ST7735(6, 7, 8);    

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
    #define minMidMAX_SwitchPin A2 //flag0, flag1

    #define fireButtonPin 34 //flag2
    #define downButtonPin 37
    #define upButtonPin   35
    
    #define ROTARY_ENCODER_MIN1_PIN1 3
    #define ROTARY_ENCODER_MIN1_PIN2 2

    #define ROTARY_ENCODER_MID1_PIN1 18
    #define ROTARY_ENCODER_MID1_PIN2 19


    #define ROTARY_ENCODER_MAX1_PIN1 21
    #define ROTARY_ENCODER_MAX1_PIN2 20

    #define ROTARY_DIVIDER 4

#endif

#ifdef TREE_ENCODERS_ONE_POTENTIOMETER_IN_LINE
  // Change these two numbers to the pins connected to your encoder.
  //   Best Performance: both pins have interrupt capability
  //   Good Performance: only the first pin has interrupt capability
  //   Low Performance:  neither pin has interrupt capability
  Encoder myEncMin1(ROTARY_ENCODER_MIN1_PIN1, ROTARY_ENCODER_MIN1_PIN2);
  Encoder myEncMid1(ROTARY_ENCODER_MID1_PIN1, ROTARY_ENCODER_MID1_PIN2);

  Encoder myEncMax1(ROTARY_ENCODER_MAX1_PIN1, ROTARY_ENCODER_MAX1_PIN2);

  long newPosition[3] ={   0,    0,    0};
  long oldPosition[3]= {-999, -999, -999};
  long oldEncoderPosition[3]= {-999, -999, -999};
  long newEncoderPosition[3] = {255,  255,  255};
#endif

uint16_t servoPulseIndex =0;
bool data_changed;

uint16_t analogValuePot0 = 0;
uint16_t prevAnalogValuePot0 = 0;

int prevAnalogValue_3StateSwitch = 0;
int analogValue_3StateSwitch=0;



uint8_t servoIndexForAnalog=0;

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
char servo[]="S"; //"Srv";//"Servo ";
char colon[]=":";//": ";
uint8_t upButtonState = 0;
uint8_t downButtonState = 0;

int16_t AntiActiveServoSet =0;
int16_t activeServoSet = 0;

uint8_t fireBtnState = 0;
uint8_t minMidMAXState = 0;
uint8_t previousState = 0;
uint8_t previousFireBtnState = 0;

#ifdef USE_PWM_DRIVER
  bool pwmAvailable = false;
#endif

//#define SERVOPULSE_CONVERSION_NEEDED
uint16_t prevServoPulse[64] ={0,0,0,0,0,0,0,0,0,0,
                              0,0,0,0,0,0,0,0,0,0,
                              0,0,0,0,0,0,0,0,0,0,
                              0,0,0,0,0,0,0,0,0,0,
                              0,0,0,0,0,0,0,0,0,0,
                              0,0,0,0,0,0,0,0,0,0,
                              0,0,0,0};

uint16_t servoPulse[64] =    {
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
                            SERVO_MID_eyeLeftUD       ,
                            SERVO_MID_eyeLeftLR       ,
                            SERVO_MID_eyeRightUD      ,
                            SERVO_MID_eyeRightLR      ,
                            SERVO_MID_eyelidLeftUpper ,
                            SERVO_MID_eyelidLeftLower ,
                            SERVO_MID_eyelidRightUpper,
                            SERVO_MID_eyelidRightLower,
                            SERVO_MID_eyebrowRight    ,
                            SERVO_MID_eyebrowLeft     ,
                            SERVO_MID_cheekRight      ,
                            SERVO_MID_cheekLeft       ,
                            SERVO_MID_upperLip        ,
                            SERVO_MID_forheadRight    ,
                            SERVO_MID_forheadLeft     ,
                            SERVO_MID_Jaw_UpDown      ,
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
                            SERVO_MID_eyeLeftUD       ,
                            SERVO_MID_eyeLeftLR       ,
                            SERVO_MID_eyeRightUD      ,
                            SERVO_MID_eyeRightLR      ,
                            SERVO_MID_eyelidLeftUpper ,
                            SERVO_MID_eyelidLeftLower ,
                            SERVO_MID_eyelidRightUpper,
                            SERVO_MID_eyelidRightLower,
                            SERVO_MID_eyebrowRight    ,
                            SERVO_MID_eyebrowLeft     ,
                            SERVO_MID_cheekRight      ,
                            SERVO_MID_cheekLeft       ,
                            SERVO_MID_upperLip        ,
                            SERVO_MID_forheadRight    ,
                            SERVO_MID_forheadLeft     ,
                            SERVO_MID_Jaw_UpDown
                              };


#define HIGHSPEED 

#ifdef HIGHSPEED
  #define Baud 19200   // Serial monitor
  //#define BTBaud 38400 // There is only one speed for configuring HC-05, and that is 38400.
#else
  #define Baud 9600    // Serial monitor
  //#define BTBaud 4800  // HM-10, HM-19 etc
#endif

unsigned long previousMillis_BTN_Select = 0;
const long interval_BTN_Select = 150;

unsigned long previousMillis_SerialLine = 0;
const long interval_SerialLine = 150;

unsigned long previousMillis_writeToDisplay = 0;
const long interval_writeToDisplay = 350;


#ifdef USE_PWM_DRIVER
  Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#endif

#ifdef USE_RF_REMOTE
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
    pinMode(fireButtonPin, INPUT_PULLUP);

    myEncMin1.write(newEncoderPosition[0]);
    myEncMid1.write(newEncoderPosition[1]);
    myEncMax1.write(newEncoderPosition[2]);
    //myEncMin2.write(newEncoderPosition[3]);
    //myEncMid2.write(newEncoderPosition[4]);
    //myEncMax2.write(newEncoderPosition[5]);



  #endif


  prepareServoForm();
  //------------------
  #ifdef USE_RF_REMOTE
    Serial.println("setup: radio.begin()....");
    radio.begin();
    Serial.println("setup: radio.setAutoAck(f)");
    //radio.setAutoAck(false);
    radio.setAutoAck(true);
    
    //Serial.println("setup: radio.setDataRate(RF24_250KBPS)");
    //radio.setDataRate(RF24_250KBPS);
    Serial.println("setup: radio.openWritingPipe(my_radio_pipe)");
    radio.openWritingPipe(my_radio_pipe);
    Serial.println("setup: radio.setPALevel(RF24_PA_MIN)");
    radio.setPALevel(RF24_PA_MIN);
    Serial.println("setup: radio.stopListening()");
    radio.stopListening();

    Serial.println("setup: radio - OK.  radio - end");
  #endif

   Serial.println("setup:done. setup END.");
   Serial.println("starting loop.... ");
}

//-------------------------loop------------------------------------------------
//-------------------------loop------------------------------------------------
//-------------------------loop------------------------------------------------
//-------------------------loop------------------------------------------------
int origAnalogValuePot0;
void loop() {
  
  unsigned long currentMillis = millis();
  //Run function to see if buttons have been pressed, and pick a servo set accordingly
  loop_servoSet_BTN_Select(currentMillis);

  #if defined(TREE_ENCODERS_ONE_POTENTIOMETER_IN_LINE)
      RotEnc_EvaluateIncrement(&myEncMin1, RotEnc_Row1_MIN, ROTARY_DIVIDER, activeServoSet, LEFT_ARROW_STEP, LABEL_FORM_MIN);
      RotEnc_EvaluateIncrement(&myEncMid1, RotEnc_Row1_MID, ROTARY_DIVIDER, activeServoSet, LEFT_ARROW_STEP, LABEL_FORM_MID);
      
      origAnalogValuePot0 = analogRead(pot0);
      analogValuePot0 = constrain(origAnalogValuePot0, 0, 1023);
      servoIndexForAnalog = (((activeServoSet*LEFT_ARROW_STEP)) + (16 * 0));

      servoPulse[((servoIndexForAnalog) + (16 * 3))] = 
        (analogValuePot0)<512 ? 
          (map(analogValuePot0,   0, 512, servoPulse[(servoIndexForAnalog)           ], servoPulse[(servoIndexForAnalog) + (16 * 1)]) ) 
        : (map(analogValuePot0, 513, 1023, servoPulse[(servoIndexForAnalog) + (16 * 1)], servoPulse[(servoIndexForAnalog) + (16 * 2)]) );

      if(abs(prevAnalogValuePot0 - analogValuePot0)>2) {
        Serial.println("loop:origAnalogValuePot0 = "+String(origAnalogValuePot0)+", analogValuePot0 ="+String (analogValuePot0)+" servoPulse["+String(((servoIndexForAnalog) + (16 * 3)))+"] ="+String(servoPulse[((servoIndexForAnalog) + (16 * 3))])+".");
        prevAnalogValuePot0 = analogValuePot0;
      }

      //servoPulse[(activeServoSet*LEFT_ARROW_STEP) + (16 * 3)] = analogRead(pot0); // map(analogRead(pot0), 0, 1023, 255, 0);
      RotEnc_EvaluateIncrement(&myEncMax1, RotEnc_Row1_MAX, ROTARY_DIVIDER, activeServoSet, LEFT_ARROW_STEP, LABEL_FORM_MAX);
  #endif


  //Clear the previous number, and write the new pulsewidths for the active servo set to the monitor
  loop_writePulsesToDisplay(currentMillis); //here

  
  #ifdef USE_RF_REMOTE
    loop_WriteTo_RF_Line(currentMillis);
  #endif

  #ifdef USE_PWM_DRIVER
  if(pwmAvailable) {
    //Using the servo driver board, set the active servos to the position  specified by the potentiometers
    pwm.setPWM((activeServoSet*LEFT_ARROW_STEP)+0, 0, map(servoPulse[(activeServoSet*LEFT_ARROW_STEP)+0], 0, 255, 0, 1023));
  }
  #endif  
  //delay(150);
}

int16_t RotEnc_EvaluateIncrement(Encoder *myEnc, uint16_t encoderIndex, uint16_t divider, uint16_t active_ServoSet, uint16_t left_arrow_step, uint16_t Min_Mid_Max) {
    ////Read endoders and compute values for all servos.
    ////rotary encoder handling
    //--------------------------------------------------------------------------
    int16_t increment = 0;
    uint16_t servoPulseIndex = (active_ServoSet * left_arrow_step) + (16 * Min_Mid_Max);
  //uint16_t servoPulseIndex = (((active_ServoSet*LEFT_ARROW_STEP)+i)*4) + 16;

    newPosition[encoderIndex] = myEnc->read();
    if (newPosition[encoderIndex] != oldPosition[encoderIndex]) {
      oldPosition[encoderIndex] = newPosition[encoderIndex];
      //Serial.println("encoder newPosition[encoderIndex] = "+String(newPosition[encoderIndex]));
      newEncoderPosition[encoderIndex] = (newPosition[encoderIndex]/divider);
      if(oldEncoderPosition[encoderIndex] != newEncoderPosition[encoderIndex]) {
        Serial.println("RotEnc_EvaluateIncrement: newEncoderPosition["+String(encoderIndex)+"] ="+String(newEncoderPosition[encoderIndex]));
        if(newEncoderPosition[encoderIndex] > oldEncoderPosition[encoderIndex]) {
          increment = 1;
          if(servoPulse[servoPulseIndex]<1023){ 
            servoPulse[servoPulseIndex] = servoPulse[servoPulseIndex] +1;
            Serial.println("RotEnc_EvaluateIncrement [+] : active_ServoSet= "+String(active_ServoSet) + ", left_arrow_step = "+String(left_arrow_step)+", Min_Mid_Max = "+String(Min_Mid_Max)+", servoPulse["+String(servoPulseIndex)+"] = "+String(servoPulse[servoPulseIndex]));
          }
        }
        if(newEncoderPosition[encoderIndex] < oldEncoderPosition[encoderIndex]) {
          increment = -1;
          if(servoPulse[servoPulseIndex]>0){
            servoPulse[servoPulseIndex] = servoPulse[servoPulseIndex] -1;
            Serial.println("RotEnc_EvaluateIncrement -: active_ServoSet= "+String(active_ServoSet) + ", left_arrow_step = "+String(left_arrow_step)+", Min_Mid_Max = "+String(Min_Mid_Max)+", servoPulse["+String(servoPulseIndex)+"] = "+String(servoPulse[servoPulseIndex]));
          }
        }
        oldEncoderPosition[encoderIndex] = newEncoderPosition[encoderIndex];
      }//if(oldEncoderPosition != newEncoderPosition)
    }
  return increment;
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
//-------end of loop_WriteTo_RF_Line----------------------------------------
//-------end of loop_WriteTo_RF_Line----------------------------------------

String i_str ="";
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
  //servo ="S".....
  for (uint8_t count = 0; count <= ((16/LEFT_ARROW_STEP) - 1); count ++){ 
    for (uint8_t i = 0; i <=(LEFT_ARROW_STEP - 1); i ++){
        //char numRead[4];
        //dtostrf(servoPulse[servoNum], 4, 0, numRead);
        //tft.drawString((((strlen(servo) + 2)) * 8), yPos, numRead, YELLOW);
        writeMINPulsesToDisplay((count*LEFT_ARROW_STEP)+i, servoPulse[servoNum], true);

        //char numRead2[4];
        //dtostrf(servoPulse[servoNum + 16], 4, 0, numRead2);
        //tft.drawString((((strlen(servo) + 2 + 4)) * 8), yPos, numRead2, YELLOW);
        writeMIDPulsesToDisplay((count*LEFT_ARROW_STEP)+i, servoPulse[servoNum+16], true);

        writeCurrPulsesToDisplay((count*LEFT_ARROW_STEP)+i, servoPulse[servoNum+48], true);

        //char numRead3[4];
        //dtostrf(servoPulse[servoNum + 32], 4, 0, numRead3);
        //tft.drawString((((strlen(servo) + 2 + 8)) * 8), yPos, numRead3, YELLOW);
        writeMAXPulsesToDisplay((count*LEFT_ARROW_STEP)+i, servoPulse[servoNum+32], true);
        //Serial.print("prepareServoForm: y:"+String(yPos)+", count:"+String(count)+", i:"+String(i)+".");
      i_str = String(i);
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
    servoPulse[i] = map(servoPulse[i], 0,1023, 0,255);
    Serial.println(" new="+String(servoPulse[i]));
  }
  Serial.println("servopulse_initial_conversion. end");
}

void ReadHwData() {
  mydata_send.servoSet = activeServoSet;

  mydata_send.s1min = servoPulse[ activeServoSet +  0];
  mydata_send.s1mid = servoPulse[ activeServoSet + 16];

  mydata_send.s1max = servoPulse[ activeServoSet + 32];

  mydata_send.s1curr= servoPulse[ activeServoSet + 48];
  
  mydata_send.devType =  2; // mode:  0 = fourSticksController (8 chanels) ,   1 = ServoConfigurator (16 chanels) , 2 = MinMaxServoConfig (min max for 2 chanels)
  //mydata_send.flags = 0;

  mydata_send.switchPos = minMidMAXState;
  mydata_send.fireBtn1 = fireBtnState;
}
//------------------BtWriteEvent-------------------------------------
bool write_succeeded = false;
void RF_Line_WriteEvent (unsigned long currentMillis) {
  #ifdef USE_RF_REMOTE
    //Serial.println("s1min: "+String(mydata_send.s1min)+", s1curr: "+String(mydata_send.s1curr)+", s1mid: "+String(mydata_send.s1mid)+", s1max: "+String(mydata_send.s1max)+", servoSet: "+String(mydata_send.servoSet)+", devType:"+String(mydata_send.devType));
    radio.write(&mydata_send, sizeof(TX_DATA_STRUCTURE));
    //Serial.println("RF_Line_WriteEvent: Data sent");
    
    //write_succeeded = radio.write(&mydata_send, sizeof(TX_DATA_STRUCTURE)); 
    //if(write_succeeded) {
    //  Serial.println("RF_Line_WriteEvent: Data sent");
    //} else {
    //  Serial.println("RF_Line_WriteEvent: Write failed!");
    //}
    
  #endif
}

void loop_writePulsesToDisplay (unsigned long currentMillis){
  if (currentMillis - previousMillis_writeToDisplay >= interval_writeToDisplay) {  // start timed event for read and send
    previousMillis_writeToDisplay = currentMillis;
    data_changed = false;
    servoNum = 0;
    yPos = 2 + (activeServoSet*((8+2) * LEFT_ARROW_STEP));
  
    //Serial.println("loop_writePulsesToDisplay: for{...} start");
    //if(previousServoPulse[(activeServoSet*LEFT_ARROW_STEP)+servoNum] != servoPulse[(activeServoSet*LEFT_ARROW_STEP)+servoNum]) {
      //for (uint8_t i = 0; i <=(LEFT_ARROW_STEP - 1); i ++){
        uint8_t i = 0;
          servoPulseIndex = (((activeServoSet*LEFT_ARROW_STEP)+i)) + 0;
          if(prevServoPulse[servoPulseIndex] != servoPulse[servoPulseIndex]) {
            data_changed = true;
            writeMINPulsesToDisplay((activeServoSet*LEFT_ARROW_STEP)+i,servoPulse[servoPulseIndex]);
            prevServoPulse[servoPulseIndex] = servoPulse[servoPulseIndex];
          }
          
          servoPulseIndex = (((activeServoSet*LEFT_ARROW_STEP)+i)) + 16;
          if(prevServoPulse[servoPulseIndex] != servoPulse[servoPulseIndex]) {
            data_changed = true;
            writeMIDPulsesToDisplay((activeServoSet*LEFT_ARROW_STEP)+i,servoPulse[servoPulseIndex]);
            prevServoPulse[servoPulseIndex] = servoPulse[servoPulseIndex];
          }
          //-------------------------------------------------------------
          servoPulseIndex = (((activeServoSet*LEFT_ARROW_STEP)+i)) + 48;
          if(prevServoPulse[servoPulseIndex] != servoPulse[servoPulseIndex]) {
            data_changed = true;
            writeCurrPulsesToDisplay((activeServoSet*LEFT_ARROW_STEP)+i,servoPulse[servoPulseIndex]);
            prevServoPulse[servoPulseIndex] = servoPulse[servoPulseIndex];
          }

          //-------------------------------------------------------------
          servoPulseIndex = (((activeServoSet*LEFT_ARROW_STEP)+i)) + 32;
          if(prevServoPulse[servoPulseIndex] != servoPulse[servoPulseIndex]) {
            data_changed = true;
            writeMAXPulsesToDisplay((activeServoSet*LEFT_ARROW_STEP)+i,servoPulse[servoPulseIndex]);
            prevServoPulse[servoPulseIndex] = servoPulse[servoPulseIndex];
          }
          //Serial.print(" loop_writePulsesToDisplay: yPos:"+String(yPos)+" , inChar:"+String(inChar)+". ");
        servoNum ++;
        yPos += spacing;    
      //}
    //}
    yPos += (2*LEFT_ARROW_STEP); //8;
    //Serial.println("loop_writePulsesToDisplay: end");
  }
}

void writeMINPulsesToDisplay (uint8_t chanelNum, uint16_t SERVO_MIN){
  writeOneFieldToDisplay (chanelNum, LABEL_FORM_MIN, SERVO_MIN, false);  
}
void writeMINPulsesToDisplay (uint8_t chanelNum, uint16_t SERVO_MIN, bool showDebug){
  writeOneFieldToDisplay (chanelNum, LABEL_FORM_MIN, SERVO_MIN, showDebug);  
}

void writeMIDPulsesToDisplay (uint8_t chanelNum, uint16_t servo_Pwm){
  writeOneFieldToDisplay (chanelNum, LABEL_FORM_MID, servo_Pwm, false);
}
void writeMIDPulsesToDisplay (uint8_t chanelNum, uint16_t servo_Pwm, bool showDebug){
  writeOneFieldToDisplay (chanelNum, LABEL_FORM_MID, servo_Pwm, showDebug);
}

void writeMAXPulsesToDisplay (uint8_t chanelNum, uint16_t SERVO_MAX){
  writeOneFieldToDisplay (chanelNum, LABEL_FORM_MAX+1, SERVO_MAX, false);
}

void writeMAXPulsesToDisplay (uint8_t chanelNum, uint16_t SERVO_MAX, bool showDebug){
  writeOneFieldToDisplay (chanelNum, (LABEL_FORM_MAX+1) , SERVO_MAX, showDebug);
}

void writeCurrPulsesToDisplay (uint8_t chanelNum, uint16_t SERVO_MAX, bool showDebug){
  writeOneFieldToDisplay (chanelNum, 2, SERVO_MAX, showDebug);
}

void writeCurrPulsesToDisplay (uint8_t chanelNum, uint16_t SERVO_MAX){
  writeOneFieldToDisplay (chanelNum, 2, SERVO_MAX, false);
}

#define char_width_x 8

#define char_height_y 8  
#define char_shift_x  2
#define chr_point_shift_x  1

void writeOneFieldToDisplay (uint8_t chanelNum,uint8_t form_label_Min_Mid_Max, uint16_t servo_Pwm, bool showDebug){
  uint8_t modulo = chanelNum % LEFT_ARROW_STEP;
  uint8_t div_result =chanelNum / LEFT_ARROW_STEP;
  uint8_t yPos = 2 + (div_result * ((LEFT_ARROW_STEP*8)+2)) + (modulo*8);

  if(showDebug == true) {
    Serial.print("writePulsesToDisplay: ");
    Serial.print("chanelNum:"+String(chanelNum)+", form_label_Min_Mid_Max:"+String(form_label_Min_Mid_Max)+", servo_Pwm:"+String(servo_Pwm)+",  ");
    Serial.print("div_result = "+String(div_result)+", modulo = "+String(modulo)+", ");
    Serial.println("yPos:"+String(yPos)+", ");
  } else {
    //Serial.println("writeOneFieldToDisplay: yPos:"+String(yPos)+", chanelNum:"+String(chanelNum)+", form_label_Min_Mid_Max:"+String(form_label_Min_Mid_Max)+", servo_Pwm:"+String(servo_Pwm)+", servoPulseIndex:"+String(servoPulseIndex));
  }

  uint8_t xPos = (((char_shift_x + (form_label_Min_Mid_Max*3)) * char_width_x));
  writeOneFieldToDisplay_innerPart(xPos, chr_point_shift_x, yPos, char_height_y, form_label_Min_Mid_Max, servo_Pwm, chanelNum, showDebug);
}



void writeOneFieldToDisplay_innerPart (uint8_t xPos, uint16_t _chr_point_shift_x, uint8_t yPos, uint16_t _char_height_y, uint16_t form_label_Min_Mid_Max, uint16_t servo_Pwm,uint16_t chanelNum, bool showDebug)
{
  //uint8_t modulo2 = (chanelNum + form_label_Min_Mid_Max)%2;
    //if(modulo2 ==0) 
    //{
      tft.fillRect((xPos + _chr_point_shift_x), yPos, (3*char_width_x)-_chr_point_shift_x, _char_height_y, BLACK);
    //} else {
    //    tft.fillRect((xPos + _chr_point_shift_x), yPos, (3*char_width_x)-_chr_point_shift_x, _char_height_y, WHITE);
    //}
    char numRead3[4];
    dtostrf(servo_Pwm, 4, 0, numRead3);
    tft.drawString(xPos, yPos, numRead3, YELLOW);
}



void loop_servoSet_BTN_Select(unsigned long currentMillis){
  if (currentMillis - previousMillis_BTN_Select >= interval_BTN_Select) {  // start timed event for read and send
    previousMillis_BTN_Select = currentMillis;
    #if defined(ANALOG_POTENTIOMENTERS_READ) || defined (TREE_ENCODERS_ONE_POTENTIOMETER_IN_LINE)
      upButtonState = digitalRead(upButtonPin);
      downButtonState = digitalRead(downButtonPin);

      analogValue_3StateSwitch = analogRead(minMidMAX_SwitchPin);
      if(abs(analogValue_3StateSwitch - prevAnalogValue_3StateSwitch)>20) {
        Serial.println("3-state-switch Button state: analogValue_3StateSwitch =" + String(analogValue_3StateSwitch)+". ");
        prevAnalogValue_3StateSwitch = analogValue_3StateSwitch;
      }

      if(analogValue_3StateSwitch < 100) minMidMAXState = 0;
      else if(analogValue_3StateSwitch > 900) minMidMAXState = 2;
      else minMidMAXState = 1;
    
      if(previousState != minMidMAXState) {
    
        previousState = minMidMAXState;
        Serial.print("3-state-switch Button state: ");
        Serial.println(minMidMAXState);
      }

      //fireBtnState = (analogRead(fireButtonPin )>127 ? LOW : HIGH);
      fireBtnState = digitalRead(fireButtonPin);

      if(fireBtnState == LOW) {
          Serial.println("Button - fire pressed");
      }
      previousFireBtnState = fireBtnState;

    #endif

    if (upButtonState == LOW){
      Serial.print("Button Up pressed. ");
      activeServoSet ++;
      Serial.print("activeServoSet = "+String(activeServoSet)+" ");
      if (activeServoSet >((16/LEFT_ARROW_STEP) - 1)){
        activeServoSet = 0;
        Serial.print("activeServoSet reset.  to val "+String(activeServoSet)+" ");
      }
      Serial.println(" new activeServoSet ="+String(activeServoSet));
      
      tft.fillRect((128-(LEFT_ARROW_SIZE*8)), 0, (LEFT_ARROW_SIZE*8), 160, BLACK);
      tft.drawString((128-(LEFT_ARROW_SIZE*8)), ((activeServoSet * ((2+8) * LEFT_ARROW_STEP))+3), "<", WHITE, LEFT_ARROW_SIZE);
      delay(150);
    }

    if (downButtonState == LOW){
      Serial.print("Button Down pressed. old-activeServoSet ="+String(activeServoSet));
      activeServoSet --;
      if (activeServoSet < 0){
        activeServoSet = 15;
        Serial.println("activeServoSet reset.  to val "+String(activeServoSet)+". ");
      }
      Serial.println(" new activeServoSet ="+String(activeServoSet));
      /*
      if (activeServoSet <= 0){
        activeServoSet = int(15);
        activeServoSet = int(activeServoSet);
        Serial.println("activeServoSet reset.  to val "+String(activeServoSet)+". ");
      } else {
        activeServoSet --;
        activeServoSet =int(activeServoSet);
        Serial.println(" new activeServoSet ="+String(activeServoSet));
      }
      */

      tft.fillRect((128-(LEFT_ARROW_SIZE*8)), 0, (LEFT_ARROW_SIZE*8), 160, BLACK);
      tft.drawString((128-(LEFT_ARROW_SIZE*8)), ((activeServoSet * ((8+2) * LEFT_ARROW_STEP))+3), "<", WHITE, LEFT_ARROW_SIZE);
      delay(150);
    }
    //ToDoHere;
  }
}
