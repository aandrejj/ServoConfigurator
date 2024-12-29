struct TX_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  
    uint16_t devType;  // mode:  0 = fourSticksController (8 chanels) ,   1 = ServoConfigurator (16 chanels) , 2 = MinMaxServoConfig (min max for 2 chanels)

    byte servoSet;

    uint16_t s1min;
    uint16_t s1mid;
    uint16_t s1curr;
    uint16_t s1max;

    byte fireBtn1;

    byte switchPos;

    //byte flags;

};

struct RX_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to receive
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  int16_t mode;
  int16_t count;
};

