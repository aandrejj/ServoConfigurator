struct TX_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  
    int16_t devType;  // mode:  0 = fourSticksController (8 chanels) ,   1 = ServoConfigurator (16 chanels) , 2 = MinMaxServoConfig (min max for 2 chanels)

    byte servoSet;

    byte s1min;
    byte s1mid;
    byte s1curr;
    byte s1max;

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

