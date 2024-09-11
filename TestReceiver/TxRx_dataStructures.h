struct RX_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  
    int16_t mode;  

    uint16_t s00;
    uint16_t s01;
    uint16_t s02;
    uint16_t s03;
    uint16_t s04;
    uint16_t s05;
    uint16_t s06;
    uint16_t s07;
    uint16_t s08;
    uint16_t s09;
    uint16_t s10;
    uint16_t s11;
    uint16_t s12;
    uint16_t s13;
    uint16_t s14;
    uint16_t s15;
};

struct TX_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to receive
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  int16_t mode;
  int16_t count;
};

