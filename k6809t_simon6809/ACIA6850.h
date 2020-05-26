// 6850 ACIA serial

#include <Arduino.h>

class ACIA6850 {

  private:
    unsigned int statusaddress;
    unsigned int rxdataaddress;
    unsigned int controladdress;
    unsigned int txdataaddress;
    unsigned int baudrate;
    
    unsigned int statusregister;
    unsigned int rxdataregister;
    unsigned int controlregister;
    unsigned int txdataregister;

    uint8_t usart;

  public:
    ACIA6850(uint8_t usart,unsigned int baseaddress, unsigned int baudrate);
    void init();
    void restart(unsigned int value);
    void write(unsigned int address, unsigned int value);
    unsigned int read(unsigned int address);
    boolean here(unsigned int address);
    void event();
  
};
