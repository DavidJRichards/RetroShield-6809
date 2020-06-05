// LCD device driver

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>

class dd_LCD {

  private:
    unsigned int value;
    unsigned int address;
    
    unsigned int command_address;
    unsigned int data_address;
   
    unsigned int statusregister;
    unsigned int dataregister;


  public:
    dd_LCD(unsigned int address);
    void init();
    void write(unsigned int address, unsigned int value);
    unsigned int read(unsigned int address);
    boolean here(unsigned int address);
    void event();
  
};
