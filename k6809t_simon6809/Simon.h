// 6809 Simon ROM

#include <Arduino.h>
#include <avr/pgmspace.h>

class Simon {

  private:
    const uint16_t *_ROM;

 
  public:
    Simon(void);
    void init();
    unsigned int read(unsigned int address);
    boolean here(unsigned int address);
    void event();  
};
