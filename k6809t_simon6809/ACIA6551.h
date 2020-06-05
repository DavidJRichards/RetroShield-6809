// 6551 ACIA serial

#include <Arduino.h>

// Bit definitions for status registers
#define ACIA6551_STATUS_PE    0
#define ACIA6551_STATUS_FE    1
#define ACIA6551_STATUS_OR    2
#define ACIA6551_STATUS_RDRF  3
#define ACIA6551_STATUS_TDRE  4
#define ACIA6551_NDCD         5
#define ACIA6551_NDSR         6
#define ACIA6551_IRQ          7


class ACIA6551 {

  private:
    unsigned int dataaddress;
    unsigned int statusaddress;
    unsigned int commandaddress;
    unsigned int controladdress;
    
    unsigned int baudrate;
    
    unsigned int txdataregister;
    unsigned int rxdataregister;
    unsigned int statusregister;
    unsigned int commandregister;
    unsigned int controlregister;

    uint8_t usart;

  public:
    ACIA6551(uint8_t usart,unsigned int baseaddress, unsigned int baudrate);
    void init();
    void write(unsigned int address, unsigned int value);
    unsigned int read(unsigned int address);
    boolean here(unsigned int address);
    void event();
  
};
