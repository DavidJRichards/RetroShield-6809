// DEC KY11 Operators Console 

#include "KY11.H"
#include <TM1638.h>
#include <TM1638QYF.h>



extern TM1638QYF* _tm1638;
extern unsigned long deposit_data;

KY11::KY11(unsigned int address,unsigned int value) {
  this->value=value;
  this->address=address;
  init();
}

void KY11::init() {
}

void KY11::write(unsigned int address, unsigned int value) {
  char tmp[80];
        sprintf(tmp,  "op-console WRITE data %o %o ", this->address, this->value);
        Serial.println(tmp);
    if (address == this->address) {
      this->value = value;  
//      _tm1638->setDisplayToDecNumber(value, 1<<5, false);       
      _tm1638->setDisplayToHexNumber(value, 1<<5, false);  
      deposit_data=this->value;     
    }
}

unsigned int KY11::read(unsigned int address) {
  char tmp[80];
    if (address == this->address) {
      this->value = deposit_data;
        sprintf(tmp,  "op-console READ data %o ", this->value);
        Serial.println(tmp);
      return this->value;      
    }
    return 0; // suppress compiler warning
#warning incomplete code
}

boolean KY11::here(unsigned int address) {
  return((this->address == address));
}

void KY11::event() {
}
