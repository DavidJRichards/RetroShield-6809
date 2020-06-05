#include "dd_LCD.h"
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd_(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display

dd_LCD::dd_LCD(unsigned int address) {
  this->command_address=address;
  this->data_address=address+1;
  init();
}

void dd_LCD::init() {
  lcd_.init();                      // initialize the lcd 
  // Print a message to the LCD.
  lcd_.backlight();
//  lcd_.cursor();
  lcd_.blink();
  lcd_.setCursor(1,0);
  lcd_.print("djrm HD63C09EP");
  lcd_.setCursor(2,1);
  lcd_.print("Retro Shield");
}

void dd_LCD::write(unsigned int address, unsigned int value) {
  char tmp[80];
        sprintf(tmp,  "dd LCD WRITE ");
        Serial.print(tmp);
    if (address == this->data_address) {
        sprintf(tmp,  "data %x %x ", address, value);
        Serial.println(tmp);
        lcd_.write(value);
    }
    else
    if(address == this->command_address) {
        sprintf(tmp,  "command %x %x ", address, value);
        Serial.println(tmp);
      switch(value){
        case 0x01:
          lcd_.clear();
          break;
        case 0x05:
//          lcd_.cmd(5);      
//          lcd_.printLeft();
          break;
        case 0x07:
//          lcd_.cmd(value);      
//          lcd_.printRight();
          break;
        case 0xA8:
          lcd_.home();
          break;
        case 0x30:
        case 0x06:
        case 0x0e:
          break;
        default:    
//          lcd_.cmd(value);      
          break;
      }
    }
}

unsigned int dd_LCD::read(unsigned int address) {
  char tmp[80];
    if (address == this->command_address) {
      this->value = 0;
        sprintf(tmp,  "dd LCD READ status %o ", this->value);
        Serial.println(tmp);
      return this->value;      
    }
    else
    return 0; // suppress compiler warning
}

boolean dd_LCD::here(unsigned int address) {
  return((this->data_address == address) || (this->command_address == address));
}

void dd_LCD::event() {
}
