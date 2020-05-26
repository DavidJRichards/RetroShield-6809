// 6850 ACIA serial
#include "ACIA6850.H"

#define nACIA_DEBUG

// Bit definitions for status registers
#define ACIA6850_STATUS_RDRF 0
#define ACIA6850_STATUS_TDRE 1

ACIA6850::ACIA6850(uint8_t usart,unsigned int baseaddress,unsigned int baudrate) {
  
  this->statusaddress   = baseaddress;    // read only
  this->controladdress  = baseaddress;    // write only
  this->rxdataaddress   = baseaddress+1;  // read only
  this->txdataaddress   = baseaddress+1;  // write only
  this->usart = usart;
  if (baudrate > 0)
  {
    switch (this->usart) {
      case 0:
        break;
      case 1:
        this->baudrate=baudrate;
        Serial1.begin(baudrate);
        Serial1.println("Serial 1 initialised");
        break;
      case 5:
        this->baudrate=baudrate;
        Serial5.begin(baudrate);
        Serial5.println("Serial 5 initialised");
        break;
      default:
        break;
    }
  }
  init();
}

void ACIA6850::init() {
  this->rxdataregister = 0;
  this->txdataregister = 0;
  this->statusregister = (1 << ACIA6850_STATUS_TDRE) ;
  this->controlregister = 0;
}

void ACIA6850::write(unsigned int address, unsigned int value) {
  char tmp[80];
  if (address == this->controladdress) 
  {
    this->controlregister = value;
    #ifdef ACIA_DEBUG
    sprintf(tmp,  "<command write %02x>", value);
    Serial.println(tmp);
    #endif // ACIA_DEBUG          
  } 
  else if (address == this->txdataaddress) 
  {
    this->txdataregister = value;
    switch (this->usart) {
      case 1:
        Serial1.write(this->txdataregister);
        #ifdef ACIA_DEBUG
        sprintf(tmp,  "<usart 1 tx data write %02x>", this->txdataregister);
        Serial.print(tmp);
        #endif // ACIA_DEBUG          
        break;
      case 5: 
        Serial5.write(this->txdataregister);
        sprintf(tmp,  "<usart 5 tx data write %02x>", this->txdataregister);
        Serial.print(tmp);
        break;
      default:
        break;
    }
  }
}

unsigned int ACIA6850::read(unsigned int address) {
  char tmp[80];
  switch (this->usart) {
    case 1:
      #ifdef ACIA_DEBUG
      sprintf(tmp,  "<usart 1 read");
      Serial.print(tmp);
      #endif // ACIA_DEBUG          
      break;
    case 5: 
      sprintf(tmp,  "<usart 5 read");
      Serial.print(tmp);
      break;
    default:
      Serial.print("<usart ?>");
      break;
  }
  if (address == this->statusaddress) 
  {
    #ifdef ACIA_DEBUG
    sprintf(tmp,  " status %02x>", this->statusregister);
    Serial.print(tmp);
    #endif // ACIA_DEBUG          
    return this->statusregister;      
  } 
  else if (address == this->rxdataaddress) 
  {
    #ifdef ACIA_DEBUG
    sprintf(tmp,  " data %02x>", this->rxdataregister);
    Serial.print(tmp);
    #endif // ACIA_DEBUG          
    this->statusregister = this->statusregister &~(1<<ACIA6850_STATUS_RDRF);   
    return this->rxdataregister;      
  } 
  #warning address error code here?    
  Serial.println("address error");
  return 0;// suppress compiler warning
}

boolean ACIA6850::here(unsigned int address) {
  return((this->statusaddress == address) || (this->rxdataaddress == address) || (this->txdataaddress == address) || (this->controladdress == address) );
}

void ACIA6850::event() {
  char tmp[80];
  switch (this->usart)
  {
    case 1:
      if(Serial1.availableForWrite()) {
        this->statusregister = this->statusregister | (1<<ACIA6850_STATUS_TDRE); 
      } else {
        this->statusregister = this->statusregister &~(1<<ACIA6850_STATUS_TDRE);
      }

      if( (Serial1.available()) && !(this->statusregister & (1<<ACIA6850_STATUS_RDRF))) {
        this->rxdataregister = Serial1.read();
        #ifdef ACIA_DEBUG
        sprintf(tmp,  "[usart 1 receive data %02x] ", this->rxdataregister);
        Serial.print(tmp);
        #endif // ACIA_DEBUG          
        this->statusregister = this->statusregister | (1<<ACIA6850_STATUS_RDRF);
      }
      break;
      
    case 5:
      if( (Serial5.availableForWrite())) {
        this->statusregister = this->statusregister | (1<<ACIA6850_STATUS_TDRE);
      } else {
        this->statusregister = this->statusregister &~(1<<ACIA6850_STATUS_TDRE);
      }

      if( (Serial5.available()) && !(this->statusregister & (1<<ACIA6850_STATUS_RDRF))) {
        #ifdef ACIA_DEBUG
        sprintf(tmp,  "[usart 5 receive data %02x]", this->rxdataregister);
        Serial.print(tmp);
        #endif // ACIA_DEBUG          
        this->rxdataregister = Serial5.read();
        this->statusregister = this->statusregister | (1<<ACIA6850_STATUS_RDRF);
      }
      break;
      
    default:
      break;
  }
}
