// 6551 ACIA serial
#include "ACIA6551.H"

#define nACIA_DEBUG


ACIA6551::ACIA6551(uint8_t usart,unsigned int baseaddress,unsigned int baudrate) {
  
  this->dataaddress   = baseaddress;        // read rx, write tx
  this->statusaddress   = baseaddress+1;    // read status, write reset
  this->commandaddress  = baseaddress+2;    // read write
  this->controladdress  = baseaddress+3;    // read write
  this->usart = usart;
  if (baudrate > 0)
  {
    switch (this->usart) {
      case 0:
        break;
      case 1:
        this->baudrate=baudrate;
        Serial1.begin(baudrate);
        Serial1.println("Serial 1 ACIA 6551 initialised");
        break;
      case 5:
        this->baudrate=baudrate;
        Serial5.begin(baudrate);
        Serial5.println("Serial 5 ACIA 6551 initialised");
        break;
      default:
        break;
    }
  }
  init();
}

void ACIA6551::init() {
  this->rxdataregister = 0;
  this->txdataregister = 0;
  this->statusregister = (1 << ACIA6551_STATUS_TDRE) ;
  this->controlregister = 0;
  this->commandregister = 0;
}

void ACIA6551::write(unsigned int address, unsigned int value) {
  char tmp[80];
  if (address == this->commandaddress) 
  {
    this->commandregister = value;
    #ifdef ACIA_DEBUG
    sprintf(tmp,  "<command write %02x>", value);
    Serial.println(tmp);
    #endif // ACIA_DEBUG          
  } 
  else if (address == this->controladdress) 
  {
    this->controlregister = value;
    #ifdef ACIA_DEBUG
    sprintf(tmp,  "<control write %02x>", value);
    Serial.println(tmp);
    #endif // ACIA_DEBUG          
  } 
  else if (address == this->statusaddress) 
  {
//    this->controlregister = value;
    #ifdef ACIA_DEBUG
    sprintf(tmp,  "<reset write %02x>", value);
    Serial.println(tmp);
    #endif // ACIA_DEBUG          
  } 
  else if (address == this->dataaddress) 
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

unsigned int ACIA6551::read(unsigned int address) {
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
  if (address == this->controladdress) 
  {
    #ifdef ACIA_DEBUG
    sprintf(tmp,  " control %02x>", this->controlregister);
    Serial.print(tmp);
    #endif // ACIA_DEBUG          
    return this->controlregister;      
  } 
  else if (address == this->commandaddress) 
  {
    #ifdef ACIA_DEBUG
    sprintf(tmp,  " command %02x>", this->commandregister);
    Serial.print(tmp);
    #endif // ACIA_DEBUG          
    return this->commandregister;      
  } 
  else if (address == this->statusaddress) 
  {
    #ifdef ACIA_DEBUG
    sprintf(tmp,  " status %02x>", this->statusregister);
    Serial.print(tmp);
    #endif // ACIA_DEBUG          
    return this->statusregister;      
  } 
  else if (address == this->dataaddress) 
  {
    #ifdef ACIA_DEBUG
    sprintf(tmp,  " data %02x>", this->rxdataregister);
    Serial.print(tmp);
    #endif // ACIA_DEBUG          
    this->statusregister = this->statusregister &~(1<<ACIA6551_STATUS_RDRF);   
    return this->rxdataregister;      
  } 
  #warning address error code here?    
  Serial.println("address error");
  return 0;// suppress compiler warning
}

boolean ACIA6551::here(unsigned int address) {
  return((this->statusaddress == address) || (this->dataaddress == address) || (this->commandaddress == address) || (this->controladdress == address) );
}

void ACIA6551::event() {
  char tmp[80];
  switch (this->usart)
  {
    case 1:
      if(Serial1.availableForWrite()) {
        this->statusregister = this->statusregister | (1<<ACIA6551_STATUS_TDRE); 
      } else {
        this->statusregister = this->statusregister &~(1<<ACIA6551_STATUS_TDRE);
      }

      if( (Serial1.available()) && !(this->statusregister & (1<<ACIA6551_STATUS_RDRF))) {
        this->rxdataregister = Serial1.read();
        #ifdef ACIA_DEBUG
        sprintf(tmp,  "[usart 1 receive data %02x] ", this->rxdataregister);
        Serial.print(tmp);
        #endif // ACIA_DEBUG          
        this->statusregister = this->statusregister | (1<<ACIA6551_STATUS_RDRF);
      }
      break;
      
    case 5:
      if( (Serial5.availableForWrite())) {
        this->statusregister = this->statusregister | (1<<ACIA6551_STATUS_TDRE);
      } else {
        this->statusregister = this->statusregister &~(1<<ACIA6551_STATUS_TDRE);
      }

      if( (Serial5.available()) && !(this->statusregister & (1<<ACIA6551_STATUS_RDRF))) {
        #ifdef ACIA_DEBUG
        sprintf(tmp,  "[usart 5 receive data %02x]", this->rxdataregister);
        Serial.print(tmp);
        #endif // ACIA_DEBUG          
        this->rxdataregister = Serial5.read();
        this->statusregister = this->statusregister | (1<<ACIA6551_STATUS_RDRF);
      }
      break;
      
    default:
      break;
  }
}
