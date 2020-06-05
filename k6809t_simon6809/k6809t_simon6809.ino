
////////////////////////////////////////////////////////////////////
// RetroShield 6809 for Teensy 3.5
// Simon6809
//
// 2019/01/23
// Version 1.0

// The MIT License (MIT)

// Copyright (c) 2019 Erturk Kocalar, 8Bitforce.com

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

// Date         Comments                                            Author
// -----------------------------------------------------------------------------
// 9/29/2019    Bring-up on Teensy 3.5.                             Erturk
// 19 Jan 2019  Add Reset and NMI buttons                           DJRM
// 26 May 2020  Add ACIA 6850

//  File dataFile = SD.open("ef09.bin");
//  File dataFile = SD.open("figF09.bin");
//  #define DATAFILE "basic2.bin"
#define DATAFILE "monitor.bin"

#define nWRITE_ROM

#define TM1638_DISPLAY
#define OLED_DISPLAY

#include <TimerOne.h>
#include <SD.h>
#include <SPI.h>

#ifdef OLED_DISPLAY
#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"

// 0X3C+SA0 - 0x3C or 0x3D
#define I2C_ADDRESS 0x3C

// Define proper RST_PIN if required.
#define RST_PIN -1

SSD1306AsciiWire oled;
#endif // OLED_DISPLAY

const int chipSelect = BUILTIN_SDCARD;

#ifdef TM1638_DISPLAY
#include <TM1638.h>
#include <TM1638QYF.h>
#include "keypad.h"
//#include "GString.h"
// define a module on data pin 18, clock pin 19 and strobe pin 31
TM1638QYF tm1638(44,45,46);
TM1638QYF* _tm1638;
word mode;
unsigned long startTime;
#endif
#include "KY11.h"
#define KY11_BASE         0xff78 //0xC000 //0xFF78 //0177570       // KY11-LB Switch Register
KY11 operatorconsole(KY11_BASE,1);

//#include "ACIA6551.h"
#include "ACIA6850.h"
//#define CONSOLE_BASE      0xC400 // ACIA base address
#define CONSOLE_BASE      0xFFD0 // ACIA base address
ACIA6850 console(1,CONSOLE_BASE,115200);
//M7856 tu58(5,TU58_BASE,9600);


#include "dd_LCD.h"
dd_LCD lcd(0xc800);
////////////////////////////////////////////////////////////////////
// Options
//   outputDEBUG: Print memory access debugging messages.
////////////////////////////////////////////////////////////////////
#define outputDEBUG     0

////////////////////////////////////////////////////////////////////
// 6809 DEFINITIONS
////////////////////////////////////////////////////////////////////
// MEMORY
// 32K MEMORY
#define RAM_START   0x0000
#define RAM_END     0x7FFF
byte    RAM[RAM_END-RAM_START+1];
byte    *pRAM = RAM;
extern const byte simon09_bin[];


// ROMs
#define ROM_START   0xE000
#define ROM_END     0xFFFF
byte    ROM[ROM_END-ROM_START+1];
byte    *pROM = ROM;
#include "rom_eprom.h"



////////////////////////////////////////////////////////////////////
// 6809 Processor Control
////////////////////////////////////////////////////////////////////
//
// #define GPIO?_PDOR    (*(volatile uint32_t *)0x400FF0C0) // Port Data Output Register
// #define GPIO?_PSOR    (*(volatile uint32_t *)0x400FF0C4) // Port Set Output Register
// #define GPIO?_PCOR    (*(volatile uint32_t *)0x400FF0C8) // Port Clear Output Register
// #define GPIO?_PTOR    (*(volatile uint32_t *)0x400FF0CC) // Port Toggle Output Register
// #define GPIO?_PDIR    (*(volatile uint32_t *)0x400FF0D0) // Port Data Input Register
// #define GPIO?_PDDR    (*(volatile uint32_t *)0x400FF0D4) // Port Data Direction Register

/* Digital Pin Assignments */

// read bits raw
#define xDATA_DIR_IN()      (GPIOD_PDDR = (GPIOD_PDDR & 0xFFFFFF00))
#define xDATA_DIR_OUT()     (GPIOD_PDDR = (GPIOD_PDDR | 0x000000FF))
#define SET_DATA_OUT(D) (GPIOD_PDOR = (GPIOD_PDOR & 0xFFFFFF00) | (D))
#define xDATA_IN        ((byte) (GPIOD_PDIR & 0xFF))

// Teensy has an LED on its digital pin13 (PTC5). which interferes w/
// level shifters.  So we instead pick-up A5 from PTA5 port and use
// PTC5 for PG0 purposes.
//
#define ADDR_H          ((word) (GPIOA_PDIR & 0b1111000000100000))
#define ADDR_L          ((word) (GPIOC_PDIR & 0b0000111111011111))
#define ADDR            ((word) (ADDR_H | ADDR_L))

#define MEGA_PD7  (24)
#define MEGA_PG0  (13)
#define MEGA_PG1  (16)
#define MEGA_PG2  (17)
#define MEGA_PB0  (28)
#define MEGA_PB1  (39)
#define MEGA_PB2  (29)
#define MEGA_PB3  (30)

#define uP_RESET_N  MEGA_PD7
#define uP_RW_N     MEGA_PG1
#define uP_FIRQ_N   MEGA_PG0
#define uP_GPIO     MEGA_PG2
#define uP_IRQ_N    MEGA_PB3
#define uP_NMI_N    MEGA_PB2
#define uP_E        MEGA_PB1
#define uP_Q        MEGA_PB0

#define BUTTON_P  A21
#define BUTTON_C  A22

// Fast routines to drive signals high/low; faster than digitalWrite
//
#define CLK_E_HIGH          (GPIOA_PSOR = 0x20000)
#define CLK_E_LOW           (GPIOA_PCOR = 0x20000)
#define CLK_Q_HIGH          (GPIOA_PSOR = 0x10000)
#define CLK_Q_LOW           (GPIOA_PCOR = 0x10000)
#define STATE_RW_N          ((byte) (GPIOB_PDIR & 0x01) )

unsigned long clock_cycle_count;

word          uP_ADDR;
byte          uP_DATA;


void uP_init()
{
  // Set directions for ADDR & DATA Bus.
  
  byte pinTable[] = {
    5,21,20,6,8,7,14,2,     // D7..D0
    27,26,4,3,38,37,36,35,  // A15..A8
    12,11,25,10,9,23,22,15  // A7..A0
  };
  for (int i=0; i<24; i++)
  {
    pinMode(pinTable[i],INPUT);
  } 
  
  pinMode(uP_RESET_N, OUTPUT);
  pinMode(uP_RW_N, INPUT_PULLUP);
  pinMode(uP_FIRQ_N, OUTPUT);
  pinMode(uP_GPIO, OUTPUT);
  pinMode(uP_IRQ_N, OUTPUT);
  pinMode(uP_NMI_N, OUTPUT);
  pinMode(uP_E, OUTPUT);
  pinMode(uP_Q, OUTPUT);
  
  uP_assert_reset();
  digitalWrite(uP_E, LOW);
  digitalWrite(uP_Q, LOW);

  clock_cycle_count = 0;
}
  
void uP_assert_reset()
{
  // Drive RESET conditions
  digitalWrite(uP_RESET_N, LOW);
  digitalWrite(uP_FIRQ_N, HIGH);
  digitalWrite(uP_GPIO, LOW);
  digitalWrite(uP_IRQ_N, HIGH);
  digitalWrite(uP_NMI_N, HIGH);
}

void uP_release_reset()
{
  // Drive RESET conditions
  digitalWrite(uP_RESET_N, HIGH);
}

////////////////////////////////////////////////////////////////////
// Processor Control Loop
////////////////////////////////////////////////////////////////////

volatile byte DATA_OUT;
volatile byte DATA_IN;

#if outputDEBUG
#define DELAY_FACTOR() delayMicroseconds(1000)
#define DELAY_FACTOR_H()DELAY_FACTOR()
#define DELAY_FACTOR_L() DELAY_FACTOR()
#else
#define DELAY_FACTOR_H() asm volatile("nop\nnop\nnop\nnop\n");
#define DELAY_FACTOR_L() asm volatile("nop\nnop\nnop\nnop\n");
#endif

inline __attribute__((always_inline))
void cpu_tick()
{    

  //////////////////////////////////////////////////////////////
  CLK_Q_HIGH;           // digitalWrite(uP_Q, HIGH);   // CLK_Q_HIGH;  // PORTB = PORTB | 0x01;     // Set Q = high
  DELAY_FACTOR_H();
  CLK_E_HIGH;           // digitalWrite(uP_E, HIGH);    // PORTB = PORTB | 0x02;     // Set E = high
  // DELAY_FACTOR_H();
  
  uP_ADDR = ADDR;

  CLK_Q_LOW;            // digitalWrite(uP_Q, LOW);    // CLK_Q_LOW; // PORTB = PORTB & 0xFE;     // Set Q = low
  // DELAY_FACTOR_L();

  if (STATE_RW_N)      // Check R/W
  //////////////////////////////////////////////////////////////
  // R/W = high = READ?
  {
    // DATA_DIR = DIR_OUT;
    xDATA_DIR_OUT();

    if (console.here(uP_ADDR)) {
      DATA_OUT = console.read(uP_ADDR);
    } 
    else 
    if (lcd.here(uP_ADDR))
    {
      DATA_OUT = lcd.read(uP_ADDR);
    }
    else 
    // ROM?
#ifdef WRITE_ROM    
    if ( (ROM_START <= uP_ADDR) && (uP_ADDR <= ROM_END) )
      DATA_OUT = pROM [ (uP_ADDR - ROM_START) ];
    else
#endif    
    if ( (ROM_START <= uP_ADDR) && (uP_ADDR <= ROM_END) )
      DATA_OUT = pROM [ (uP_ADDR - ROM_START) ];
    else 
    // RAM?
    if ( (RAM_START <= uP_ADDR) && (uP_ADDR <= RAM_END) )
      DATA_OUT = RAM[uP_ADDR - RAM_START];
    else
    // FTDI?
    if (uP_ADDR == 0xD000)
      DATA_OUT = FTDI_Read();
    else 
    if (console.here(uP_ADDR)) {
      DATA_OUT = console.read(uP_ADDR);
    } 
    else 
    if (operatorconsole.here(uP_ADDR)) 
    {
      DATA_OUT = operatorconsole.read(uP_ADDR);
    }
    
    else
      DATA_OUT = 0xFF;

    // Start driving the databus out
    SET_DATA_OUT( DATA_OUT );
    
#if outputDEBUG
    char tmp[40];
    sprintf(tmp, "-- A=%04X D=%02X\n", uP_ADDR, DATA_OUT);
    Serial1.write(tmp);
#endif

  } 
  else 
  //////////////////////////////////////////////////////////////
  // R/W = low = WRITE?
  {
    DATA_IN = xDATA_IN;
    
    if (console.here(uP_ADDR)) 
    {
      console.write(uP_ADDR,DATA_IN);
    }
    else 
    if (lcd.here(uP_ADDR))
    {
      lcd.write(uP_ADDR,DATA_IN);
    }
    else    
    // ROM?
#ifdef WRITE_ROM
    if ( (ROM_START <= uP_ADDR) && (uP_ADDR <= ROM_END) )
      ROM[uP_ADDR - ROM_START] = DATA_IN;
    else
#endif    
    // RAM?
    if ( (RAM_START <= uP_ADDR) && (uP_ADDR <= RAM_END) )
      RAM[uP_ADDR - RAM_START] = DATA_IN;
    else
    // FTDI?
    if (uP_ADDR == 0xD000)
      Serial.write(DATA_IN);
    else
    if (console.here(uP_ADDR)) 
    {
      console.write(uP_ADDR,DATA_IN);
    } 
    else    
    if (operatorconsole.here(uP_ADDR)) 
    {
      operatorconsole.write(uP_ADDR,DATA_IN);
    }

#if outputDEBUG
    char tmp[40];
    sprintf(tmp, "WR A=%04X D=%02X\n", uP_ADDR, DATA_IN);
    Serial1.write(tmp);
#endif
  }

  //////////////////////////////////////////////////////////////
  // cycle complete
  
  CLK_E_LOW;    // digitalWrite(uP_E, LOW);      // CLK_E_LOW; // PORTB = PORTB & 0xFC;     // Set E = low
  
  // natural delay for data hold
  xDATA_DIR_IN();

  // DELAY_FACTOR_L();

#if (outputDEBUG)  
  clock_cycle_count ++;
#endif
}

////////////////////////////////////////////////////////////////////
// Serial Functions
////////////////////////////////////////////////////////////////////

/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so any delay inside loop will delay
 response.  
 Warning: Multiple bytes of data may be available.
 */
 
inline __attribute__((always_inline))
void serialEvent0() 
{
  // If serial data available, assert FIRQ so process can grab it.
  if (Serial.available())
      digitalWrite(uP_FIRQ_N, LOW);
  return;
}

inline __attribute__((always_inline))
byte FTDI_Read()
{
  byte x = Serial.read();
  // If no more characters left, deassert FIRQ to the processor.
  if (Serial.available() == 0)
      digitalWrite(uP_FIRQ_N, HIGH);
  return x;
}

// This function is not used by the timer loop
// because it takes too long to call a subrouting.
inline __attribute__((always_inline))
void FTDI_Write(byte x)
{
  Serial.write(x);
}

////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////
// LCD/Button Shield functions from vendor
////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////
// Button Press callbacks
////////////////////////////////////////////////////////////////////


void btn_Pressed_Up()
{
  Serial.println("Up");

  // deassert reset
  //digitalWrite(uP_RESET_N, HIGH);
}

void btn_Pressed_Down()
{
  Serial.println("Down");

  // reset microprocessor
  //digitalWrite(uP_RESET_N, LOW);
  // flush serial port
  while (Serial.available() > 0)
    Serial.read();
}

void process_buttons()
{
  static int Plast=1;
  static int Clast=1;
  int Pval,Cval;
  // Handle key presses
  //
#if 1  
    Pval = analogRead(BUTTON_P);
    if(Pval>500) // button not pressed
    {
      if(Plast == 0)
      {
        Plast = 1;
        //digitalWrite(LED_BUILTIN,0);      
        //btn_Pressed_Up();
        uP_release_reset();

      }
    }
    else  // button pressed
    {
      if(Plast == 1)
      {
         Plast = 0;
         //digitalWrite(LED_BUILTIN,1);    
         //btn_Pressed_Down();  
          uP_assert_reset();
          for(int i=0;i<25;i++) cpu_tick();  
      }
    }
#endif    
  //
    Cval = analogRead(BUTTON_C);
    if(Cval>500) // button not pressed
    {
      if(Clast == 0)
      {
        Clast = 1;
        //digitalWrite(LED_BUILTIN,0);      
        //btn_Pressed_Up();
        digitalWrite(uP_NMI_N, HIGH);

      }
    }
    else  // button pressed
    {
      if(Clast == 1)
      {
         Clast = 0;
         //digitalWrite(LED_BUILTIN,1);    
         //btn_Pressed_Down();  
        digitalWrite(uP_NMI_N, LOW);
        for(int i=0;i<25;i++) cpu_tick();  
      }
    }
 
 
}


// 
// interruptEventNMI - generates NMI pulse periodically
////////////////////////////////////////////////////////////////////
#define CONST_NMI_TIMER_STARTUP_DELAY 1000000
#define CONST_NMI_TIMER_CYCLE_COUNT 1000
#define NMI_PULSE_WIDTH 10

inline __attribute__((always_inline))
void interruptEventNMI()
{
  // Set initially to start-up delay so NMI does not
  // fire immediately before SW has time to set up
  // any RAM pointers (if used)
  static long cycleCount = CONST_NMI_TIMER_STARTUP_DELAY;

  cycleCount--;
  if (cycleCount == 0)
  {
    if (digitalRead(uP_NMI_N) == true)     // NMI# was high
    {
      // High -> Low edge
      digitalWrite(uP_NMI_N, LOW);
      cycleCount = NMI_PULSE_WIDTH;
      // Serial.print("*");
    } else {                             // NMI# was low
      // Low -> High edge
      digitalWrite(uP_NMI_N, HIGH);
      cycleCount = CONST_NMI_TIMER_CYCLE_COUNT;      
      // Serial.print("-");
    }
  }
}



void FIRQpulse(void)
{
//12        digitalWrite(uP_IRQ_N, LOW);
        digitalWrite(uP_GPIO, HIGH);
        for(int i=0;i<25;i++) cpu_tick(); 
        digitalWrite(uP_GPIO, LOW);
        digitalWrite(uP_IRQ_N, HIGH);
        
}


////////////////////////////////////////////////////////////////////
// Setup
////////////////////////////////////////////////////////////////////

void setup() 
{
  unsigned long i;
  unsigned char buff;
  unsigned char * data;
#ifdef OLED_DISPLAY  
  Wire.begin();
  Wire.setClock(400000L);
  #if RST_PIN >= 0
  oled.begin(&Adafruit128x64, I2C_ADDRESS, RST_PIN);
  #else // RST_PIN >= 0
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
  #endif // RST_PIN >= 0
#endif // OLED_DISPLAY

#ifdef TM1638_DISPLAY
  startTime = millis();
  _tm1638 = &tm1638;
  tm1638.setupDisplay(true, 7);
  tm1638.setDisplayToString("", 1);
  cmdindex = 0;
  cmdline[cmdindex] = 0;
  mode = 0;
#endif

#ifdef OLED_DISPLAY
  oled.setFont(X11fixed7x14);
  oled.clear();
  oled.set2X();
  oled.println("6809 CPU");
  oled.set2X();
  oled.println("1234 ABCD");
//  oled.println("5678 90EF");
#endif // OLED_DISPLAY

  Serial.begin(115200);
#if outputDEBUG
  Serial1.begin(115200);
#endif
  while (!Serial);

  Serial.write(27);       // ESC command
  Serial.print("[2J");    // clear screen command
  Serial.write(27);
  Serial.print("[H");
  Serial.println("\n");
  Serial.println("Configuration:");
  Serial.println("==============");
  Serial.print("Debug:      "); Serial.println(outputDEBUG, HEX);
  Serial.print("SRAM Size:  "); Serial.print(RAM_END - RAM_START + 1, DEC); Serial.println(" Bytes");
  Serial.print("SRAM_START: 0x"); Serial.println(RAM_START, HEX); 
  Serial.print("SRAM_END:   0x"); Serial.println(RAM_END, HEX); 

  Serial.println();
  Serial.println("Initializing SD card...");
  
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    //return;
  }
 // Serial.println("card initialized.");

 #if 0 
  File romFile = SD.open("monitor.bin");
  // if the file is available, read it:
  if (romFile) {
    i=0x7800;
    while (romFile.available()) {
      romFile.read(&buff,1);
      RAM[i]=buff;
      i++;
    }
    romFile.close();
    Serial.println(RAM[0x7ffe]);
    Serial.println(RAM[0x7fff]);
    Serial.print(i-0x7800);
    Serial.println(" bytes monitor rom read from file");   
//    pROM = ROM; 
  }
//  else
#endif  
  {
    pROM = (byte*) simon09_bin; // read only, declared const
  }

#if 1
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open(DATAFILE);
  // if the file is available, read it:
  if (dataFile) {
    i=0x400;
    while (dataFile.available()) {
      dataFile.read(&buff,1);
      RAM[i]=buff;
      i++;
    }
    dataFile.close();
    Serial.print(i);
    Serial.println(" bytes application data read from file");    
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening file, using default");
    // copy forth image to ram
    data=(unsigned char*)eprom;
    for(i=eprom_start; i<=eprom_finish; i++)
    {
      RAM[i]=*data++;
    }
  } 
#endif

  uP_init();
  pinMode(BUTTON_P, INPUT);
  pinMode(BUTTON_C, INPUT);
  //pinMode(LED_BUILTIN, OUTPUT);
  //digitalWrite(LED_BUILTIN,1);      

//  Timer1.initialize(150000);
//  Timer1.attachInterrupt(blinkLED); // blinkLED to run every 0.15 seconds
  
  // Reset processor
  uP_assert_reset();
  for(int i=0;i<25;i++) cpu_tick();  
  uP_release_reset();

  delay(500);
  Serial.println("\n");
#ifdef OLED_DISPLAY  
  oled.clear();
#endif //OLED_DISPLAY  
}

////////////////////////////////////////////////////////////////////
// Loop
////////////////////////////////////////////////////////////////////

void loop()
{
    static long lastMillis = 0;
    static int cycles = 0;

  word j = 0;

  // Loop forever
  //  
  while(1)
  {

   long currentMillis = millis();

    serialEvent0();
    
  // Check for ACIA 6850 state changes
    console.event();
    
    cpu_tick();
    cycles++;

    if (j-- == 0)
    {
      process_buttons();
      Serial.flush();
      j = 500;
    }

#if 1
    if (currentMillis - lastMillis > 100)
//if(uP_ADDR<0xE000)
//delay(50);
    {

#ifdef OLED_DISPLAY
      char tmp[20];
      oled.home();
//      oled.println("CPS:");
//      oled.println(cycles*10);
      sprintf(tmp, "A=%04X\n\rD=%02X", uP_ADDR, DATA_OUT);
      oled.print(tmp);
#endif // OLED_DISPLAY    
      
#ifdef TM1638_DISPLAY
      panel_update();
#endif
      lastMillis = currentMillis;
      cycles = 0;
      //digitalWrite(uP_IRQ_N, HIGH);
    }
#endif
        
  }
}


void blinkLED(void)
{
  static  int ledState = LOW;

  if (ledState == LOW) {
    ledState = HIGH;
//    blinkCount = blinkCount + 1;  // increase when LED turns on
  } else {
    ledState = LOW;
  }
  digitalWrite(LED_BUILTIN, ledState);
}
