/*
command line example for TM1638.
djrm 22 Dec 2018, based on PDP-11/34 console
*/

#include <TM1638.h>
#include <TM1638QYF.h>

// define a module on data pin 18, clock pin 19 and strobe pin 31
//TM1638QYF tm1638(44,45,46);
//TM1638QYF* _tm1638;
extern TM1638QYF tm1638;
extern TM1638QYF* _tm1638;

unsigned char ledbit = 1;
char buf[80];

// global text command buffer used to hold digits from keypad
#define CMDLEN 8        // max no of digits
char cmdline[CMDLEN+1]; // add 1 for NULL terminator;
int cmdindex = 0;       // actual number of digits

// global variables
//#define _BASE 8
//unsigned long load_address = 0777777; // read by DISAD, set by LAD
//unsigned long deposit_data = 0123;    // set by DEP, read by EXAM
//unsigned long load_status = 0567;     // set by LSR
#define _BASE 16
unsigned long load_address = 0xc000; // read by DISAD, set by LAD
unsigned long deposit_data = 0x24;    // set by DEP, read by EXAM
unsigned long load_status = 0x55;     // set by LSR

// lookup table entry used to find ASCII value from button press
/*typedef*/ struct t_keymap {
  const word buttoncode; // binary button code
  const char character;  // ASCII key value
};

// 4x4 keypad single press button values mapped to ASCII characters
#define NUM_KEY_LOOKUP 17
t_keymap const keymap[NUM_KEY_LOOKUP] =
{
  {0x0,  '.'},   // default value when search fails
  {0x1,   'A'},  {0x2,   '7'},  {0x4,   'C'},  {0x8,   'D'},
  {0x10,  'E'},  {0x20,  '4'},  {0x40,  '5'},  {0x80,  '6'},
  {0x100, 'I'},  {0x200, '1'},  {0x400, '2'},  {0x800, '3'},
  {0x1000,'M'},  {0x2000,'0'},  {0x4000,'O'},  {0x8000,'P'},
};

// non numeric buttons have commands associated
#define BTN_DISAD 'A'
#define BTN_EXAM  'C'
#define BTN_DEP   'D'
#define BTN_LAD   'E'
#define BTN_LSR   'I'
#define BTN_CLR   'M'
#define BTN_INIT  'O'
#define BTN_CNTRL 'P'

// search for ACSII equivalent to button press
char lookupkey(word mybutton)
{
  int i;
  for(i = 0; i < NUM_KEY_LOOKUP; i++)
  {
    if(keymap[i].buttoncode == mybutton)
      return keymap[i].character; // key found
  }
  return keymap[0].character;     // default
}      

// add key character to line buffer
int bufferupdate(char mykey)
{
  if (BTN_CLR == mykey)        // clear all key buffer
  {
    cmdindex = 0;
    cmdline[cmdindex] = 0;
  }
  else if (BTN_CNTRL == mykey) // erase last key only
  {
    if(cmdindex > 0)
    {
      cmdindex -= 1;
      cmdline[cmdindex] = 0;
    }
  }
  // add digit 0 to 7 to end of buffer
  else if(isdigit(mykey))
  {
    cmdline[cmdindex] = mykey;
    cmdindex += 1;
    cmdline[cmdindex] = 0;
  }
  else // must be a command key
  {
    return mykey;
  }
  return 0;
}

// right justify command line in display
void displayupdate(char* mycmdline)
{
  int i;
  int j = 0;
  char displaybuffer[CMDLEN+1];  
  for(i = cmdindex; i < CMDLEN; i++)
  {
      displaybuffer[j++] = ' ';
  }
  for(i = 0; i < cmdindex; i++)
  {
      displaybuffer[j++] = mycmdline[i];
  }
  displaybuffer[CMDLEN] = 0;
  _tm1638->setDisplayToString(displaybuffer, 1);   
}

void commandprocess(int mycommand)
{
  switch (mycommand) {
  case BTN_DISAD:
//    _tm1638->setDisplayToDecNumber(load_address, 1<<7,false);
    _tm1638->setDisplayToHexNumber(load_address, 1<<7,false);
    break;
  case BTN_DEP:
    deposit_data = strtol(cmdline, NULL, _BASE);  
    break;
  case BTN_EXAM:
//    _tm1638->setDisplayToDecNumber(deposit_data, 1<<6, false);
    _tm1638->setDisplayToHexNumber(deposit_data, 1<<6, false);
    break;
  case BTN_LAD:
    load_address = strtol(cmdline, NULL, _BASE); 
    break;
  case BTN_LSR:
    load_status = strtol(cmdline, NULL, _BASE); 
    break;
  default:
    displayupdate(cmdline);       
    break;
  }
}

//--------------------------------------------

void panel_update(void) {

  static word prevbutton = 0; // used to prevent button autorepeat
  word buttons;               // actual key code of button pressed
  char keypress;              // ASCII translation 
  int command;                // non numeric command key

  buttons = _tm1638->getButtons();
  if (buttons != 0) // button pressed 
  {
    if (buttons != prevbutton) // if new key press
    {
        keypress = lookupkey(buttons);  // convert to ASCII
        command=bufferupdate(keypress); // add digit to commandline buffer
        commandprocess(command);        // process command button press
    }
    prevbutton=buttons; // save key press for test above
  }
  else
  {
    if(prevbutton != 0)   // only update prevbutton if neccessary
    {
      prevbutton=buttons; // allow new button when finger off button (0)
    }
  }
}

void setu2(void) {
  Serial.begin(115200);
  Serial.println("Command parser");

  _tm1638 = &tm1638;
  _tm1638->setupDisplay(true, 7);
  _tm1638->setDisplayToString("", 1);
  cmdindex = 0;
  cmdline[cmdindex] = 0;
}
