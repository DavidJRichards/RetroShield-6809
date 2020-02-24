#include <TM1638.h>
#include <TM1638QYF.h>


extern unsigned char ledbit;
extern char buf[80];

// global text command buffer used to hold digits from keypad
#define CMDLEN 8        // max no of digits
extern char cmdline[CMDLEN+1]; // add 1 for NULL terminator;
extern int cmdindex;       // actual number of digits

// global variables
extern unsigned long load_address; // read by DISAD, set by LAD
extern unsigned long deposit_data;    // set by DEP, read by EXAM
extern unsigned long load_status;     // set by LSR


extern void panel_update(void);
