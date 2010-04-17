#ifndef __PIN_SETUP_H__
#define __PIN_SETUP_H__

/* pin number aliases */
// analog pins
#define TEMP1 0     // fermentation temp sensor -- lm34 
#define TEMP2 1     // ambient temp -- lm34
#define TEMPCONTROL 5 // wiper of pot for temp setting

// interrupt pins
//#define DISP_BUTTON 1 // LCD display change interrupt pin

// digital pins
// digital pin 2 and pin 3 are interrupt pins
#define LCD_RX 6    // dummy pin for soft serial to 16x2 lcd
#define LCD_TX 5    // xmit pin for soft serial to 16x2 lcd
#define X10DTR 8    // X10 firecracker pin 4 (pin 5 is GND)
#define X10RTS 7    // X10 firecracker pin 7 (pin 5 is GND)
#define LOGSWITCH 9 // DPST for toggling logging

#endif
