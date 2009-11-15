  // Example to read from LM34 temp sensor, simple circuit
  // pin 1 to 5V, 2 to analog input, 3 to GND.
  //
  // Switched to Fat16lib 11/2009
  
  #include <SoftwareSerial.h>
  #include <Fat16.h>
  #include <Fat16util.h> // use functions to print strings from flash memory

  #include "pin_setup.h"

  #define SYNC_INTERVAL 1000 // mills between calls to sync()
  uint32_t syncTime = 0;     // time of last sync()
  
  #define SERIAL_DEBUG 1 // serial console output for development
  
  SdCard card;
  Fat16 file;
  
  SoftwareSerial lcdSerial = SoftwareSerial(LCD_RX, LCD_TX);
  
  int tempf = 0;
  int samples[8];
  int maxtemp = -1000;
  int mintemp = 1000;
  unsigned long oldtime;
  int i = 0;
  
  
  unsigned long startTime;
  unsigned long lastTime;
  
  char logstring[30];
  int result=0;
  
  // store error strings in flash to save RAM
  #define error(s) error_P(PSTR(s))
  void error_P(const char *str)
  {
    PgmPrint("error: ");
    SerialPrintln_P(str);
    while(1);
  }
  
  void printTTY(){
    Serial.print(tempf,DEC);
    Serial.print(" Fahrenheit -> ");
    Serial.print(maxtemp,DEC);
    Serial.print(" Max, ");
    Serial.print(mintemp,DEC);
    Serial.println(" Min ");
  }
  
  void printSD(){
    sprintf(logstring,"%d,%lu",tempf,lastTime);
    file.println(logstring);  // fat16write example suggests this is a supported method -- need to check arguments.
  }
  
  void printLCD(){
      lcdSerial.print("?x00?y0");
      lcdSerial.print("CUR   MIN   MAX");
  
      lcdSerial.print("?x00?y1");
      lcdSerial.print(tempf/10,DEC);
      lcdSerial.print(".");
      lcdSerial.print(tempf%10,DEC);
  
      lcdSerial.print("?t");
      
      lcdSerial.print(mintemp/10,DEC);
      lcdSerial.print(".");
      lcdSerial.print(mintemp%10,DEC);
    
      lcdSerial.print("?t");
      
      lcdSerial.print(maxtemp/10,DEC);
      lcdSerial.print(".");
      lcdSerial.print(maxtemp%10,DEC);
  }
  
  void setup(){
    // initialize the SD card
    if (!card.init()) error("card.init");
  
    // initialize a FAT16 volume
    if (!Fat16::init(card)) error("Fat16::init");
  
    // create a new file
    char name[] = "LOGGER00.CSV";
    for (uint8_t i = 0; i < 100; i++) {
      name[6] = i/10 + '0';
      name[7] = i%10 + '0';
      // O_CREAT - create the file if it does not exist
      // O_EXCL - fail if the file exists
      // O_WRITE - open for write only
      if (file.open(name, O_CREAT | O_EXCL | O_WRITE))break;
    }
    if (!file.isOpen()) error ("create");
    #if SERIAL_DEBUG
      PgmPrint("Logging to: ");
      Serial.println(name);
    #endif //only use serial comm for develpment
    
    startTime = millis();
    lastTime = startTime;
    
    pinMode(BUTTON, INPUT);
    
    pinMode(LCD_TX, OUTPUT);
    lcdSerial.begin(9600); // setup LCD comm
    
   // lcdSerial.print("?B66");    // set backlight to ff hex, maximum brightness
   //  delay(1000);                // pause to allow LCD EEPROM to program
    
    lcdSerial.print("?f");
    delay(10);
    
    Serial.begin(9600); // setup tty comm
    analogReference(INTERNAL); // set A/D to use 1.1V reference instead of 5V for better accuracy
  }
  
  void loop(){
      
  if(millis()-250 >= lastTime) {    
    samples[i] = ( analogRead(TEMP1) * 1.1 * 1000 ) / 1024; // temp in deg F
    // temp is read out in 10mV/degree F.  We are using analog reference of 1.1V.
    tempf = tempf + samples[i];
    lastTime = millis();
    i++;
  //delay(250);
  }
    
    
    if(i>=8) {
      tempf = tempf/(i);  
      if(lastTime-startTime > 6000) {
        if(tempf > maxtemp){maxtemp = tempf;}
        if(tempf < mintemp){mintemp = tempf;}
        printSD();
        printLCD(); 
      }
      i=0;
      tempf = 0;
    }
    
    
  //don't sync too often - requires 2048 bytes of I/O to SD card
  if ((millis() - syncTime) <  SYNC_INTERVAL) return;
  syncTime = millis();
  if (!file.sync()) error("sync");
  //  delay(500);
  }
