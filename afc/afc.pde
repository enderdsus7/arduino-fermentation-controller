// Example to read from LM34 temp sensor, simple circuit
// pin 1 to 5V, 2 to analog input, 3 to GND.
//
// Switched to Fat16lib 11/2009

#include <SoftwareSerial.h>
#include <Fat16.h>
#include <Fat16util.h> // use functions to print strings from flash memory
#include <X10Firecracker.h>

#include "pin_setup.h"

#define SYNC_INTERVAL 1000 // mills between calls to sync()
uint32_t syncTime = 0;     // time of last sync()

#define SERIAL_DEBUG 1 // serial console output for development

#define LCD_DISPLAY_COUNT 3

SdCard card;
Fat16 file;

SoftwareSerial lcdSerial = SoftwareSerial(LCD_RX, LCD_TX);

// tracks which lcd display we're showing.
unsigned char lcdDisplay = 0;

int tempf[3] = {0};
int sample = 0;
int numSamples = 0;
int maxtemp[3] = {-1000};
int mintemp[3] = {1000};

unsigned long oldtime;
unsigned long startTime;
unsigned long lastTime;

char logstring[30];

// store error strings in flash to save RAM
#define error(s) error_P(PSTR(s))
void error_P(const char *str)
{
    PgmPrint("error: ");
    SerialPrintln_P(str);
    while(1);
}

void printTTY(){
    for(int i=0; i<3; i++){
        Serial.print(tempf[i],DEC);
        Serial.print(" Fahrenheit -> ");
        Serial.print(maxtemp[i],DEC);
        Serial.print(" Max, ");
        Serial.print(mintemp[i],DEC);
        Serial.println(" Min ");
    }
}

void printSD(){
    sprintf(logstring,"%d,%d,%d,%lu",tempf[0],tempf[1],tempf[2],lastTime);
    file.println(logstring);  // fat16write example suggests this is a supported method -- need to check arguments.

#if SERIAL_DEBUG
    Serial.println(logstring);
#endif //only use serial comm for develpment
}

void printLCD(){
    lcdSerial.print("?x00?y0");
    lcdSerial.print("CUR   MIN   MAX");

    lcdSerial.print("?x00?y1");
    lcdSerial.print(tempf[1]/10,DEC);
    lcdSerial.print(".");
    lcdSerial.print(tempf[1]%10,DEC);

    lcdSerial.print("?t");

    lcdSerial.print(mintemp[1]/10,DEC);
    lcdSerial.print(".");
    lcdSerial.print(mintemp[1]%10,DEC);

    lcdSerial.print("?t");

    lcdSerial.print(maxtemp[1]/10,DEC);
    lcdSerial.print(".");
    lcdSerial.print(maxtemp[1]%10,DEC);
}

void sampleThermistor()
{
    for(int i = 0; i < 3; i++){
        sample = ( analogRead(TEMP1+i) * 1.1 * 1000 ) / 1024; // temp in deg F
        // TODO change to shift -- C. Sklnd - 11-15-09
        // temp is read out in 10mV/degree F.  We are using analog reference of 1.1V.
        tempf[i] = tempf[i] + sample;
    }
    lastTime = millis();
    //delay(250);
}

/*************************************************************************
 *  LCD display switch interrupt handler
 *      Changes which output is shown on the LCD
 ************************************************************************/
void displayInterrupt()
{
    static unsigned long last_interrupt_time = 0;
    unsigned long interrupt_time = millis();

    // If interrupts come faster than 200ms, assume it's a bounce and ignore
    if (interrupt_time - last_interrupt_time > 200)
    {
        lcdDisplay++;
        if(lcdDisplay > LCD_DISPLAY_COUNT)
        {
            // wrap which display we show.
            lcdDisplay = 0;
        }

#if SERIAL_DEBUG
        Serial.println("DISP button pushed");
#endif
    }

    last_interrupt_time = interrupt_time;
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

    pinMode(LCD_TX, OUTPUT);
    lcdSerial.begin(9600); // setup LCD comm

    // lcdSerial.print("?B66");    // set backlight to ff hex, maximum brightness
    //  delay(1000);                // pause to allow LCD EEPROM to program

    lcdSerial.print("?f");
    delay(10);

    Serial.begin(9600); // setup tty comm
    analogReference(INTERNAL); // set A/D to use 1.1V reference instead of 5V for better accuracy

    // init the lcd display button interrupt handler
    attachInterrupt(DISP_BUTTON, displayInterrupt, RISING);
    
}

void loop(){

    int sampleIndex = 0;

    if(millis()-250 >= lastTime) {    
        sampleThermistor();
        sampleIndex++;
    }


    if(sampleIndex>=8) {
        for(int i = 0; i < 3; i++){
            tempf[i] = tempf[i]/sampleIndex;  
            if(lastTime-startTime > 6000) {
                if(tempf[i] > maxtemp[i]){maxtemp[i] = tempf[i];}
                if(tempf[i] < mintemp[i]){mintemp[i] = tempf[i];}
                printSD();
                printLCD(); 
            }
            sampleIndex=0;
            tempf[i] = 0;
        }
    }


    //don't sync too often - requires 2048 bytes of I/O to SD card
    if ((millis() - syncTime) <  SYNC_INTERVAL) return;
    syncTime = millis();
    if (!file.sync()) error("sync");
    //  delay(500);
}
