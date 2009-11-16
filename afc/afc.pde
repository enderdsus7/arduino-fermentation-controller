// Example to read from LM34 temp sensor, simple circuit
// pin 1 to 5V, 2 to analog input, 3 to GND.

#include <SoftwareSerial.h>
#include <X10Firecracker.h>
#include <Fat16.h>
#include <Fat16util.h> // use functions to print strings from flash memory

#include "pin_setup.h"

#define SYNC_INTERVAL 60000 // mills between calls to sync() - 1 minute
                            // 2048 bytes fills the write buffer, but it will
                            // take far too long to get there with our sampling
                            // frequency.
#define LOG_INTERVAL 30000  // mills between writes to SD card -- need to log
                            // for up to two weeks, so limiting writes to save
                            // space on the SD card
uint32_t syncTime = 0;     // time of last sync()

#define SERIAL_DEBUG 1 // serial console output for development

#define LCD_DISPLAY_COUNT 4

//set the pump trip points with room for thermal coasting
#define TOO_WARM 700    // THIS DEPENDS ON OUR YEAST.  MAKE THIS PROGRAMMABLE?
                        // reprogram the board for every new yeast
#define TOO_COOL 620    // THIS DEPENDS ON OUR YEAST.  MAKE THIS PROGRAMMABLE?
                        // reprogram the board for every new yeast
boolean pump_on = false;
int num_toggles = 0;
boolean clearDisplay = false;


SdCard card;
Fat16 file;

SoftwareSerial lcdSerial = SoftwareSerial(LCD_RX, LCD_TX);

// tracks which lcd display we're showing.
unsigned char lcdDisplay = 0;

int tempf[3] = {0};
int sample = 0;
int numSamples = 0;
int maxtemp[3] = {-1000,-1000,-1000};
int mintemp[3] = {1000,1000,1000};
char* thermistorName = "AFC";   //characters to distinguish between thermistors
                                //displayed on the LCD

unsigned long oldtime;
unsigned long startTime;
unsigned long lastTime;
unsigned long lastLog = 0;

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
    sprintf(logstring,"%d,%d,%d,%d,%lu",tempf[0],tempf[1],tempf[2],pump_on,lastLog);
    file.println(logstring);  // fat16write example suggests this is a supported method -- need to check arguments.

#if SERIAL_DEBUG
    Serial.println(logstring);
#endif //only use serial comm for develpment
}

void printLCD(){
    unsigned char localLcdDisplay = lcdDisplay;
    if(clearDisplay){
        lcdSerial.print("?f");  // clear the LCD
        clearDisplay=!clearDisplay;
    }
    if(localLcdDisplay <= 2 ){
        char thermName[2] = {0};
        thermName[0] = thermistorName[localLcdDisplay];

        lcdSerial.print("?x00?y0");
        lcdSerial.print(thermName);

        lcdSerial.print("?x02?y0");
        lcdSerial.print("CUR  MIN   MAX");

        lcdSerial.print("?x01?y1");
        lcdSerial.print(tempf[localLcdDisplay]/10,DEC);
        lcdSerial.print(".");
        lcdSerial.print(tempf[localLcdDisplay]%10,DEC);

        lcdSerial.print("?x06?y1");
        lcdSerial.print(mintemp[localLcdDisplay]/10,DEC);
        lcdSerial.print(".");
        lcdSerial.print(mintemp[localLcdDisplay]%10,DEC);

        lcdSerial.print("?x12?y1");
        lcdSerial.print(maxtemp[localLcdDisplay]/10,DEC);
        lcdSerial.print(".");
        lcdSerial.print(maxtemp[localLcdDisplay]%10,DEC);
    }
    else
    {
        lcdSerial.print("?x00?y0");
        lcdSerial.print("Pump status: ");
        lcdSerial.print(pump_on,DEC);
        lcdSerial.print("?x00?y1");
        lcdSerial.print("Num Cycles: ");
        lcdSerial.print(num_toggles,DEC);
    }
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
        clearDisplay=!clearDisplay;
        if(lcdDisplay >= LCD_DISPLAY_COUNT)
        {
            // wrap which display we show.
            lcdDisplay = 0;
        }

#if SERIAL_DEBUG
        Serial.print("DISP button pushed state ");
        Serial.println(lcdDisplay,DEC);
#endif
    }

    last_interrupt_time = interrupt_time;
}

void TurnPump(boolean control){
    if(control){
        X10.sendCmd( hcD, 1, cmdOn);
    }
    else{
        X10.sendCmd( hcD, 1, cmdOff);
    }
}

void setup(){
#if SERIAL_DEBUG
    Serial.begin(9600);
    Serial.println();
#endif //only use serial comm for develpment 

// TODO want to use this even if no card present -- refactor card/FAT routines
// to not halt on error
    // initialize the SD card
    if (!card.init()) error("card.init");

    // initialize a FAT16 volume
    if (!Fat16::init(card)) error("Fat16::init");

    // create a new file
    char name[] = "LOG00.TXT";
    for (uint8_t i = 0; i < 100; i++) {
        name[3] = i/10 + '0';
        name[4] = i%10 + '0';
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

    startTime = millis();   // use to determine if A/D has settled at startup
    lastTime = startTime;   // used to keep track of sample spacing
    
    pinMode(LCD_TX, OUTPUT); // setup LCD serial comm
    lcdSerial.begin(9600); 

    // only need to do the following ONCE to change brightness -- saving here in
    // case we want to change it again, later
    // lcdSerial.print("?B66");    // set backlight to ff hex, maximum brightness
    //  delay(1000);               // pause to allow LCD EEPROM to program

    lcdSerial.print("?f");  // clear the LCD
    delay(10);              // wait for LCD to finish clearning

    analogReference(INTERNAL); // set A/D to use 1.1V reference instead of 5V for better accuracy

    // init the lcd display button interrupt handler
    attachInterrupt(DISP_BUTTON, displayInterrupt, RISING);

    X10.init(X10RTS, X10DTR, 0);
}

void loop(){

    static int sampleIndex = 0;

    // only poll thermistors every 250mS to get stable average
    if(millis()-250 >= lastTime) {    
        sampleThermistor();
        sampleIndex++;
    }

    // use 8 samples to reduce noise in measured value
    if(sampleIndex>=8) {
        for(int i = 0; i < 3; i++){
            tempf[i] = tempf[i]/sampleIndex;  
            if(tempf[i] > maxtemp[i]){maxtemp[i] = tempf[i];}
            if(tempf[i] < mintemp[i]){mintemp[i] = tempf[i];}
        }
        // only log every LOG_INTERVAL or so to conserve storage space
        if ((millis() - lastLog) >= LOG_INTERVAL){
            lastLog = millis();
            printSD();
            // now that we have new sample, check if pump should be on

            // originally wrote logic with check of pump_on value, also, but decided
            // this will cause system failure if the pump does not turn on/off the
            // first time we pass a trip point -- we do not have a closed loop
            // control.  We should try to turn on/off every time for reassurance
            if(tempf[TEMP2 - 1] >= TOO_WARM) {
                if(!pump_on) {num_toggles++;}
                pump_on = true;
                TurnPump(pump_on);
            }
            else if(tempf[TEMP2 - 1] <= TOO_COOL) {
                if(pump_on) {num_toggles++;}
                pump_on = false;
                TurnPump(pump_on);
            }
        }

        // update the LCD as often as we have data
        printLCD(); 
        for(int i=0; i < 3; i++){
            sampleIndex=0;
            tempf[i] = 0;
        }
    }

    //don't sync too often - requires 2048 bytes of I/O to SD card
    if ((millis() - syncTime) <  SYNC_INTERVAL) return;
    syncTime = millis();
    if (!file.sync()) error("sync");
}
