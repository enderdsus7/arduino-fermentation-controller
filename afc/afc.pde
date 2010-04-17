// Example to read from LM34 temp sensor, simple circuit
// pin 1 to 5V, 2 to analog input, 3 to GND.

#include <SoftwareSerial.h>
#include <X10Firecracker.h>
#include <Fat16.h>
#include <Fat16util.h> // use functions to print strings from flash memory
#include "pin_setup.h"

#define SERIAL_DEBUG 1 // serial console output for development

//############# FAT SETUP ################
#define SYNC_INTERVAL 60000 // mills between calls to sync() - 1 minute
                            // 2048 bytes fills the write buffer, but it will
                            // take far too long to get there with our sampling
                            // frequency.
#define LOG_INTERVAL 30000  // mills between writes to SD card -- need to log
                            // for up to two weeks, so limiting writes to save
                            // space on the SD card
uint32_t syncTime = 0;     // time of last sync()
SdCard card;
Fat16 file;

//############# SETUP CONSTANTS ################
#define MENU 1

//set the pump trip points with room for thermal coasting
// should read from eeprom and use last known value.
//#define TOO_WARM 700    // THIS DEPENDS ON OUR YEAST.  MAKE THIS PROGRAMMABLE?
//#define TOO_COOL 620    // THIS DEPENDS ON OUR YEAST.  MAKE THIS PROGRAMMABLE?


//############# SETUP VARIABLES ################
int setPoint = 68;
int setPointSample = 0;
int upState = 0;
int downState = 0;
int TOO_WARM = 70;
int TOO_COOL = 66;
boolean cooler_on= false;
int num_toggles = 0;
boolean clearDisplay = false;

unsigned char lcdDisplay = 0;
boolean menuPressed = false;
boolean maintenanceMode = false;
boolean logging = false;
unsigned char menuIndex = 0;

int tempf[2] = {0};
int sample = 0;
int numSamples = 0;

unsigned long oldtime;
unsigned long startTime;
unsigned long lastTime;
unsigned long lastLog = 0;

char logstring[30];


//############# SETUP LCD SERIAL ################
SoftwareSerial lcdSerial = SoftwareSerial(LCD_RX, LCD_TX);



// store error strings in flash to save RAM
#define error(s) error_P(PSTR(s))
void error_P(const char *str)
{
    PgmPrint("error: ");
    SerialPrintln_P(str);
    while(1);
}


//############# PRINT FUNCITONS ################
/*************************************************************************
 *  Write to debug serial port
 ************************************************************************/
void printTTY(){
    for(int i=0; i<3; i++){
        Serial.print(tempf[i],DEC);
        Serial.print(" Fahrenheit -> ");
    }
}


/*************************************************************************
 *  Write to SD card
 ************************************************************************/
void printSD(){
    sprintf(logstring,"%d,%d,%d,%d,%lu",tempf[0],tempf[1],tempf[2],cooler_on,lastLog);
    file.println(logstring);  // fat16write example suggests this is a supported method -- need to check arguments.

#if SERIAL_DEBUG
    Serial.println(logstring);
#endif //only use serial comm for develpment
}


/*************************************************************************
 *  Write temps to LCD
 ************************************************************************/
void printLCD(){
    unsigned char localLcdDisplay = lcdDisplay;
    if(clearDisplay){
        lcdSerial.print("?f");  // clear the LCD
        clearDisplay=!clearDisplay;
    }
    if(localLcdDisplay == 0 ){

        lcdSerial.print("?x00?y0");
        lcdSerial.print("AIR");

        lcdSerial.print("?x06?y0");
        lcdSerial.print("FERM");

        lcdSerial.print("?x12?y0");
        lcdSerial.print("SET");

        lcdSerial.print("?x00?y1");
        lcdSerial.print(tempf[1]/10,DEC);
        lcdSerial.print(".");
        lcdSerial.print(tempf[1]%10,DEC);

        lcdSerial.print("?x06?y1");
        lcdSerial.print(tempf[0]/10,DEC);
        lcdSerial.print(".");
        lcdSerial.print(tempf[0]%10,DEC);

        lcdSerial.print("?x12?y1");
        lcdSerial.print(setPoint,DEC);
    }
}

/*************************************************************************
 *  Samples the thermistors
 ************************************************************************/
void sampleThermistor()
{
    for(int i = 0; i < 2; i++){
        sample = ( analogRead(TEMP1+i) * 1.068 * 1000 ) / 1024; // temp in deg F
        // TODO change to shift -- C. Sklnd - 11-15-09
        // temp is read out in 10mV/degree F.  We are using analog reference of 1.1V.
        tempf[i] = tempf[i] + sample;
    }
    lastTime = millis();
    //delay(250);
}


/*************************************************************************
 *  Read the temperature controller set point
 ************************************************************************/
void getSetPoint()
{
    setPointSample += 32 + ( analogRead(TEMPCONTROL) * 1.1 * 50 ) / 1024; // temp in deg F
    //setPointSample += 28 + analogRead(TEMPCONTROL)/16;
    //delay(250);
}


/*************************************************************************
 *  LCD display switch interrupt handler
 *     Permits menu navigation 
 ************************************************************************/
void displayInterrupt()
{
    static unsigned long last_interrupt_time = 0;
    unsigned long interrupt_time = millis();

    // If interrupts come faster than 200ms, assume it's a bounce and ignore
    if (interrupt_time - last_interrupt_time > 250)
    {
        menuPressed=true;

#if SERIAL_DEBUG
        Serial.println("Menu Button Pressed");
#endif
    }

    last_interrupt_time = interrupt_time;
}


/*************************************************************************
 *  Toggles Cooler State
 ************************************************************************/
void TurnCooler(boolean control){
    if(control){
        X10.sendCmd( hcD, 2, cmdOn);
    }
    else{
        X10.sendCmd( hcD, 2, cmdOff);
    }
}


/*************************************************************************
 *  Toggles Cooler State
 ************************************************************************/
void initializeSD(){
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
}


void setup(){
#if SERIAL_DEBUG
    Serial.begin(9600);
    Serial.println();
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
    //attachInterrupt(DISP_BUTTON, displayInterrupt, RISING);

    // setup firecracker
    X10.init(X10RTS, X10DTR, 0);

    // setup navigation buttons
    pinMode(LOGSWITCH, INPUT);
    logging=digitalRead(LOGSWITCH);

    if(logging) {initializeSD();};
}



void loop(){

    static int sampleIndex = 0;

    // only poll thermistors every 250mS to get stable average
    if(millis()-250 >= lastTime) {    
        sampleThermistor();
        getSetPoint(); 
        sampleIndex++;
    }
    
    // use 8 samples to reduce noise in measured value
    if(sampleIndex>=8) {
        for(int i = 0; i < 2; i++){
            tempf[i] = tempf[i]/sampleIndex;  
        }
        setPoint = setPointSample/sampleIndex;
        // only log every LOG_INTERVAL or so to conserve storage space
        if ((millis() - lastLog) >= LOG_INTERVAL){
            lastLog = millis();
            if(logging){printSD();};
            // now that we have new sample, check if pump should be on

            // originally wrote logic with check of cooler_on value, also, but decided
            // this will cause system failure if the pump does not turn on/off the
            // first time we pass a trip point -- we do not have a closed loop
            // control.  We should try to turn on/off every time for reassurance
            TOO_WARM = 10*(1.5 + setPoint);
            TOO_COOL = 10*(setPoint - 1.5);
            if(tempf[TEMP1] >= TOO_WARM) {
                cooler_on = true;
                TurnCooler(cooler_on);
            }
            else if(tempf[TEMP1] <= TOO_COOL) {
                cooler_on = false;
                TurnCooler(cooler_on);
            }
        }

        // update the LCD as often as we have data
        printLCD(); 
        for(int i=0; i < 3; i++){
            tempf[i] = 0;
        }
        sampleIndex=0;
        setPointSample=0;
    }
 
    //don't sync too often - requires 2048 bytes of I/O to SD card
    if(logging){
        if ((millis() - syncTime) <  SYNC_INTERVAL) return;
        syncTime = millis();
        if (!file.sync()) error("sync");
    };
}
