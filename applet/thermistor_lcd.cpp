  // Example to read from LM34 temp sensor, simple circuit
  // pin 1 to 5V, 2 to analog input, 3 to GND.
  
  #include <SoftwareSerial.h>
  
  #define TEMP1 1
  #define rxPin 4 // any unused pin number is fine
  #define BUTTON 6
  #define transistor 7 // output for transistor switch
  #define txPin 14 // analog pin 0
  
  #include "WProgram.h"
void setup();
void printTTY();
void printLCD();
void loop();
SoftwareSerial mySerial = SoftwareSerial(rxPin, txPin);
  
  int tempf = 0;
  int samples[8];
  int maxtemp = -1000;
  int mintemp = 1000;
  int i = 0;
  
  int val = 0;
  int old_val = 0;
  int state = 0;
  
  unsigned long startTime;
  unsigned long lastTime;
  
  void setup(){
    startTime = millis();
    lastTime = startTime;
    pinMode(transistor, OUTPUT);
    digitalWrite(transistor, LOW);
    
    pinMode(BUTTON, INPUT);
    
    pinMode(txPin, OUTPUT);
    mySerial.begin(9600); // setup LCD comm
    
   // mySerial.print("?B66");    // set backlight to ff hex, maximum brightness
   //  delay(1000);                // pause to allow LCD EEPROM to program
    
    mySerial.print("?f");
    delay(10);
    
    Serial.begin(9600); // setup tty comm
    analogReference(INTERNAL); // set A/D to use 1.1V reference instead of 5V for better accuracy
  }
  
  void printTTY(){
    Serial.print(tempf,DEC);
    Serial.print(" Fahrenheit -> ");
    Serial.print(maxtemp,DEC);
    Serial.print(" Max, ");
    Serial.print(mintemp,DEC);
    Serial.println(" Min ");
  }
  
  void printLCD(){
    if(millis()-lastTime > 1000){
      mySerial.print("?x00?y0");
      mySerial.print("CUR   MIN   MAX");
  
      mySerial.print("?x00?y1");
      mySerial.print(tempf/10,DEC);
      mySerial.print(".");
      mySerial.print(tempf%10,DEC);
  
      mySerial.print("?t");
      
      mySerial.print(mintemp/10,DEC);
      mySerial.print(".");
      mySerial.print(mintemp%10,DEC);
    
      mySerial.print("?t");
      
      mySerial.print(maxtemp/10,DEC);
      mySerial.print(".");
      mySerial.print(maxtemp%10,DEC);
      
      lastTime = millis();
    }
  }
    
  
  void loop(){
    for(i = 0; i < 8; i++){
      samples[i] = ( analogRead(TEMP1) * 1.1 * 1000 ) / 1024; // temp in deg F
      // temp is read out in 10mV/degree F.  We are using analog reference of 1.1V.
      tempf = tempf + samples[i];
  //    delay(250);
    }
    
    val = digitalRead(BUTTON);
    
    if(val == HIGH && old_val == LOW) {
      state = 1 - state;
      delay(20);
    }
    
    tempf = tempf/8.0;
  
    if(millis() > 6000) {
    if(tempf > maxtemp){maxtemp = tempf;}
    if(tempf < mintemp){mintemp = tempf;}
  
    printTTY();
    printLCD();
    }
    
    old_val = val;
    
    if (state == 1) {
       digitalWrite(transistor, HIGH);
    } else {
      digitalWrite(transistor, LOW);
    }
    
    tempf = 0;
  
  //  delay(500);
  }

int main(void)
{
	init();

	setup();
    
	for (;;)
		loop();
        
	return 0;
}

