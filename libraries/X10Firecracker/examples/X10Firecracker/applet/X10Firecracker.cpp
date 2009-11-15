#include <X10Firecracker.h>

#include "WProgram.h"
void setup();
void Test();
void loop();
void setup()
{
  X10.init( 12, 13, 0 );
}


void Test()
{
  X10.sendCmd( hcD, 1, cmdOn );
  X10.sendCmd( hcD, 1, cmdOff );
}

void loop()
{
  delay(2000);
  Test();
  delay(8000);
}

int main(void)
{
	init();

	setup();
    
	for (;;)
		loop();
        
	return 0;
}

