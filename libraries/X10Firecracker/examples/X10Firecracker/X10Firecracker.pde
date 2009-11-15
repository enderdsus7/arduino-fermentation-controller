#include <X10Firecracker.h>

void setup()
{
  X10.init( 2, 3, 0 );
}


void Test()
{
  X10.sendCmd( hcA, 2, cmdOn );
  X10.sendCmd( hcA, 2, cmdDim );
  X10.sendCmd( hcA, 2, cmdDim );
  X10.sendCmd( hcA, 2, cmdDim );
  X10.sendCmd( hcA, 2, cmdDim );
  X10.sendCmd( hcA, 2, cmdDim );
  X10.sendCmd( hcA, 2, cmdBright );
  X10.sendCmd( hcA, 2, cmdBright );
  X10.sendCmd( hcA, 2, cmdBright );
  X10.sendCmd( hcA, 2, cmdBright );
  X10.sendCmd( hcA, 2, cmdBright );
  X10.sendCmd( hcA, 2, cmdOff );
}

void loop()
{
  delay(2000);
  Test();
  delay(8000);
}