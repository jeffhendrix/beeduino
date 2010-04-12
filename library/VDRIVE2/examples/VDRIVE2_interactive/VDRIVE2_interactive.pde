#include <vdrive2.h>

VDRIVE2 vd(7,8,6,5); // PINS: CLK, CS, DI, DO (DI, DO from perspective of VDRIVE2)

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  VDRIVE2_READ vdr = vd.read();
//  unsigned char stat = vd.readStatus();
//  Serial.print(" ");
//  Serial.print((unsigned int)stat);
//  Serial.print(" ");
  if( vdr.isNew ) {
    if( vdr.value != 0x0d )
      Serial.print(vdr.value);
    else
      Serial.println("");
  }

    
  if( Serial.available() ) {
    unsigned char c = Serial.read();
    if( c == '\n' )
      vd.write( 0x0d );
    else
      vd.write( c );
  }
  delay(100);
}
