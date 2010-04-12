#define VDRIVE2_DEBUG

#include <vdrive2.h>
#include <stdio.h>

VDRIVE2 vd(10,11,12,9);

void setup()
{
  Serial.begin(9600);
  
  vd.init();
  
  vd.fopen("Me.txt");
  vd.fputs("me me me");
  vd.fclose("Me.txt");
}

void loop()
{

}
