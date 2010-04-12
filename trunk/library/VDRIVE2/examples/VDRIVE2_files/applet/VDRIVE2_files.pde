#define VDRIVE2_DEBUG

#include <vdrive2.h>
#include <stdio.h>

VDRIVE2 vd(10,11,12,9);

void setup()
{
  Serial.begin(9600);
  
  vd.init();
  
  vd.fopen("Me.txt");
  int val;
  char d[2048];
  for(int i=0; i<2048; i++) { d[i] = i%10; if( i > 0 ) d[i-1] = d[i]; }
  int k=0;
  /*for(int i=0;i<100;i++)
  {
    val = analogRead(0);
    // MESSY itoa function: fails > 99999
    d[0] = '0' + val/1000;
    val %= 1000;
    d[1] = '0' + val/100;
    val %= 100;
    d[2] = '0' + val/10;
    val %= 10;
    d[3] = '0' + val;
    d[4] = ',';
    d[5] = 0;
    vd.fputs(d);
//    delay(1);
  }*/
  vd.fclose("Me.txt");
}

void loop()
{

}
