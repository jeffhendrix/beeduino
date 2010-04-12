#include "vdrive2.h"
#include "WProgram.h"
#include <string.h>

#define VDRIVE_DEBUG

#define VDRIVE2_HALF_CLOCK 1 // in microseconds; minimum is 0.08 but must be integer

VDRIVE2::VDRIVE2(int pinCLK, int pinCS, int pinDI, int pinDO)
: pinSPICLK (pinCLK),
	pinSPICS (pinCS),
	pinSPIDI (pinDI),
	pinSPIDO (pinDO)
{
    pinMode(pinSPICLK,OUTPUT);
    digitalWrite(pinSPICLK,LOW);

	pinMode(pinSPIDI,OUTPUT);
    digitalWrite(pinSPIDI,LOW);

	pinMode(pinSPIDO,INPUT);

	pinMode(pinSPICS,OUTPUT);
    digitalWrite(pinSPICS,LOW);
}

VDRIVE2_READ VDRIVE2::read()
{
	struct VDRIVE2_READ ret = {0,0};

	delay(5);

    for(int i=0; i<=12; i++)
    {
      digitalWrite(pinSPICLK,LOW);
      if( i <= 2 )
      {
        digitalWrite(pinSPICS,HIGH);
        digitalWrite(pinSPIDI,(i == 0 || i == 1) ? HIGH : LOW );
      }
      if( i == 12 )
      {
        digitalWrite(pinSPICS,LOW);
      }
      delayMicroseconds(VDRIVE2_HALF_CLOCK);
      digitalWrite(pinSPICLK,HIGH);
      delayMicroseconds(VDRIVE2_HALF_CLOCK);
      if( i >= 3 && i <= 10 )
      {
        ret.value = (ret.value << 1) | ((digitalRead(pinSPIDO) == HIGH) ? 1 : 0);
      }
      if( i == 11 )
      {
        ret.isNew = !digitalRead(pinSPIDO);
      }
    }
    digitalWrite(pinSPICLK,LOW);
    

/*	digitalWrite(pinSPICLK,LOW);
	digitalWrite(pinSPICS,HIGH);
	digitalWrite(pinSPIDI,HIGH);
	delayMicroseconds(VDRIVE2_HALF_CLOCK);
	
	digitalWrite(pinSPICLK,HIGH);
	delayMicroseconds(VDRIVE2_HALF_CLOCK);

	digitalWrite(pinSPICLK,LOW);
	delayMicroseconds(VDRIVE2_HALF_CLOCK);

	digitalWrite(pinSPICLK,HIGH);
	// R/W' = 1 (read)
	// CS = 1 (selected)
	delayMicroseconds(VDRIVE2_HALF_CLOCK);

	digitalWrite(pinSPICLK,LOW);
	digitalWrite(pinSPIDI,LOW);
	delayMicroseconds(VDRIVE2_HALF_CLOCK);

	digitalWrite(pinSPICLK,HIGH);
	// ADD = 0 (data register)
	delayMicroseconds(VDRIVE2_HALF_CLOCK);

	for(int i=0; i<8; i++)
	{
		digitalWrite(pinSPICLK,LOW);
		delayMicroseconds(VDRIVE2_HALF_CLOCK);

		digitalWrite(pinSPICLK,HIGH);
		ret = (ret<<1) | digitalRead(pinSPIDO);
		delayMicroseconds(VDRIVE2_HALF_CLOCK);
	}

	digitalWrite(pinSPICLK,LOW);
	delayMicroseconds(VDRIVE2_HALF_CLOCK);

	digitalWrite(pinSPICLK,HIGH);
	unsigned char status = digitalRead(pinSPIDO);
	delayMicroseconds(VDRIVE2_HALF_CLOCK);

	digitalWrite(pinSPICLK,LOW);
	digitalWrite(pinSPICS,LOW);
	delayMicroseconds(VDRIVE2_HALF_CLOCK);

	digitalWrite(pinSPICLK,HIGH);
	// CS = 0 (unselected)
	delayMicroseconds(VDRIVE2_HALF_CLOCK);

	digitalWrite(pinSPICLK,LOW);
	delayMicroseconds(VDRIVE2_HALF_CLOCK);*/

	ret.value = ret.value >> 1;
#ifdef VDRIVE_DEBUG
	if( ret.isNew )
		Serial.print(ret.value);
#endif

	return ret;
}

unsigned char VDRIVE2::readStatus()
{
	struct VDRIVE2_READ ret = {0,0};


    for(int i=0; i<=12; i++)
    {
      digitalWrite(pinSPICLK,LOW);
      if( i <= 2 )
      {
        digitalWrite(pinSPICS,HIGH);
        digitalWrite(pinSPIDI,(i == 0 || i == 1) ? HIGH : HIGH );
      }
      if( i == 12 )
      {
        digitalWrite(pinSPICS,LOW);
      }
      delayMicroseconds(VDRIVE2_HALF_CLOCK);
      digitalWrite(pinSPICLK,HIGH);
      delayMicroseconds(VDRIVE2_HALF_CLOCK);
      if( i >= 3 && i <= 10 )
      {
        ret.value = (ret.value << 1) | ((digitalRead(pinSPIDO) == HIGH) ? 1 : 0);
      }
      if( i == 11 )
      {
        ret.isNew = !digitalRead(pinSPIDO);
      }
    }
    digitalWrite(pinSPICLK,LOW);

	ret.value = ret.value >> 1;
	return ret.value;
}

 
unsigned char VDRIVE2::write(unsigned char out)
{
#ifdef VDRIVE_DEBUG
    if( out == 0x0d )
      Serial.println();
    else
      Serial.print(out);
#endif
    unsigned char ret = 0;
    for(int i=0; i<=12; i++)
    {
      digitalWrite(pinSPICLK,LOW);
      if( i <= 2 )
      {
        digitalWrite(pinSPICS,HIGH);
        digitalWrite(pinSPIDI,(i == 0) ? HIGH : LOW );
      }
      if( i == 12 )
      {
        digitalWrite(pinSPICS,LOW);
      }
      if( i >= 3 && i <= 10 )
      {
        digitalWrite(pinSPIDI, (out & (1<<(8-(i-3)-1))) ? HIGH : LOW);
      }
      delayMicroseconds(VDRIVE2_HALF_CLOCK);
      digitalWrite(pinSPICLK,HIGH);
      delayMicroseconds(VDRIVE2_HALF_CLOCK);
      if( i == 11 )
      {
        ret = digitalRead(pinSPIDO);
      }
    }
    digitalWrite(pinSPICLK,LOW);
    
	delay(5);

    return ret;
}

unsigned char VDRIVE2::writeString(char* in)
{
	unsigned char ret=0;
	int i=0;
	while( in[i] != '\0' ) {
		ret |= this->write(in[i]);
		i++;
	}
	return ret;
}

int isPrompt(char* in)
{
#ifdef VDRIVE_DEBUG
	Serial.print("Is this a prompt? ");
	Serial.println(in);
#endif
	if( strstr(in,"D:\\>") )
		return 1;
	return 0;
}

void VDRIVE2::clearConsole()
{
	char response[64];
	do
	{
		int i=0;
		//char response[64];
		VDRIVE2_READ vdr = this->read();
		while( vdr.isNew ) {
			response[i++] = vdr.value;
			vdr = this->read();
		}
		response[i] = 0;
	} while( !isPrompt(response) );

#ifdef VDRIVE_DEBUG
	Serial.print("Clear Response: ");
	Serial.println(response);
#endif
}

void VDRIVE2::init()
{
	int i=0;
	char response[32];
	int tries=0;
	VDRIVE2_READ vdr;

	this->writeString("E");
	this->write(0x0d);

get_response:
	i=0;
	vdr = this->read();
	while( vdr.isNew ) {
		response[i++] = vdr.value;
		vdr = this->read();
	}
	response[i] = 0;

	if( i-2 >= 0 && response[i-2] == 'E' && response[i-1] == 0x0d )
	{
#ifdef VDRIVE_DEBUG
		Serial.println("Sync\'ed");
#endif
	}
	else
	{
#ifdef VDRIVE_DEBUG
		Serial.println("Unsync\'ed");
		Serial.println(response);
#endif
		delay(1000);
		tries++;
		if( tries > 4 ) {
#ifdef VDRIVE_DEBUG
			Serial.println("giving up");
#endif
			goto get_response;
//			return;
		}
		goto get_response;
	}

	// use ASCII representations of numbers (command IPA)
	this->writeString("IPA");
	this->write(0x0d);

	delay(500);

	i=0;
	vdr = this->read();
	while( vdr.isNew ) {
		response[i++] = vdr.value;
		vdr = this->read();
	}
	response[i] = 0;

#ifdef VDRIVE_DEBUG
	Serial.print("IPA Response: ");
	Serial.println(response);
#endif

}

int VDRIVE2::fopen(char* input)
{
	this->writeString("OPW ");

    int i=0;
    while( input[i] != 0 ) {
      this->write(input[i++]);
    }
    
    this->write(0x0d);

	delay(500);

	i=0;
	char response[32];
	VDRIVE2_READ vdr = this->read();
	while( vdr.isNew ) {
		response[i++] = vdr.value;
		vdr = this->read();
	}
	response[i] = 0;

#ifdef VDRIVE_DEBUG
	Serial.print("OPW Response: ");
	Serial.println(response);
#endif
}
  
int VDRIVE2::fputs(char* data)
{
    int len = 0;
    while( data[len++] != 0 );
    
	this->writeString("WRF ");
    
    int i =0;
    char usezero=0;
    long temlen = len-1;

	// MESSY itoa function: fails > 99999
    for(int j=10000; j>=1; j/=10) {
      if( temlen/j == 0 )
      {
        if( usezero )
           { this->write('0'); }
      }
      else
      {
        usezero = 1;
        this->write( '0' + (temlen/j) );
        temlen %= j;
      }
    }

    this->write(0x0d);
    
    for(i=0; i<(len-1); i++)
      { this->write( data[i] ); }
      
	delay(10);
	// wait for rts
//#define VDRIVE_IRQREC 
//	while( !(this->readStatus() | VDRIVE_TXE) ) {}

	i=0;
	char response[32];
	VDRIVE2_READ vdr = this->read();
	while( vdr.isNew ) {
		response[i++] = vdr.value;
		vdr = this->read();
	}
	response[i] = 0;

#ifdef VDRIVE_DEBUG
	Serial.print("WRF Response: ");
	Serial.println(response);
#endif

//#ifdef VDRIVE_DEBUG
//    for(int j=0; j<20; j++)
//      Serial.print( this->read().value );
//    Serial.println();
//#endif
  }
  
int VDRIVE2::fclose(char* input)
{
	this->writeString("CLF ");

    int i=0;
    while( input[i] != 0 ) {
      this->write(input[i++]);
    }
    
    this->write(0x0d);

	delay(500);

	i=0;
	char response[32];
	VDRIVE2_READ vdr = this->read();
	while( vdr.isNew ) {
		response[i++] = vdr.value;
		vdr = this->read();
	}
	response[i] = 0;

#ifdef VDRIVE_DEBUG
	Serial.print("CLF Response: ");
	Serial.println(response);
#endif
}