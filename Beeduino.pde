/****************************************************

Authors: Kevin Savidge and Spencer Miles

Description: Beeduino Main Program

At initialization, sets a time on the datetime-keeper that is defined below.

Every x minutes (x to be determined in-situ), a call to read_sensors_and_log is fired. The read_sensors_and_log
function does this:

1) Turns on sensors
2) Gathers 256 samples of audio, stores in AtMega RAM
3) Gathers the temperature in degrees Fahrenheit from the MAX6675
4) Gathers the ADC value of the load cell circuit output and converts ADC value to a weight
5) Gathers the date and time from the DS1307
6) Opens a file on the USB flash drive (via the VDRIVE2 module)
7) Outputs all the data gathered above to the USB flash drive
8) Closes the file
9) Turns off sensors

Inputs: Audio data, temperature, weight, and datetime information

Outputs: A file on a USB flash drive, named by the datetime, that contains all the above inputs

Pin Connections:

MAX6675 and VDRIVE2 pins are defined below (they are Arduino digital pins).
DS1307 must be connected to analog pins 4 (SDA) and analog pin 5 (SCL)
Analog pin 0 is connected to an electret microphone through a bandpass, 100x op-amp.
Analog pins 1-3 are connected to load cells through an amplifier circuit.

******************************************************/

///////////////////////////////////////////////////////////////////////////////
// include libraries for VDRIVE2 module and MAX6675 module
//

#include <vdrive2.h>
#include <MAX6675.h>
#include <string.h>
#include <Wire.h>

///////////////////////////////////////////////////////////////////////////////
// Define BEE_DEBUG to produce verbose output to console
// Note: _DO NOT_ define BEE_DEBUG if using digital pins 0/1 for something else (e.g., cell phone)

#define BEE_DEBUG


///////////////////////////////////////////////////////////////////////////////
// Initial datetime used by Arduino to start the datetime-keeper chip (DS1307)
//

/*#define SETUP_TIMESET_YEAR 1990
#define SETUP_TIMESET_MONTH 1
#define SETUP_TIMESET_DAY 5
#define SETUP_TIMESET_HOUR 12
#define SETUP_TIMESET_MINUTE 0
#define SETUP_TIMESET_SECOND 0*/


///////////////////////////////////////////////////////////////////////////////
// Pin-outs/Connections that may be changed
// Note, all values are Arduino _digital_ pins 

#define SENSORS_ENABLE_PIN 9

// SDO and SDI relative to VDRIVE Module (_NOT_ Arduino)
#define VDRIVE2_SDO 5
#define VDRIVE2_SDI 6
#define VDRIVE2_CS 8
#define VDRIVE2_SCLK 7

#define MAX6675_S0 2
#define MAX6675_CS 3
#define MAX6675_SCK 4


///////////////////////////////////////////////////////////////////////////////
// Audio definitions and functions
//

// Set and Clear bit macros
// (used by audio recording to get better performance from AtMega uC)
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))


// Function record() puts NUM_SAMPLES audio samples (fs=19500Hz) into array d

#define NUM_SAMPLES 256
unsigned char d[NUM_SAMPLES];
int num_d=0;

void record();


///////////////////////////////////////////////////////////////////////////////
// DS1307 datetime functions and definitions
//

/*#define DS1307_I2C_ADDRESS 0x68  // This is the I2C address of the DS1307
byte second, myminute, hour, dayOfWeek, dayOfMonth, month, year; // "temp" variables

// sets the date
void setDateDs1307(char iyear, char imonth, char idayOfMonth, char idayOfWeek, char ihour, char iminute, char isecond);

// gets the datetime from DS1307 and stores it in above "temp" variables
void getDateDs1307();*/

// creates a filename from the above "temp" variables (must call getDateDs1307 first for valid result)
// placed here for ease-of-change
int sequence_num=1;
char file[16];
void make_filename(char* filename)
{
  filename[0] = '0'+ sequence_num/10000000;
  filename[1] = '0'+ (sequence_num%10000000)/1000000;
  filename[2] = '0'+ (sequence_num%1000000)/100000;
  filename[3] = '0'+(sequence_num%100000)/10000;
  filename[4] = '0'+(sequence_num%10000)/1000;
  filename[5] = '0'+(sequence_num%1000)/100;
  filename[6] = '0'+(sequence_num%100)/10;
  filename[7] = '0'+(sequence_num%10);
  filename[8] = '.'; filename[9] = 'T'; filename[10] = 'X'; filename[11] = 'T'; filename[12] = 0;
  sequence_num+=1;
}


///////////////////////////////////////////////////////////////////////////////
// Weight functions: conversion routine for ADC value to an actual weight value
//

//unsigned int ADC_to_weight_value(unsigned int ADC)
//{
//  return ADC;
//}

///////////////////////////////////////////////////////////////////////////////
// Float output routines
//
void fmtDouble(double val, byte precision, char *buf, unsigned bufLen = 0xffff);
unsigned fmtUnsigned(unsigned long val, char *buf, unsigned bufLen = 0xffff, byte width = 0);

//
// Produce a formatted string in a buffer corresponding to the value provided.
// If the 'width' parameter is non-zero, the value will be padded with leading
// zeroes to achieve the specified width.  The number of characters added to
// the buffer (not including the null termination) is returned.
//
unsigned
fmtUnsigned(unsigned long val, char *buf, unsigned bufLen, byte width)
{
  if (!buf || !bufLen)
    return(0);

  // produce the digit string (backwards in the digit buffer)
  char dbuf[10];
  unsigned idx = 0;
  while (idx < sizeof(dbuf))
  {
    dbuf[idx++] = (val % 10) + '0';
    if ((val /= 10) == 0)
      break;
  }

  // copy the optional leading zeroes and digits to the target buffer
  unsigned len = 0;
  byte padding = (width > idx) ? width - idx : 0;
  char c = '0';
  while ((--bufLen > 0) && (idx || padding))
  {
    if (padding)
      padding--;
    else
      c = dbuf[--idx];
    *buf++ = c;
    len++;
  }

  // add the null termination
  *buf = '\0';
  return(len);
}

//
// Format a floating point value with number of decimal places.
// The 'precision' parameter is a number from 0 to 6 indicating the desired decimal places.
// The 'buf' parameter points to a buffer to receive the formatted string.  This must be
// sufficiently large to contain the resulting string.  The buffer's length may be
// optionally specified.  If it is given, the maximum length of the generated string
// will be one less than the specified value.
//
// example: fmtDouble(3.1415, 2, buf); // produces 3.14 (two decimal places)
//
void
fmtDouble(double val, byte precision, char *buf, unsigned bufLen)
{
  if (!buf || !bufLen)
    return;

  // limit the precision to the maximum allowed value
  const byte maxPrecision = 6;
  if (precision > maxPrecision)
    precision = maxPrecision;

  if (--bufLen > 0)
  {
    // check for a negative value
    if (val < 0.0)
    {
      val = -val;
      *buf = '-';
      bufLen--;
    }

    // compute the rounding factor and fractional multiplier
    double roundingFactor = 0.5;
    unsigned long mult = 1;
    for (byte i = 0; i < precision; i++)
    {
      roundingFactor /= 10.0;
      mult *= 10;
    }

    if (bufLen > 0)
    {
      // apply the rounding factor
      val += roundingFactor;

      // add the integral portion to the buffer
      unsigned len = fmtUnsigned((unsigned long)val, buf, bufLen);
      buf += len;
      bufLen -= len;
    }

    // handle the fractional portion
    if ((precision > 0) && (bufLen > 0))
    {
      *buf++ = '.';
      if (--bufLen > 0)
        buf += fmtUnsigned((unsigned long)((val - (unsigned long)val) * mult), buf, bufLen, precision);
    }
  }

  // null-terminate the string
  *buf = '\0';
} 



///////////////////////////////////////////////////////////////////////////////
// Instantiate sensor classes
//

MAX6675 temp_sensor(MAX6675_CS,MAX6675_S0,MAX6675_SCK,0,-10.0); // MAX6675 - pinout then fahrenheit (0) and no error compensation (0.0)
VDRIVE2 usb_data_log(VDRIVE2_SCLK,VDRIVE2_CS,VDRIVE2_SDI,VDRIVE2_SDO);



///////////////////////////////////////////////////////////////////////////////
// Function: setup
// Description: 
//   Initializes the serial port (if BEE_DEBUG is defined)
//   Initializes the timekeeper DS1307 with the start datetime

void setup()
{

#ifdef BEE_DEBUG
  Serial.begin(9600);
  Serial.println("Hello from setup()");
#endif

  // DS1307 (Timekeeper) setup
//  Wire.begin();
//  setDateDs1307(10,10,01,1,12,10,0);
  
  pinMode(SENSORS_ENABLE_PIN,OUTPUT);
  
  // power off sensors
  digitalWrite(SENSORS_ENABLE_PIN,1);
  delay(5000);
}

///////////////////////////////////////////////////////////////////////////////
// Function: loop
// Description: Infinite loop executed by Arduino platform
//   calls read_sensors_and_log, then waits a minute

void loop()
{
  read_sensors_and_log();
  delay(60000);
}


///////////////////////////////////////////////////////////////////////////////
// Function: read_sensors_and_log
// Description: Does the meat of the Beeduino work. This function (copied from intro):
//   1) Turns on sensors
//   2) Gathers 256 samples of audio, stores in AtMega RAM
//   3) Gathers the temperature in degrees Fahrenheit from the MAX6675
//   4) Gathers the ADC value of the load cell circuit output and converts ADC value to a weight
//   5) Gathers the date and time from the DS1307
//   6) Opens a file on the USB flash drive (via the VDRIVE2 module)
//   7) Outputs all the data gathered above to the USB flash drive
//   8) Closes the file
//   9) Turns off sensors

// temp variables
float temperature;
unsigned int weight0, weight1, weight2;

void read_sensors_and_log()
{
#ifdef BEE_DEBUG
  Serial.println("Hello from read_sensors_and_log()");
#endif
  
  // power on sensors ///////////////////////////
  digitalWrite(SENSORS_ENABLE_PIN,0);
  delay(5000);

  
  // record audio ///////////////////////////////
#ifdef BEE_DEBUG
  Serial.println("Recording sound...");
#endif

  record();
  
#ifdef BEE_DEBUG
  Serial.println("Done:");
//  for(int i=0; i<NUM_SAMPLES; i++) // uncomment for verbose output of audio data
//    Serial.println((int)d[i]);
#endif



  // record temperature /////////////////////////
/* Commented out b/c we can't test with MAX6675 until it's on the board */
#ifdef BEE_DEBUG
  Serial.println("Recording temperature...");
#endif

  temperature = temp_sensor.read_temp(5); // take 5 temperature measurements and average, then convert to Fahrenheit

#ifdef BEE_DEBUG  
  if(temperature == -1) {                   // If there is an error with the TC, temperature will be -1
    Serial.println("Thermocouple Error!!"); // Temperature is -1 and there is a thermocouple error
  } else {
    Serial.print("Current Temperature: ");
    Serial.println( temperature );          // Print the temperature to Serial 
  }
  Serial.print("Done: T(F)=");
  Serial.println( temperature );
#endif



  // record weight /////////////////////////////////////
#ifdef BEE_DEBUG
  Serial.println("Recording weight...");
#endif

  weight0 = analogRead(2);
//  weight0 = ADC_to_weight_value(weight0);

#ifdef BEE_DEBUG
  Serial.print("Done: ");
  Serial.println( weight0 );
#endif


  // get datetime from DS1307 and make a filename //////
#ifdef BEE_DEBUG
  Serial.println("Gathering date-time...");
#endif

//  getDateDs1307();
  make_filename(file);

#ifdef BEE_DEBUG
  Serial.print("Done: ");
  Serial.println(file);
#endif
  
  // write gathered data to file on USB drive /////////
#ifdef BEE_DEBUG
  Serial.print("Writing to file...");
  Serial.println(file);
#endif

  usb_data_log.init();

  usb_data_log.fopen(file);
  delay(1000);

  // output weight
  char data_point[5];
  data_point[0] = '0'+weight0/100;
  data_point[1] = '0'+(weight0%100)/10;
  data_point[2] = '0'+weight0%10;
  data_point[3] = '\n';
  data_point[4] = 0;
  usb_data_log.fputs(data_point);
  delay(10);
  
  // TODO: output temperature (which is a float value)
  char float_point[10];
  fmtDouble(temperature,2,float_point,10);
  usb_data_log.fputs(float_point);
  usb_data_log.fputs("\n");
  
  
  // output audio
  for(int i=0; i<NUM_SAMPLES; i++) {
    data_point[0] = '0'+(d[i]/100);
    data_point[1] = '0'+(d[i]%100)/10;
    data_point[2] = '0'+(d[i]%10);
    if( i != NUM_SAMPLES-1 )
    {
      data_point[3] = '\n';
      data_point[4] = 0;
    }
    else
    {
      data_point[3] = 0;
    }
    usb_data_log.fputs(data_point);
    delay(10);
  }
  
  // close file
  usb_data_log.fclose(file);
  delay(2000);

#ifdef BEE_DEBUG
  Serial.println("Done");
#endif

  // done -- power down sensors ///////////////////////////
  digitalWrite(SENSORS_ENABLE_PIN,1);
  delay(1000);
}

///////////////////////////////////////////////////////////////////////////////
// DS1307 datetime functions
//
/*
// Convert normal decimal numbers to binary coded decimal
byte decToBcd(byte val)
{
  return ( (val/10*16) + (val%10) );
}

// Convert binary coded decimal to normal decimal numbers
byte bcdToDec(byte val)
{
  return ( (val/16*10) + (val%16) );
}

// Function: setDateDs1307( datetime in descending order, year to second )
// Description:
//   1) Sets the date and time on the ds1307
//   2) Starts the clock
//   3) Sets hour mode to 24 hour clock
// Assumes you're passing in valid numbers, Probably need to put in checks for valid numbers.
void setDateDs1307(char iyear, char imonth, char idayOfMonth, char idayOfWeek, char ihour, char iminute, char isecond)                
{

   second = (byte) isecond; // Use of (byte) type casting and ascii math to achieve result.  
   myminute = (byte) iminute;
   hour  = (byte) ihour;
   dayOfWeek = (byte) idayOfWeek;
   dayOfMonth = (byte) idayOfMonth;
   month = (byte) imonth;
   year= (byte) iyear;
   
   Wire.beginTransmission(DS1307_I2C_ADDRESS);
   Wire.send(0x00);
   Wire.send(decToBcd(second));    // 0 to bit 7 starts the clock
   Wire.send(decToBcd(myminute));
   Wire.send(decToBcd(hour));      // If you want 12 hour am/pm you need to set
                                   // bit 6 (also need to change readDateDs1307)
   Wire.send(decToBcd(dayOfWeek));
   Wire.send(decToBcd(dayOfMonth));
   Wire.send(decToBcd(month));
   Wire.send(decToBcd(year));
   Wire.endTransmission();
}

// Function: getDateDs1307
// Description:
// Gets the date and time from the ds1307 stores result into global "temp" variables
void getDateDs1307()
{
  // Reset the register pointer
  Wire.beginTransmission(DS1307_I2C_ADDRESS);
  Wire.send(0x00);
  Wire.endTransmission();

  Wire.requestFrom(DS1307_I2C_ADDRESS, 7);

  // A few of these need masks because certain bits are control bits
  second     = bcdToDec(Wire.receive() & 0x7f);
  myminute     = bcdToDec(Wire.receive());
  hour       = bcdToDec(Wire.receive() & 0x3f);  // Need to change this if 12 hour am/pm
  dayOfWeek  = bcdToDec(Wire.receive());
  dayOfMonth = bcdToDec(Wire.receive());
  month      = bcdToDec(Wire.receive());
  year       = bcdToDec(Wire.receive());
}
*/
///////////////////////////////////////////////////////////////////////////////
// Audio definitions and functions
//

void record()
{
  // set adc prescaler  to 64 for 19kHz sampling frequency
  sbi(ADCSRA, ADPS2);
  sbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);

  sbi(ADMUX,ADLAR);  // 8-Bit ADC in ADCH Register
  sbi(ADMUX,REFS0);  // VCC Reference
  cbi(ADMUX,REFS1);
  cbi(ADMUX,MUX0);   // Set Input Multiplexer to Channel 0
  cbi(ADMUX,MUX1);
  cbi(ADMUX,MUX2);
  cbi(ADMUX,MUX3);

  num_d = 0;
  sbi(ADCSRA,ADIE); // enable ADC interrupt
  sbi(ADCSRA,ADATE); // free-running mode
  cbi(ADCSRB,ADTS0);
  cbi(ADCSRB,ADTS1);
  cbi(ADCSRB,ADTS2);
  sbi(ADCSRA,ADSC); // start conversion
  
  while( num_d < NUM_SAMPLES ) { // wait while audio data is read
    delay(100);
  }
  
  cbi(ADCSRA,ADIE); // disable ADC interrupt
  cbi(ADCSRA,ADATE); // disable free-running mode
}

ISR(ADC_vect)
{
  if( num_d < NUM_SAMPLES )
  {
    d[num_d] = ADCH;
    num_d++;
  }
}
