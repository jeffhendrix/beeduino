#ifndef __VDRIVE2_H_
#define __VDRIVE2_H_

#include <inttypes.h>

//#define VDRIVE_DEBUG

struct VDRIVE2_READ
{
	unsigned char value;
	unsigned char isNew;
};

class VDRIVE2
{
private:
	int pinSPICLK;
	int pinSPICS;
	int pinSPIDI;
	int pinSPIDO;

public:
	VDRIVE2();
	VDRIVE2(int pinCLK, int pinCS, int pinDI, int pinDO);
  
	VDRIVE2_READ read();
	unsigned char readStatus();
	unsigned char write(unsigned char out);
	unsigned char writeString(char* in);
	void clearConsole();
	void init();

	int fopen(char* input);
	int fputs(char* data);
	int fclose(char* input);
};

#endif