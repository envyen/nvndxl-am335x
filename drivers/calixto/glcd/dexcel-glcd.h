/*
    Calixto GLCD Driver for Dexcel

    Copyright (C) 2014 Calixto Systems 
    Naveen Karuthedath <naveen.k@calixto.co.in>
      
*/

#include <linux/ioctl.h>

struct gLcd_Pos
{
	unsigned char page;
	unsigned char col;
};

#define CALIXTO_GLCD_IOC_MAGIC	'n'

//GLCD IOCTLs
#define GLCD_SET_XY		_IOW(CALIXTO_GLCD_IOC_MAGIC, 0x01, struct cLcd_Pos *)

#define GLCD_SEND_CMD	_IOW(CALIXTO_GLCD_IOC_MAGIC, 0x02, unsigned char)
#define GLCD_SEND_DATA	_IOW(CALIXTO_GLCD_IOC_MAGIC, 0x03, unsigned char)

#define GLCD_CLEAR		_IO(CALIXTO_GLCD_IOC_MAGIC, 0x04)
#define GLCD_TEST		_IO(CALIXTO_GLCD_IOC_MAGIC, 0x05)

#define GLCD_INVERT		_IOW(CALIXTO_GLCD_IOC_MAGIC, 0x06, int)

#define GLCD_SEL_LEFT   _IO(CALIXTO_GLCD_IOC_MAGIC, 0x07)
#define GLCD_SEL_RIGHT  _IO(CALIXTO_GLCD_IOC_MAGIC, 0x08)
