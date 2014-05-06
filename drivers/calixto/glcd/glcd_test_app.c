/*
    Calixto GLCD Driver Test Application for Dexcel

    Copyright (C) 2014 Calixto Systems 
    Naveen Karuthedath <naveen.k@calixto.co.in>

    Compilation:
    # arm-linux-gnueabihf-gcc glcd_test_app.c -o glcd-test
    
    
*/

#include <stdio.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>
#include "dexcel-glcd.h"

int fd;

// test volume icon
// 0x3c, 0x3c, 0x7e, 0xff, 0x00, 0x24, 0x18, 0x42, 0x3c

void LCD_print(char* txt)
{
	write(fd,txt,strlen(txt));
}

void glcd_fill()
{
  int i,j, cnt=0;

}


int main(int argc, char *argv[])
{
    char *file_name = "/dev/glcd";
    int i,j, cnt=0;
    
    struct gLcd_Pos cursor;

    if ((fd = open(file_name, O_WRONLY)) < 0) {
        perror("Failed to open.");
        return 1;
    }

    ioctl(fd, GLCD_CLEAR, 0);

    cursor.page = 0;
    cursor.col  = 0;
    ioctl(fd, GLCD_SET_XY, &cursor);
    
    printf("Graphical LCD Test\n");    
    LCD_print("LCD Test");


    printf("Command: Display OFF. Press enter \n");    
    getchar();
    ioctl(fd, GLCD_SEND_CMD, 0xAE);

    printf("Command: Display ON. Press enter \n");    
    getchar();
    ioctl(fd, GLCD_SEND_CMD, 0xAF);
    
    printf("Custom Data.. Press enter \n");            
    getchar();
    
    LCD_print(" ");    
    ioctl(fd, GLCD_SEND_DATA, 0x3c);
    ioctl(fd, GLCD_SEND_DATA, 0x3c);
    ioctl(fd, GLCD_SEND_DATA, 0x7e);
    ioctl(fd, GLCD_SEND_DATA, 0xff);
    ioctl(fd, GLCD_SEND_DATA, 0x00);
    ioctl(fd, GLCD_SEND_DATA, 0x18);                        
    ioctl(fd, GLCD_SEND_DATA, 0x42);                    

    printf("Inverted text. Press enter \n");    
    getchar();

    ioctl(fd, GLCD_INVERT, 1);
    LCD_print(" Inverted ");
    ioctl(fd, GLCD_INVERT, 0);

    printf("Position: Press enter \n");    
    getchar();

    cursor.page = 2;
    cursor.col = 20;
    ioctl(fd, GLCD_SET_XY, &cursor);

    LCD_print("Position test");

    printf("To Clear & Exit, Press enter \n");    
    getchar();

    ioctl(fd, GLCD_CLEAR, 0);
    
    close(fd);
    return 0;

}