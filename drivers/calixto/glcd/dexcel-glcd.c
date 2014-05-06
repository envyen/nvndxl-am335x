/*
    Calixto GLCD Driver for Dexcel

    Copyright (C) 2014 Calixto Systems 
    Naveen Karuthedath <naveen.k@calixto.co.in>
      
*/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/file.h>
#include <asm/uaccess.h>
#include <linux/pm_runtime.h>
#include <linux/clk.h>

#include "dexcel-glcd.h"

#define CHARLCD_DRIVER_MAJOR    162
#define CHARLCD_DRIVER_MINOR	0	
#define NUMBER_OF_DEVICE		1 

#define BITVAL(x,bit) 			((x&(1<<bit))?1:0)

#define GPIO_TO_PIN(bank, gpio) (32 * (bank) + (gpio))

#define XMAX                    121
#define MAXPAGE                 3
#define MAXCOLUMN 			    60

#define E1MIN                   0
#define E1MAX                   60
#define E2MIN                   61
#define E2MAX                   121

//Chip enable
#define CS_BOTH				    2
#define CS_RIGHT			    1
#define CS_LEFT				    0

#define D0						(GPIO_TO_PIN(2,6))
#define D1						(GPIO_TO_PIN(2,7))
#define D2						(GPIO_TO_PIN(2,8))
#define D3						(GPIO_TO_PIN(2,9))
#define D4						(GPIO_TO_PIN(2,10))
#define D5						(GPIO_TO_PIN(2,11))
#define D6						(GPIO_TO_PIN(2,12))
#define D7						(GPIO_TO_PIN(2,13))

#define EN1(x)   				gpio_set_value(GPIO_TO_PIN(2,14),x)
#define EN2(x)   				gpio_set_value(GPIO_TO_PIN(2,15),x)
#define RST(x)                  gpio_set_value(GPIO_TO_PIN(2,16),x)
#define A0_CMD()                gpio_set_value(GPIO_TO_PIN(2,22),0)
#define A0_DATA()               gpio_set_value(GPIO_TO_PIN(2,22),1)


//LCD Commands for PT6520 
#define CMD_DISPLAY_OFF 	    0xAE /* turn LCD panel OFF 		*/
#define CMD_DISPLAY_ON 		    0xAF /* turn LCD panel ON 		*/
#define CMD_SET_START_LINE 		0xC0 /* set line 			    */
#define CMD_SET_PAGE 			0xB8 /* set page address		*/
#define CMD_SET_COLUMN 	    	0x00 /* set column address 		*/
#define CMD_SET_ADC_NORMAL 		0xA0 /* ADC set for normal direction 	*/
#define CMD_SET_ADC_REVERSE 	0xA1 /* ADC set for reverse direction 	*/
#define CMD_INTERNAL_RESET 		0xE2 /* soft reset DISPLAY 		*/
#define CMD_SET_STATIC_OFF 		0xA4 /* normal drive 			*/
#define CMD_SET_STATIC_ON 		0xA5 /* static drive (power save) 	    */
#define CMD_SET_DUTY_16			0XA8 /* driving duty 1/16 		*/
#define CMD_SET_DUTY_32			0XA9 /* driving duty 1/32 		*/

//Driver Infos
#define CAL_GLCD_DRIVER_MAJOR   162
#define CAL_GLCD_DRIVER_MINOR	0
#define CAL_GLCD_NAME			"Calixto glcd"
#define NUMBER_OF_DEVICE	    1
#define BUFFER_SIZE			    1024

static unsigned char PAGE = 0, COL = 0;
static int invert_data = 0;

//TODO! Test font.. move to include
static const char cal_glcd_font1[94][6] = {
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},  // (space) Ascii(32)
    { 0x00, 0x00, 0x5F, 0x00, 0x00, 0x00},  // !
    { 0x00, 0x07, 0x00, 0x07, 0x00, 0x00},  // "
    { 0x14, 0x7F, 0x14, 0x7F, 0x14, 0x00},  // #
    { 0x24, 0x2A, 0x7F, 0x2A, 0x12, 0x00},  // $
    { 0x23, 0x13, 0x08, 0x64, 0x62, 0x00},  // %
    { 0x36, 0x49, 0x55, 0x22, 0x50, 0x00},  // &
    { 0x00, 0x05, 0x03, 0x00, 0x00, 0x00},  // '
    { 0x00, 0x1C, 0x22, 0x41, 0x00, 0x00},  // (
    { 0x00, 0x41, 0x22, 0x1C, 0x00, 0x00},  // )
    { 0x08, 0x2A, 0x1C, 0x2A, 0x08, 0x00},  // *
    { 0x08, 0x08, 0x3E, 0x08, 0x08, 0x00},  // +
    { 0x00, 0x50, 0x30, 0x00, 0x00, 0x00},  // ,
    { 0x08, 0x08, 0x08, 0x08, 0x08, 0x00},  // -
    { 0x00, 0x60, 0x60, 0x00, 0x00, 0x00},  // .
    { 0x20, 0x10, 0x08, 0x04, 0x02, 0x00},  // /
    { 0x3E, 0x51, 0x49, 0x45, 0x3E, 0x00},  // 0
    { 0x00, 0x42, 0x7F, 0x40, 0x00, 0x00},  // 1
    { 0x42, 0x61, 0x51, 0x49, 0x46, 0x00},  // 2
    { 0x21, 0x41, 0x45, 0x4B, 0x31, 0x00},  // 3
    { 0x18, 0x14, 0x12, 0x7F, 0x10, 0x00},  // 4
    { 0x27, 0x45, 0x45, 0x45, 0x39, 0x00},  // 5
    { 0x3C, 0x4A, 0x49, 0x49, 0x30, 0x00},  // 6
    { 0x01, 0x71, 0x09, 0x05, 0x03, 0x00},  // 7
    { 0x36, 0x49, 0x49, 0x49, 0x36, 0x00},  // 8
    { 0x06, 0x49, 0x49, 0x29, 0x1E, 0x00},  // 9
    { 0x00, 0x36, 0x36, 0x00, 0x00, 0x00},  // :
    { 0x00, 0x56, 0x36, 0x00, 0x00, 0x00},  // ;
    { 0x00, 0x08, 0x14, 0x22, 0x41, 0x00},  // <
    { 0x14, 0x14, 0x14, 0x14, 0x14, 0x00},  // =
    { 0x41, 0x22, 0x14, 0x08, 0x00, 0x00},  // >
    { 0x02, 0x01, 0x51, 0x09, 0x06, 0x00},  // ?
    { 0x32, 0x49, 0x79, 0x41, 0x3E, 0x00},  // @
    { 0x7E, 0x11, 0x11, 0x11, 0x7E, 0x00},  // A
    { 0x7F, 0x49, 0x49, 0x49, 0x36, 0x00},  // B
    { 0x3E, 0x41, 0x41, 0x41, 0x22, 0x00},  // C
    { 0x7F, 0x41, 0x41, 0x22, 0x1C, 0x00},  // D
    { 0x7F, 0x49, 0x49, 0x49, 0x41, 0x00},  // E
    { 0x7F, 0x09, 0x09, 0x01, 0x01, 0x00},  // F
    { 0x3E, 0x41, 0x41, 0x51, 0x32, 0x00},  // G
    { 0x7F, 0x08, 0x08, 0x08, 0x7F, 0x00},  // H
    { 0x00, 0x41, 0x7F, 0x41, 0x00, 0x00},  // I
    { 0x20, 0x40, 0x41, 0x3F, 0x01, 0x00},  // J
    { 0x7F, 0x08, 0x14, 0x22, 0x41, 0x00},  // K
    { 0x7F, 0x40, 0x40, 0x40, 0x40, 0x00},  // L
    { 0x7F, 0x02, 0x04, 0x02, 0x7F, 0x00},  // M
    { 0x7F, 0x04, 0x08, 0x10, 0x7F, 0x00},  // N
    { 0x3E, 0x41, 0x41, 0x41, 0x3E, 0x00},  // O
    { 0x7F, 0x09, 0x09, 0x09, 0x06, 0x00},  // P
    { 0x3E, 0x41, 0x51, 0x21, 0x5E, 0x00},  // Q
    { 0x7F, 0x09, 0x19, 0x29, 0x46, 0x00},  // R
    { 0x46, 0x49, 0x49, 0x49, 0x31, 0x00},  // S
    { 0x01, 0x01, 0x7F, 0x01, 0x01, 0x00},  // T
    { 0x3F, 0x40, 0x40, 0x40, 0x3F, 0x00},  // U
    { 0x1F, 0x20, 0x40, 0x20, 0x1F, 0x00},  // V
    { 0x7F, 0x20, 0x18, 0x20, 0x7F, 0x00},  // W
    { 0x63, 0x14, 0x08, 0x14, 0x63, 0x00},  // X
    { 0x03, 0x04, 0x78, 0x04, 0x03, 0x00},  // Y
    { 0x61, 0x51, 0x49, 0x45, 0x43, 0x00},  // Z
    { 0x00, 0x00, 0x7F, 0x41, 0x41, 0x00},  // [
    { 0x02, 0x04, 0x08, 0x10, 0x20, 0x00},  // "\"
    { 0x41, 0x41, 0x7F, 0x00, 0x00, 0x00},  // ]
    { 0x04, 0x02, 0x01, 0x02, 0x04, 0x00},  // ^
    { 0x40, 0x40, 0x40, 0x40, 0x40, 0x00},  // _
    { 0x00, 0x01, 0x02, 0x04, 0x00, 0x00},  // `
    { 0x20, 0x54, 0x54, 0x54, 0x78, 0x00},  // a
    { 0x7F, 0x48, 0x44, 0x44, 0x38, 0x00},  // b
    { 0x38, 0x44, 0x44, 0x44, 0x20, 0x00},  // c
    { 0x38, 0x44, 0x44, 0x48, 0x7F, 0x00},  // d
    { 0x38, 0x54, 0x54, 0x54, 0x18, 0x00},  // e
    { 0x08, 0x7E, 0x09, 0x01, 0x02, 0x00},  // f
    { 0x08, 0x14, 0x54, 0x54, 0x3C, 0x00},  // g
    { 0x7F, 0x08, 0x04, 0x04, 0x78, 0x00},  // h
    { 0x00, 0x44, 0x7D, 0x40, 0x00, 0x00},  // i
    { 0x20, 0x40, 0x44, 0x3D, 0x00, 0x00},  // j
    { 0x00, 0x7F, 0x10, 0x28, 0x44, 0x00},  // k
    { 0x00, 0x41, 0x7F, 0x40, 0x00, 0x00},  // l
    { 0x7C, 0x04, 0x18, 0x04, 0x78, 0x00},  // m
    { 0x7C, 0x08, 0x04, 0x04, 0x78, 0x00},  // n
    { 0x38, 0x44, 0x44, 0x44, 0x38, 0x00},  // o
    { 0x7C, 0x14, 0x14, 0x14, 0x08, 0x00},  // p
    { 0x08, 0x14, 0x14, 0x18, 0x7C, 0x00},  // q
    { 0x7C, 0x08, 0x04, 0x04, 0x08, 0x00},  // r
    { 0x48, 0x54, 0x54, 0x54, 0x20, 0x00},  // s
    { 0x04, 0x3F, 0x44, 0x40, 0x20, 0x00},  // t
    { 0x3C, 0x40, 0x40, 0x20, 0x7C, 0x00},  // u
    { 0x1C, 0x20, 0x40, 0x20, 0x1C, 0x00},  // v
    { 0x3C, 0x40, 0x30, 0x40, 0x3C, 0x00},  // w
    { 0x44, 0x28, 0x10, 0x28, 0x44, 0x00},  // x
    { 0x0C, 0x50, 0x50, 0x50, 0x3C, 0x00},  // y
    { 0x44, 0x64, 0x54, 0x4C, 0x44, 0x00},  // z
    { 0x00, 0x08, 0x36, 0x41, 0x00, 0x00},  // {
    { 0x00, 0x00, 0x7F, 0x00, 0x00, 0x00},  // |
    { 0x00, 0x41, 0x36, 0x08, 0x00, 0x00}   // }
};

struct cal_glcd_data {
	dev_t		    devt;
	struct cdev	    cdev;
	struct class    *class;
   	struct device   *cal_glcd_device;
	struct clk      *lcdc_clk;	            //power enable for the LCDC
};

struct cal_glcd_data *cal_glcd_dev;


static void lcd_write(int x) 
{
	gpio_set_value(D0,BITVAL(x,0)); 
	gpio_set_value(D1,BITVAL(x,1)); 
	gpio_set_value(D2,BITVAL(x,2)); 
	gpio_set_value(D3,BITVAL(x,3)); 	
	gpio_set_value(D4,BITVAL(x,4)); 
	gpio_set_value(D5,BITVAL(x,5)); 
	gpio_set_value(D6,BITVAL(x,6)); 
	gpio_set_value(D7,BITVAL(x,7)); 	
	
}

static void chip_en(int cs, int val)
{
    if(cs==0)
    {
        EN1(val);        
    }
    else if(cs==1)
    {
        EN2(val);
    }
    else if(cs==2)
    {
        EN1(val);
        EN2(val);
    }
}

static void cal_glcd_write_cmd(int val, int cs)
{
    //a0 = 1
    A0_CMD();
    // d0-d7
    lcd_write(val);
    //enable strobe Ex
    chip_en(cs,1);      
    chip_en(cs,0);  
}

static void cal_glcd_write_data(int val, int cs)
{
    //a0=0
    A0_DATA();
    //d0-d7
    lcd_write(val);
    chip_en(cs,0);    
    chip_en(cs,1);      
    chip_en(cs,0);      
}


void cal_glcd_fill(char c, int cs)
{
  int i,j;
  for(i=0; i<=MAXPAGE; i++){
    for(j=0; j<=MAXCOLUMN; j++){
      /* pages */
      cal_glcd_write_cmd(CMD_SET_PAGE|i, cs);
      /*cloumns*/
      cal_glcd_write_cmd(CMD_SET_COLUMN|j, cs);      
      cal_glcd_write_data(c, cs);
    }
  }
}

static void	cal_glcd_init_display(void)
{
    cal_glcd_write_cmd(CMD_INTERNAL_RESET, 	CS_BOTH); 	
	cal_glcd_write_cmd(CMD_SET_DUTY_32,     CS_BOTH);  
	cal_glcd_write_cmd(CMD_SET_ADC_NORMAL,  CS_BOTH);  
	cal_glcd_write_cmd(CMD_DISPLAY_ON,      CS_BOTH);  
	cal_glcd_write_cmd(CMD_SET_STATIC_OFF,	CS_BOTH);
	cal_glcd_write_cmd(CMD_SET_START_LINE,  CS_BOTH);			
	cal_glcd_fill(0x00, CS_BOTH); 
}

static int cal_glcd_open(struct inode *inode, struct file *filp)
{   
	return 0;
}

unsigned char cal_glcd_setColumn(unsigned char Xcolumn, unsigned char Ypage)
{
    unsigned char enable = CS_LEFT;
    unsigned char offset = E1MIN;
    
    if(Ypage <= MAXPAGE)
    {
        if(Xcolumn <= E1MAX)
        {     
            enable = CS_LEFT;
            offset = E1MIN;
        }
        else if(Xcolumn <= E2MAX)
        { 
            enable = CS_RIGHT;
            offset = E2MIN;
        }
    }

    if(enable >= CS_LEFT)
    { 
        cal_glcd_write_cmd(CMD_SET_PAGE + Ypage, enable);
        cal_glcd_write_cmd(CMD_SET_COLUMN + Xcolumn - offset, enable);
    }

    return enable;
}

/* Do this on every char write to pdate position*/
static void cal_glcd_update_cursor(void)
{
	if (COL >= XMAX)
	{
		PAGE += 1;
		COL = 0; 

	    if(PAGE	> (MAXPAGE))
	    {
		    PAGE = 0;
	    } 
	}

	cal_glcd_setColumn(COL, PAGE);	

}


/* Sent a char from font table to LCD */
static void cal_glcd_print(unsigned char Temp_data)
{
	unsigned char i,index;

	//Our Ascii font table start at 32(space)
	index = Temp_data - 32; 

    if((index>=0) && (index<94)){
        
        //print a char
	    for(i = 0; i < 6; i++) {            
            
            cal_glcd_update_cursor();
                        
            if(COL <= E1MAX){
                if(invert_data==1)
              		cal_glcd_write_data(cal_glcd_font1[index][i] ^ 0xFF, CS_LEFT);
        		else
                    cal_glcd_write_data(cal_glcd_font1[index][i], CS_LEFT);
            }
            else if(COL <= E2MAX) {
                if(invert_data==1)
              		cal_glcd_write_data(cal_glcd_font1[index][i] ^ 0xFF, CS_RIGHT);
                else
                    cal_glcd_write_data(cal_glcd_font1[index][i], CS_RIGHT);
            }
            
	        COL += 1;

            if(COL == 120) { 
               COL += 2; 
               cal_glcd_update_cursor();
            }
            
	    }
    }
}


ssize_t cal_glcd_write(struct file *tty_dev, const char __user *data_buf,
						size_t count, loff_t *offset)
{
	ssize_t retval;
	char *message;
   	int i=0;
    
	message = kmalloc(count, GFP_KERNEL);

	if (!message)
		retval = -ENOMEM;

	if (count > BUFFER_SIZE)
		count = BUFFER_SIZE;

	if (copy_from_user(message, data_buf, count)) 
		retval = -EFAULT;

    // push char by char to lcd 
  	for(i=0;i<count;i++)
		cal_glcd_print(message[i]);
	
	return count;
}

static long cal_glcd_ioctl(struct file *filp, unsigned int cmd, 
                            unsigned long arg)
{
	struct 	gLcd_Pos lcd_pos;
	int chip = CS_LEFT;
	/* Check if its our ioctl */
	if (_IOC_TYPE(cmd) != CALIXTO_GLCD_IOC_MAGIC)
		return -ENOTTY;

	switch (cmd) {
	
	case GLCD_CLEAR:
    	cal_glcd_fill(0x00, CS_BOTH); 
		return 0;	
		break;
		
	case GLCD_SET_XY:
    	if (!arg)
	    return -EINVAL;
	    
    	/* Get the struct with page and col from user */
    	if(copy_from_user(&lcd_pos, (struct gLcd_Pos *)arg, 
    	                             sizeof(struct gLcd_Pos)))
		    return -EFAULT;

        COL = lcd_pos.col;
        PAGE = lcd_pos.page;
        cal_glcd_update_cursor();

        return 0;	
        break;

	case GLCD_SEL_LEFT:
    	chip = CS_LEFT;
        return 0;	
		break;
    	
	case GLCD_SEL_RIGHT:
    	chip = CS_RIGHT;
        return 0;	
		break;
    	
	case GLCD_SEND_CMD:
	    	cal_glcd_write_cmd((unsigned char)arg, chip);
		return 0;	
		break;

	case GLCD_SEND_DATA:
    		cal_glcd_write_data((unsigned char)arg, chip);
		return 0;	
		break;
		
	case GLCD_INVERT:
	    	invert_data = (int)arg;
   	    	return 0;
	    	break;

		
	default:
		printk(KERN_ERR CAL_GLCD_NAME": Invalid IOCTL: 0x%04x\n", cmd);
		return -ENOTTY;
		break;
	}
	
	return 0;
}

static int cal_glcd_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static const struct file_operations cal_glcd_fops = {
        .owner          = THIS_MODULE,
        .open           = cal_glcd_open,
        .write          = cal_glcd_write, 
        .unlocked_ioctl	= cal_glcd_ioctl, 
        .release        = cal_glcd_release,
};

static int __devinit cal_glcd_probe(struct platform_device *pdev)
{
    int  status;

	cal_glcd_dev = kzalloc(sizeof(*cal_glcd_dev), GFP_KERNEL);
	if (!cal_glcd_dev)
		return -ENOMEM;

    cal_glcd_dev->devt = MKDEV(CAL_GLCD_DRIVER_MAJOR,CAL_GLCD_DRIVER_MINOR);

	status = register_chrdev_region(cal_glcd_dev->devt, NUMBER_OF_DEVICE,
									 "glcd");
    	if (status < 0) {
		return -EINVAL;
	}

	cdev_init (&cal_glcd_dev->cdev, &cal_glcd_fops);
	cal_glcd_dev->cdev.owner = THIS_MODULE;
	cal_glcd_dev->cdev.ops   = &cal_glcd_fops;
	status                  = cdev_add (&cal_glcd_dev->cdev, cal_glcd_dev->devt,
							NUMBER_OF_DEVICE);

	if(status < 0) {
	    goto free_chrdev_region;
	    printk(KERN_ERR CAL_GLCD_NAME ": Driver Registration Failed\n");
	}

	cal_glcd_dev->class = class_create(THIS_MODULE, "glcd");

	if (IS_ERR(cal_glcd_dev->class)) {
		status = PTR_ERR(cal_glcd_dev->class);
		goto unregister_chrdev;
	}

	cal_glcd_dev->cal_glcd_device = device_create(cal_glcd_dev->class, NULL,
	                          cal_glcd_dev->devt, NULL, "glcd");

	if (IS_ERR(cal_glcd_dev->cal_glcd_device)) {
		status = PTR_ERR(cal_glcd_dev->cal_glcd_device);
		goto unregister_chrdev;
	}

	printk(KERN_INFO CAL_GLCD_NAME ": Driver loaded \n");

	status = gpio_request(GPIO_TO_PIN(2,6),"GLCD_D0");
	status = gpio_request(GPIO_TO_PIN(2,7),"GLCD_D1");
	status = gpio_request(GPIO_TO_PIN(2,8),"GLCD_D2");
	status = gpio_request(GPIO_TO_PIN(2,9),"GLCD_D3");
	status = gpio_request(GPIO_TO_PIN(2,10),"GLCD_D4");
    status = gpio_request(GPIO_TO_PIN(2,11),"GLCD_D5");
	status = gpio_request(GPIO_TO_PIN(2,12),"GLCD_D6");
	status = gpio_request(GPIO_TO_PIN(2,13),"GLCD_D7");

	status = gpio_request(GPIO_TO_PIN(2,22),"GLCD_A0");
	status = gpio_request(GPIO_TO_PIN(2,23),"GLCD_RW");
						
	status = gpio_request(GPIO_TO_PIN(2,14),"GLCD_E1");
    status = gpio_request(GPIO_TO_PIN(2,15),"GLCD_E2");
    
	gpio_direction_output(GPIO_TO_PIN(2,6),0);
	gpio_direction_output(GPIO_TO_PIN(2,7),0);
	gpio_direction_output(GPIO_TO_PIN(2,8),0);
	gpio_direction_output(GPIO_TO_PIN(2,9),0);
	gpio_direction_output(GPIO_TO_PIN(2,10),0);
	gpio_direction_output(GPIO_TO_PIN(2,11),0);
	gpio_direction_output(GPIO_TO_PIN(2,12),0);
	gpio_direction_output(GPIO_TO_PIN(2,13),0);
	
	gpio_direction_output(GPIO_TO_PIN(2,14),0);
	gpio_direction_output(GPIO_TO_PIN(2,15),0);
	gpio_direction_output(GPIO_TO_PIN(2,22),0);
	gpio_direction_output(GPIO_TO_PIN(2,23),0);

    gpio_set_value(GPIO_TO_PIN(2,23),0);
    gpio_set_value(GPIO_TO_PIN(2,14),1);
    gpio_set_value(GPIO_TO_PIN(2,15),1);
    gpio_set_value(GPIO_TO_PIN(2,16),1); //rst test
	cal_glcd_init_display();

	//our display is ready now !
	return status;

unregister_chrdev:
	cdev_del(&cal_glcd_dev->cdev);
	class_destroy(cal_glcd_dev->class);

free_chrdev_region:
	unregister_chrdev_region(cal_glcd_dev->devt, NUMBER_OF_DEVICE);
	kfree(cal_glcd_dev);
    	return status;
}

static int __devexit cal_glcd_remove(struct platform_device *pdev)
{
   	cdev_del(&cal_glcd_dev->cdev);
	unregister_chrdev_region(cal_glcd_dev->devt, NUMBER_OF_DEVICE);
    class_destroy(cal_glcd_dev->class);
	kfree(cal_glcd_dev);

	return 0;
}

static struct platform_driver cal_glcd_driver = {
	.driver = {
		.name   = "dxlLcd",
		.owner  = THIS_MODULE,
	},
	.probe      = cal_glcd_probe,
	.remove     = __devexit_p(cal_glcd_remove),
};

MODULE_ALIAS("Platform:dxlLcd");

static int __init cal_glcd_init(void)
{
	return platform_driver_register(&cal_glcd_driver);
}

static void __exit cal_glcd_exit(void)
{
	platform_driver_unregister(&cal_glcd_driver);

}

module_init(cal_glcd_init);
module_exit(cal_glcd_exit);

MODULE_AUTHOR("Calixto Systems, <naveen.k@calixto.co.in>");
MODULE_DESCRIPTION("Calixto GLCD LIDD driver");
MODULE_LICENSE("GPL");