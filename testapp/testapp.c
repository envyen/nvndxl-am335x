#include <stdio.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

#define DEXCEL_IOC_MAGIC  'd'
#define START_DEXCEL_IOCTL_OUT  _IOW(DEXCEL_IOC_MAGIC, 1, int)
#define START_DEXCEL_IOCTL_IN   _IOR(DEXCEL_IOC_MAGIC, 2, struct gpio_info *)
#define TOGGLE_DEXCEL_IOS       _IOW(DEXCEL_IOC_MAGIC, 3, int)

struct gpio_info{

  int id;
  int value;
};

struct gpio_info data;
int fd;

int blink_led(void)
{

   int loop;
   int error = 0;

   for (loop = 0; loop <=5; loop++){


	data.id = 3;
	data.value = 1;
	error= ioctl(fd, START_DEXCEL_IOCTL_OUT, &data);

	data.id = 4;
        data.value = 1;
        error= ioctl(fd, START_DEXCEL_IOCTL_OUT, &data);
	
	sleep(2);

 	data.id = 3;
	data.value = 0;
        error= ioctl(fd, START_DEXCEL_IOCTL_OUT, &data);

        data.id = 4;
        data.value = 0;
        error= ioctl(fd, START_DEXCEL_IOCTL_OUT, &data);

        sleep(2);

   } 
     return error;
}

int gpio_jmp_test(void)
{
	int error = 0;

	  data.id = 3;
          error = ioctl(fd, START_DEXCEL_IOCTL_IN, &data);

          printf("Input GPIO ID No -> %d\n", data.id);
          printf("Input GPIO Value -> %d\n", data.value);

          data.id = 4;
          error = ioctl(fd, START_DEXCEL_IOCTL_IN, &data);

          printf("\nInput GPIO ID No -> %d\n", data.id);
          printf("Input GPIO Value -> %d\n", data.value);

	return error;
}


int main(void)
{
    int error;
    int loop = 0;

    fd= open("/dev/gpio_driver", O_RDWR);
    
     if(fd == -1){

	   printf("Error in opening Driver\n");
           return 0;
     }

	//error = blink_led();	 
	
	  error = gpio_jmp_test(); 
		
	close(fd);
	return error;
}
