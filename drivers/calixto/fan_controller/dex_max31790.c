#include <linux/module.h> 		// For supporting module interface
#include <linux/moduleparam.h>		// For module_param interface
#include <linux/init.h>			// For module interface
#include <linux/kernel.h>		// For supporting i2c cleint driver
#include <linux/i2c.h>			// For supporting i2c cleint driver
#include <linux/interrupt.h>		// For supporting interrupt handling
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/slab.h>

#define PRINTK_FLAG  KERN_EMERG
#define DEBUG_PRINT  1

#define REG_FAN1_DYNAMIC 0x08

//Local strcuture defined to represent the max31790 device
struct max31790_device_struct{
	struct i2c_client *client;
};

#if 0
int max31790_read_value(struct i2c_client *client , unsigned char reg)
{
	if(reg < 0x10)  // byte-sized register
		return i2c_smbus_read_data(client , reg);
	else		// word sized register
		return i2c_smbus_read_word(client , reg);
}


int max31790_write_value(struct i2c_client *client , unsigned char reg,
						    unsigned short value)
{

	if (reg == 0x10)     //Impossible to write-driver error
		return -EINVAL;
	else if(reg < 0x10)  // byte-sized register
		return i2c_smbus_write_data(client , reg);
	else		     // word sized register
		return i2c_smbus_write_word(client , reg);

}

#endif

static int max31790_probe_func(struct i2c_client *client ,
			const struct i2c_device_id *id)
{
	struct max31790_device_struct 	*local_dev_ptr;
	int                     	tmp;
	int 				ret=0;

#ifdef DEBUG_PRINT
	printk("[dex_31790] In PROBE routine of module \n");
#endif

	//Checking if the I2c adapter supports required behaviour
	
	if(!i2c_check_functionality(client->adapter,
				I2C_FUNC_SMBUS_BYTE_DATA |
			        I2C_FUNC_SMBUS_WORD_DATA |
				I2C_FUNC_SMBUS_I2C_BLOCK)){
		printk("[dex_31790] Needed function not supported \n");
		return -ENODEV;
	}//End brace of check_functionality function

	local_dev_ptr = kzalloc(sizeof(*local_dev_ptr), GFP_KERNEL);
	if(!local_dev_ptr){
		printk("\n [dex_31790] No sufficient memory \n");
		return -ENOMEM;
	}

	local_dev_ptr->client = client;

	tmp = i2c_smbus_read_byte_data(client, REG_FAN1_DYNAMIC);
	
	 if(tmp < 0)
		printk("Reg Read Error\n");
	
	printk("Read Fan1 Dynamic Register-> %x\n", tmp);

	i2c_set_clientdata(client , local_dev_ptr);

	printk("[dex_31790] Sample_Client_created \n");	
	return 0;
}


static int max31790_remove_func(struct i2c_client *client)
{

	struct max31790_device_struct *dev_ptr = i2c_get_clientdata(client);
	kfree(dev_ptr);
	
#ifdef DEBUG_PRINT
	printk("[dex_31790] In REMOVE routine of module \n");
#endif
	return 0;
}


static int max31790_suspend_func(struct i2c_client *client , pm_message_t msg)
{

	struct max31790_device_struct *dev_ptr = i2c_get_clientdata(client);
#ifdef DEBUG_PRINT
	printk("[dex_31790] In SUSPEND routine of module \n");
#endif
	return 0;
}


static int max31790_resume_func(struct i2c_client *client)
{

	struct max31790_device_struct *dev_ptr = i2c_get_clientdata(client); 
#ifdef DEBUG_PRINT
	printk("[dex_31790] In RESUME routine of module \n");
#endif

	return 0;
}


//List of ID supported by the driver
static const struct i2c_device_id max31790_id_table[]={
	{ "max31790_controller",0}, 
	{ }  
};	
MODULE_DEVICE_TABLE(i2c,max31790_id_table);


static struct i2c_driver max31790_driver_struct={
	.driver   = {
		.name  = "max31790_controller",
		.owner = THIS_MODULE
	},
        .id_table = max31790_id_table,
	.probe    = max31790_probe_func,
	.remove   = max31790_remove_func,
	.suspend  = max31790_suspend_func,
	.resume   = max31790_resume_func,
};

static int __init test_module_init(void)
{

	int local_ret=0;

#ifdef DEBUG_PRINT
	printk("[dex_31790] In INIT routine of module \n");
#endif

	local_ret = i2c_add_driver(&max31790_driver_struct);

	if(local_ret == 0)
		printk("\n[dex_31790] I2C_ADD_DRIVER returned 0 \n");
	else
		printk("\n[dex_31790] I2c_ADD_DRIVER returned %d \n",local_ret);
	
	return(local_ret);
}

static void __exit test_module_exit(void)
{
	int local_ret=0;

#ifdef DEBUG_PRINT
	printk("\n[dex_max31790] In EXIT routine of module\n");
#endif
	i2c_del_driver(&max31790_driver_struct); 
}

module_init(test_module_init);
module_exit(test_module_exit);

MODULE_DESCRIPTION("MAX31790_i2c_client_driver");
MODULE_AUTHOR("Ashish_Kumar_Mishra");
MODULE_LICENSE("GPL");
