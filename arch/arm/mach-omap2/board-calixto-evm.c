/*
 * Code for AM335X Dexcel Custom Board
 *
 * Copyright (C) 2011 Texas Instruments, Inc. - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/i2c/at24.h>
#include <linux/phy.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/gpio_info.h>
#include <linux/input.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/wl12xx.h>
#include <linux/ethtool.h>
#include <linux/mfd/tps65910.h>
//#include <linux/input/ti_tsc.h>
#include <linux/platform_data/ti_adc.h>
#include <linux/mfd/ti_tscadc.h>
#include <linux/reboot.h>
#include <linux/pwm/pwm.h>
#include <linux/rtc/rtc-omap.h>
#include <linux/opp.h>

/* LCD controller is similar to DA850 */
#include <video/da8xx-fb.h>

#include <mach/hardware.h>
#include <mach/board-am335xevm.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/hardware/asp.h>

#include <plat/omap_device.h>
#include <plat/omap-pm.h>
#include <plat/irqs.h>
#include <plat/board.h>
#include <plat/common.h>
#include <plat/lcdc.h>
#include <plat/usb.h>
#include <plat/mmc.h>
#include <plat/emif.h>
#include <plat/nand.h>

#include "board-flash.h"
#include "cpuidle33xx.h"
#include "mux.h"
#include "devices.h"
#include "hsmmc.h"
#include "common.h"

/* Am335x-ADC Init */
static struct adc_data am335x_adc_data = {
	.adc_channels = 8,
};

static struct mfd_tscadc_board tscadc = {
	.adc_init = &am335x_adc_data,
};

/*OMAP MMC Init */
static struct omap2_hsmmc_info am335x_mmc[] __initdata = {
	{
		.mmc            = 1,
		.caps           = MMC_CAP_4_BIT_DATA,
		.gpio_cd        = GPIO_TO_PIN(1, 24),
		.gpio_wp        = GPIO_TO_PIN(0, 7),
		.ocr_mask       = MMC_VDD_32_33 | MMC_VDD_33_34, /* 3V3 */
	},
	{
		.mmc            = 0,	/* will be set at runtime */
	},
	{
		.mmc            = 0,	/* will be set at runtime */
	},
	{}      /* Terminator */
};


#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	/*
	 * Setting SYSBOOT[5] should set xdma_event_intr0 pin to mode 3 thereby
	 * allowing clkout1 to be available on xdma_event_intr0.
	 * However, on some boards (like EVM-SK), SYSBOOT[5] isn't properly
	 * latched.
	 * To be extra cautious, setup the pin-mux manually.
	 * If any modules/usecase requries it in different mode, then subsequent
	 * module init call will change the mux accordingly.
	 */
	AM33XX_MUX(XDMA_EVENT_INTR0, OMAP_MUX_MODE3 | AM33XX_PIN_OUTPUT),
	AM33XX_MUX(I2C0_SDA, OMAP_MUX_MODE0 | AM33XX_SLEWCTRL_SLOW |
			AM33XX_INPUT_EN | AM33XX_PIN_OUTPUT),
	AM33XX_MUX(I2C0_SCL, OMAP_MUX_MODE0 | AM33XX_SLEWCTRL_SLOW |
			AM33XX_INPUT_EN | AM33XX_PIN_OUTPUT),
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define	board_mux	NULL
#endif

/* module pin mux structure */
struct pinmux_config {
	const char *string_name; /* signal name format */
	int val; /* Options for the mux register value */
};

struct evm_dev_cfg {
	void (*device_init)(int evm_id, int profile);

#define DEV_ON_BASEBOARD	0
	u32 device_on;
	u32 profile;
};

static bool daughter_brd_detected;
static int am33xx_evmid = -EINVAL;

void am335x_evm_set_id(unsigned int evmid)
{
	am33xx_evmid = evmid;
	return;
}

int am335x_evm_get_id(void)
{
	return am33xx_evmid;
}
EXPORT_SYMBOL(am335x_evm_get_id);

static struct pinmux_config lcdc_pin_mux_gpio[] = {
        {"lcd_data0.gpio2_6",         OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT },
        {"lcd_data1.gpio2_7",         OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT },
        {"lcd_data2.gpio2_8",         OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT },
        {"lcd_data3.gpio2_9",         OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT },
        {"lcd_data4.gpio2_10",         OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT },
        {"lcd_data5.gpio2_11",         OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT },
        {"lcd_data6.gpio2_12",         OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT },
        {"lcd_data7.gpio2_13",         OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT },

        {"lcd_data8.gpio2_14",          OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
        {"lcd_data9.gpio2_15",          OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
        {"lcd_data10.gpio2_16",         OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},

        {"lcd_vsync.gpio2_22",         OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
        {"lcd_hsync.gpio2_23",         OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
        {"lcd_pclk.gpio2_24",           OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},

        {"lcd_ac_bias_en.lcd_ac_bias_en", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
        {NULL, 0},
};
/* Pin mux for nand flash module */
static struct pinmux_config nand_pin_mux[] = {
	{"gpmc_ad0.gpmc_ad0",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad1.gpmc_ad1",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad2.gpmc_ad2",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad3.gpmc_ad3",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad4.gpmc_ad4",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad5.gpmc_ad5",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad6.gpmc_ad6",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad7.gpmc_ad7",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_wait0.gpmc_wait0", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_wpn.gpmc_wpn",	  OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_csn0.gpmc_csn0",	  OMAP_MUX_MODE0 | AM33XX_PULL_DISA},
	{"gpmc_advn_ale.gpmc_advn_ale",  OMAP_MUX_MODE0 | AM33XX_PULL_DISA},
	{"gpmc_oen_ren.gpmc_oen_ren",	 OMAP_MUX_MODE0 | AM33XX_PULL_DISA},
	{"gpmc_wen.gpmc_wen",     OMAP_MUX_MODE0 | AM33XX_PULL_DISA},
	{"gpmc_ben0_cle.gpmc_ben0_cle",	 OMAP_MUX_MODE0 | AM33XX_PULL_DISA},
	{NULL, 0},
};

/* Module pin mux for SPI0 flash */
static struct pinmux_config spi0_pin_mux[] = {
	{"spi0_sclk.spi0_sclk", OMAP_MUX_MODE0 | AM33XX_PULL_ENBL
							| AM33XX_INPUT_EN},
	{"spi0_d0.spi0_d0", OMAP_MUX_MODE0 | AM33XX_PULL_ENBL | AM33XX_PULL_UP
							| AM33XX_INPUT_EN},
	{"spi0_d1.spi0_d1", OMAP_MUX_MODE0 | AM33XX_PULL_ENBL
							| AM33XX_INPUT_EN},
	{"spi0_cs0.spi0_cs0", OMAP_MUX_MODE0 | AM33XX_PULL_ENBL | AM33XX_PULL_UP
							| AM33XX_INPUT_EN},
	{NULL, 0},
};

/* Module pin mux for SPI1 flash */
static struct pinmux_config spi1_pin_mux[] = {
	{"mcasp0_aclkx.spi1_sclk", OMAP_MUX_MODE3 | AM33XX_PULL_ENBL
							| AM33XX_INPUT_EN},
	{"mcasp0_fsx.spi1_d0", OMAP_MUX_MODE3 | AM33XX_PULL_ENBL | AM33XX_PULL_UP
							| AM33XX_INPUT_EN},
	{"mcasp0_axr0.spi1_d1", OMAP_MUX_MODE3 | AM33XX_PULL_ENBL
							| AM33XX_INPUT_EN},
	{"mcasp0_ahclkr.spi1_cs0", OMAP_MUX_MODE3 | AM33XX_PULL_ENBL | AM33XX_PULL_UP
							| AM33XX_INPUT_EN},
	{"ecap0_in_pwm0_out.spi1_cs1", OMAP_MUX_MODE2 | AM33XX_PULL_ENBL | AM33XX_PULL_UP
                                                        | AM33XX_INPUT_EN},
	{NULL, 0},
};

/* Module pin mux for rmii1 */
static struct pinmux_config rmii1_pin_mux[] = {
	{"mii1_crs.rmii1_crs_dv", OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_rxerr.mii1_rxerr", OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_txen.mii1_txen", OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"mii1_txd1.mii1_txd1", OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"mii1_txd0.mii1_txd0", OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"mii1_rxd1.mii1_rxd1", OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_rxd0.mii1_rxd0", OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLDOWN},
	{"rmii1_refclk.rmii1_refclk", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mdio_data.mdio_data", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mdio_clk.mdio_clk", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT_PULLUP},
	{NULL, 0},
};

/* Module pin mux for mmc0 */
static struct pinmux_config mmc0_pin_mux[] = {
        {"mmc0_dat3.mmc0_dat3", OMAP_MUX_MODE0 	| AM33XX_PIN_INPUT_PULLUP},
        {"mmc0_dat2.mmc0_dat2", OMAP_MUX_MODE0 	| AM33XX_PIN_INPUT_PULLUP},
        {"mmc0_dat1.mmc0_dat1", OMAP_MUX_MODE0 	| AM33XX_PIN_INPUT_PULLUP},
        {"mmc0_dat0.mmc0_dat0", OMAP_MUX_MODE0 	| AM33XX_PIN_INPUT_PULLUP},
        {"mmc0_clk.mmc0_clk",   OMAP_MUX_MODE0 	| AM33XX_PIN_INPUT_PULLUP},
        {"mmc0_cmd.mmc0_cmd",   OMAP_MUX_MODE0 	| AM33XX_PIN_INPUT_PULLUP},
        {"gpmc_a8.gpio1_24",  	OMAP_MUX_MODE7 	| AM33XX_PIN_INPUT_PULLUP},
        {NULL, 0},
};

/* Module pin mux for uart2 */
static struct pinmux_config uart2_pin_mux[] = {
        {"mii1_txclk.uart2_rxd", OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLUP},
        {"mii1_rxclk.uart2_txd", OMAP_MUX_MODE1 | AM33XX_PULL_ENBL},
        {NULL, 0},
};

/* Module pin mux for uart3 */
static struct pinmux_config uart3_pin_mux[] = {
        {"spi0_cs1.uart3_rxd",  OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLUP},
        {"mii1_rxd2.uart3_txd", OMAP_MUX_MODE1 | AM33XX_PULL_ENBL},
        {NULL, 0},
};

/* Module pin mux for uart4 */
static struct pinmux_config uart4_pin_mux[] = {
        {"uart0_ctsn.uart4_rxd", OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLUP},
        {"uart0_rtsn.uart4_txd", OMAP_MUX_MODE1 | AM33XX_PULL_ENBL},
        {NULL, 0},
};

/* Module pin mux for uart5 */
static struct pinmux_config uart5_pin_mux[] = {
        {"mii1_col.uart5_rxd",  OMAP_MUX_MODE3 | AM33XX_PIN_INPUT_PULLUP},
        {"mii1_rxdv.uart5_txd", OMAP_MUX_MODE3 | AM33XX_PULL_ENBL},
        {NULL, 0},
};

/* Module pin Mux for gpio */
static struct pinmux_config gpio_pin_mux[] = {
	{"mii1_rxd3.gpio2_18",   OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT}, /* GPIO_LED_1  */ 
	{"mcasp0_axr1.gpio3_20", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT}, /* GPIO_LED_2  */
	{"gpmc_csn1.gpio1_30",	 OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},  /* GPIO_JMP_2  */
	{"gpmc_csn2.gpio1_31",   OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},  /* GPIO_JMP_1  */
	{"gpmc_a7.gpio1_23",	 OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},  /* GPIO_SWITCH */
	{NULL, 0},
};
/*
* @pin_mux - single module pin-mux structure which defines pin-mux
*			details for all its pins.
*/
static void setup_pin_mux(struct pinmux_config *pin_mux)
{
	int i;

	for (i = 0; pin_mux->string_name != NULL; pin_mux++)
		omap_mux_init_signal(pin_mux->string_name, pin_mux->val);

}

/* EVM Board Configuration */
static void _configure_device(int evm_id, struct evm_dev_cfg *dev_cfg,
	int profile)
{
	int i;

	am335x_evm_set_id(evm_id);

	if (profile == PROFILE_NONE) {
		for (i = 0; dev_cfg->device_init != NULL; dev_cfg++) {
			if (dev_cfg->device_on == DEV_ON_BASEBOARD)
				dev_cfg->device_init(evm_id, profile);
			else if (daughter_brd_detected == true)
				dev_cfg->device_init(evm_id, profile);
		}
	} else {
		for (i = 0; dev_cfg->device_init != NULL; dev_cfg++) {
			if (dev_cfg->profile & profile) {
				if (dev_cfg->device_on == DEV_ON_BASEBOARD)
					dev_cfg->device_init(evm_id, profile);
				else if (daughter_brd_detected == true)
					dev_cfg->device_init(evm_id, profile);
			}
		}
	}
}

/* pinmux for usb0 drvvbus */
static struct pinmux_config usb0_pin_mux[] = {
	{"usb0_drvvbus.usb0_drvvbus",    OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};

#define AM335XEVM_WLAN_PMENA_GPIO	GPIO_TO_PIN(0, 27)
#define AM335XEVM_WLAN_IRQ_GPIO		GPIO_TO_PIN(0, 26)

struct wl12xx_platform_data am335xevm_wlan_data = {
        .irq = OMAP_GPIO_IRQ(AM335XEVM_WLAN_IRQ_GPIO),
        .board_ref_clock = WL12XX_REFCLOCK_38_XTAL, /* 38.4Mhz */
        .bt_enable_gpio = GPIO_TO_PIN(0, 23),
        .wlan_enable_gpio = GPIO_TO_PIN(0, 27),
};

/* Module pin mux for wlan and bluetooth */                                          
static struct pinmux_config mmc2_wl12xx_pin_mux[] = {                                
        {"gpmc_ad12.mmc2_dat0", OMAP_MUX_MODE3 | AM33XX_PIN_INPUT_PULLUP},           
        {"gpmc_ad13.mmc2_dat1", OMAP_MUX_MODE3 | AM33XX_PIN_INPUT_PULLUP},           
        {"gpmc_ad14.mmc2_dat2", OMAP_MUX_MODE3 | AM33XX_PIN_INPUT_PULLUP},           
        {"gpmc_ad15.mmc2_dat3", OMAP_MUX_MODE3 | AM33XX_PIN_INPUT_PULLUP},           
        {"gpmc_csn3.mmc2_cmd",  OMAP_MUX_MODE3 | AM33XX_PIN_INPUT_PULLUP},           
        {"gpmc_clk.mmc2_clk",   OMAP_MUX_MODE3 | AM33XX_PIN_INPUT_PULLUP},           
        {NULL, 0},                                                                   
};

static struct pinmux_config uart1_wl12xx_pin_mux[] = {                    
        {"uart1_ctsn.uart1_ctsn", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},    
        {"uart1_rtsn.uart1_rtsn", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT}, 
        {"uart1_rxd.uart1_rxd", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
        {"uart1_txd.uart1_txd", OMAP_MUX_MODE0 | AM33XX_PULL_ENBL},       
        {NULL, 0},
};      

static struct pinmux_config wl12xx_pin_mux[] = {
        {"gpmc_ad11.gpio0_27",  OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
        {"gpmc_ad10.gpio0_26",  OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
        {"gpmc_ad9.gpio0_23",   OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
        {NULL, 0},
};

static int __init conf_disp_pll(int rate)
{
	struct clk *disp_pll;
	int ret = -EINVAL;

	disp_pll = clk_get(NULL, "dpll_disp_ck");
	if (IS_ERR(disp_pll)) {
		pr_err("Cannot clk_get disp_pll\n");
		goto out;
	}

	ret = clk_set_rate(disp_pll, rate);
	clk_put(disp_pll);
out:
	return ret;
}


#define  LCD_BASE_ADDR	0x4830E000

static struct resource dxlLcd_resources[] = {
	[0] = {
		.start = LCD_BASE_ADDR,	
		.end   = LCD_BASE_ADDR+SZ_4K-1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = AM33XX_IRQ_LCD,
		.end   = AM33XX_IRQ_LCD,
		.flags = IORESOURCE_IRQ,
	}
};

static struct platform_device dxlLcd_device = {
	.name          = "dxlLcd",
	.id            = 0,
	.num_resources = ARRAY_SIZE(dxlLcd_resources),
	.resource      = dxlLcd_resources,
};

static void dxlLcd_display_init(int evm_id, int profile)
{
	setup_pin_mux(lcdc_pin_mux_gpio);

	platform_device_register(&dxlLcd_device);
	pr_info("Initialized dxlLcd LCD driver.\n");

}

static void rmii1_init(int evm_id, int profile)
{
	setup_pin_mux(rmii1_pin_mux);
	return;
}

static void usb0_init(int evm_id, int profile)
{
	setup_pin_mux(usb0_pin_mux);
	return;
}

/* setup uart2 */
static void uart2_init(int evm_id, int profile)
{
        setup_pin_mux(uart2_pin_mux);
        return;
}

/* setup uart3 */
static void uart3_init(int evm_id, int profile)
{
	setup_pin_mux(uart3_pin_mux);
	return;
}

/* setup uart4 */
static void uart4_init(int evm_id, int profile)
{
        setup_pin_mux(uart4_pin_mux);
        return;
}

/* setup uart5 */
static void uart5_init(int evm_id, int profile)
{
        setup_pin_mux(uart5_pin_mux);
        return;
}

/* NAND partition information */
static struct mtd_partition am335x_nand_partitions[] = {
/* All the partition sizes are listed in terms of NAND block size */
	{
		.name           = "Kernel",
		.offset         = 0,  			 /* Offset = 0x000000 */
		.size           = 80 * SZ_128K,
	},
	{
		.name           = "File System",
		.offset         = MTDPART_OFS_APPEND,   /* Offset = 0x780000 */
		.size           = MTDPART_SIZ_FULL,
	},
};

/* SPI 0/1 Platform Data */
static struct mtd_partition am335x_spi_partitions[] = {
	/* All the partition sizes are listed in terms of erase size */
	{
		.name       = "SPL",
		.offset     = 0,			/* Offset = 0x0 */
		.size       = SZ_128K,
	},
	{
		.name       = "U-Boot",
		.offset     = MTDPART_OFS_APPEND,	/* Offset = 0x20000 */
		.size       = SZ_512K + SZ_256K,
	},
	{
		.name       = "U-Boot Env",
		.offset     = MTDPART_OFS_APPEND,	/* Offset = 0x60000 */
		.size       = MTDPART_SIZ_FULL,
	},
};

static const struct flash_platform_data am335x_spi_flash = {
	.type      = "st25vf016b",
	.name      = "spi_flash",
	.parts     = am335x_spi_partitions,
	.nr_parts  = ARRAY_SIZE(am335x_spi_partitions),
};

/*
 * SPI Flash works at 80Mhz however SPI Controller works at 48MHz.
 * So setup Max speed to be less than that of Controller speed
 */
static struct spi_board_info am335x_spi0_slave_info[] = {
	{
		.modalias      = "m25p80",
		.platform_data = &am335x_spi_flash,
		.irq           = -1,
		.max_speed_hz  = 24000000,
		.bus_num       = 1,
		.chip_select   = 0,
	},
};

static struct spi_board_info am335x_spi1_slave_info[] = {
        {
                .modalias      = "spidev1",
                //.platform_data = &user_defined,
                .irq           = -1,
                .max_speed_hz  = 8000000,
                .bus_num       = 2,
		.mode	       = SPI_MODE_0,
                .chip_select   = 0,
        },
	{
                .modalias      = "spidev2",
                //.platform_data = &user_defined,
                .irq           = -1,
                .max_speed_hz  = 8000000,
                .bus_num       = 2,
		.mode          = SPI_MODE_0,
                .chip_select   = 1,
        },
};

static struct gpmc_timings am335x_nand_timings = {
	.sync_clk = 0,

	.cs_on = 0,
	.cs_rd_off = 44,
	.cs_wr_off = 44,

	.adv_on = 6,
	.adv_rd_off = 34,
	.adv_wr_off = 44,
	.we_off = 40,
	.oe_off = 54,

	.access = 64,
	.rd_cycle = 82,
	.wr_cycle = 82,

	.wr_access = 40,
	.wr_data_mux_bus = 0,
};

static void evm_nand_init(int evm_id, int profile)
{
	struct omap_nand_platform_data *pdata;
	struct gpmc_devices_info gpmc_device[2] = {
		{ NULL, 0 },
		{ NULL, 0 },
	};

	setup_pin_mux(nand_pin_mux);
	pdata = omap_nand_init(am335x_nand_partitions,
		ARRAY_SIZE(am335x_nand_partitions), 0, 0,
		&am335x_nand_timings);
	if (!pdata)
		return;
	pdata->ecc_opt =OMAP_ECC_BCH8_CODE_HW;
	pdata->elm_used = true;
	gpmc_device[0].pdata = pdata;
	gpmc_device[0].flag = GPMC_DEVICE_NAND;

	omap_init_gpmc(gpmc_device, sizeof(gpmc_device));
	omap_init_elm();
}

/*Custom GPIO ID Table */
static struct gpio_info_button dexcel_gpio_buttons[] = {
	{
		.gpio			= GPIO_TO_PIN(2, 18),
		.direction		= OUTPUT,
		.default_level		= LOW,
		.id			= 1,
		.name			= "gpio_led_1",
	},
	{
		.gpio           	= GPIO_TO_PIN(3, 20),
                .direction      	= OUTPUT,
                .default_level  	= LOW,
                .id             	= 2,
                .name           	= "gpio_led_2",
        },
	{
		.gpio           	= GPIO_TO_PIN(1, 30),
                .direction      	= INPUT,
                .id             	= 3,
                .name           	= "gpio_jmp_2",
	},
	{
                .gpio           	= GPIO_TO_PIN(1, 31),
                .direction      	= INPUT,
                .id             	= 4,
                .name           	= "gpio_jmp_1",
        },
	{
                .gpio           	= GPIO_TO_PIN(1, 23),
                .direction      	= INPUT,
                .irq_enable     	= ENABLE,
		.debounce_interval	= 5,
                .id             	= 5,
                .name           	= "gpio_switch",
        },
};

/* Calixto EVM MAC Address initialization */
void calixto_evm_phy_int(void){

#define EEPROM_NO_OF_MAC_ADDR         3
static char am335x_mac_addr[EEPROM_NO_OF_MAC_ADDR][ETH_ALEN];

  am335x_mac_addr[0][0] = 0xd4;
  am335x_mac_addr[0][1] = 0x94;
  am335x_mac_addr[0][2] = 0xa1;
  am335x_mac_addr[0][3] = 0x38;
  am335x_mac_addr[0][4] = 0xed;
  am335x_mac_addr[0][5] = 0x8b;
  am335x_mac_addr[1][0] = 0xd4;
  am335x_mac_addr[1][1] = 0x94;
  am335x_mac_addr[1][2] = 0xa1;
  am335x_mac_addr[1][3] = 0x38;
  am335x_mac_addr[1][4] = 0xed;
  am335x_mac_addr[1][5] = 0x8c;

  am33xx_cpsw_macidfillup(&am335x_mac_addr[0][0],
                          &am335x_mac_addr[1][0]);

}

static void mmc2_wl12xx_init(int evm_id, int profile)
{
	setup_pin_mux(mmc2_wl12xx_pin_mux);

	am335x_mmc[1].mmc = 3;
	am335x_mmc[1].name = "wl1271";
	am335x_mmc[1].caps = MMC_CAP_4_BIT_DATA | MMC_CAP_POWER_OFF_CARD;
	am335x_mmc[1].nonremovable = true;
	am335x_mmc[1].gpio_cd = -EINVAL;
	am335x_mmc[1].gpio_wp = -EINVAL;
	am335x_mmc[1].ocr_mask = MMC_VDD_32_33 | MMC_VDD_33_34; /* 3V3 */

	/* mmc will be initialized when mmc0_init is called */
	return;
}

static void uart1_wl12xx_init(int evm_id, int profile)
{
	setup_pin_mux(uart1_wl12xx_pin_mux);
}

static void wl12xx_bluetooth_enable(void)
{
	int status = gpio_request(am335xevm_wlan_data.bt_enable_gpio,
		"bt_en\n");
	if (status < 0)
		pr_err("Failed to request gpio for bt_enable");

	pr_info("Configure Bluetooth Enable pin...\n");
	gpio_direction_output(am335xevm_wlan_data.bt_enable_gpio, 0);
}

static int wl12xx_set_power(struct device *dev, int slot, int on, int vdd)
{
	if (on) {
		gpio_direction_output(am335xevm_wlan_data.wlan_enable_gpio, 1);
		mdelay(70);
	} else {
		gpio_direction_output(am335xevm_wlan_data.wlan_enable_gpio, 0);
	}

	return 0;
}

static void wl12xx_init(int evm_id, int profile)
{
	struct device *dev;
	struct omap_mmc_platform_data *pdata;
	int ret;

	setup_pin_mux(wl12xx_pin_mux);
	wl12xx_bluetooth_enable();

	if (wl12xx_set_platform_data(&am335xevm_wlan_data))
		pr_err("error setting wl12xx data\n");

	dev = am335x_mmc[1].dev;
	if (!dev) {
		pr_err("wl12xx mmc device initialization failed\n");
		goto out;
	}

	pdata = dev->platform_data;
	if (!pdata) {
		pr_err("Platfrom data of wl12xx device not set\n");
		goto out;
	}

	ret = gpio_request_one(am335xevm_wlan_data.wlan_enable_gpio,
		GPIOF_OUT_INIT_LOW, "wlan_en");
	if (ret) {
		pr_err("Error requesting wlan enable gpio: %d\n", ret);
		goto out;
	}

	pdata->slots[0].set_power = wl12xx_set_power;
out:
	return;
}

/*Calixto EVM MMC0 Interface */
static void mmc0_init(int evm_id, int profile)
{
     setup_pin_mux(mmc0_pin_mux);
     omap2_hsmmc_init(am335x_mmc);
     return;
}

/* setup spi0 */
static void spi0_init(int evm_id, int profile)
{
	setup_pin_mux(spi0_pin_mux);
	spi_register_board_info(am335x_spi0_slave_info,
			ARRAY_SIZE(am335x_spi0_slave_info));
	return;
}

/* setup spi1 */
static void spi1_init(int evm_id, int profile)
{
        setup_pin_mux(spi1_pin_mux);
        spi_register_board_info(am335x_spi1_slave_info,
                        ARRAY_SIZE(am335x_spi1_slave_info));
        return;
}

/*setup ADC for am335x */
static void mfd_adc_init(int evm_id, int profile)
{
	int err;
	
	err = am33xx_register_mfd_tscadc(&tscadc);
	if (err)
		pr_err("failed to register touchscreen device\n");
}

/* gpio key init */
static struct gpio_info_platform_data dexcel_gpio_key_info = {
                .buttons        = dexcel_gpio_buttons,
                .nbuttons       = ARRAY_SIZE(dexcel_gpio_buttons),
};

/* gpio key platform data */
static struct platform_device dexcel_gpio_regs = {
                .name                   = "gpio_driver",
                .id                     = -1,
                .dev.platform_data      = &dexcel_gpio_key_info,
};

static void gpio_init(int evm_id, int profile)
{
	int err;
	setup_pin_mux(gpio_pin_mux);
	err = platform_device_register(&dexcel_gpio_regs);

	  if(err)
		printk("Error while registering gpio_driver\n ");

	return;
}

static struct omap_rtc_pdata am335x_rtc_info = {
	.pm_off		= false,
	.wakeup_capable	= 0,
};

static void am335x_rtc_init(int evm_id, int profile)
{
	void __iomem *base;
	struct clk *clk;
	struct omap_hwmod *oh;
	struct platform_device *pdev;
	char *dev_name = "am33xx-rtc";

	clk = clk_get(NULL, "rtc_fck");
	if (IS_ERR(clk)) {
		pr_err("rtc : Failed to get RTC clock\n");
		return;
	}

	if (clk_enable(clk)) {
		pr_err("rtc: Clock Enable Failed\n");
		return;
	}

	base = ioremap(AM33XX_RTC_BASE, SZ_4K);

	if (WARN_ON(!base))
		return;

	/* Unlock the rtc's registers */
	writel(0x83e70b13, base + 0x6c);
	writel(0x95a4f1e0, base + 0x70);

	/*
	 * Enable the 32K OSc
	 * TODO: Need a better way to handle this
	 * Since we want the clock to be running before mmc init
	 * we need to do it before the rtc probe happens
	 */
	writel(0x48, base + 0x54);

	iounmap(base);

	switch (evm_id) {
	case BEAGLE_BONE_A3:
	case BEAGLE_BONE_OLD:
		am335x_rtc_info.pm_off = true;
		break;
	default:
		break;
	}

	clk_disable(clk);
	clk_put(clk);

	if (omap_rev() >= AM335X_REV_ES2_0)
		am335x_rtc_info.wakeup_capable = 1;

	oh = omap_hwmod_lookup("rtc");
	if (!oh) {
		pr_err("could not look up %s\n", "rtc");
		return;
	}

	pdev = omap_device_build(dev_name, -1, oh, &am335x_rtc_info,
			sizeof(struct omap_rtc_pdata), NULL, 0, 0);
	WARN(IS_ERR(pdev), "Can't build omap_device for %s:%s.\n",
			dev_name, oh->name);
}

/* Enable clkout2 */
static struct pinmux_config clkout2_pin_mux[] = {
	{"xdma_event_intr1.clkout2", OMAP_MUX_MODE3 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};

static void clkout2_enable(int evm_id, int profile)
{
	struct clk *ck_32;

	ck_32 = clk_get(NULL, "clkout2_ck");
	if (IS_ERR(ck_32)) {
		pr_err("Cannot clk_get ck_32\n");
		return;
	}

	clk_enable(ck_32);
	setup_pin_mux(clkout2_pin_mux);
}

static void sgx_init(int evm_id, int profile)
{
	if (omap3_has_sgx()) {
		am33xx_gpu_init();
	}
}

/* Calixto EVM Interface Configuration */
static struct evm_dev_cfg calixto_dev_cfg[] = {
	{am335x_rtc_init, DEV_ON_BASEBOARD, PROFILE_NONE},
	{clkout2_enable,  DEV_ON_BASEBOARD, PROFILE_NONE},
	{dxlLcd_display_init, DEV_ON_BASEBOARD,PROFILE_NONE},
	{mfd_adc_init,	  DEV_ON_BASEBOARD, PROFILE_NONE},
	{spi0_init,	  DEV_ON_BASEBOARD, PROFILE_NONE},
	{spi1_init,	  DEV_ON_BASEBOARD, PROFILE_NONE},
	{evm_nand_init,   DEV_ON_BASEBOARD, PROFILE_NONE},
	{rmii1_init,	  DEV_ON_BASEBOARD, PROFILE_NONE},
	{usb0_init,	  DEV_ON_BASEBOARD, PROFILE_NONE},
        {mmc2_wl12xx_init,	DEV_ON_BASEBOARD, PROFILE_NONE},
        {uart1_wl12xx_init,	DEV_ON_BASEBOARD, PROFILE_NONE},
        {mmc0_init,	DEV_ON_BASEBOARD, PROFILE_NONE},
        {wl12xx_init,	DEV_ON_BASEBOARD, PROFILE_NONE},
        {uart2_init,	DEV_ON_BASEBOARD, PROFILE_NONE},
        {uart3_init,	DEV_ON_BASEBOARD, PROFILE_NONE},
        {uart4_init,	DEV_ON_BASEBOARD, PROFILE_NONE},
        {uart5_init,	DEV_ON_BASEBOARD, PROFILE_NONE},
        {gpio_init,	DEV_ON_BASEBOARD, PROFILE_NONE},
	{NULL, 0, 0},
};

#define AM33XX_VDD_CORE_OPP50_UV	1100000
#define AM33XX_OPP120_FREQ		600000000
#define AM33XX_OPPTURBO_FREQ		720000000

#define AM33XX_ES2_0_VDD_CORE_OPP50_UV	950000
#define AM33XX_ES2_0_OPP120_FREQ	720000000
#define AM33XX_ES2_0_OPPTURBO_FREQ	800000000
#define AM33XX_ES2_0_OPPNITRO_FREQ	1000000000

#define AM33XX_ES2_1_VDD_CORE_OPP50_UV	950000
#define AM33XX_ES2_1_OPP120_FREQ	720000000
#define AM33XX_ES2_1_OPPTURBO_FREQ	800000000
#define AM33XX_ES2_1_OPPNITRO_FREQ	1000000000

static void am335x_opp_update(void)
{
	u32 rev;
	int voltage_uv = 0;
	struct device *core_dev, *mpu_dev;
	struct regulator *core_reg;

	core_dev = omap_device_get_by_hwmod_name("l3_main");
	mpu_dev = omap_device_get_by_hwmod_name("mpu");

	if (!mpu_dev || !core_dev) {
		pr_err("%s: Aiee.. no mpu/core devices? %p %p\n", __func__,
		       mpu_dev, core_dev);
		return;
	}

	core_reg = regulator_get(core_dev, "vdd_core");
	if (IS_ERR(core_reg)) {
		pr_err("%s: unable to get core regulator\n", __func__);
		return;
	}

	/*
	 * Ensure physical regulator is present.
	 * (e.g. could be dummy regulator.)
	 */
	voltage_uv = regulator_get_voltage(core_reg);
	if (voltage_uv < 0) {
		pr_err("%s: physical regulator not present for core" \
		       "(%d)\n", __func__, voltage_uv);
		regulator_put(core_reg);
		return;
	}

	pr_debug("%s: core regulator value %d\n", __func__, voltage_uv);
	if (voltage_uv > 0) {
		rev = omap_rev();
		switch (rev) {
		case AM335X_REV_ES1_0:
			if (voltage_uv <= AM33XX_VDD_CORE_OPP50_UV) {
				/*
				 * disable the higher freqs - we dont care about
				 * the results
				 */
				opp_disable(mpu_dev, AM33XX_OPP120_FREQ);
				opp_disable(mpu_dev, AM33XX_OPPTURBO_FREQ);
			}
			break;
		case AM335X_REV_ES2_0:
			if (voltage_uv <= AM33XX_ES2_0_VDD_CORE_OPP50_UV) {
				/*
				 * disable the higher freqs - we dont care about
				 * the results
				 */
				opp_disable(mpu_dev,
					    AM33XX_ES2_0_OPP120_FREQ);
				opp_disable(mpu_dev,
					    AM33XX_ES2_0_OPPTURBO_FREQ);
				opp_disable(mpu_dev,
					    AM33XX_ES2_0_OPPNITRO_FREQ);
			}
			break;
		case AM335X_REV_ES2_1:
		/* FALLTHROUGH */
		default:
			if (voltage_uv <= AM33XX_ES2_1_VDD_CORE_OPP50_UV) {
				/*
				 * disable the higher freqs - we dont care about
				 * the results
				 */
				opp_disable(mpu_dev,
					    AM33XX_ES2_1_OPP120_FREQ);
				opp_disable(mpu_dev,
					    AM33XX_ES2_1_OPPTURBO_FREQ);
				opp_disable(mpu_dev,
					    AM33XX_ES2_1_OPPNITRO_FREQ);
			}
			break;
		}
	}
}

static void setup_calixto_evm_board(void)
{
        
    pr_info("SOM Configuration : AM335x Calixto NXT Module.\n");
    pr_info("Custom : Dexcel Viper Interface Card\n");

    calixto_evm_phy_int();
    /* EVM has Micro-SD slot which doesn't have Write Protect pin */
    am335x_mmc[0].gpio_wp = -EINVAL;

    _configure_device(CALIXTO_EVM, calixto_dev_cfg, PROFILE_NONE);

     am33xx_cpsw_init(CALIXTO_EVM_ETHERNET_INTERFACE, NULL, NULL);

     //am335x_opp_update();
}

static struct at24_platform_data am335x_eeprom_info = {
	.byte_len       = (16*1024) / 8,
	.page_size      = 256,
	//.flags          = AT24_FLAG_ADDR8,
	//.setup          = am335x_setup_daughter_board,
	.context        = (void *)NULL,
};

static struct i2c_board_info __initdata am335x_i2c0_boardinfo[] = {
	{
		I2C_BOARD_INFO("24mac402", 0x52),
		.platform_data = &am335x_eeprom_info,
	},
	{
                I2C_BOARD_INFO("pcf8563", 0x51),
        },
	{
		I2C_BOARD_INFO("max31790_controller", 0x20),
	},
};

static struct omap_musb_board_data musb_board_data = {
	.interface_type	= MUSB_INTERFACE_ULPI,
	/*
	 * mode[0:3] = USB0PORT's mode
	 * mode[4:7] = USB1PORT's mode
	 * AM335X beta EVM has USB0 in OTG mode and USB1 in host mode.
	 */
	.mode           = MUSB_HOST,
	.power		= 500,
	.instances	= 1,
};

static void __iomem *am33xx_i2c0_base;

int am33xx_map_i2c0(void)
{
	am33xx_i2c0_base = ioremap(AM33XX_I2C0_BASE, SZ_4K);

	if (!am33xx_i2c0_base)
		return -ENOMEM;

	return 0;
}

void __iomem *am33xx_get_i2c0_base(void)
{
	return am33xx_i2c0_base;
}

static void __init am335x_evm_i2c_init(void)
{
    omap_register_i2c_bus(1, 100, am335x_i2c0_boardinfo,
                             ARRAY_SIZE(am335x_i2c0_boardinfo));
}

void __iomem *am33xx_emif_base;

void __iomem * __init am33xx_get_mem_ctlr(void)
{

	am33xx_emif_base = ioremap(AM33XX_EMIF0_BASE, SZ_32K);

	if (!am33xx_emif_base)
		pr_warning("%s: Unable to map DDR2 controller",	__func__);

	return am33xx_emif_base;
}

void __iomem *am33xx_get_ram_base(void)
{
	return am33xx_emif_base;
}

void __iomem *am33xx_gpio0_base;

void __iomem *am33xx_get_gpio0_base(void)
{
	am33xx_gpio0_base = ioremap(AM33XX_GPIO0_BASE, SZ_4K);

	return am33xx_gpio0_base;
}

static struct resource am33xx_cpuidle_resources[] = {
	{
		.start		= AM33XX_EMIF0_BASE,
		.end		= AM33XX_EMIF0_BASE + SZ_32K - 1,
		.flags		= IORESOURCE_MEM,
	},
};

static struct am33xx_cpuidle_config am33xx_cpuidle_pdata = {
};

static struct platform_device am33xx_cpuidle_device = {
	.name			= "cpuidle-am33xx",
	.num_resources		= ARRAY_SIZE(am33xx_cpuidle_resources),
	.resource		= am33xx_cpuidle_resources,
	.dev = {
		.platform_data	= &am33xx_cpuidle_pdata,
	},
};

static void __init am33xx_cpuidle_init(void)
{
	int ret;

	am33xx_cpuidle_pdata.emif_base = am33xx_get_mem_ctlr();

	ret = platform_device_register(&am33xx_cpuidle_device);

	if (ret)
		pr_warning("AM33XX cpuidle registration failed\n");

}

static void __init am335x_evm_init(void)
{
	am33xx_cpuidle_init();
	am33xx_mux_init(board_mux);
	omap_serial_init();
	am335x_evm_i2c_init();
	omap_sdrc_init(NULL, NULL);
	usb_musb_init(&musb_board_data);
	/* Create an alias for icss clock */
	if (clk_add_alias("pruss", NULL, "pruss_uart_gclk", NULL))
		pr_warn("failed to create an alias: icss_uart_gclk --> pruss\n");
	/* Create an alias for gfx/sgx clock */
	if (clk_add_alias("sgx_ck", NULL, "gfx_fclk", NULL))
		pr_warn("failed to create an alias: gfx_fclk --> sgx_ck\n");

	setup_calixto_evm_board();
}

static void __init am335x_evm_map_io(void)
{
	omap2_set_globals_am33xx();
	omapam33xx_map_common_io();
}

MACHINE_START(AM335XEVM, "am335xevm")
	/* Maintainer: Texas Instruments */
	.atag_offset	= 0x100,
	.map_io		= am335x_evm_map_io,
	.init_early	= am33xx_init_early,
	.init_irq	= ti81xx_init_irq,
	.handle_irq     = omap3_intc_handle_irq,
	.timer		= &omap3_am33xx_timer,
	.init_machine	= am335x_evm_init,
MACHINE_END

MACHINE_START(AM335XIAEVM, "am335xiaevm")
	/* Maintainer: Texas Instruments */
	.atag_offset	= 0x100,
	.map_io		= am335x_evm_map_io,
	.init_irq	= ti81xx_init_irq,
	.init_early	= am33xx_init_early,
	.timer		= &omap3_am33xx_timer,
	.init_machine	= am335x_evm_init,
MACHINE_END