/*
 * Copyright (C) 2013 HandEra, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/nodemask.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/i2c.h>
#include <linux/i2c/pca953x.h>
#include <linux/ata.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/regulator/consumer.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/ipu.h>
#include <linux/mxcfb.h>
#include <linux/pwm_backlight.h>
#include <linux/fec.h>
#include <linux/memblock.h>
#include <linux/gpio.h>
#include <linux/etherdevice.h>
#include <linux/regulator/anatop-regulator.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/mfd/mxc-hdmi-core.h>

#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/mxc_dvfs.h>
#include <mach/memory.h>
#include <mach/iomux-mx6q.h>
#include <mach/imx-uart.h>
#include <mach/viv_gpu.h>
#include <mach/ahci_sata.h>
#include <mach/ipu-v3.h>
#include <mach/mxc_hdmi.h>
#include <mach/mxc_asrc.h>
#include <mach/mipi_dsi.h>

#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>

#include "usb.h"
#include "devices-imx6q.h"
#include "crm_regs.h"
#include "cpu_op-mx6.h"
#include "board-amherst.h"

/* SPI Chip Selects */
#define AMHERST_SPI2_SS2	IMX_GPIO_NR(3, 24)
#define AMHERST_SPI2_SS3	IMX_GPIO_NR(3, 25)

/* Ethernet GPIOs */
#define AMHERST_ENET_RSTF	IMX_GPIO_NR(1, 3)
#define AMHERST_ENET_INTF	IMX_GPIO_NR(4, 10)

/* USB GPIOs */
#define AMHERST_USB_OTG_PWR	IMX_GPIO_NR(3, 22)
#define AMHERST_USB_H1_PWR	IMX_GPIO_NR(3, 31)

/* SDHC */
#define AMHERST_SD2_PWR	IMX_GPIO_NR(7, 13)
#define AMHERST_SD2_CD	IMX_GPIO_NR(1, 4)
#define AMHERST_SD2_WP	IMX_GPIO_NR(1, 2)

/* WiFi */
#define AMHERST_WIFI_PWR	IMX_GPIO_NR(7, 1)
#define AMHERST_WIFI_RSTF	IMX_GPIO_NR(7, 8)
#define AMHERST_WIFI_WAKEUP	IMX_GPIO_NR(7, 0)

/* Option PWM */
#define AMHERST_OPTION_PWM	IMX_GPIO_NR(1, 19)

/* Display PWM and Control */
#define AMHERST_EXT_PWM	IMX_GPIO_NR(1, 9)
#define AMHERST_EXT_VID_EN	IMX_GPIO_NR(3, 19)

/* Relay */
#define AMHERST_EXT_RELAY	IMX_GPIO_NR(3, 20)

/* PCIe */
#define AMHERST_PCIE_WAKEF	IMX_GPIO_NR(5, 20)
#define AMHERST_PCIE_RSTF	IMX_GPIO_NR(7, 12)
#define AMHERST_PCIE_DISF	IMX_GPIO_NR(4, 13)

/* LEDs */
#define AMHERST_LED_BLU	IMX_GPIO_NR(4, 28)
#define AMHERST_LED_GRN	IMX_GPIO_NR(4, 29)


static int caam_enabled;

extern char *gp_reg_id;
extern char *soc_reg_id;
extern char *pu_reg_id;
extern int epdc_enabled;

/* SDHC Setup */
// micro-SD card
static const struct esdhc_platform_data amherst_sd2_data __initconst = {
	.cd_gpio = AMHERST_SD2_CD,
	.wp_gpio = AMHERST_SD2_WP,
	.keep_power_at_suspend = 1,
	.support_18v = 0,
	.support_8bit = 0,
	.delay_line = 0,
	.cd_type = ESDHC_CD_GPIO,
	.cd_inverted = 1,
};

// SDIO WLAN
static const struct esdhc_platform_data amherst_sd3_data __initconst = {
	.always_present = 1,
	.keep_power_at_suspend = 1,
	.support_18v = 0,
	.support_8bit = 0,
	.delay_line = 0,
	.cd_type = ESDHC_CD_PERMANENT,
};

// eMMC
static const struct esdhc_platform_data amherst_sd4_data __initconst = {
	.always_present = 1,
	.keep_power_at_suspend = 1,
	.support_18v = 0,
	.support_8bit = 1,
	.delay_line = 0,
	.cd_type = ESDHC_CD_PERMANENT,
};

static inline void imx6_init_sdhc(void)
{
	int ret = 0;

	// Move sd4 to first because sd4 connect to emmc.
	// Mfgtools want emmc is mmcblk0 and other sd cards mmcblkX.
	// ToDo: Add/Edit udev rule to always make sd4 mmcblk0
	imx6q_add_sdhci_usdhc_imx(3, &amherst_sd4_data);
	imx6q_add_sdhci_usdhc_imx(1, &amherst_sd2_data);
	imx6q_add_sdhci_usdhc_imx(2, &amherst_sd3_data);

	// Turn on power to SD2
	ret = gpio_request(AMHERST_SD2_PWR, "sdhc2-pwr");
	if (ret) {
		pr_err("failed to get GPIO AMHERST_SD2_PWR: %d\n",
			ret);
		return;
	}
	gpio_direction_output(AMHERST_SD2_PWR, 1);
}

/* Thermal Setup */
static const struct anatop_thermal_platform_data
	amherst_anatop_thermal_data __initconst = {
		.name = "anatop_thermal",
};

/* UART Setup */
static inline void amherst_init_uart(void)
{
	imx6q_add_imx_uart(0, NULL);
	imx6q_add_imx_uart(1, NULL);
	imx6q_add_imx_uart(3, NULL);
	imx6q_add_imx_uart(4, NULL);
}

/* Ethernet setup */
static int amherst_fec_phy_init(struct phy_device *phydev)
{
	unsigned short val;

	// Check phy power
	val = phy_read(phydev, MII_BMCR);

	if (val & BMCR_PDOWN)
	{
		phy_write(phydev, MII_BMCR, (val & ~BMCR_PDOWN));
	}

	// Enable auto-negotiation
	val = phy_read(phydev, MII_BMCR);
	val |= BMCR_ANENABLE;
	phy_write(phydev, MII_BMCR, val);

	return 0;
}

static struct fec_platform_data fec_data __initdata = {
	.init = amherst_fec_phy_init,
	.phy = PHY_INTERFACE_MODE_RMII,
	.gpio_irq = -1,
	.gpio_reset = AMHERST_ENET_RSTF,
	.phy_reset_usec = 750,		// 500 usec min reset pulse for SMSC LAN8720A
};

/* SPI Setup */
static int amherst_spi_cs[] = {
	AMHERST_SPI2_SS2,
	AMHERST_SPI2_SS3,
};

static const struct spi_imx_master amherst_spi_data __initconst = {
	.chipselect     = amherst_spi_cs,
	.num_chipselect = ARRAY_SIZE(amherst_spi_cs),
};

#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_MTD_M25P80_MODULE)
static struct mtd_partition amherst_spi_nor_partitions[] = {
	{
	 .name = "bootloader",
	 .offset = 0,
	 .size = 0x00100000,
	},
	{
	 .name = "kernel",
	 .offset = MTDPART_OFS_APPEND,
	 .size = MTDPART_SIZ_FULL,
	},
};

static struct flash_platform_data amherst__spi_flash_data = {
	.name = "m25p80",
/*	.parts = imx6_sabresd_spi_nor_partitions,
	.nr_parts = ARRAY_SIZE(imx6_sabresd_spi_nor_partitions),*/
	.type = "sst25vf016b",
};
#endif

static struct spi_board_info amherst_spi_nor_device[] __initdata = {
#if defined(CONFIG_MTD_M25P80)
	{
		.modalias = "m25p80",
		.max_speed_hz = 20000000, /* max spi clock (SCK) speed in HZ */
		.bus_num = 1,
		.chip_select = 1,
		.platform_data = &amherst__spi_flash_data,
	},
#endif
};

static void spi_device_init(void)
{
	imx6q_add_ecspi(1, &amherst_spi_data);
	spi_register_board_info(amherst_spi_nor_device,
				ARRAY_SIZE(amherst_spi_nor_device));
}

/* Audio Setup */
static struct imx_ssi_platform_data amherst_ssi_pdata = {
	.flags = IMX_SSI_DMA | IMX_SSI_SYN,
};

static struct mxc_audio_platform_data amherst_audio_data;

static int amherst_sgtl5000_init(void)
{
	struct clk *clko;
	struct clk *new_parent;
	int rate;

	clko = clk_get(NULL, "clko_clk");
	if (IS_ERR(clko)) {
		pr_err("can't get CLKO clock.\n");
		return PTR_ERR(clko);
	}
	new_parent = clk_get(NULL, "ahb");
	if (!IS_ERR(new_parent)) {
		clk_set_parent(clko, new_parent);
		clk_put(new_parent);
	}
	rate = clk_round_rate(clko, 24000000);
	if (rate < 8000000 || rate > 27000000) {
		pr_err("Error:SGTL5000 mclk freq %d out of range!\n", rate);
		clk_put(clko);
		return -1;
	}

	amherst_audio_data.sysclk = rate;
	clk_set_rate(clko, rate);
	clk_enable(clko);

	return 0;
}

static struct mxc_audio_platform_data amherst_audio_data = {
	.ssi_num = 1,
	.src_port = 2,
	.ext_port = 3,
	.init = amherst_sgtl5000_init,
	.hp_gpio = -1,
};

static struct platform_device amherst_audio_device = {
	.name = "imx-sgtl5000",
};

#ifdef CONFIG_SND_SOC_SGTL5000

static struct regulator_consumer_supply sgtl5000_amherst_consumer_vdda = {
	.supply = "VDDA",
	.dev_name = "0-000a",
};

static struct regulator_consumer_supply sgtl5000_amherst_consumer_vddio = {
	.supply = "VDDIO",
	.dev_name = "0-000a",
};

static struct regulator_consumer_supply sgtl5000_amherst_consumer_vddd = {
	.supply = "VDDD",
	.dev_name = "0-000a",
};

static struct regulator_init_data sgtl5000_amherst_vdda_reg_initdata = {
	.num_consumer_supplies = 1,
	.consumer_supplies = &sgtl5000_amherst_consumer_vdda,
};

static struct regulator_init_data sgtl5000_amherst_vddio_reg_initdata = {
	.num_consumer_supplies = 1,
	.consumer_supplies = &sgtl5000_amherst_consumer_vddio,
};

static struct regulator_init_data sgtl5000_amherst_vddd_reg_initdata = {
	.num_consumer_supplies = 1,
	.consumer_supplies = &sgtl5000_amherst_consumer_vddd,
};

static struct fixed_voltage_config sgtl5000_amherst_vdda_reg_config = {
	.supply_name		= "VDDA",
	.microvolts		= 3300000,
	.gpio			= -1,
	.init_data		= &sgtl5000_amherst_vdda_reg_initdata,
};

static struct fixed_voltage_config sgtl5000_amherst_vddio_reg_config = {
	.supply_name		= "VDDIO",
	.microvolts		= 3300000,
	.gpio			= -1,
	.init_data		= &sgtl5000_amherst_vddio_reg_initdata,
};

static struct fixed_voltage_config sgtl5000_amherst_vddd_reg_config = {
	.supply_name		= "VDDD",
	.microvolts		= 0,
	.gpio			= -1,
	.init_data		= &sgtl5000_amherst_vddd_reg_initdata,
};

static struct platform_device sgtl5000_amherst_vdda_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 0,
	.dev	= {
		.platform_data = &sgtl5000_amherst_vdda_reg_config,
	},
};

static struct platform_device sgtl5000_amherst_vddio_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 1,
	.dev	= {
		.platform_data = &sgtl5000_amherst_vddio_reg_config,
	},
};

static struct platform_device sgtl5000_amherst_vddd_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 2,
	.dev	= {
		.platform_data = &sgtl5000_amherst_vddd_reg_config,
	},
};

#endif /* CONFIG_SND_SOC_SGTL5000 */

static int imx6q_init_audio(void)
{
	mxc_register_device(&amherst_audio_device,
			    &amherst_audio_data);
	imx6q_add_imx_ssi(1, &amherst_ssi_pdata);
#ifdef CONFIG_SND_SOC_SGTL5000
	platform_device_register(&sgtl5000_amherst_vdda_reg_devices);
	platform_device_register(&sgtl5000_amherst_vddio_reg_devices);
	platform_device_register(&sgtl5000_amherst_vddd_reg_devices);
#endif
	return 0;
}

/* I2C setup */
static struct imxi2c_platform_data amherst_i2c_data = {
	.bitrate = 100000,
};

static struct i2c_board_info mxc_i2c0_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("sgtl5000", 0x0a),
	},
};

static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {
	{
		// MOE: Check this address
		I2C_BOARD_INFO("mxc_hdmi_i2c", 0x50),
	},
};

static struct i2c_board_info mxc_i2c2_board_info[] __initdata = {
	{
		// MOE: Check this address
		I2C_BOARD_INFO("cmeh19baaa", 0x20),
	},
};

static void __init amherst_init_i2c(void)
{
	imx6q_add_imx_i2c(0, &amherst_i2c_data);
	imx6q_add_imx_i2c(1, &amherst_i2c_data);
	imx6q_add_imx_i2c(2, &amherst_i2c_data);
	i2c_register_board_info(0, mxc_i2c0_board_info,
			ARRAY_SIZE(mxc_i2c0_board_info));
	i2c_register_board_info(1, mxc_i2c1_board_info,
			ARRAY_SIZE(mxc_i2c1_board_info));
	i2c_register_board_info(2, mxc_i2c2_board_info,
			ARRAY_SIZE(mxc_i2c2_board_info));
}

/* USB Setup */
static void amherst_usbotg_vbus(bool on)
{
	if (on)
		gpio_set_value(AMHERST_USB_OTG_PWR, 1);
	else
		gpio_set_value(AMHERST_USB_OTG_PWR, 0);
}

static void __init amherst_init_usb(void)
{
	int ret = 0;

	imx_otg_base = MX6_IO_ADDRESS(MX6Q_USB_OTG_BASE_ADDR);
	ret = gpio_request(AMHERST_USB_OTG_PWR, "usb-pwr");
	if (ret) {
		pr_err("failed to get GPIO AMHERST_USB_OTG_PWR: %d\n",
			ret);
		return;
	}
	gpio_direction_output(AMHERST_USB_OTG_PWR, 0);

	// Note: SMSC USB2512BI Hub datasheet says VBUS_DET input signal must be strapped 
	// to VCC, so set the connected CPU pin to input (ie. high-impedence).
	
    /* keep USB host1 VBUS always on */
	ret = gpio_request(AMHERST_USB_H1_PWR, "usb-h1-pwr");
	if (ret) {
		pr_err("failed to get GPIO AMHERST_USB_H1_PWR: %d\n",
			ret);
		return;
	}
	gpio_direction_output(AMHERST_USB_H1_PWR, 1);
    mx6_set_otghost_vbus_func(amherst_usbotg_vbus);

	// Set bit in GPR to set USB OTG ID pin to GPIO_1
	mxc_iomux_set_gpr_register(1, 13, 1, 1);
}

/* Display Setup */
static struct viv_gpu_platform_data imx6q_gpu_pdata __initdata = {
	.reserved_mem_size = SZ_128M,
};

static struct imx_asrc_platform_data imx_asrc_data = {
	.channel_bits = 4,
	.clk_map_ver = 2,
};

static struct mipi_dsi_platform_data mipi_dsi_pdata = {
	.ipu_id		= 0,
	.disp_id	= 1,
	.lcd_panel	= "TRULY-WVGA",
};

// HandEra: based on the SabreSD BSP, it looks like the third and fourth entries
// are only used for the Quad version of the processor (verify and delete if true)
static struct ipuv3_fb_platform_data amherst_fb_data[] = {
	{ //fb0
	.disp_dev = "ldb",
	.interface_pix_fmt = IPU_PIX_FMT_RGB666,
	.mode_str = "LDB-XGA",
	.default_bpp = 16,
	.int_clk = false,
	}, {
	.disp_dev = "lcd",
	.interface_pix_fmt = IPU_PIX_FMT_RGB565,
	.mode_str = "CLAA-WVGA",
	.default_bpp = 16,
	.int_clk = false,
	}, {
	.disp_dev = "ldb",
	.interface_pix_fmt = IPU_PIX_FMT_RGB666,
	.mode_str = "LDB-SVGA",
	.default_bpp = 16,
	.int_clk = false,
	}, {
	.disp_dev = "ldb",
	.interface_pix_fmt = IPU_PIX_FMT_RGB666,
	.mode_str = "LDB-VGA",
	.default_bpp = 16,
	.int_clk = false,
	},
};

/* HDMI Setup */
static void hdmi_init(int ipu_id, int disp_id)
{
	int hdmi_mux_setting;

	if ((ipu_id > 1) || (ipu_id < 0)) {
		pr_err("Invalid IPU select for HDMI: %d. Set to 0\n", ipu_id);
		ipu_id = 0;
	}

	if ((disp_id > 1) || (disp_id < 0)) {
		pr_err("Invalid DI select for HDMI: %d. Set to 0\n", disp_id);
		disp_id = 0;
	}

	/* Configure the connection between IPU1/2 and HDMI */
	hdmi_mux_setting = 2*ipu_id + disp_id;

	/* GPR3, bits 2-3 = HDMI_MUX_CTL */
	mxc_iomux_set_gpr_register(3, 2, 2, hdmi_mux_setting);

	/* Set HDMI event as SDMA event2 while Chip version later than TO1.2 */
	if (hdmi_SDMA_check())
		mxc_iomux_set_gpr_register(0, 0, 1, 1);
}

static void hdmi_enable_ddc_pin(void)
{
	mxc_iomux_v3_setup_multiple_pads(amherst_hdmi_ddc_pads,
		ARRAY_SIZE(amherst_hdmi_ddc_pads));
}

static void hdmi_disable_ddc_pin(void)
{
}

static struct fsl_mxc_hdmi_platform_data hdmi_data = {
	.init = hdmi_init,
	.enable_pins = hdmi_enable_ddc_pin,
	.disable_pins = hdmi_disable_ddc_pin,
};

static struct fsl_mxc_hdmi_core_platform_data hdmi_core_data = {
	.ipu_id = 0,
	.disp_id = 0,
};

static struct fsl_mxc_lcd_platform_data lcdif_data = {
	.ipu_id = 0,
	.disp_id = 0,
	.default_ifmt = IPU_PIX_FMT_RGB565,
};

static struct fsl_mxc_ldb_platform_data ldb_data = {
	.ipu_id = 1,
	.disp_id = 1,
	.ext_ref = 1,
	.mode = LDB_SEP1,
	.sec_ipu_id = 1,
	.sec_disp_id = 0,
};

static struct imx_ipuv3_platform_data ipu_data[] = {
	{
	.rev = 4,
	.csi_clk[0] = "clko_clk",
	.bypass_reset = false,
	}, {
	.rev = 4,
	.csi_clk[0] = "clko_clk",
	.bypass_reset = false,
	},
};

static struct fsl_mxc_capture_platform_data capture_data[] = {
	{
		.csi = 0,
		.ipu = 0,
		.mclk_source = 0,
		.is_mipi = 0,
	}, {
		.csi = 1,
		.ipu = 0,
		.mclk_source = 0,
		.is_mipi = 1,
	},
};

struct imx_vout_mem {
	resource_size_t res_mbase;
	resource_size_t res_msize;
};

static struct imx_vout_mem vout_mem __initdata = {
	.res_msize = SZ_128M,
};

static void amherst_suspend_enter(void)
{
}

static void amherst_suspend_exit(void)
{
}

static const struct pm_platform_data amherst_pm_data __initconst = {
	.name = "imx_pm",
	.suspend_enter = amherst_suspend_enter,
	.suspend_exit = amherst_suspend_exit,
};

static struct regulator_consumer_supply amherst_vmmc_consumers[] = {
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.1"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.2"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.3"),
};

static struct regulator_init_data amherst_vmmc_init = {
	.num_consumer_supplies = ARRAY_SIZE(amherst_vmmc_consumers),
	.consumer_supplies = amherst_vmmc_consumers,
};

static struct fixed_voltage_config amherst_vmmc_reg_config = {
	.supply_name		= "vmmc",
	.microvolts		= 3300000,
	.gpio			= -1,
	.init_data		= &amherst_vmmc_init,
};

static struct platform_device amherst_vmmc_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 3,
	.dev	= {
		.platform_data = &amherst_vmmc_reg_config,
	},
};

#ifndef CONFIG_IMX_PCIE
static void pcie_3v3_reset(void)
{
	/* reset miniPCIe */
	gpio_request(AMHERST_PCIE_RSTF, "pcie_reset_rebB");
	gpio_direction_output(AMHERST_PCIE_RSTF, 0);
	/* The PCI Express Mini CEM specification states that PREST# is
	deasserted minimum 1ms after 3.3vVaux has been applied and stable*/
	mdelay(1);
	gpio_set_value(AMHERST_PCIE_RSTF, 1);
	gpio_free(AMHERST_PCIE_RSTF);
}
#endif


/* LED Setup */
#if defined(CONFIG_LEDS_TRIGGER) || defined(CONFIG_LEDS_GPIO)

#define GPIO_LED(gpio_led, name_led, act_low, state_suspend, trigger)	\
{									\
	.gpio			= gpio_led,				\
	.name			= name_led,				\
	.active_low		= act_low,				\
	.retain_state_suspended = state_suspend,			\
	.default_state		= 0,					\
	.default_trigger	= "max8903-"trigger,		\
}

static struct gpio_led imx6q_gpio_leds[] = {
	GPIO_LED(AMHERST_LED_BLU, "led_blue", 0, 1,
		"user_led_blue"),
	GPIO_LED(AMHERST_LED_GRN, "led_green", 0, 1,
		"user_led_green"),
};

static struct gpio_led_platform_data imx6q_gpio_leds_data = {
	.leds		= imx6q_gpio_leds,
	.num_leds	= ARRAY_SIZE(imx6q_gpio_leds),
};

static struct platform_device imx6q_gpio_led_device = {
	.name		= "leds-gpio",
	.id		= -1,
	.num_resources  = 0,
	.dev		= {
		.platform_data = &imx6q_gpio_leds_data,
	}
};

static void __init imx6q_add_device_gpio_leds(void)
{
	platform_device_register(&imx6q_gpio_led_device);
}
#else
static void __init imx6q_add_device_gpio_leds(void) {}
#endif

/* DVFS Setup */
static struct mxc_dvfs_platform_data amherst_dvfscore_data = {
	.reg_id = "VDDCORE",
	.soc_id	= "VDDSOC",
	.clk1_id = "cpu_clk",
	.clk2_id = "gpc_dvfs_clk",
	.gpc_cntr_offset = MXC_GPC_CNTR_OFFSET,
	.ccm_cdcr_offset = MXC_CCM_CDCR_OFFSET,
	.ccm_cacrr_offset = MXC_CCM_CACRR_OFFSET,
	.ccm_cdhipr_offset = MXC_CCM_CDHIPR_OFFSET,
	.prediv_mask = 0x1F800,
	.prediv_offset = 11,
	.prediv_val = 3,
	.div3ck_mask = 0xE0000000,
	.div3ck_offset = 29,
	.div3ck_val = 2,
	.emac_val = 0x08,
	.upthr_val = 25,
	.dnthr_val = 9,
	.pncthr_val = 33,
	.upcnt_val = 10,
	.dncnt_val = 10,
	.delay_time = 80,
};

static void __init fixup_mxc_board(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
}

static struct mipi_csi2_platform_data mipi_csi2_pdata = {
	.ipu_id	 = 0,
	.csi_id = 1,
	.v_channel = 0,
	.lanes = 2,
	.dphy_clk = "mipi_pllref_clk",
	.pixel_clk = "emi_clk",
};

static int __init caam_setup(char *__unused)
{
	caam_enabled = 1;
	return 1;
}
early_param("caam", caam_setup);

#define SNVS_LPCR 0x38
static void mx6_snvs_poweroff(void)
{

	void __iomem *mx6_snvs_base =  MX6_IO_ADDRESS(MX6Q_SNVS_BASE_ADDR);
	u32 value;
	value = readl(mx6_snvs_base + SNVS_LPCR);
	/*set TOP and DP_EN bit*/
	writel(value | 0x60, mx6_snvs_base + SNVS_LPCR);
}

static const struct imx_pcie_platform_data amherst_pcie_data __initconst = {
	.pcie_rst	= AMHERST_PCIE_RSTF,
	.pcie_wake_up	= AMHERST_PCIE_WAKEF,
	.pcie_dis	= AMHERST_PCIE_DISF,
};

/*!
 * Board specific initialization.
 */
static void __init mx6_amherst_board_init(void)
{
	int i;
	struct clk *clko, *clko2;
	struct clk *new_parent;
	int rate;
	struct platform_device *voutdev;

	mxc_iomux_v3_setup_multiple_pads(amherst_pads,
		ARRAY_SIZE(amherst_pads));

#ifdef CONFIG_FEC_1588
	/* Set GPIO_16 input for IEEE-1588 ts_clk and RMII reference clock
	 * For MX6 GPR1 bit21 meaning:
	 * Bit21:       0 - GPIO_16 get reference clk from pad
	 *              1 - GPIO_16 get reference clk from internal clk,
	 *		    also sent out to external PHY
	 */
	mxc_iomux_set_gpr_register(1, 21, 1, 1);

	// also setup PHY reset line
	if (gpio_request(AMHERST_ENET_RSTF, "enet-rstf") == 0)
		gpio_direction_output(AMHERST_ENET_RSTF, 1);
	else
		pr_err("failed to get GPIO AMHERST_ENET_RSTF\n");
#endif

	gp_reg_id = amherst_dvfscore_data.reg_id;
	soc_reg_id = amherst_dvfscore_data.soc_id;

	/* UARTs */
	amherst_init_uart();

	/*
	 * MX6DL/Solo only supports single IPU
	 * The following codes are used to change ipu id
	 * and display id information for MX6DL/Solo. Then
	 * register 1 IPU device and up to 2 displays for
	 * MX6DL/Solo
	 */
	if (cpu_is_mx6dl()) {
		ldb_data.ipu_id = 0;
		ldb_data.sec_ipu_id = 0;
	}

	/* MIPI Display */
	imx6q_add_mxc_hdmi_core(&hdmi_core_data);
	imx6q_add_ipuv3(0, &ipu_data[0]);
	for (i = 0; i < 2 && i < ARRAY_SIZE(amherst_fb_data); i++)
		imx6q_add_ipuv3fb(i, &amherst_fb_data[i]);

	imx6q_add_vdoa();
	imx6q_add_mipi_dsi(&mipi_dsi_pdata);
	imx6q_add_lcdif(&lcdif_data);
	imx6q_add_ldb(&ldb_data);
	imx6q_add_v4l2_output(0);
	voutdev = imx6q_add_v4l2_output(0);
	if (vout_mem.res_msize && voutdev) {
		dma_declare_coherent_memory(&voutdev->dev,
					    vout_mem.res_mbase,
					    vout_mem.res_mbase,
					    vout_mem.res_msize,
					    (DMA_MEMORY_MAP |
					     DMA_MEMORY_EXCLUSIVE));
	}

	imx6q_add_v4l2_capture(0, &capture_data[0]);
	imx6q_add_v4l2_capture(1, &capture_data[1]);
	imx6q_add_mipi_csi2(&mipi_csi2_pdata);
	imx6q_add_imx_snvs_rtc();

	/* Crypto engine */
	if (1 == caam_enabled)
		imx6q_add_imx_caam();

	/* LEDs */
	imx6q_add_device_gpio_leds();

	/* I2C */
	amherst_init_i2c();

	/* SPI */
	spi_device_init();

	/* HDMI */
	imx6q_add_mxc_hdmi(&hdmi_data);

	/* Thermal */
	imx6q_add_anatop_thermal_imx(1, &amherst_anatop_thermal_data);

	/* Ethernet */
	imx6_init_fec(fec_data);

	/* Suspend Power Management */
	imx6q_add_pm_imx(0, &amherst_pm_data);

	/* SDHC */
	imx6_init_sdhc();

	imx_add_viv_gpu(&imx6_gpu_data, &imx6q_gpu_pdata);

	/* USB */
	amherst_init_usb();

	imx6q_add_vpu();

	/* Audio */
	imx6q_init_audio();

	platform_device_register(&amherst_vmmc_reg_devices);
	imx_asrc_data.asrc_core_clk = clk_get(NULL, "asrc_clk");
	imx_asrc_data.asrc_audio_clk = clk_get(NULL, "asrc_serial_clk");
	imx6q_add_asrc(&imx_asrc_data);

	/* HDMI audio */
	imx6q_add_hdmi_soc();
	imx6q_add_hdmi_soc_dai();

	/* PWM */
	imx6q_add_mxc_pwm(0);
	imx6q_add_mxc_pwm(1);

	imx6q_add_otp();
	imx6q_add_viim();
	imx6q_add_imx2_wdt(0, NULL);
	imx6q_add_dma();

	imx6q_add_dvfs_core(&amherst_dvfscore_data);

	/* PCIe */
#ifndef CONFIG_IMX_PCIE
	/* enable pcie 3v3 power without pcie driver */
	pcie_3v3_reset();
#endif
	/* Add PCIe RC interface support */
	imx6q_add_pcie(&amherst_pcie_data);

	/* Clocks */
	clko2 = clk_get(NULL, "clko2_clk");
	if (IS_ERR(clko2))
		pr_err("can't get CLKO2 clock.\n");

	new_parent = clk_get(NULL, "osc_clk");
	if (!IS_ERR(new_parent)) {
		clk_set_parent(clko2, new_parent);
		clk_put(new_parent);
	}
	rate = clk_round_rate(clko2, 24000000);
	clk_set_rate(clko2, rate);
	clk_enable(clko2);

	/* Camera and audio use osc clock */
	clko = clk_get(NULL, "clko_clk");
	if (!IS_ERR(clko))
		clk_set_parent(clko, clko2);
	
	imx6q_add_busfreq();

	imx6q_add_perfmon(0);
	imx6q_add_perfmon(1);
	imx6q_add_perfmon(2);
}

extern void __iomem *twd_base;
static void __init mx6_amherst_timer_init(void)
{
	struct clk *uart_clk;
#ifdef CONFIG_LOCAL_TIMERS
	twd_base = ioremap(LOCAL_TWD_ADDR, SZ_256);
	BUG_ON(!twd_base);
#endif
	mx6_clocks_init(32768, 24000000, 0, 0);

	uart_clk = clk_get_sys("imx-uart.0", NULL);
	early_console_setup(UART1_BASE_ADDR, uart_clk);
}

static struct sys_timer mx6_amherst_timer = {
	.init   = mx6_amherst_timer_init,
};

static void __init mx6q_amherst_reserve(void)
{
#if defined(CONFIG_MXC_GPU_VIV) || defined(CONFIG_MXC_GPU_VIV_MODULE)
	phys_addr_t phys;

	if (imx6q_gpu_pdata.reserved_mem_size) {
		phys = memblock_alloc_base(imx6q_gpu_pdata.reserved_mem_size,
					   SZ_4K, SZ_1G);
		memblock_remove(phys, imx6q_gpu_pdata.reserved_mem_size);
		imx6q_gpu_pdata.reserved_mem_base = phys;
	}
#endif
}

/*
 * initialize __mach_desc_AMHERST data structure.
 */
MACHINE_START(AMHERST, "HandEra i.MX6 Amherst Board")
	/* Maintainer: HandEra, Inc. */
	.boot_params = MX6_PHYS_OFFSET + 0x100,
	.fixup = fixup_mxc_board,
	.map_io = mx6_map_io,
	.init_irq = mx6_init_irq,
	.init_machine = mx6_amherst_board_init,
	.timer = &mx6_amherst_timer,
	.reserve = mx6q_amherst_reserve,
MACHINE_END
