/*
 *  linux/arch/arm/mach-bcm2708/bcm2708.c
 *
 *  Copyright (C) 2010 Broadcom
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/init.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/serial_8250.h>
#include <linux/platform_device.h>
#include <linux/syscore_ops.h>
#include <linux/interrupt.h>
#include <linux/amba/bus.h>
#include <linux/amba/clcd.h>
#include <linux/clockchips.h>
#include <linux/cnt32_to_63.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/w1-gpio.h>
#include <linux/pps-gpio.h>

#include <linux/version.h>
#include <linux/clkdev.h>
#include <asm/system.h>
#include <mach/hardware.h>
#include <asm/irq.h>
#include <linux/leds.h>
#include <asm/mach-types.h>
#include <linux/sched_clock.h>

#include <asm/mach/arch.h>
#include <asm/mach/flash.h>
#include <asm/mach/irq.h>
#include <asm/mach/time.h>
#include <asm/mach/map.h>

#include <mach/timex.h>
#include <mach/dma.h>
#include <mach/vcio.h>
#include <mach/system.h>
#include <mach/gpio.h>

#include <linux/delay.h>

#include <linux/mfd/rc5t619.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/rc5t619-regulator.h>
#include <linux/rtc/rtc-rc5t619.h>
#include <linux/power/rc5t619-battery.h>

#include "bcm2708.h"
#include "armctrl.h"
#include "clock.h"

#ifdef CONFIG_BCM_VC_CMA
#include <linux/broadcom/vc_cma.h>
#endif


/* Effectively we have an IOMMU (ARM<->VideoCore map) that is set up to
 * give us IO access only to 64Mbytes of physical memory (26 bits).  We could
 * represent this window by setting our dmamasks to 26 bits but, in fact
 * we're not going to use addresses outside this range (they're not in real
 * memory) so we don't bother.
 *
 * In the future we might include code to use this IOMMU to remap other
 * physical addresses onto VideoCore memory then the use of 32-bits would be
 * more legitimate.
 */
#define DMA_MASK_BITS_COMMON 32

// use GPIO 4 for the one-wire GPIO pin, if enabled
#define W1_GPIO 4
// ensure one-wire GPIO pullup is disabled by default
#define W1_PULLUP -1

/* command line parameters */
static unsigned boardrev, serial;
static unsigned uart_clock;
static unsigned disk_led_gpio = 16;
static unsigned disk_led_active_low = 1;
static unsigned reboot_part = 0;
static unsigned w1_gpio_pin = W1_GPIO;
static unsigned w1_gpio_pullup = W1_PULLUP;
static int pps_gpio_pin = -1;
static unsigned bcm2835_mmc = 1;
static bool vc_i2c_override = true;

static void __init bcm2708_init_led(void);

void __init bcm2708_init_irq(void)
{
	armctrl_init(__io_address(ARMCTRL_IC_BASE), 0, 0, 0);
}

static struct map_desc bcm2708_io_desc[] __initdata = {
	{
	 .virtual = IO_ADDRESS(ARMCTRL_BASE),
	 .pfn = __phys_to_pfn(ARMCTRL_BASE),
	 .length = SZ_4K,
	 .type = MT_DEVICE},
	{
	 .virtual = IO_ADDRESS(UART0_BASE),
	 .pfn = __phys_to_pfn(UART0_BASE),
	 .length = SZ_4K,
	 .type = MT_DEVICE},
	{
	 .virtual = IO_ADDRESS(UART1_BASE),
	 .pfn = __phys_to_pfn(UART1_BASE),
	 .length = SZ_4K,
	 .type = MT_DEVICE},
	{
	 .virtual = IO_ADDRESS(DMA_BASE),
	 .pfn = __phys_to_pfn(DMA_BASE),
	 .length = SZ_4K,
	 .type = MT_DEVICE},
	{
	 .virtual = IO_ADDRESS(MCORE_BASE),
	 .pfn = __phys_to_pfn(MCORE_BASE),
	 .length = SZ_4K,
	 .type = MT_DEVICE},
	{
	 .virtual = IO_ADDRESS(ST_BASE),
	 .pfn = __phys_to_pfn(ST_BASE),
	 .length = SZ_4K,
	 .type = MT_DEVICE},
	{
	 .virtual = IO_ADDRESS(USB_BASE),
	 .pfn = __phys_to_pfn(USB_BASE),
	 .length = SZ_128K,
	 .type = MT_DEVICE},
	{
	 .virtual = IO_ADDRESS(PM_BASE),
	 .pfn = __phys_to_pfn(PM_BASE),
	 .length = SZ_4K,
	 .type = MT_DEVICE},
	{
	 .virtual = IO_ADDRESS(GPIO_BASE),
	 .pfn = __phys_to_pfn(GPIO_BASE),
	 .length = SZ_4K,
	 .type = MT_DEVICE}
};

void __init bcm2708_map_io(void)
{
	iotable_init(bcm2708_io_desc, ARRAY_SIZE(bcm2708_io_desc));
}

/* The STC is a free running counter that increments at the rate of 1MHz */
#define STC_FREQ_HZ 1000000

static inline uint32_t timer_read(void)
{
	/* STC: a free running counter that increments at the rate of 1MHz */
	return readl(__io_address(ST_BASE + 0x04));
}

static unsigned long bcm2708_read_current_timer(void)
{
	return timer_read();
}

static u32 notrace bcm2708_read_sched_clock(void)
{
	return timer_read();
}

static cycle_t clksrc_read(struct clocksource *cs)
{
	return timer_read();
}

static struct clocksource clocksource_stc = {
	.name = "stc",
	.rating = 300,
	.read = clksrc_read,
	.mask = CLOCKSOURCE_MASK(32),
	.flags = CLOCK_SOURCE_IS_CONTINUOUS,
};

unsigned long frc_clock_ticks32(void)
{
	return timer_read();
}

static void __init bcm2708_clocksource_init(void)
{
	if (clocksource_register_hz(&clocksource_stc, STC_FREQ_HZ)) {
		printk(KERN_ERR "timer: failed to initialize clock "
		       "source %s\n", clocksource_stc.name);
	}
}


/*
 * These are fixed clocks.
 */
static struct clk ref24_clk = {
	.rate = UART0_CLOCK,	/* The UART is clocked at 3MHz via APB_CLK */
};

static struct clk osc_clk = {
#ifdef CONFIG_ARCH_BCM2708_CHIPIT
	.rate = 27000000,
#else
	.rate = 500000000,	/* ARM clock is set from the VideoCore booter */
#endif
};

/* warning - the USB needs a clock > 34MHz */

static struct clk sdhost_clk = {
#ifdef CONFIG_ARCH_BCM2708_CHIPIT
	.rate = 4000000,	/* 4MHz */
#else
	.rate = 250000000,	/* 250MHz */
#endif
};

static struct clk_lookup lookups[] = {
	{			/* UART0 */
	 .dev_id = "dev:f1",
	 .clk = &ref24_clk,
	 },
	{			/* USB */
	 .dev_id = "bcm2708_usb",
	 .clk = &osc_clk,
	 }, {	/* SPI */
		 .dev_id = "bcm2708_spi.0",
		 .clk = &sdhost_clk,
	 }, {	/* BSC0 */
		 .dev_id = "bcm2708_i2c.0",
		 .clk = &sdhost_clk,
	 }, {	/* BSC1 */
		 .dev_id = "bcm2708_i2c.1",
		 .clk = &sdhost_clk,
	 }
};

#define UART0_IRQ	{ IRQ_UART, 0 /*NO_IRQ*/ }
#define UART0_DMA	{ 15, 14 }

AMBA_DEVICE(uart0, "dev:f1", UART0, NULL);

static struct amba_device *amba_devs[] __initdata = {
	&uart0_device,
};

static struct resource bcm2708_dmaman_resources[] = {
	{
	 .start = DMA_BASE,
	 .end = DMA_BASE + SZ_4K - 1,
	 .flags = IORESOURCE_MEM,
	 }
};

static struct platform_device bcm2708_dmaman_device = {
	.name = BCM_DMAMAN_DRIVER_NAME,
	.id = 0,		/* first bcm2708_dma */
	.resource = bcm2708_dmaman_resources,
	.num_resources = ARRAY_SIZE(bcm2708_dmaman_resources),
};

#if defined(CONFIG_W1_MASTER_GPIO) || defined(CONFIG_W1_MASTER_GPIO_MODULE)
static struct w1_gpio_platform_data w1_gpio_pdata = {
	.pin = W1_GPIO,
        .ext_pullup_enable_pin = W1_PULLUP,
	.is_open_drain = 0,
};

static struct platform_device w1_device = {
	.name = "w1-gpio",
	.id = -1,
	.dev.platform_data = &w1_gpio_pdata,
};
#endif

static struct pps_gpio_platform_data pps_gpio_info = {
	.assert_falling_edge = false,
	.capture_clear = false,
	.gpio_pin = -1,
	.gpio_label = "PPS",
};

static struct platform_device pps_gpio_device = {
	.name = "pps-gpio",
	.id = PLATFORM_DEVID_NONE,
	.dev.platform_data = &pps_gpio_info,
};

static u64 fb_dmamask = DMA_BIT_MASK(DMA_MASK_BITS_COMMON);

static struct platform_device bcm2708_fb_device = {
	.name = "bcm2708_fb",
	.id = -1,		/* only one bcm2708_fb */
	.resource = NULL,
	.num_resources = 0,
	.dev = {
		.dma_mask = &fb_dmamask,
		.coherent_dma_mask = DMA_BIT_MASK(DMA_MASK_BITS_COMMON),
		},
};

static struct plat_serial8250_port bcm2708_uart1_platform_data[] = {
	{
	 .mapbase = UART1_BASE + 0x40,
	 .irq = IRQ_AUX,
	 .uartclk = 125000000,
	 .regshift = 2,
	 .iotype = UPIO_MEM,
	 .flags = UPF_FIXED_TYPE | UPF_IOREMAP | UPF_SKIP_TEST,
	 .type = PORT_8250,
	 },
	{},
};

static struct platform_device bcm2708_uart1_device = {
	.name = "serial8250",
	.id = PLAT8250_DEV_PLATFORM,
	.dev = {
		.platform_data = bcm2708_uart1_platform_data,
		},
};

static struct resource bcm2708_usb_resources[] = {
	[0] = {
	       .start = USB_BASE,
	       .end = USB_BASE + SZ_128K - 1,
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
		.start = MPHI_BASE,
		.end = MPHI_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	      },
	[2] = {
	       .start = IRQ_HOSTPORT,
	       .end = IRQ_HOSTPORT,
	       .flags = IORESOURCE_IRQ,
	       },
	[3] = {
			.start = IRQ_USB,
			.end = IRQ_USB,
			.flags = IORESOURCE_IRQ,
			},
};


static u64 usb_dmamask = DMA_BIT_MASK(DMA_MASK_BITS_COMMON);

static struct platform_device bcm2708_usb_device = {
	.name = "bcm2708_usb",
	.id = -1,		/* only one bcm2708_usb */
	.resource = bcm2708_usb_resources,
	.num_resources = ARRAY_SIZE(bcm2708_usb_resources),
	.dev = {
		.dma_mask = &usb_dmamask,
		.coherent_dma_mask = DMA_BIT_MASK(DMA_MASK_BITS_COMMON),
		},
};

static struct resource bcm2708_vcio_resources[] = {
	[0] = {			/* mailbox/semaphore/doorbell access */
	       .start = MCORE_BASE,
	       .end = MCORE_BASE + SZ_4K - 1,
	       .flags = IORESOURCE_MEM,
	       },
};

static u64 vcio_dmamask = DMA_BIT_MASK(DMA_MASK_BITS_COMMON);

static struct platform_device bcm2708_vcio_device = {
	.name = BCM_VCIO_DRIVER_NAME,
	.id = -1,		/* only one VideoCore I/O area */
	.resource = bcm2708_vcio_resources,
	.num_resources = ARRAY_SIZE(bcm2708_vcio_resources),
	.dev = {
		.dma_mask = &vcio_dmamask,
		.coherent_dma_mask = DMA_BIT_MASK(DMA_MASK_BITS_COMMON),
		},
};

#ifdef CONFIG_BCM2708_GPIO
#define BCM_GPIO_DRIVER_NAME "bcm2708_gpio"

static struct resource bcm2708_gpio_resources[] = {
	[0] = {			/* general purpose I/O */
	       .start = GPIO_BASE,
	       .end = GPIO_BASE + SZ_4K - 1,
	       .flags = IORESOURCE_MEM,
	       },
};

static u64 gpio_dmamask = DMA_BIT_MASK(DMA_MASK_BITS_COMMON);

static struct platform_device bcm2708_gpio_device = {
	.name = BCM_GPIO_DRIVER_NAME,
	.id = -1,		/* only one VideoCore I/O area */
	.resource = bcm2708_gpio_resources,
	.num_resources = ARRAY_SIZE(bcm2708_gpio_resources),
	.dev = {
		.dma_mask = &gpio_dmamask,
		.coherent_dma_mask = DMA_BIT_MASK(DMA_MASK_BITS_COMMON),
		},
};
#endif

static struct resource bcm2708_systemtimer_resources[] = {
	[0] = {			/* system timer access */
	       .start = ST_BASE,
	       .end = ST_BASE + SZ_4K - 1,
	       .flags = IORESOURCE_MEM,
	       },
	{
	 .start = IRQ_TIMER3,
	 .end = IRQ_TIMER3,
	 .flags = IORESOURCE_IRQ,
	 }

};

static u64 systemtimer_dmamask = DMA_BIT_MASK(DMA_MASK_BITS_COMMON);

static struct platform_device bcm2708_systemtimer_device = {
	.name = "bcm2708_systemtimer",
	.id = -1,		/* only one VideoCore I/O area */
	.resource = bcm2708_systemtimer_resources,
	.num_resources = ARRAY_SIZE(bcm2708_systemtimer_resources),
	.dev = {
		.dma_mask = &systemtimer_dmamask,
		.coherent_dma_mask = DMA_BIT_MASK(DMA_MASK_BITS_COMMON),
		},
};

#ifdef CONFIG_MMC_SDHCI_BCM2708	/* Arasan emmc SD */
static struct resource bcm2708_emmc_resources[] = {
	[0] = {
	       .start = EMMC_BASE,
	       .end = EMMC_BASE + SZ_256 - 1,	/* we only need this area */
	       /* the memory map actually makes SZ_4K available  */
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = IRQ_ARASANSDIO,
	       .end = IRQ_ARASANSDIO,
	       .flags = IORESOURCE_IRQ,
	       },
};

static u64 bcm2708_emmc_dmamask = 0xffffffffUL;

struct platform_device bcm2708_emmc_device = {
	.name = "bcm2708_sdhci",
	.id = 0,
	.num_resources = ARRAY_SIZE(bcm2708_emmc_resources),
	.resource = bcm2708_emmc_resources,
	.dev = {
		.dma_mask = &bcm2708_emmc_dmamask,
		.coherent_dma_mask = 0xffffffffUL},
};
#endif /* CONFIG_MMC_SDHCI_BCM2708 */

#ifdef CONFIG_MMC_BCM2835	/* Arasan emmc SD (new) */
static struct resource bcm2835_emmc_resources[] = {
	[0] = {
	       .start = EMMC_BASE,
	       .end = EMMC_BASE + SZ_256 - 1,	/* we only need this area */
	       /* the memory map actually makes SZ_4K available  */
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = IRQ_ARASANSDIO,
	       .end = IRQ_ARASANSDIO,
	       .flags = IORESOURCE_IRQ,
	       },
};

static u64 bcm2835_emmc_dmamask = 0xffffffffUL;

struct platform_device bcm2835_emmc_device = {
	.name = "mmc-bcm2835",
	.id = 0,
	.num_resources = ARRAY_SIZE(bcm2835_emmc_resources),
	.resource = bcm2835_emmc_resources,
	.dev = {
		.dma_mask = &bcm2835_emmc_dmamask,
		.coherent_dma_mask = 0xffffffffUL},
};
#endif /* CONFIG_MMC_BCM2835 */

static struct resource bcm2708_powerman_resources[] = {
	[0] = {
	       .start = PM_BASE,
	       .end = PM_BASE + SZ_256 - 1,
	       .flags = IORESOURCE_MEM,
	       },
};

static u64 powerman_dmamask = DMA_BIT_MASK(DMA_MASK_BITS_COMMON);

struct platform_device bcm2708_powerman_device = {
	.name = "bcm2708_powerman",
	.id = 0,
	.num_resources = ARRAY_SIZE(bcm2708_powerman_resources),
	.resource = bcm2708_powerman_resources,
	.dev = {
		.dma_mask = &powerman_dmamask,
		.coherent_dma_mask = 0xffffffffUL},
};


static struct platform_device bcm2708_alsa_devices[] = {
	[0] = {
	       .name = "bcm2835_AUD0",
	       .id = 0,		/* first audio device */
	       .resource = 0,
	       .num_resources = 0,
	       },
	[1] = {
	       .name = "bcm2835_AUD1",
	       .id = 1,		/* second audio device */
	       .resource = 0,
	       .num_resources = 0,
	       },
	[2] = {
	       .name = "bcm2835_AUD2",
	       .id = 2,		/* third audio device */
	       .resource = 0,
	       .num_resources = 0,
	       },
	[3] = {
	       .name = "bcm2835_AUD3",
	       .id = 3,		/* forth audio device */
	       .resource = 0,
	       .num_resources = 0,
	       },
	[4] = {
	       .name = "bcm2835_AUD4",
	       .id = 4,		/* fifth audio device */
	       .resource = 0,
	       .num_resources = 0,
	       },
	[5] = {
	       .name = "bcm2835_AUD5",
	       .id = 5,		/* sixth audio device */
	       .resource = 0,
	       .num_resources = 0,
	       },
	[6] = {
	       .name = "bcm2835_AUD6",
	       .id = 6,		/* seventh audio device */
	       .resource = 0,
	       .num_resources = 0,
	       },
	[7] = {
	       .name = "bcm2835_AUD7",
	       .id = 7,		/* eighth audio device */
	       .resource = 0,
	       .num_resources = 0,
	       },
};

static struct resource bcm2708_spi_resources[] = {
	{
		.start = SPI0_BASE,
		.end = SPI0_BASE + SZ_256 - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = IRQ_SPI,
		.end = IRQ_SPI,
		.flags = IORESOURCE_IRQ,
	}
};


static u64 bcm2708_spi_dmamask = DMA_BIT_MASK(DMA_MASK_BITS_COMMON);
static struct platform_device bcm2708_spi_device = {
	.name = "bcm2708_spi",
	.id = 0,
	.num_resources = ARRAY_SIZE(bcm2708_spi_resources),
	.resource = bcm2708_spi_resources,
	.dev = {
		.dma_mask = &bcm2708_spi_dmamask,
		.coherent_dma_mask = DMA_BIT_MASK(DMA_MASK_BITS_COMMON)},
};

#ifdef CONFIG_BCM2708_SPIDEV
static struct spi_board_info bcm2708_spi_devices[] = {
#ifdef CONFIG_SPI_SPIDEV
	{
		.modalias = "spidev",
		.max_speed_hz = 500000,
		.bus_num = 0,
		.chip_select = 0,
		.mode = SPI_MODE_0,
	}, {
		.modalias = "spidev",
		.max_speed_hz = 500000,
		.bus_num = 0,
		.chip_select = 1,
		.mode = SPI_MODE_0,
	}
#endif
};
#endif

static struct resource bcm2708_bsc0_resources[] = {
	{
		.start = BSC0_BASE,
		.end = BSC0_BASE + SZ_256 - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = INTERRUPT_I2C,
		.end = INTERRUPT_I2C,
		.flags = IORESOURCE_IRQ,
	}
};

static struct platform_device bcm2708_bsc0_device = {
	.name = "bcm2708_i2c",
	.id = 0,
	.num_resources = ARRAY_SIZE(bcm2708_bsc0_resources),
	.resource = bcm2708_bsc0_resources,
};


static struct resource bcm2708_bsc1_resources[] = {
	{
		.start = BSC1_BASE,
		.end = BSC1_BASE + SZ_256 - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = INTERRUPT_I2C,
		.end = INTERRUPT_I2C,
		.flags = IORESOURCE_IRQ,
	}
};

static struct platform_device bcm2708_bsc1_device = {
	.name = "bcm2708_i2c",
	.id = 1,
	.num_resources = ARRAY_SIZE(bcm2708_bsc1_resources),
	.resource = bcm2708_bsc1_resources,
};

static struct platform_device bcm2835_hwmon_device = {
	.name = "bcm2835_hwmon",
};

static struct platform_device bcm2835_thermal_device = {
	.name = "bcm2835_thermal",
};

#ifdef CONFIG_SND_BCM2708_SOC_I2S_MODULE
static struct resource bcm2708_i2s_resources[] = {
	{
		.start = I2S_BASE,
		.end = I2S_BASE + 0x20,
		.flags = IORESOURCE_MEM,
	},
        {
		.start = PCM_CLOCK_BASE,
		.end = PCM_CLOCK_BASE + 0x02,
		.flags = IORESOURCE_MEM,
	}
};

static struct platform_device bcm2708_i2s_device = {
	.name = "bcm2708-i2s",
	.id = 0,
	.num_resources = ARRAY_SIZE(bcm2708_i2s_resources),
	.resource = bcm2708_i2s_resources,
};
#endif

#if defined(CONFIG_SND_BCM2708_SOC_HIFIBERRY_DAC) || defined(CONFIG_SND_BCM2708_SOC_HIFIBERRY_DAC_MODULE)
static struct platform_device snd_hifiberry_dac_device = {
        .name = "snd-hifiberry-dac",
        .id = 0,
        .num_resources = 0,
};

static struct platform_device snd_pcm5102a_codec_device = {
        .name = "pcm5102a-codec",
        .id = -1,
        .num_resources = 0,
};
#endif

#if defined(CONFIG_SND_BCM2708_SOC_HIFIBERRY_DACPLUS) || defined(CONFIG_SND_BCM2708_SOC_HIFIBERRY_DACPLUS_MODULE)
static struct platform_device snd_rpi_hifiberry_dacplus_device = {
        .name = "snd-rpi-hifiberry-dacplus",
        .id = 0,
        .num_resources = 0,
};

static struct i2c_board_info __initdata snd_pcm512x_hbdacplus_i2c_devices[] = {
        {
                I2C_BOARD_INFO("pcm5122", 0x4d)
        },
};
#endif

#if defined(CONFIG_SND_BCM2708_SOC_HIFIBERRY_DIGI) || defined(CONFIG_SND_BCM2708_SOC_HIFIBERRY_DIGI_MODULE)
static struct platform_device snd_hifiberry_digi_device = {
        .name = "snd-hifiberry-digi",
        .id = 0,
        .num_resources = 0,
};

static struct i2c_board_info __initdata snd_wm8804_i2c_devices[] = {
        {
                I2C_BOARD_INFO("wm8804", 0x3b)
        },
};

#endif

#if defined(CONFIG_SND_BCM2708_SOC_HIFIBERRY_AMP) || defined(CONFIG_SND_BCM2708_SOC_HIFIBERRY_AMP_MODULE)
static struct platform_device snd_hifiberry_amp_device = {
        .name = "snd-hifiberry-amp",
        .id = 0,
        .num_resources = 0,
};

static struct i2c_board_info __initdata snd_tas5713_i2c_devices[] = {
        {
                I2C_BOARD_INFO("tas5713", 0x1b)
        },
};
#endif

#if defined(CONFIG_SND_BCM2708_SOC_RPI_DAC) || defined(CONFIG_SND_BCM2708_SOC_RPI_DAC_MODULE)
static struct platform_device snd_rpi_dac_device = {
        .name = "snd-rpi-dac",
        .id = 0,
        .num_resources = 0,
};

static struct platform_device snd_pcm1794a_codec_device = {
        .name = "pcm1794a-codec",
        .id = -1,
        .num_resources = 0,
};
#endif


#if defined(CONFIG_SND_BCM2708_SOC_IQAUDIO_DAC) || defined(CONFIG_SND_BCM2708_SOC_IQAUDIO_DAC_MODULE)
static struct platform_device snd_rpi_iqaudio_dac_device = {
        .name = "snd-rpi-iqaudio-dac",
        .id = 0,
        .num_resources = 0,
};

// Use the actual device name rather than generic driver name
static struct i2c_board_info __initdata snd_pcm512x_i2c_devices[] = {
	{
		I2C_BOARD_INFO("pcm5122", 0x4c)
	},
};
#endif

#if defined(CONFIG_MFD_RC5T619)
#define IRQ_BOARD_BASE        (GPIO_IRQ_START + GPIO_IRQS)
#define RC5T619_HOST_IRQ_GPIO 32
#define RC5T619_HOST_IRQ_ONOB 33

static struct rc5t619 *Rc5t619;
static int rc5t619_pre_init(struct rc5t619 *rc5t619)
{
	int ret;
	Rc5t619 = 	rc5t619;

	ret = rc5t619_set_bits(rc5t619->dev,RC5T619_PWR_REP_CNT,(0 << 0));  //Disable restart when power off
	/****************set Re-charging voltage*****************/
	ret = rc5t619_set_bits(rc5t619->dev,BATSET2_REG,(3 << 0)); 
	ret = rc5t619_clr_bits(rc5t619->dev,BATSET2_REG,(1 << 2)); //set vrchg 4v
	/********************************************************/

	return 0;
}

static int rc5t619_post_init(struct rc5t619 *rc5t619)
{
	int ret=0;
	
	ret = rc5t619_clr_bits(rc5t619->dev,0xb1,(7<<0));  //set vbatdec voltage  3.0v
	ret = rc5t619_set_bits(rc5t619->dev,0xb1,(3<<0));  //set vbatdec voltage 3.0v

	printk("%s,line=%d END\n", __func__,__LINE__);

	return ret;
}

#if defined(CONFIG_REGULATOR_RC5T619)
static struct regulator_consumer_supply rc5t619_dcdc1_supply[] = {
	REGULATOR_SUPPLY("p3v3", NULL),
};
static struct regulator_consumer_supply rc5t619_dcdc2_supply[] = {
	REGULATOR_SUPPLY("p1v8", NULL),
};

static struct regulator_consumer_supply rc5t619_ldo1_supply[] = {
	REGULATOR_SUPPLY("p2v5", NULL),
};
static struct regulator_consumer_supply rc5t619_ldo2_supply[] = {
	REGULATOR_SUPPLY("tflash_io", NULL),
};

static struct regulator_init_data rc5t619_dcdc1 = {
	.constraints = {
		.name           = "DC1",
		.min_uV			= 3300000,
		.max_uV			= 3300000,
		.apply_uV		= 1,
		.always_on = 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_MODE,
		.valid_modes_mask = REGULATOR_MODE_STANDBY | REGULATOR_MODE_NORMAL | REGULATOR_MODE_FAST,

	},
	.num_consumer_supplies = ARRAY_SIZE(rc5t619_dcdc1_supply),
	.consumer_supplies =  rc5t619_dcdc1_supply,
};

static struct regulator_init_data rc5t619_dcdc2 = {
	.constraints = {
		.name           = "DC2",
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.apply_uV		= 1,
		.always_on = 1,
	
		.valid_ops_mask = REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_MODE,
		.valid_modes_mask = REGULATOR_MODE_STANDBY | REGULATOR_MODE_NORMAL | REGULATOR_MODE_FAST,

	},
	.num_consumer_supplies = ARRAY_SIZE(rc5t619_dcdc2_supply),
	.consumer_supplies =  rc5t619_dcdc2_supply,
};

static struct regulator_init_data rc5t619_ldo1 = {
	.constraints = {
		.name           = "LDO1",
		.min_uV			= 2500000,
		.max_uV			= 2500000,
		.apply_uV		= 1,
		.always_on = 1,
		
		.valid_ops_mask = REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_MODE,
		.valid_modes_mask = REGULATOR_MODE_STANDBY | REGULATOR_MODE_NORMAL,

	},
	.num_consumer_supplies = ARRAY_SIZE(rc5t619_ldo1_supply),
	.consumer_supplies =  rc5t619_ldo1_supply,
};

/* */
static struct regulator_init_data rc5t619_ldo2 = {
	.constraints = {
		.name           = "LDO2",
		.min_uV			= 2850000,
		.max_uV			= 2850000,
		.apply_uV		= 1,
		.always_on = 1,
		
		.valid_ops_mask = REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_MODE,
		.valid_modes_mask = REGULATOR_MODE_STANDBY | REGULATOR_MODE_NORMAL,

	},
	.num_consumer_supplies = ARRAY_SIZE(rc5t619_ldo2_supply),
	.consumer_supplies =  rc5t619_ldo2_supply,
};
#endif //defined(CONFIG_REGULATOR_RC5T619)


#if defined(CONFIG_BATTERY_RC5T619)
static struct rc5t619_battery_platform_data rc5t619_battery = {
	.irq = IRQ_BOARD_BASE,
	.alarm_vol_mv = 3300,
	.multiple = 100,
	.monitor_time = 20,
	
	//4200mv type battery
	.ch_vfchg		= 0xFF,	/* VFCHG		= 0x0-0x4 (4.05v 4.10v 4.15v 4.20v 4.35v) 0xFF is OTP setting*/
	.ch_vrchg		= 0xFF,	/* VRCHG		= 0x0-0x4 (3.85v 3.90v 3.95v 4.00v 4.10v) 0xFF is OTP setting*/
	.ch_vbatovset	= 0xFF,	/* VBATOVSET	= 0 or 1 (0 : 4.38V(up)/3.95V(down) 1: 4.53V(up)/4.10V(down)) 
							   0xFF is OTP setting*/
	.ch_ichg		= 0xFF,	/* ICHG			= 0x0-0x1D (100mA - 3000mA)  0xFF is OTP setting*/
	.ch_icchg		= 0x03,	/* ICCHG		= 0x0-3 (50mA 100mA 150mA 200mA)  0xFF is OTP setting*/
	.ch_ilim_adp	= 0xFF,	/* ILIM_ADP		= 0x0-0x1D (100mA - 3000mA) */
	.ch_ilim_usb	= 0xFF,	/* ILIM_USB		= 0x0-0x1D (100mA - 3000mA) */
	.fg_target_vsys	= 3200, /* This value is the target one to DSOC=0% */
	.fg_target_ibat	= 2000, /* This value is the target one to DSOC=0% */
	.jt_en			= 0,	/* JEITA Switch	= 1:enable, 0:disable */
	.jt_hw_sw		= 1,	/* JEITA is controlled by 1:HW, 0:SW */
	.jt_temp_h		= 50,	/* degree C */ 
	.jt_temp_l		= 12,	/* degree C */
	.jt_vfchg_h		= 0x03,	/* VFCHG High  	= 0 - 0x04 (4.05, 4.10, 4.15, 4.20, 4.35V) */
	.jt_vfchg_l		= 0,	/* VFCHG Low  	= 0 - 0x04 (4.05, 4.10, 4.15, 4.20, 4.35V) */
	.jt_ichg_h		= 0x0D,	/* ICHG High   	= 0 - 0x1D (100 - 3000mA) */
	.jt_ichg_l		= 0x09,	/* ICHG Low   	= 0 - 0x1D (100 - 3000mA) */
};
#endif //defined(CONFIG_BATTERY_RC5T619)

#if defined(CONFIG_RTC_DRV_RC5T619)
static struct rtc_time rtc_tm = {
    .tm_wday = 0,
    .tm_year = 114,
    .tm_mon  = 0,
    .tm_mday = 1,
    .tm_hour = 0,
    .tm_min  = 0,
    .tm_sec  = 0,
};
static struct rc5t619_rtc_platform_data rc5t619_rtc_pdata = {
	.irq = IRQ_BOARD_BASE,
	.time = &rtc_tm,
};
#endif //defined(CONFIG_RTC_DRV_RC5T619)

#if defined(CONFIG_INPUT_RC5T619_PWRKEY)
static struct rc5t619_pwrkey_platform_data rc5t619_pwrkey_data= { 
	.irq = gpio_to_irq(RC5T619_HOST_IRQ_ONOB),
	.delay_ms = 20, 
}; 
#endif //defined(CONFIG_INPUT_RC5T619_PWRKEY)

#define RC5T619_REGULATOR(_id, _data)  \
{ \
		.id = RC5T619_ID_##_id, \
		.name = "rc5t619-regulator", \
		.platform_data = _data, \
} \

static struct rc5t619_subdev_info rc5t619_devs[] = {
	RC5T619_REGULATOR(DC1, &rc5t619_dcdc1),
	RC5T619_REGULATOR(DC2, &rc5t619_dcdc2),
	
	RC5T619_REGULATOR(LDO1, &rc5t619_ldo1),
	RC5T619_REGULATOR(LDO2, &rc5t619_ldo2),

#if defined(CONFIG_BATTERY_RC5T619)
	{
		.id = 5,  
		.name ="rc5t619-battery",
		.platform_data = &rc5t619_battery,
	},
#endif //defined(CONFIG_BATTERY_RC5T619)

#if defined(CONFIG_RTC_DRV_RC5T619)
	{
		.id = 6,  
		.name ="rc5t619-rtc",
		.platform_data = &rc5t619_rtc_pdata,
	},
#endif //defined(CONFIG_RTC_DRV_RC5T619)

#if defined(CONFIG_INPUT_RC5T619_PWRKEY)
	{
		.id = 7,  
		.name ="rc5t619-pwrkey",
		.platform_data = &rc5t619_pwrkey_data,
	},
#endif //defined(CONFIG_INPUT_RC5T619_PWRKEY)

#if defined(CONFIG_RC5T619_ADC)
	{
		.id = 8,  
		.name ="rc5t619-adc",
	},
#endif //defined(CONFIG_INPUT_RC5T619_PWRKEY)

};

#define RC5T619_GPIO_INIT(_init_apply, _output_mode, _output_val, _led_mode, _led_func)  \
{  \
	.output_mode_en = _output_mode,  \
	.output_val = _output_val,  \
	.init_apply = _init_apply,   \
	.led_mode = _led_mode,  \
	.led_func = _led_func,  \
}  \

struct rc5t619_gpio_init_data rc5t619_gpio_data[] = { 
	RC5T619_GPIO_INIT(0, 0, 0, 0, 0), 
	RC5T619_GPIO_INIT(0, 0, 0, 0, 0), 
	RC5T619_GPIO_INIT(0, 0, 0, 0, 0), 
	RC5T619_GPIO_INIT(0, 0, 0, 0, 0), 
	RC5T619_GPIO_INIT(0, 0, 0, 0, 0), 
}; 

static struct rc5t619_platform_data rc5t619_pdata = {
	.irq_base          = IRQ_BOARD_BASE,
	.subdevs           = rc5t619_devs,
	.num_subdevs       = ARRAY_SIZE(rc5t619_devs),
	.gpio_base         = BCM2708_NR_GPIOS,
	.gpio_init_data    = rc5t619_gpio_data, 
	.num_gpioinit_data = ARRAY_SIZE(rc5t619_gpio_data),
	.pre_init          = rc5t619_pre_init,
	.post_init         = rc5t619_post_init,
};

static struct i2c_board_info __initdata i2c0_devs[] = {
	{
		I2C_BOARD_INFO("rc5t619", 0x32),
		.platform_data = &rc5t619_pdata,
		.irq = gpio_to_irq(RC5T619_HOST_IRQ_GPIO),
	},
};
#endif // defined(CONFIG_MFD_RC5T619) || defined(CONFIG_MFD_RC5T619_MODULE)

int __init bcm_register_device(struct platform_device *pdev)
{
	int ret;

	ret = platform_device_register(pdev);
	if (ret)
		pr_debug("Unable to register platform device '%s': %d\n",
			 pdev->name, ret);

	return ret;
}

int calc_rsts(int partition)
{
	return PM_PASSWORD |
		((partition & (1 << 0))  << 0) |
		((partition & (1 << 1))  << 1) |
		((partition & (1 << 2))  << 2) |
		((partition & (1 << 3))  << 3) |
		((partition & (1 << 4))  << 4) |
		((partition & (1 << 5))  << 5);
}

static void bcm2708_restart(enum reboot_mode mode, const char *cmd)
{
	extern char bcm2708_reboot_mode;
	uint32_t pm_rstc, pm_wdog;
	uint32_t timeout = 10;
	uint32_t pm_rsts = 0;

	if(bcm2708_reboot_mode == 'q')
	{
		// NOOBS < 1.3 booting with reboot=q
		pm_rsts = readl(__io_address(PM_RSTS));
		pm_rsts = PM_PASSWORD | pm_rsts | PM_RSTS_HADWRQ_SET;
	}
	else if(bcm2708_reboot_mode == 'p')
	{
		// NOOBS < 1.3 halting
		pm_rsts = readl(__io_address(PM_RSTS));
		pm_rsts = PM_PASSWORD | pm_rsts | PM_RSTS_HADWRH_SET;
	}
	else
	{
		pm_rsts = calc_rsts(reboot_part);
	}

	writel(pm_rsts, __io_address(PM_RSTS));

	/* Setup watchdog for reset */
	pm_rstc = readl(__io_address(PM_RSTC));

	pm_wdog = PM_PASSWORD | (timeout & PM_WDOG_TIME_SET); // watchdog timer = timer clock / 16; need password (31:16) + value (11:0)
	pm_rstc = PM_PASSWORD | (pm_rstc & PM_RSTC_WRCFG_CLR) | PM_RSTC_WRCFG_FULL_RESET;

	writel(pm_wdog, __io_address(PM_WDOG));
	writel(pm_rstc, __io_address(PM_RSTC));
}

/* We can't really power off, but if we do the normal reset scheme, and indicate to bootcode.bin not to reboot, then most of the chip will be powered off */
static void bcm2708_power_off(void)
{
	extern char bcm2708_reboot_mode;

#if defined(CONFIG_MFD_RC5T619)
	rc5t619_power_off(); //pmic shutdown
#endif

	if(bcm2708_reboot_mode == 'q')
	{
		// NOOBS < v1.3
		bcm2708_restart('p', "");
	}
	else
	{
		/* partition 63 is special code for HALT the bootloader knows not to boot*/
		reboot_part = 63;
		/* continue with normal reset mechanism */
		bcm2708_restart(0, "");
	}
}

void __init bcm2708_init(void)
{
	int i;

#if defined(CONFIG_BCM_VC_CMA)
	vc_cma_early_init();
#endif
	printk("bcm2708.uart_clock = %d\n", uart_clock);
	pm_power_off = bcm2708_power_off;

	if (uart_clock)
		lookups[0].clk->rate = uart_clock;

	for (i = 0; i < ARRAY_SIZE(lookups); i++)
		clkdev_add(&lookups[i]);

	bcm_register_device(&bcm2708_dmaman_device);
	bcm_register_device(&bcm2708_vcio_device);
#ifdef CONFIG_BCM2708_GPIO
	bcm_register_device(&bcm2708_gpio_device);
	if (pps_gpio_pin >= 0) {
		pr_info("bcm2708: GPIO %d setup as pps-gpio device\n", pps_gpio_pin);
		pps_gpio_info.gpio_pin = pps_gpio_pin;
		pps_gpio_device.id = pps_gpio_pin;
		bcm_register_device(&pps_gpio_device);
	}
#endif
#if defined(CONFIG_W1_MASTER_GPIO) || defined(CONFIG_W1_MASTER_GPIO_MODULE)
	w1_gpio_pdata.pin = w1_gpio_pin;
	w1_gpio_pdata.ext_pullup_enable_pin = w1_gpio_pullup;
	platform_device_register(&w1_device);
#endif
	bcm_register_device(&bcm2708_systemtimer_device);
	bcm_register_device(&bcm2708_fb_device);
	bcm_register_device(&bcm2708_usb_device);
	bcm_register_device(&bcm2708_uart1_device);
	bcm_register_device(&bcm2708_powerman_device);

#ifdef CONFIG_MMC_SDHCI_BCM2708
	if (!bcm2835_mmc)
		bcm_register_device(&bcm2708_emmc_device);
#endif
#ifdef CONFIG_MMC_BCM2835
	if (bcm2835_mmc)
		bcm_register_device(&bcm2835_emmc_device);
#endif
	bcm2708_init_led();
	for (i = 0; i < ARRAY_SIZE(bcm2708_alsa_devices); i++)
		bcm_register_device(&bcm2708_alsa_devices[i]);

	bcm_register_device(&bcm2708_spi_device);

	if (vc_i2c_override) {
		bcm_register_device(&bcm2708_bsc0_device);
		bcm_register_device(&bcm2708_bsc1_device);
	} else if ((boardrev & 0xffffff) == 0x2 || (boardrev & 0xffffff) == 0x3) {
		bcm_register_device(&bcm2708_bsc0_device);
	} else {
		bcm_register_device(&bcm2708_bsc1_device);
	}

	bcm_register_device(&bcm2835_hwmon_device);
	bcm_register_device(&bcm2835_thermal_device);

#ifdef CONFIG_SND_BCM2708_SOC_I2S_MODULE
	bcm_register_device(&bcm2708_i2s_device);
#endif

#if defined(CONFIG_SND_BCM2708_SOC_HIFIBERRY_DAC) || defined(CONFIG_SND_BCM2708_SOC_HIFIBERRY_DAC_MODULE)
        bcm_register_device(&snd_hifiberry_dac_device);
        bcm_register_device(&snd_pcm5102a_codec_device);
#endif

#if defined(CONFIG_SND_BCM2708_SOC_HIFIBERRY_DACPLUS) || defined(CONFIG_SND_BCM2708_SOC_HIFIBERRY_DACPLUS_MODULE)
        bcm_register_device(&snd_rpi_hifiberry_dacplus_device);
        i2c_register_board_info(1, snd_pcm512x_hbdacplus_i2c_devices, ARRAY_SIZE(snd_pcm512x_hbdacplus_i2c_devices));
#endif

#if defined(CONFIG_SND_BCM2708_SOC_HIFIBERRY_DIGI) || defined(CONFIG_SND_BCM2708_SOC_HIFIBERRY_DIGI_MODULE)
        bcm_register_device(&snd_hifiberry_digi_device);
        i2c_register_board_info(1, snd_wm8804_i2c_devices, ARRAY_SIZE(snd_wm8804_i2c_devices));
#endif

#if defined(CONFIG_SND_BCM2708_SOC_HIFIBERRY_AMP) || defined(CONFIG_SND_BCM2708_SOC_HIFIBERRY_AMP_MODULE)
        bcm_register_device(&snd_hifiberry_amp_device);
        i2c_register_board_info(1, snd_tas5713_i2c_devices, ARRAY_SIZE(snd_tas5713_i2c_devices));
#endif


#if defined(CONFIG_SND_BCM2708_SOC_RPI_DAC) || defined(CONFIG_SND_BCM2708_SOC_RPI_DAC_MODULE)
        bcm_register_device(&snd_rpi_dac_device);
        bcm_register_device(&snd_pcm1794a_codec_device);
#endif

#if defined(CONFIG_SND_BCM2708_SOC_IQAUDIO_DAC) || defined(CONFIG_SND_BCM2708_SOC_IQAUDIO_DAC_MODULE)
        bcm_register_device(&snd_rpi_iqaudio_dac_device);
        i2c_register_board_info(1, snd_pcm512x_i2c_devices, ARRAY_SIZE(snd_pcm512x_i2c_devices));
#endif

#if defined(CONFIG_MFD_RC5T619) || defined(CONFIG_MFD_RC5T619_MODULE)
	i2c_register_board_info(0, i2c0_devs, ARRAY_SIZE(i2c0_devs));
#endif

	for (i = 0; i < ARRAY_SIZE(amba_devs); i++) {
		struct amba_device *d = amba_devs[i];
		amba_device_register(d, &iomem_resource);
	}
	system_rev = boardrev;
	system_serial_low = serial;

#ifdef CONFIG_BCM2708_SPIDEV
	spi_register_board_info(bcm2708_spi_devices,
			ARRAY_SIZE(bcm2708_spi_devices));
#endif
}

static void timer_set_mode(enum clock_event_mode mode,
			   struct clock_event_device *clk)
{
	switch (mode) {
	case CLOCK_EVT_MODE_ONESHOT: /* Leave the timer disabled, .set_next_event will enable it */
	case CLOCK_EVT_MODE_SHUTDOWN:
		break;
	case CLOCK_EVT_MODE_PERIODIC:

	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_RESUME:

	default:
		printk(KERN_ERR "timer_set_mode: unhandled mode:%d\n",
		       (int)mode);
		break;
	}

}

static int timer_set_next_event(unsigned long cycles,
				struct clock_event_device *unused)
{
	unsigned long stc;
	do {
		stc = readl(__io_address(ST_BASE + 0x04));
		/* We could take a FIQ here, which may push ST above STC3 */
		writel(stc + cycles, __io_address(ST_BASE + 0x18));
	} while ((signed long) cycles >= 0 &&
				(signed long) (readl(__io_address(ST_BASE + 0x04)) - stc)
				>= (signed long) cycles);
	return 0;
}

static struct clock_event_device timer0_clockevent = {
	.name = "timer0",
	.shift = 32,
	.features = CLOCK_EVT_FEAT_ONESHOT,
	.set_mode = timer_set_mode,
	.set_next_event = timer_set_next_event,
};

/*
 * IRQ handler for the timer
 */
static irqreturn_t bcm2708_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = &timer0_clockevent;

	writel(1 << 3, __io_address(ST_BASE + 0x00));	/* stcs clear timer int */

	evt->event_handler(evt);

	return IRQ_HANDLED;
}

static struct irqaction bcm2708_timer_irq = {
	.name = "BCM2708 Timer Tick",
	.flags = IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler = bcm2708_timer_interrupt,
};

/*
 * Set up timer interrupt, and return the current time in seconds.
 */

static struct delay_timer bcm2708_delay_timer = {
	.read_current_timer = bcm2708_read_current_timer,
	.freq = STC_FREQ_HZ,
};

static void __init bcm2708_timer_init(void)
{
	/* init high res timer */
	bcm2708_clocksource_init();

	/*
	 * Initialise to a known state (all timers off)
	 */
	writel(0, __io_address(ARM_T_CONTROL));
	/*
	 * Make irqs happen for the system timer
	 */
	setup_irq(IRQ_TIMER3, &bcm2708_timer_irq);

	setup_sched_clock(bcm2708_read_sched_clock, 32, STC_FREQ_HZ);

	timer0_clockevent.mult =
	    div_sc(STC_FREQ_HZ, NSEC_PER_SEC, timer0_clockevent.shift);
	timer0_clockevent.max_delta_ns =
	    clockevent_delta2ns(0xffffffff, &timer0_clockevent);
	timer0_clockevent.min_delta_ns =
	    clockevent_delta2ns(0xf, &timer0_clockevent);

	timer0_clockevent.cpumask = cpumask_of(0);
	clockevents_register_device(&timer0_clockevent);

	register_current_timer_delay(&bcm2708_delay_timer);
}

#if defined(CONFIG_LEDS_GPIO) || defined(CONFIG_LEDS_GPIO_MODULE)
#include <linux/leds.h>

static struct gpio_led bcm2708_leds[] = {
	[0] = {
	       .gpio = 16,
	       .name = "led0",
	       .default_trigger = "mmc0",
	       .active_low = 1,
	       },
};

static struct gpio_led_platform_data bcm2708_led_pdata = {
	.num_leds = ARRAY_SIZE(bcm2708_leds),
	.leds = bcm2708_leds,
};

static struct platform_device bcm2708_led_device = {
	.name = "leds-gpio",
	.id = -1,
	.dev = {
		.platform_data = &bcm2708_led_pdata,
		},
};

static void __init bcm2708_init_led(void)
{
  bcm2708_leds[0].gpio = disk_led_gpio;
  bcm2708_leds[0].active_low = disk_led_active_low;
  platform_device_register(&bcm2708_led_device);
}
#else
static inline void bcm2708_init_led(void)
{
}
#endif

void __init bcm2708_init_early(void)
{
	/*
	 * Some devices allocate their coherent buffers from atomic
	 * context. Increase size of atomic coherent pool to make sure such
	 * the allocations won't fail.
	 */
	init_dma_coherent_pool_size(SZ_4M);
}

static void __init board_reserve(void)
{
#if defined(CONFIG_BCM_VC_CMA)
	vc_cma_reserve();
#endif
}

MACHINE_START(BCM2708, "BCM2708")
    /* Maintainer: Broadcom Europe Ltd. */
	.map_io = bcm2708_map_io,
	.init_irq = bcm2708_init_irq,
	.init_time = bcm2708_timer_init,
	.init_machine = bcm2708_init,
	.init_early = bcm2708_init_early,
	.reserve = board_reserve,
	.restart	= bcm2708_restart,
MACHINE_END

module_param(boardrev, uint, 0644);
module_param(serial, uint, 0644);
module_param(uart_clock, uint, 0644);
module_param(disk_led_gpio, uint, 0644);
module_param(disk_led_active_low, uint, 0644);
module_param(reboot_part, uint, 0644);
module_param(w1_gpio_pin, uint, 0644);
module_param(w1_gpio_pullup, uint, 0644);
module_param(pps_gpio_pin, int, 0644);
MODULE_PARM_DESC(pps_gpio_pin, "Set GPIO pin to reserve for PPS");
module_param(bcm2835_mmc, uint, 0644);
module_param(vc_i2c_override, bool, 0644);
MODULE_PARM_DESC(vc_i2c_override, "Allow the use of VC's I2C peripheral.");
