#include <linux/mmc/host.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <asm/io.h>
#include <asm/rb.h>
#include <gxio/gpio.h>

#include <hv/drv_srom_intf.h>

static int devhdl = -1;
static DEFINE_SPINLOCK(nor_lock);

static unsigned long mmc_pins = MMC_PINS;
static unsigned long gpi_sd_cdn = GPI_SD_CDn;
static unsigned long gpo_sd_pwr = GPO_SD_PWR;

#define FN_ARG int (*fn)(int, __hv32, HV_VirtAddr, __hv32, __hv64)

static int nor_fn(void *buf, int count, unsigned long offset, FN_ARG) {
    int retval;
    int retries = 1000;
    unsigned long flags;

    spin_lock_irqsave(&nor_lock, flags);
    for (;;) {
	retval = fn(devhdl, 0, (HV_VirtAddr) buf, count, offset);
	if (retval == HV_EBUSY && --retries > 0) {
	    mdelay(20);
	    continue;
	}
	else if (retval < 0) {
	    continue;
	}
	else break;
    }
    spin_unlock_irqrestore(&nor_lock, flags);

    return retval;
}  

int nor_read(void *buf, int count, unsigned long offset) {
    return nor_fn(buf, count, offset, hv_dev_pread);
}  
EXPORT_SYMBOL(nor_read);

static int nor_write(void *buf, int count, unsigned long offset) {
    return nor_fn(buf, count, offset, hv_dev_pwrite);
}

int nor_program(void *buf, int count, unsigned long offset) {
    char dummy;
    int retval;
    retval = nor_write(buf, count, offset);
    nor_write(&dummy, 1, SROM_FLUSH_OFF);
    return retval;
}
EXPORT_SYMBOL(nor_program);

static unsigned read_unsigned(unsigned *offset) {
    unsigned val = 0;
    nor_read(&val, sizeof(unsigned), *offset);
    *offset += sizeof(unsigned);
    return val;    
}

int read_booter_cfg(unsigned id, void *buf, int amount) {
    unsigned offset = 0x30000;
    if (read_unsigned(&offset) != 0x64726148) return 0;
    while (offset < 0x31000) {
	unsigned data = read_unsigned(&offset);
	unsigned tag = data & 0xffff;	
	int len = data >> 16;	
	if (len == 0 || len > 0x1000 || tag == 0) {
	    break;
	} else if (tag == id) {
	    amount = min(len, amount);
	    nor_read(buf, amount, offset);
	    return amount;
	}
	offset = offset + len;
    }
    return 0;
}
EXPORT_SYMBOL(read_booter_cfg);

static struct hrtimer beep_timer;
static unsigned beep_interval = 0;
static gxio_gpio_context_t gpio_ctx;

void set_gpio(unsigned long value, unsigned long mask) {
    gxio_gpio_set(&gpio_ctx, value, mask);
}
EXPORT_SYMBOL(set_gpio);

unsigned long get_gpio(void) {
    return gxio_gpio_get(&gpio_ctx);
}
EXPORT_SYMBOL(get_gpio);

static enum hrtimer_restart beep_callback(struct hrtimer *t) {
    static unsigned state = 1;
    set_gpio((state = !state) ? GPO_BEEP : 0, GPO_BEEP);
    hrtimer_forward_now(&beep_timer, ns_to_ktime(beep_interval));
    return HRTIMER_RESTART;
}

void router_beep(int freq) {
    unsigned long flags;
    printk("tile_beep: %d\n", freq);

    spin_lock_irqsave(&nor_lock, flags);
    if (freq > 0) {
        beep_interval = 1000000000 / (2 * freq);
        hrtimer_start(&beep_timer, ns_to_ktime(0), HRTIMER_MODE_REL_PINNED);
    }
    else {
        hrtimer_cancel(&beep_timer);
	gxio_gpio_set(&gpio_ctx, 0, GPO_BEEP);
    }
    spin_unlock_irqrestore(&nor_lock, flags);
}
EXPORT_SYMBOL(router_beep);

static spinlock_t latch_lock;
static unsigned hw_options = 0;

int ccr_init(void) {
    char name[64];
    spin_lock_init(&latch_lock);
    devhdl = hv_dev_open((HV_VirtAddr) "srom/0/0", 0);
    read_booter_cfg(ID_BOARD_NAME, name, sizeof(name));
    read_booter_cfg(ID_HW_OPTIONS, &hw_options, sizeof(unsigned));
    printk("Hello [%s] HW=%08x\n", name, hw_options);
    return 0;
}
early_initcall(ccr_init);

unsigned long gpio_irq_refresh(unsigned long pin) {
    uint64_t s1 = pin, s2 = pin;
    gxio_gpio_report_reset_interrupt(&gpio_ctx, &s1, &s2);
    return (s1 | s2);
}
EXPORT_SYMBOL(gpio_irq_refresh);

void gpio_irq(unsigned irq, unsigned long pin) {
    int my_cpu = smp_processor_id();
    int x = cpu_x(my_cpu);
    int y = cpu_y(my_cpu);
    gxio_gpio_cfg_interrupt(&gpio_ctx, x, y, KERNEL_PL, irq, pin, pin);
    gpio_irq_refresh(pin);
}
EXPORT_SYMBOL(gpio_irq);

static struct spi_board_info ccr_spi_microsd = {
	.modalias = "mmc_spi",
	.max_speed_hz = 10 * 1000 * 1000,
	.bus_num = 0,
	.chip_select = 0,
	.controller_data = (void*)56,
	.mode = SPI_MODE_3,
};

static struct spi_board_info *ccr_spi_info[] = {
	&ccr_spi_microsd, NULL
};

static struct platform_device tile_mmc_device = {
	.name	= "ccr-mmc-spi",
	.id	= -1,
	.dev	= {
		.platform_data = ccr_spi_info,
	},
};

static struct sim_gpio ccr1009_sim_gpio = {
	.io	= GPIO_SIM_IO,
	.clk	= GPO_SIM_CLK,
	.rst	= GPO_SIM_RST,
	.en	= GPO_SIM_EN,
	.vsel	= GPO_SIM_3V_1V8n,
};
static struct platform_device ccr1009_sim_device = {
	.name	= "ccr-sim",
	.id	= -1,
	.dev	= {
		.platform_data = &ccr1009_sim_gpio,
	},
};

static struct sim_gpio ccr1072_sim_gpio = {
	.io	= (1ULL << 35),
	.clk	= (1ULL << 36),
	.rst	= (1ULL << 37),
	.en	= (1ULL << 38),
	.vsel	= (1ULL << 39),
};
static struct platform_device ccr1072_sim_device = {
	.name	= "ccr-sim",
	.id	= -1,
	.dev	= {
		.platform_data = &ccr1072_sim_gpio,
	},
};

static struct platform_device tile_led_device = {
	.name	= "ccr-led",
	.id	= -1,
};

static struct platform_device tile_uart_device = {
	.name	= "ccr-uart",
	.id	= -1,
};

static struct platform_device tile_crypto_device = {
	.name	= "tile",
	.id	= -1,
};

static struct platform_device tile_eth_device = {
	.name	= "tilegx",
	.id	= -1,
};

static struct platform_device *tile_devices[] = {
	&tile_led_device,
	&tile_uart_device,
	&tile_crypto_device,
	&tile_eth_device,
};

static struct platform_device *tile_mmc_devices[] = {
	&tile_mmc_device,
};

static int tile_gpio_get(struct gpio_chip *chip, unsigned offset) {
    return (get_gpio() & BIT(offset)) != 0;
}

static void tile_gpio_set(struct gpio_chip *chip, unsigned offset, int value) {    
    unsigned long bit = BIT(offset);
    set_gpio(value ? bit : 0, bit);
}

static int tile_direction_input(struct gpio_chip *chip, unsigned offset) {
    gxio_gpio_set_dir(&gpio_ctx, 0, BIT(offset), 0, 0);
    return 0;
}

static int tile_direction_output(struct gpio_chip *chip, unsigned offset, int value) {
    tile_gpio_set(chip, offset, value);
    gxio_gpio_set_dir(&gpio_ctx, 0, 0, BIT(offset), 0);
    tile_gpio_set(chip, offset, value);
    return 0;
}

static struct gpio_chip tile_gpio_chip = {
    .label			= "gpio",
    .direction_input		= tile_direction_input,
    .direction_output		= tile_direction_output,
    .set			= tile_gpio_set,
    .get			= tile_gpio_get,
    .base			= 0,
    .ngpio			= 64,
};

static void micro_sd_power_toggle(void) {
    int card_in = (get_gpio() & gpi_sd_cdn) == 0;
    printk("MicroSD power %s\n", card_in ? "on" : "off");
    set_gpio(card_in ? gpo_sd_pwr : 0, gpo_sd_pwr);
}

irqreturn_t mmc_cd_irq(int irq, void *ptr) {
    struct spi_device *spi = ptr;
    void *mmc_host = dev_get_drvdata(&spi->dev);
    printk("mmc card detect irq\n");
    if (gpio_irq_refresh(gpi_sd_cdn) & gpi_sd_cdn && mmc_host) {
	printk("toggle power and detect\n");
	micro_sd_power_toggle();
	mmc_detect_change(mmc_host, msecs_to_jiffies(100));
    }
    return IRQ_HANDLED;
}

void init_ccr_spi(struct spi_master *master) {
    int irq_num;
    struct spi_device *spi_dev;
    spi_dev = spi_new_device(master, &ccr_spi_microsd);

    irq_num = irq_alloc_hwirq(-1);
    if (irq_num <= 0)
    {
        printk("failed to allocate irq for microsd\n");
        return;
    }
    tile_irq_activate(irq_num, TILE_IRQ_PERCPU);
    if (request_irq(irq_num, &mmc_cd_irq, IRQF_SHARED, "MicroSD", spi_dev)) {
        printk("failed to request irq for microsd\n");
	irq_free_hwirq(irq_num);
	return;
    }
    gpio_irq(irq_num, gpi_sd_cdn);

}
EXPORT_SYMBOL(init_ccr_spi);

static int is_prefix(char *str, char *base) {
    unsigned len = strlen(str);
    if (strlen(base) < len) return 0;
    return (memcmp(base, str, len) == 0) ? 1 : 0;
}

#define FETCH_NAME(id) \
    static int fetched = 0; \
    static char name[64]; \
    if (!fetched) { \
	read_booter_cfg(id, name, sizeof(name)); \
	fetched = 1; \
    }

int is_name_prefix(char *str) {
    FETCH_NAME(ID_BOARD_NAME);
    return is_prefix(str, name);
}
EXPORT_SYMBOL(is_name_prefix);

int is_board_type(char *str) {
    FETCH_NAME(ID_BOARD_TYPE_NAME);
    return strcmp(str, name) == 0;
}
EXPORT_SYMBOL(is_board_type);

int fan_sense_count(void) {
    return is_board_type("ccr1009c") ? 1 : 2;
}
EXPORT_SYMBOL(fan_sense_count);

unsigned long fan_sense_gpios(void) {
    return GPI_SENSE(1) | (fan_sense_count() < 2 ? 0 : GPI_SENSE(2));
}
EXPORT_SYMBOL(fan_sense_gpios);

struct latch_info {
    unsigned long strobe;
    unsigned long state;
    int input;
    int output;
};

#define latch_delay() __insn_mf(); ndelay(15)
unsigned long access_latch(int num, unsigned long value, unsigned long mask) {
    static struct latch_info info[] = {
	{ .strobe = GPO_PR_STROBE1,
	  .state = RST_USB | RST_PCI | RST_ETH | RST_LCD | RST_LED | RST_I2C,
	  .input = 0,
	  .output = 16,
	},
	{ .strobe = GPO_PR_STROBE2,
	  .state = 0,
	  .input = 32,
	  .output = 40,
	},
    };

    int i;
    unsigned long flags;
    unsigned long ret = 0;
    struct latch_info *latch = &info[num];
    spin_lock_irqsave(&latch_lock, flags);
    latch->state = (latch->state & ~mask) | (value & mask);
    set_gpio(0, GPO_AUX_RESET | latch->strobe);
    for (i = 0; i < latch->output; ++i) {
	unsigned long pin = (latch->state & (1ULL << i)) ? GPO_PR_MOSI : 0;
	set_gpio(pin, GPO_PR_MOSI | GPO_PR_CLK);
	latch_delay();
	set_gpio(pin | GPO_PR_CLK, GPO_PR_MOSI | GPO_PR_CLK);
	latch_delay();
    }
    set_gpio(latch->strobe, latch->strobe);
    for (i = 0; i < latch->input; ++i) {
	set_gpio(0, GPO_PR_CLK);
	latch_delay();
	ret = (ret << 1) | ((get_gpio() & GPI_PR_MISO) ? 1 : 0);
	set_gpio(GPO_PR_CLK, GPO_PR_CLK);
	latch_delay();
    }
    spin_unlock_irqrestore(&latch_lock, flags);
    return ret;
}
EXPORT_SYMBOL(access_latch);

void ccr_deassert_usb_reset(unsigned i) {
    static int nr[] = { 13, 12 };
    if (is_name_prefix("CCR1072") && i < ARRAY_SIZE(nr)) {
	printk("CCR1072: deasserting USB%d reset (pin%d)\n", i, nr[i]);
	access_latch(0, 0, BIT(nr[i]));
	udelay(1000);
    }
}
EXPORT_SYMBOL(ccr_deassert_usb_reset);

struct class_attribute_gpio {
    struct class_attribute class_attr;
    unsigned long gpio_bit;
};

static ssize_t show_gpio_state(struct class *class,
			struct class_attribute *attr,
			char *output_buffer)
{
    struct class_attribute_gpio *info;
    info = container_of(attr, struct class_attribute_gpio, class_attr);
    return scnprintf(output_buffer, PAGE_SIZE, "%d\n",
		     !!(get_gpio() & info->gpio_bit));
}

static ssize_t store_nothing(struct class *class,
			     struct class_attribute *attr,
			     const char *buf, size_t count)
{
    return count;
}

unsigned long add_sysfs_gpio_state_file(struct class *hw_info,
					const char *filename,
					int gpio_num) {

	struct class_attribute_gpio *info;
	info = kmalloc(sizeof(struct class_attribute_gpio), GFP_KERNEL);
	memset(info, 0, sizeof(struct class_attribute_gpio));

	info->class_attr.attr.name = filename;
	info->class_attr.attr.mode = S_IWUSR | S_IRUGO;
	info->class_attr.show = &show_gpio_state;
	info->class_attr.store = &store_nothing;
	info->gpio_bit = BIT(gpio_num);

	if (class_create_file(hw_info, &info->class_attr) < 0) {
	    printk("add_sysfs_gpio_state_file: failed\n");
	}

	return info->gpio_bit;
}

unsigned long add_psu_state(unsigned gpio_numA, unsigned gpio_numB) {
    unsigned long pins = 0;
    struct class *hw_info = class_create(THIS_MODULE, "hw_info");
    pins |= add_sysfs_gpio_state_file(hw_info, "psu1_state", gpio_numA);
    pins |= add_sysfs_gpio_state_file(hw_info, "psu2_state", gpio_numB);
    return pins;
}

void usb_power_enable(void) {
    set_gpio(is_board_type("ccr1009c") ? GPO_USB_POWER : 0, GPO_USB_POWER);
}
EXPORT_SYMBOL(usb_power_enable);

int ccr_platform_init(void) {
    unsigned long in_pins, out_pins, od_pins, all_pins = 0;
    struct platform_device *pdev_sim = NULL;
    in_pins = GPI_BUTTON
	| fan_sense_gpios()
	| GPI_PIN_HOLE;
    out_pins = GPO_BEEP
	| GPO_USER_LED
	| GPO_MON_SEL
	| GPO_LCD_LED
	| GPO_USB_POWER;
    od_pins = GPO_FAN_ON(0);

    if (is_name_prefix("CCR1036-8G")
	|| is_name_prefix("CCR1036-12S")
	|| is_name_prefix("CCR1016-12S")) {
	od_pins |= GPO_FAN_ON(1);
    }

    if (is_name_prefix("CCR1072")) {
	struct spi_board_info **spi_info;
	in_pins = GPI_PR_MISO | GPI_BUTTON;
	out_pins = GPO_PR_STROBE1
	    | GPO_PR_STROBE2
	    | GPO_PR_CLK
	    | GPO_PR_MOSI
	    | GPO_BEEP
	    | GPO_USER_LED
	    | GPO_MON_SEL72
	    | GPO_LCD_LED
	    | GPO_AUX_RESET;

	gpi_sd_cdn = (1ULL << 40);
	gpo_sd_pwr = (1ULL << 41);
	mmc_pins = (0x3FULL << 42);

	tile_mmc_device.name = "ccr-mmc-spi-ccr1072";
	spi_info = tile_mmc_device.dev.platform_data;
	spi_info[0]->controller_data = (void *) 44;
    }

    if (is_name_prefix("CCR1009-8G-1S-1S+")) {
	in_pins |= add_psu_state(35, 34);
    }

    if (is_name_prefix("CCR1016-12S-1S+")) {
	in_pins |= add_psu_state(48, 50);
    }

    if (hw_options & HW_OPT_HAS_uSD) {
	in_pins |= gpi_sd_cdn;
	out_pins |= gpo_sd_pwr;
	all_pins |= mmc_pins;
    }
    if (hw_options & HW_OPT_HAS_SIM) {
	struct sim_gpio *gpio;
	pdev_sim = &ccr1009_sim_device;
	if (is_name_prefix("CCR1072")) pdev_sim = &ccr1072_sim_device;

	gpio = (struct sim_gpio *)pdev_sim->dev.platform_data;
	od_pins |= gpio->io;
	out_pins |= gpio->clk | gpio->rst | gpio->en | gpio->vsel;
    }

    all_pins |= out_pins | in_pins | od_pins;

    gxio_gpio_init(&gpio_ctx, 0);
    gxio_gpio_attach(&gpio_ctx, all_pins);
    gxio_gpio_set_dir(&gpio_ctx, 0, in_pins, out_pins, od_pins);
    usb_power_enable();

    hrtimer_init(&beep_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    beep_timer.function = beep_callback;
    gpiochip_add(&tile_gpio_chip);

    if (hw_options & HW_OPT_HAS_uSD) {
	printk("CCR with MicroSD slot!\n");
	set_gpio(0, gpo_sd_pwr);
	udelay(10000);
	micro_sd_power_toggle();
	printk("platfork_add_devices(tile_mmc_devices) = %d\n", platform_add_devices(tile_mmc_devices, ARRAY_SIZE(tile_mmc_devices)));
    }
/*
    if (pdev_sim) {
	struct sim_gpio *gpio = (struct sim_gpio *)pdev_sim->dev.platform_data;
	printk("CCR with SIM card slot!\n");
	set_gpio(gpio->vsel,
		 gpio->io | gpio->clk | gpio->rst | gpio->en | gpio->vsel);
	platform_device_register(pdev_sim);
    }
*/
    if (is_name_prefix("CCR1072")) {
	/* use USB type A connector by default */
	access_latch(0, BIT(0), BIT(0));
    }

    return platform_add_devices(tile_devices, ARRAY_SIZE(tile_devices));
}

arch_initcall(ccr_platform_init);

