/*
 *  Kernel ATX-Raspi, kernel level driver to allow reboot/shutdown of RPI with the ATX-Raspi 
 *  board from Low Power Labs LLC.
 *
 *  Copyright (c) 2015 Paul Downs
 *
 *  Based on the mk_arcade_joystick_rpi driver by Matthieu Proucelle
 */


/*
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
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/slab.h>

#include <linux/ioport.h>
#include <linux/reboot.h>
#include <asm/io.h>

MODULE_AUTHOR("Paul Downs");
MODULE_DESCRIPTION("ATX-Raspi Driver");
MODULE_LICENSE("GPL");

#define MK_MAX_DEVICES		1

#ifdef RPI2
#define PERI_BASE        0x3F000000
#else
#define PERI_BASE        0x20000000
#endif

#define GPIO_BASE                (PERI_BASE + 0x200000) /* GPIO controller */

#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))

#define GPIO_SET *(gpio+7)
#define GPIO_CLR *(gpio+10)

#define GET_GPIO(g) (*(gpio+13)&(1<<g)) // 0 if LOW, (1<<g) if HIGH

#define BSC1_BASE		(PERI_BASE + 0x804000)

#define MAX_ARGS 4

static volatile unsigned *gpio;

struct mk_config {
  int args[MAX_ARGS];
  unsigned int nargs;
};

static struct mk_config mk_cfg __initdata;

module_param_array_named(map, mk_cfg.args, int, &(mk_cfg.nargs), 0);
MODULE_PARM_DESC(map, "Enable or disable GPIO and ATX-Raspi Board");

/* This gives us, in theory, a 50m/s tick on kernel module which is fine for 
 * a button that needs to be held down for AT LEAST 200m/s to do anything (4 ticks)
 */
#define MK_REFRESH_TIME	HZ/50

struct mk {
  int button_pin;
  int shutdown_pin;
  int reboot_ticks;
  int max_ticks;
  int num_ticks_held; // How many of our modules ticks has the button been held for?
  int prev_down; // Were we held down before?
  int cur_down; // Are we held down now?
  int used;
};

static struct timer_list timer;
static struct mk *mk_base;

/* GPIO UTILS */
static void setGpioPullUps(int pullUps) {
  *(gpio + 37) = 0x02;
  udelay(10);
  *(gpio + 38) = pullUps;
  udelay(10);
  *(gpio + 37) = 0x00;
  *(gpio + 38) = 0x00;
}

static void setGpioAsInput(int gpioNum) {
  INP_GPIO(gpioNum);
}

static void setGpioAsOutput(int gpioNum) {
  INP_GPIO(gpioNum);
  OUT_GPIO(gpioNum);
}

static void mk_gpio_read_button(struct mk *mk) {
  if (GET_GPIO(mk->button_pin)) { // Means pushed, held for a 'tick' ~ 50ms.
	if (mk->cur_down == 0) {
		printk("Button is down\n");
		mk->num_ticks_held = 1;
		mk->cur_down = 1;
	} else {
		printk("Button still down\n");
		mk->prev_down = 1;
		mk->num_ticks_held++;
	}
  } else {
	if (mk->cur_down) {
		mk->prev_down = 1;
		printk("Button is up\n");
	}
	mk->cur_down = 0;
  }
}

/*
 * mk_timer() initiates reads of power button.
 */
static void mk_timer(unsigned long private) {
  struct mk *mk = (void *) private;
  mk_gpio_read_button(mk);
  if (mk->num_ticks_held >= mk->max_ticks) {
	// Initiate shutdown.
	printk("Shutdown initiated by ATX-Raspi Driver\n");
	//kernel_power_off();
  } else if (mk->prev_down == 1 && mk->cur_down == 0 && mk->num_ticks_held >= mk->reboot_ticks) {
	// Initiate reboot.
	printk("Reboot initiated by ATX-Raspi Driver\n");
	//kernel_restart(NULL);
  }
  printk("Status %d/%d is cur_down %d and prev_down %d\n",mk->num_ticks_held,mk->reboot_ticks,mk->cur_down,mk->prev_down);
  if (mk->cur_down == 0 && mk->prev_down == 1) { // RESET
	printk("Resetting status\n");
	mk->prev_down = 0;
	mk->num_ticks_held = 0;
	mk->cur_down = 0;
  }
  mod_timer(&timer, jiffies + MK_REFRESH_TIME);
}

static void __init mk_setup_pins(struct mk *mk, int *pins) {
  mk->button_pin = pins[0];
  mk->shutdown_pin = pins[1];
  
  setGpioAsInput(mk->button_pin);
  //setGpioPullUps(0x00100000);
  setGpioAsOutput(mk->shutdown_pin);
  GPIO_SET = 1 << mk->shutdown_pin;
}

static struct mk __init *mk_probe(int *pins, int n_pins) {
  struct mk *mk;
  int err;
  int ret;

  mk = kzalloc(sizeof (struct mk), GFP_KERNEL);
  if (!mk) {
    pr_err("Not enough memory\n");
    err = -ENOMEM;
    goto err_out;
  }
  
  mk->num_ticks_held = 0;
  mk->prev_down = 0;
  mk->cur_down = 0;
  
  if (n_pins > MAX_ARGS) {
    pr_err("Incorrect number of pins\n");
    err = -EINVAL;
    goto err_out;
  }

  mk_setup_pins(mk,pins);
  printk("Button listening on pin %d\n",mk->button_pin);
  printk("Pin %d set high for ATX-Raspi\n",mk->shutdown_pin);

  if(n_pins >= 3) {
	mk->reboot_ticks = pins[2];
  } else {
	mk->reboot_ticks = 10; // 10 ticks is 200ms
  }
  if (n_pins >= 4) {
	mk->max_ticks = pins[3];
  } else {
	mk->max_ticks = 30;
  }
  printk("Will reboot if held for over %d, and less than %d\n",mk->reboot_ticks,mk->max_ticks);
  printk("Will shutdown if held for %d or more\n",mk->max_ticks);
  
  if (n_pins > 4) {
	pr_err("Extended configuration incorrect\n");
	err = -EINVAL;
	goto err_out;
  }
  
  setup_timer(&timer, mk_timer, (long) mk);
  ret = mod_timer(&timer, jiffies + MK_REFRESH_TIME);
  if (ret)
	printk("ATX-Raspi Driver timer is being pissy\n");
  else
  	printk("Waking up %p again in %d jiffies\n",&timer,MK_REFRESH_TIME);
  
  return mk;

  err_out:
  return ERR_PTR(err);
}

static int __init mk_init(void) {
  /* Set up gpio pointer for direct register access */
  if ((gpio = ioremap(GPIO_BASE, 0xB0)) == NULL) {
    pr_err("io remap failed\n");
    return -EBUSY;
  }
  if (mk_cfg.nargs < 1) {
    pr_err("at least one device must be specified\n");
    return -EINVAL;
  } else {
    mk_base = mk_probe(mk_cfg.args, mk_cfg.nargs);
    printk("All your MK_BASE are belong to us @ %p\n",mk_base);
    if (IS_ERR(mk_base))
      return -ENODEV;
  }
  printk("ATX-Raspi Driver timer %p good to go\n",&timer);
  return 0;
}

static void __exit mk_exit(void) {
  int ret;
  printk("Uninstalling ATX-Raspi Driver\n");
  ret = del_timer(&timer);
  if (ret) 
	printk("ATX-Raspi Driver timer still in use\n");
  if (mk_base)
    kfree(mk_base);

  iounmap(gpio);
}

module_init(mk_init);
module_exit(mk_exit);
