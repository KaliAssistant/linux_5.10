// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Lichee-Jack MOD-SWITCH (SP3T) GPIO Driver
 * -----------------------------------------
 *
 * This driver provides support for the SP3T RF mod-switch found on
 * Lichee-Jack boards (CV181X SoC). The switch is connected to GPIOA_15
 * and GPIOA_24 (SPK_EN / EMMC_DAT1 muxed pins). The driver reads the GPIO state
 * and reports the current switch position through a misc character device.
 *
 * Features:
 *   - Polls switch state via GPIO inputs every 20 ms.
 *   - Reports switch position as '0', '1', or '2' when read from userspace.
 *   - Simple misc device interface: /dev/modsw
 *   - Minimal resource usage (lightweight polling thread).
 *
 * Device Node:
 *   /dev/modsw
 *
 * Read Interface:
 *   - open("/dev/modsw", O_RDONLY);
 *   - read(fd, &buf, 1);
 *   - buf contains ASCII '0', '1', or '2':
 *       '0' = unknown / idle
 *       '1' = switch position 1
 *       '2' = switch position 2
 *
 * Example Userspace Program:
 *   int fd = open("/dev/modsw", O_RDONLY);
 *   char pos;
 *   read(fd, &pos, 1);
 *   printf("Switch position = %c\n", pos);
 *
 * Limitations:
 *   - Polling interval is fixed at 20 ms (can be tuned).
 *   - Only supports the SP3T switch via GPIOA_15/GPIOA_24.
 *   - No interrupt-driven support (GPIO IRQ not used).
 *
 * Author: KaliAssistant <work.kaliassistant.github@gmail.com>
 * Copyright (C) 2025 KaliAssistant - All Rights Reserved
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/gpio.h>

#define DEVICE_NAME "modsw"
#define GPIO_BASE   0x03020000
#define GPIO_EXT    0x50   /* offset: GPIO_EXT_PORTA */
#define GPIO_LINE1  15     /* GPIOA_15 */
#define GPIO_LINE2  24     /* GPIOA_24 */

static unsigned long PINCFG_A15_REG = 0x03001908UL;
static unsigned long PINCFG_A24_REG = 0x0300192CUL;

static void __iomem *gpio_base;
static void __iomem *pincfg_a15_v;
static void __iomem *pincfg_a24_v;
static struct task_struct *modsw_thread;
static char switch_state = '0';

/**
 * modsw_get_state - Read GPIO pins and determine switch state
 *
 * This function reads the values of GPIOA_15 and GPIOA_24, then
 * maps the pin combination to a logical switch state.
 *
 * Return:
 * * '1' - switch in position 1 (GPIO15=1, GPIO24=0)
 * * '2' - switch in position 2 (GPIO15=0, GPIO24=1)
 * * '0' - idle or invalid state (any other combination)
 */
static char modsw_get_state(void)
{
    u32 val = readl(gpio_base + GPIO_EXT);
    int a15 = !!(val & (1 << GPIO_LINE1));
    int a24 = !!(val & (1 << GPIO_LINE2));

    if (!a15 && a24)
        return '1';
    else if (a15 && !a24)
        return '2';
    else
        return '0';
}

/**
 * modsw_thread_fn - Polling thread for MOD-SWITCH state
 * @data: unused thread data
 *
 * This kernel thread runs in the background and periodically polls
 * the GPIO pins to detect switch state changes. It updates the
 * global @switch_state variable every 20 ms.
 *
 * Return: always 0 when thread exits
 */
static int modsw_thread_fn(void *data)
{
    while (!kthread_should_stop()) {
        switch_state = modsw_get_state();
        msleep(20);
    }
    return 0;
}

/**
 * modsw_read - Read current switch state from device
 * @file: pointer to struct file
 * @buf: user-space buffer to copy state into
 * @count: number of bytes requested
 * @ppos: file position pointer
 *
 * Reads one byte representing the switch state ('0', '1', '2')
 * into the user buffer.
 *
 * Return:
 * * 1 if successful (one byte copied)
 * * -EFAULT if copy_to_user fails
 */
static ssize_t modsw_read(struct file *file, char __user *buf,
                          size_t count, loff_t *ppos)
{
    if (*ppos > 0)
        return 0;

    if (copy_to_user(buf, &switch_state, 1))
        return -EFAULT;

    *ppos += 1;
    return 1;
}

static inline void set_pin_pu_pd(void __iomem *paddr, int pu_bit, int pd_bit)
{
    u32 v;

    /* read-modify-write only the documented bits; leave other bits unchanged */
    v = readl(paddr);
    /* set PU bit */
    v |= (1u << pu_bit);
    /* clear PD bit */
    v &= ~(1u << pd_bit);
    writel(v, paddr);
    /* read back to ensure write posted */
    readl(paddr);
}

static const struct file_operations modsw_fops = {
    .owner = THIS_MODULE,
    .read  = modsw_read,
};

static struct miscdevice modsw_dev = {
    .minor = MISC_DYNAMIC_MINOR,
    .name  = DEVICE_NAME,
    .fops  = &modsw_fops,
};

/**
 * modsw_init - Module initialization
 *
 * Maps GPIO registers, registers the misc device, and starts the
 * polling thread for monitoring MOD-SWITCH state.
 *
 * Return:
 * * 0 on success
 * * negative error code on failure
 */
static int __init modsw_init(void)
{
    int ret;

    pincfg_a15_v = ioremap(PINCFG_A15_REG, 0x1000);
    if (!pincfg_a15_v) {
        pr_err("modsw: failed to map pincfg_a15 page\n");
        ret = -ENOMEM;
        goto out;
    }

    pincfg_a24_v = ioremap(PINCFG_A24_REG, 0x1000);
    if (!pincfg_a24_v) {
        pr_err("modsw: failed to map pincfg_a24 page\n");
        ret = -ENOMEM;
        goto unmap_a15;
    }

    gpio_base = ioremap(GPIO_BASE, 0x1000);
    if (!gpio_base) {
        pr_err("modsw: failed to map GPIO registers\n");
        ret = -ENOMEM;
        goto unmap_a24;
    }

    set_pin_pu_pd(pincfg_a15_v, 2, 3);
    set_pin_pu_pd(pincfg_a24_v, 2, 3);

    ret = misc_register(&modsw_dev);
    if (ret) {
        pr_err("modsw: failed to register misc device\n");
        goto unmap_gpio;
    }

    modsw_thread = kthread_run(modsw_thread_fn, NULL, "modsw_thread");
    if (IS_ERR(modsw_thread)) {
        ret = PTR_ERR(modsw_thread);
        pr_err("modsw: failed to create polling thread\n");
        modsw_thread = NULL;
        goto dereg_misc;
    }

    pr_info("modsw: driver loaded\n");
    return 0;

dereg_misc:
    misc_deregister(&modsw_dev);

unmap_gpio:
    if (gpio_base) {
        iounmap(gpio_base);
        gpio_base = NULL;
    }

unmap_a24:
    if (pincfg_a24_v) {
        iounmap(pincfg_a24_v);
        pincfg_a24_v = NULL;
    }

unmap_a15:
    if (pincfg_a15_v) {
        iounmap(pincfg_a15_v);
        pincfg_a15_v = NULL;
    }

out:
    return ret;

}

/**
 * modsw_exit - Module cleanup
 *
 * Stops the polling thread, deregisters the misc device,
 * and unmaps GPIO registers.
 */
static void __exit modsw_exit(void)
{
    if (modsw_thread)
        kthread_stop(modsw_thread);

    misc_deregister(&modsw_dev);

    if (gpio_base) {
        iounmap(gpio_base);
        gpio_base = NULL;
    }

    if (pincfg_a15_v) {
        iounmap(pincfg_a15_v);
        pincfg_a15_v = NULL;
    }

    if (pincfg_a24_v) {
        iounmap(pincfg_a24_v);
        pincfg_a24_v = NULL;
    }

    pr_info("modsw: driver unloaded\n");
}

module_init(modsw_init);
module_exit(modsw_exit);

MODULE_LICENSE("GPL v3");
MODULE_AUTHOR("KaliAssistant <work.kaliassistant.github@gmail.com>");
MODULE_DESCRIPTION("Kernel-space MOD-SWITCH (SP3T) GPIO driver for Lichee-Jack");
