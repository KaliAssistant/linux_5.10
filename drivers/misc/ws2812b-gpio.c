// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Lichee-Jack WS2812B GPIO-only Driver
 * ------------------------------------
 *
 * This driver provides basic support for WS2812B (NeoPixel) addressable RGB LEDs
 * using GPIO bit-banging on the Lichee-Jack platform. It is implemented entirely
 * in kernel space for precise timing control.
 *
 * Features:
 *   - GPIO-based WS2812B signal generation (no DMA/PWM/PCM required).
 *   - Configurable power GPIO line for LED supply switching.
 *   - Periodic refresh handled by a kernel thread (~30 FPS).
 *   - Shared framebuffer exposed to userspace via mmap().
 *   - Userspace can write RGB data directly into the mapped buffer.
 *
 * Device Node:
 *   /dev/ws2812b
 *
 * Framebuffer Layout:
 *   - Each LED uses 3 bytes: [G, R, B].
 *   - LED_COUNT determines the number of addressable LEDs.
 *   - Default build sets LED_COUNT = 1, FRAME_SIZE = 3.
 *
 * Timing:
 *   - Bitstream generated with local_irq_disable() and preempt_disable()
 *     to ensure nanosecond-level accuracy required by WS2812B.
 *   - T0H = 150 ns, T0L = 600 ns
 *   - T1H = 600 ns, T1L = 200 ns
 *
 * Limitations:
 *   - Busy-waits during LED refresh, consumes CPU cycles.
 *   - Accuracy depends on CPU clock speed and may drift on underclock/overclock.
 *   - Not suitable for high LED counts at low CPU frequency.
 *
 * Example Usage (userspace):
 *   int fd = open("/dev/ws2812b", O_RDWR);
 *   uint8_t *fb = mmap(NULL, PAGE_SIZE, PROT_WRITE, MAP_SHARED, fd, 0);
 *   fb[0] = 0x00; // G
 *   fb[1] = 0xFF; // R
 *   fb[2] = 0x00; // B  → LED shows red
 *
 * Power Control:
 *   - PWR_GPIO_LINE controls external LED VDD power (on by default at init).
 *   - Power is turned off during module exit.
 *
 * Author: KaliAssistant <work.kaliassistant.github@gmail.com>
 * Copyright (C) 2025 KaliAssistant - All Rights Reserved
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/preempt.h>
#include <linux/irqflags.h>

#define DEVICE_NAME "ws2812b"
#define LED_COUNT   1
#define FRAME_SIZE  (LED_COUNT*3)
#define BUF_SIZE  PAGE_SIZE   // page-aligned for mmap

// WS2812B timing (ns)
#define T0H 150
#define T0L 600
#define T1H 600
#define T1L 200

// GPIO registers
#define GPIO_BASE 0x03020000
#define GPIO_DIR  0x03020004

#define GPIO_LINE 25
#define PWR_GPIO_LINE 27

static void __iomem *gpio_base;
static void __iomem *gpio_dir;
static uint8_t *framebuf;
static struct task_struct *ws_thread;



/**
 * gpio_set - Set WS2812B GPIO line high
 */
static inline void gpio_set(void)
{
    uint32_t val = readl(gpio_base);
    val |= (1 << GPIO_LINE);
    writel(val, gpio_base);
}

/**
 * gpio_clear - Set WS2812B GPIO line low
 */
static inline void gpio_clear(void)
{
    uint32_t val = readl(gpio_base);
    val &= ~(1 << GPIO_LINE);
    writel(val, gpio_base);
}

/**
 * power_on - Enable LED power by setting PWR_GPIO_LINE high
 */
static inline void power_on(void)
{
    uint32_t val = readl(gpio_base);
    val |= (1 << PWR_GPIO_LINE);
    writel(val, gpio_base);
}


/**
 * power_off - Disable LED power by setting PWR_GPIO_LINE low
 */
static inline void power_off(void)
{
    uint32_t val = readl(gpio_base);
    val &= ~(1 << PWR_GPIO_LINE);
    writel(val, gpio_base);
}

/**
 * ws2812b_send_led - Transmit a single LED color frame
 * @g: Green value (0–255)
 * @r: Red value (0–255)
 * @b: Blue value (0–255)
 *
 * Sends a 24-bit GRB sequence to the LED using precise timing with
 * interrupts and preemption disabled.
 */
static void ws2812b_send_led(uint8_t g, uint8_t r, uint8_t b)
{
    int i, c;
    uint8_t colors[3] = { g, r, b };

    local_irq_disable();
    preempt_disable();

    /* Must clear gpio here*/
    gpio_clear();
    ndelay(T1H);
  
    for (c = 0; c < 3; c++) {
        uint8_t byte = colors[c];
        for (i = 7; i >= 0; i--) {
            if (byte & (1 << i)) {
                gpio_set();
                ndelay(T1H);
                gpio_clear();
                ndelay(T1L);
            } else {
                gpio_set();
                ndelay(T0H);
                gpio_clear();
                ndelay(T0L);
            }
        }
    }

    preempt_enable();
    local_irq_enable();
}

/**
 * ws_thread_fn - Kernel thread to refresh LED data
 * @data: unused
 *
 * Periodically transmits the framebuffer contents to all LEDs at ~30 FPS.
 *
 * Return: always 0
 */
static int ws_thread_fn(void *data)
{
    while (!kthread_should_stop()) {
        int i;
        for (i = 0; i < LED_COUNT; i++) {
            ws2812b_send_led(framebuf[i*3 + 0], framebuf[i*3 + 1], framebuf[i*3 + 2]);
        }
        usleep_range(31000, 33333);
    }
    return 0;
}

/**
 * ws2812_mmap - Map framebuffer into userspace
 * @file: file pointer
 * @vma: VMA region to map into
 *
 * Provides mmap() access to the LED framebuffer. Userspace can write
 * RGB data directly into this buffer.
 *
 * Return: 0 on success, -EINVAL if requested size exceeds one page
 */
static int ws2812_mmap(struct file *file, struct vm_area_struct *vma)
{
    unsigned long pfn = virt_to_phys(framebuf) >> PAGE_SHIFT;
    size_t size = vma->vm_end - vma->vm_start;

    if (size > PAGE_SIZE)
        return -EINVAL;

    return remap_pfn_range(vma,
                           vma->vm_start,
                           pfn,
                           size,
                           vma->vm_page_prot);
}

static const struct file_operations ws_fops = {
    .owner = THIS_MODULE,
    .mmap  = ws2812_mmap,
};

static struct miscdevice ws_dev = {
    .minor = MISC_DYNAMIC_MINOR,
    .name  = DEVICE_NAME,
    .fops  = &ws_fops,
};


/**
 * ws2812b_init - Module initialization
 *
 * Maps GPIO registers, configures LED and power pins as outputs,
 * allocates framebuffer, registers misc device, and starts refresh thread.
 *
 * Return: 0 on success, negative error code on failure
 */
static int __init ws2812b_init(void)
{
    int ret;
    
    uint32_t val;
    
    gpio_dir = ioremap(GPIO_DIR, 0x1000);
    if (!gpio_dir) {
        pr_err("Failed to map GPIO direction registers\n");
        return -ENOMEM;
    }
    
    gpio_base = ioremap(GPIO_BASE, 0x1000);
    if (!gpio_base) {
        pr_err("Failed to map GPIO registers\n");
        iounmap(gpio_dir);
        return -ENOMEM;
    }

    
    val = readl(gpio_dir); // GPIO DIR
    val |= (1 << PWR_GPIO_LINE); //output
    writel(val, gpio_dir);
    val = readl(gpio_base); // GPIO BASE
    val &= ~(1 << PWR_GPIO_LINE); // low
    writel(val, gpio_base);
  
    val = readl(gpio_dir); // GPIO DIR
    val |= (1 << GPIO_LINE); //output
    writel(val, gpio_dir);
    val = readl(gpio_base); // GPIO BASE
    val &= ~(1 << GPIO_LINE); // low
    writel(val, gpio_base);
            
    framebuf = (uint8_t *)__get_free_pages(GFP_KERNEL, 0); // 1 page
    if (!framebuf) {
        iounmap(gpio_dir);
        iounmap(gpio_base);
        return -ENOMEM;
    }
    memset(framebuf, 0, PAGE_SIZE);
    
    ret = misc_register(&ws_dev);
    if (ret) {
        pr_err("Failed to register misc device\n");
        free_pages((unsigned long)framebuf, 0);
        iounmap(gpio_dir);
        iounmap(gpio_base);
        return ret;
    }

    power_on();
    
    ws_thread = kthread_run(ws_thread_fn, NULL, "ws2812b_thread");
    if (IS_ERR(ws_thread)) {
        misc_deregister(&ws_dev);
        free_pages((unsigned long)framebuf, 0);
        iounmap(gpio_dir);
        iounmap(gpio_base);
        return PTR_ERR(ws_thread);
    }

    pr_info("WS2812B driver loaded\n");
    return 0;
}


/**
 * ws2812b_exit - Module cleanup
 *
 * Stops kernel thread, unmaps registers, frees framebuffer,
 * deregisters misc device, and powers off LEDs.
 */
static void __exit ws2812b_exit(void)
{
    power_off();

    if (ws_thread)
        kthread_stop(ws_thread);

    misc_deregister(&ws_dev);
    free_pages((unsigned long)framebuf, 0);
    iounmap(gpio_dir);
    iounmap(gpio_base);
    pr_info("WS2812B driver unloaded\n");
}

module_init(ws2812b_init);
module_exit(ws2812b_exit);

MODULE_LICENSE("GPL v3");
MODULE_AUTHOR("KaliAssistant <work.kaliassistant.github@gmail.com>");
MODULE_DESCRIPTION("Kernel-space WS2812B driver with mmap for Lichee-Jack");
