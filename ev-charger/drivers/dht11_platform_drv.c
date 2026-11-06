#include <linux/fs.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/of_gpio.h>
#include <asm/delay.h>
#include <asm/uaccess.h>
#include <linux/spinlock.h>

extern void msleep(unsigned int msecs);

static int dht11gpiono = 0;
static spinlock_t lock;

void master_send_dht11_reset(void)
{
    gpio_set_value(dht11gpiono, 0);
    msleep(20);
    gpio_set_value(dht11gpiono, 1);

    return;
}

static int master_recv_dht11_respone(void)
{
    int timeout = 0;

    gpio_direction_input(dht11gpiono);

    while (timeout < 20 && gpio_get_value(dht11gpiono))
    {
        udelay(10);
        timeout++;
    }

    if (timeout >= 20)
    {
        return -1;
    }

    while (!gpio_get_value(dht11gpiono));
    while (gpio_get_value(dht11gpiono));

    return 0;
}

static void master_recv_dht11_data(unsigned char *pdata, int len)
{
    int i = 0;
    int n = 0;
    int cnt = 0;

    for (i = 0; i < len; i++) {
        for (n = 7; n >= 0; n--) {
            cnt = 0;
            while (!gpio_get_value(dht11gpiono));
            while (gpio_get_value(dht11gpiono)) {
                udelay(10);
                cnt++;
            }
            if (cnt > 5) {
               pdata[i] |= (0x1 << n);
            }
        }
    }

    gpio_direction_output(dht11gpiono, 1);

    return;
}

static int master_check_dht11_data(unsigned char *pdata, int len)
{
    if (pdata[4] == pdata[0] + pdata[1] + pdata[2] + pdata[3])
        return 0;
    else 
        return -2;
}

static ssize_t dht11_read(struct file *fp, char __user *puser, size_t n, loff_t *off)
{
    int ret = 0;
    unsigned char data[5] = {0};
    unsigned long nret = 0;
    unsigned long flags = 0;

    master_send_dht11_reset();
    
    spin_lock_irqsave(&lock, flags);
    ret = master_recv_dht11_respone();
    if (-1 == ret) {
        spin_unlock_irqrestore(&lock, flags);   /* 失败也要解锁, 否则锁泄漏触发 scheduling while atomic */
        return ret;
    }
    master_recv_dht11_data(data, 5);
    spin_unlock_irqrestore(&lock, flags);

    ret = master_check_dht11_data(data, 5);
    if (-2 == ret)
        return ret;
   
    nret = copy_to_user(puser, data, sizeof(unsigned char) * 4);
    if (nret) {
        pr_info("copy_to_user failed\n");
        return -3;
    }

    return (sizeof(unsigned char) * 4);
}

static struct file_operations fops = {
    .read = dht11_read,
};

static struct miscdevice dht11_misc = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "dht11_misc",
    .fops = &fops,
};

static int dht11_probe(struct platform_device *pdevice)
{
    int ret = 0;

    dht11gpiono = of_get_named_gpio(pdevice->dev.of_node, "gpio-dht11", 0);
    if (dht11gpiono < 0) {
        pr_info("of_get_named_gpio failed\n");
        return -1;
    }
  
    spin_lock_init(&lock);

    ret = misc_register(&dht11_misc);
    if (ret) {
        pr_info("misc_register failed\n");
        return -1;
    }

    ret = devm_gpio_request(dht11_misc.this_device, dht11gpiono, "dht11-platform-drv");
    if (ret) {
        pr_info("devm_gpio_request failed\n");
        return -1;
    }

    gpio_direction_output(dht11gpiono, 1);

    return 0;
}

static int dht11_remove(struct platform_device *pdevice)
{
    int ret = 0;
    
    ret = misc_deregister(&dht11_misc);
    if (ret) {
        pr_info("misc_deregister failed\n");
        return -1;
    }

    return 0;
}

static struct platform_device_id dht11_id_tables[] = {
    {.name = "putedht11"},
    {},
};

static struct of_device_id dht11_of_match_tables[] = {
    {.compatible = "pute,putedht11"},
    {},
};

struct platform_driver dht11_drv = {
    .probe = dht11_probe,
    .remove = dht11_remove,
    .driver = {
        .name = "putedht11",
        .of_match_table = dht11_of_match_tables,
    },
    .id_table = dht11_id_tables,
};

module_platform_driver(dht11_drv);

MODULE_LICENSE("GPL");