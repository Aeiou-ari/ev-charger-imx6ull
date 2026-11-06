#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>

static struct i2c_client *lm75_client = NULL;
extern void msleep(unsigned int msecs);

static ssize_t lm75_read(struct file *fp, char __user *puser, size_t n, loff_t *off)
{
    struct i2c_msg sendmsg;
    struct i2c_msg recvmsg;
    int ret = 0;
    unsigned char tmpbuff[2] = {0};
    unsigned long nret = 0;

    memset(&sendmsg, 0, sizeof(sendmsg));
    memset(&recvmsg, 0, sizeof(recvmsg));

    sendmsg.addr = lm75_client->addr;
    tmpbuff[0] = 0x00;
    sendmsg.buf = tmpbuff;
    sendmsg.len = 1;
    ret = lm75_client->adapter->algo->master_xfer(lm75_client->adapter, &sendmsg, 1);
    if (ret != 1) {
        pr_info("master_xfer sendmsg failed ret = %d\n", ret);
        return -1;
    }

    recvmsg.addr = lm75_client->addr;
    recvmsg.flags |= I2C_M_RD;
    recvmsg.buf = tmpbuff;
    recvmsg.len = 2;
    ret = lm75_client->adapter->algo->master_xfer(lm75_client->adapter, &recvmsg, 1);
    if (ret != 1) {
        pr_info("master_xfer recvmsg failed ret = %d\n", ret);
        return -1;
    }

    nret = copy_to_user(puser, tmpbuff, sizeof(tmpbuff));
    if (nret) {
        pr_info("copy_to_user failed\n");
        return -1;
    }

    return sizeof(tmpbuff);
}

static struct file_operations fops = {
    .read = lm75_read,
};

static struct miscdevice lm75_misc = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "lm75_misc",
    .fops = &fops,
};

static int lm75_probe(struct i2c_client *pclient, const struct i2c_device_id *pdev)
{
    int ret = 0;
    
    lm75_client = pclient;
    ret = misc_register(&lm75_misc);
    if (ret) {
        pr_info("misc_register failed\n");
        return -1;
    }

    return 0;
}

static int lm75_remove(struct i2c_client *pclient)
{
    int ret = 0;
    
    ret = misc_deregister(&lm75_misc);
    if (ret) {
        pr_info("misc_deregister failed\n");
        return -1;
    }

    return 0;
}

static struct i2c_device_id lm75_id_tables[] = {
    {.name = "putelm75"},
    {},
};

static struct of_device_id lm75_of_match_tables[] = {
    {.compatible = "pute,putelm75"},
    {},
};

struct i2c_driver lm75_drv = {
    .probe = lm75_probe,
    .remove = lm75_remove,
    .driver = {
        .name = "lm75_drv",
        .owner = THIS_MODULE,
        .of_match_table = lm75_of_match_tables,
    },
    .id_table = lm75_id_tables,
};

static int __init lm75_drv_init(void)
{
    return i2c_add_driver(&lm75_drv);
}

static void __exit lm75_drv_exit(void)
{
    i2c_del_driver(&lm75_drv);

    return;
}

module_init(lm75_drv_init);
module_exit(lm75_drv_exit);

MODULE_LICENSE("GPL");
