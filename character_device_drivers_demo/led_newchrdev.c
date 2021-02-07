#include <linux/gpio.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ide.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <asm/mach/map.h>
#include <asm/uaccess.h>
#include <asm/io.h>

#define LED_CNT  1           /* 设备号个数 */
#define LED_NAME   "led_chrdev"      /* 设备名 */
#define LED_PIN   32*0+1         /* 引脚是PA1 */

/* led_new_chrdev 设备结构体 */
struct led_new_chrdev{
  dev_t devid;      /* 设备号 */
  struct cdev cdev;   /* cdev */
  struct class *class;  /* 类 */
  struct device *device; /* 设备 */
  int majoy;       /* 主设备号 */
  int minor;       /* 次设备号 */
};

struct led_new_chrdev led_chrdev;  /* led设备 */

/*
* @description : 打开设备
* @param – inode : 传递给驱动的 inode
* @param - filp : 设备文件，file 结构体有个叫做 private_data 的成员变量
* 一般在 open 的时候将 private_data 指向设备结构体。
* @return : 0 成功;其他 失败
*/
static int led_open(struct inode *inode, struct file *filp)
{
  filp->private_data = &led_chrdev; /* 设置私有数据 */
  return 0;
}

/*
* @description : 从设备读取数据
* @param - filp : 要打开的设备文件(文件描述符)
* @param - buf : 返回给用户空间的数据缓冲区
* @param - cnt : 要读取的数据长度
* @param - offt : 相对于文件首地址的偏移
* @return : 读取的字节数，如果为负值，表示读取失败
*/
static ssize_t led_read(struct file *filp, char __user *buf, size_t cnt, loff_t *offt)
{
  return 0;
}

/*
* @description : 向设备写数据
* @param - filp : 设备文件，表示打开的文件描述符
* @param - buf : 要写给设备写入的数据
* @param - cnt : 要写入的数据长度
* @param - offt : 相对于文件首地址的偏移
* @return : 写入的字节数，如果为负值，表示写入失败
*/
static ssize_t led_write(struct file *filp, const char __user *buf, size_t cnt, loff_t *offt)
{
  int retvalue = 0;
  unsigned char writebuf[1];

  /* 接收用户空间传递给内核的数据并且打印出来 */
  retvalue = copy_from_user(writebuf, buf, cnt);
  if(retvalue < 0)
  {
    printk("kernel write failed !\r\n");
    return -EFAULT;
  }

  if(writebuf[0])
  {
    gpio_direction_output(LED_PIN, 1); /* 设置输出值 */
    printk("Open led!\r\n");
  }
  else
  {
    gpio_direction_output(LED_PIN, 0);
    printk("Close led!\r\n");
  }
  return 0;
}

/*
* @description : 关闭/释放设备
* @param - filp : 要关闭的设备文件(文件描述符)
* @return : 0 成功;其他 失败
*/
static int led_release(struct inode *inode, struct file *filp)
{
  return 0;
}

/* 设备操作函数结构体 */
static struct file_operations led_chrdev_fops = {
  .owner = THIS_MODULE,
  .open = led_open,
  .read = led_read,
  .write = led_write,
  .release = led_release,
};

/*
* @description : 驱动入口函数
* @param : 无
* @return : 0 成功;其他 失败
*/
static int __init led_init(void)
{
  int retvalue = 0;
  retvalue = gpio_request(LED_PIN, "led"); /* 申请gpio端口 */
  if(retvalue < 0)
  {
    printk("get led failed\r\n");
  }

  /* 注册字符设备驱动 */
  /* 1.创建设备号 */
  if(led_chrdev.majoy)  /* 定义了设备号 */
  {
    led_chrdev.devid = MKDEV(led_chrdev.majoy, 0);
    register_chrdev_region(led_chrdev.devid, LED_CNT, LED_NAME);  /* 注册设备号 */
  }
  else          /* 没有定义设备号 */
  {
    alloc_chrdev_region(&led_chrdev.devid, 0, LED_CNT, LED_NAME); /* 申请设备号 */
    led_chrdev.majoy = MAJOR(led_chrdev.devid);           /* 获取主设备号 */
    led_chrdev.minor = MINOR(led_chrdev.devid);           /* 获取次设备号 */
  }
  printk("led_chrdev majoy=%d,minor=%d\n", led_chrdev.majoy, led_chrdev.minor);

  /* 2.初始化 cdev */
  led_chrdev.cdev.owner = THIS_MODULE;
  cdev_init(&led_chrdev.cdev, &led_chrdev_fops);

  /* 3.添加一个 cdev */
  cdev_add(&led_chrdev.cdev, led_chrdev.devid, LED_CNT);

  /* 4.创建类 */
  led_chrdev.class = class_create(THIS_MODULE, LED_NAME);
  if(IS_ERR(led_chrdev.class))
  {
    return PTR_ERR(led_chrdev.class);
  }

  /* 5.创建设备 */
  led_chrdev.device = device_create(led_chrdev.class, NULL, led_chrdev.devid, NULL, LED_NAME);
  if(IS_ERR(led_chrdev.device))
  {
    return PTR_ERR(led_chrdev.device);
  }

  printk("led_init()\r\n");
  return 0;
}

/*
* @description : 驱动出口函数
* @param : 无
* @return : 无
*/
static void __exit led_exit(void)
{
  gpio_free(LED_PIN);

  /* 注销字符设备驱动 */
  cdev_del(&led_chrdev.cdev);

  unregister_chrdev_region(led_chrdev.devid, LED_CNT);

  device_destroy(led_chrdev.class, led_chrdev.devid);
  class_destroy(led_chrdev.class);

  printk("led_exit()\r\n");
}

/* 将上面两个函数指定为驱动的入口和出口函数 */

module_init(led_init);
module_exit(led_exit);

/* LICENSE 和作者信息 */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("sanxml");
