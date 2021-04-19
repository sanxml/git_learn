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
#include <linux/of.h>
#include <linux/of_address.h>
#include <asm/mach/map.h>
#include <asm/uaccess.h>
#include <asm/io.h>


#define LED_CNT  1           /* 设备号个数 */
#define LED_NAME   "dtsled"      /* 设备名 */
#define LED_PIN   32*0+1         /* 引脚是PA1 */

/* dtsled_dev 设备结构体 */
struct dtsled_dev{
  dev_t devid;      /* 设备号 */
  struct cdev cdev;   /* cdev */
  struct class *class;  /* 类 */
  struct device *device; /* 设备 */
  int majoy;       /* 主设备号 */
  int minor;       /* 次设备号 */
  struct device_node *nd; /* 设备节点 */
};

struct dtsled_dev dtsled;  /* led设备 */

/*
* @description : 打开设备
* @param – inode : 传递给驱动的 inode
* @param - filp : 设备文件，file 结构体有个叫做 private_data 的成员变量
* 一般在 open 的时候将 private_data 指向设备结构体。
* @return : 0 成功;其他 失败
*/
static int led_open(struct inode *inode, struct file *filp)
{
  filp->private_data = &dtsled; /* 设置私有数据 */
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
static struct file_operations dtsled_fops = {
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
  /* 获取设备树中的属性数据 */

  int ret;
  u32 regdata[14];
  const char *str;
  struct property *proper;
  

  /* 1、获取设备节点：nanoled */
  dtsled.nd = of_find_node_by_path("/nanoled");
  if(dtsled.nd == NULL) {
    printk("nanoled node can not found!\r\n");
    return -EINVAL;
  } 
  else {
    printk("nanoled node has been found!\r\n");
  }

  /* 2、获取 compatible 属性内容 */
  proper = of_find_property(dtsled.nd, "compatible", NULL);
  if(proper == NULL) {
    printk("compatible property find failed\r\n");
  } 
  else {
    printk("compatible = %s\r\n", (char*)proper->value);
  }

  /* 3、获取 status 属性内容 */
  ret = of_property_read_string(dtsled.nd, "status", &str);
  if(ret < 0){
  printk("status read failed!\r\n");
  } else {
  printk("status = %s\r\n",str);
  }

  /* 4、获取 reg 属性内容 */
  ret = of_property_read_u32_array(dtsled.nd, "reg", regdata, 10);
  if(ret < 0) {
    printk("reg property read failed!\r\n");
  } 
  else {
    u8 i = 0;
    printk("reg data:\r\n");
    for(i = 0; i < 10; i++)
      printk("%#X ", regdata[i]);
    printk("\r\n");
  }


  int retvalue = 0;
  retvalue = gpio_request(LED_PIN, "led"); /* 申请gpio端口 */
  if(retvalue < 0)
  {
    printk("get led failed\r\n");
  }

  /* 注册字符设备驱动 */
  /* 1.创建设备号 */
  if(dtsled.majoy)  /* 定义了设备号 */
  {
    dtsled.devid = MKDEV(dtsled.majoy, 0);
    register_chrdev_region(dtsled.devid, LED_CNT, LED_NAME);  /* 注册设备号 */
  }
  else          /* 没有定义设备号 */
  {
    alloc_chrdev_region(&dtsled.devid, 0, LED_CNT, LED_NAME); /* 申请设备号 */
    dtsled.majoy = MAJOR(dtsled.devid);           /* 获取主设备号 */
    dtsled.minor = MINOR(dtsled.devid);           /* 获取次设备号 */
  }
  printk("dtsled majoy=%d,minor=%d\n", dtsled.majoy, dtsled.minor);

  /* 2.初始化 cdev */
  dtsled.cdev.owner = THIS_MODULE;
  cdev_init(&dtsled.cdev, &dtsled_fops);

  /* 3.添加一个 cdev */
  cdev_add(&dtsled.cdev, dtsled.devid, LED_CNT);

  /* 4.创建类 */
  dtsled.class = class_create(THIS_MODULE, LED_NAME);
  if(IS_ERR(dtsled.class))
  {
    return PTR_ERR(dtsled.class);
  }

  /* 5.创建设备 */
  dtsled.device = device_create(dtsled.class, NULL, dtsled.devid, NULL, LED_NAME);
  if(IS_ERR(dtsled.device))
  {
    return PTR_ERR(dtsled.device);
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
  cdev_del(&dtsled.cdev);

  unregister_chrdev_region(dtsled.devid, LED_CNT);

  device_destroy(dtsled.class, dtsled.devid);
  class_destroy(dtsled.class);

  printk("led_exit()\r\n");
}

/* 将上面两个函数指定为驱动的入口和出口函数 */

module_init(led_init);
module_exit(led_exit);

/* LICENSE 和作者信息 */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("sanxml");
