# **新字符设备驱动开发**

## **前言**

字符设备驱动开发重点是使用 register_chrdev 函数注册字符设备，当不再使用设备的时候就使用unregister_chrdev 函数注销字符设备，驱动模块加载成功以后还需要手动使用 mknod 命令创建设备节点。
register_chrdev 和 unregister_chrdev 这两个函数是老版本驱动使用的函数，现在新的字符设备驱动已经不再使用这两个函数，而是使用Linux内核推荐的新字符设备驱动API函数。

## **自动分配设备号**

使用 register_chrdev 函数注册字符设备的时候只需要给定一个主设备号即可，但是这样会带来两个问题：

1. 需要我们事先确定好哪些主设备号没有使用。

2. 会将一个主设备号下的所有次设备号都使用掉，比如现在设置 LED 这个主设备号为200，那么 0~1048575(2^20-1)这个区间的次设备号就全部都被 LED 一个设备分走了。这样太浪费次设备号了！一个 LED 设备肯定只能有一个主设备号，一个次设备号。

解决这两个问题最好的方法就是要使用设备号的时候向 Linux 内核申请，需要几个就申请几个，由 Linux 内核分配设备可以使用的设备号。

- **分配设备号**

  - 如果没有指定设备号的话使用以下函数来申请设备号：

  ``` c
  int alloc_chrdev_region(dev_t *dev, unsigned baseminor, unsigned count, const char *name)
  ```

  - 如果给定了设备的主设备号和次设备号就使用以下函数来注册设备号：

  ``` c
  int register_chrdev_region(dev_t from, unsigned count, const char *name)
  ```

  参数 from 是要申请的起始设备号，也就是给定的设备号；参数 count 是要申请的数量，一般都是一个；参数 name 是设备名字。

- **释放设备号**

注销字符设备之后要释放掉设备号， 不管是通过 alloc_chrdev_region 函数还是 register_chrdev_region 函数申请的设备号，统一使用如下释放函数：

``` c
void unregister_chrdev_region(dev_t from, unsigned count)
```

## **新的字符设备注册方式**

- **字符设备结构**

在 Linux 中使用 cdev 结构体表示一个字符设备，在 cdev 中有两个重要的成员变量：ops 和 dev，这两个就是字符设备文件操作函数集合file_operations 以及设备号 dev_t。

``` c
struct cdev {
  struct kobject kobj;
  struct module *owner;
  const struct file_operations *ops;
  struct list_head list;
  dev_t dev;
  unsigned int count;
};
```

- **cdev_init 函数**

定义好 cdev 变量以后就要使用 cdev_init 函数对其进行初始化，cdev_init 函数原型如下：

``` c
void cdev_init(struct cdev *cdev, const struct file_operations *fops)
```

参数 cdev 就是要初始化的 cdev 结构体变量，参数 fops 就是字符设备文件操作函数集合。

- **cdev_add 函数**

cdev_add 函数用于向 Linux 系统添加字符设备(cdev 结构体变量)，cdev_add 函数原型如下：

``` c
int cdev_add(struct cdev *p, dev_t dev, unsigned count)
```

参数 p 指向要添加的字符设备(cdev 结构体变量)，参数 dev 就是设备所使用的设备号，参数 count 是要添加的设备数量。

- **cdev_del 函数**

卸载驱动的时候一定要使用 cdev_del 函数从 Linux 内核中删除相应的字符设备，cdev_del函数原型如下：

``` c
int cdev_add(struct cdev *p, dev_t dev, unsigned count)
```

参数 p 就是要删除的字符设备。

## **自动创建设备节点**

- **udev机制**

udev 是一个用户程序，在 Linux 下通过 udev 来实现设备文件的创建与删除，udev 可以检测系统中硬件设备状态，可以根据系统中硬件设备状态来创建或者删除设备文件。

- **创建和删除类**

自动创建设备节点的工作是在驱动程序的入口函数中完成的，一般在 cdev_add 函数后面添加自动创建设备节点相关代码。

创建一个 class 类，class 是个结构体，定义在文件 include/linux/device.h 里面，函数原型如下：

``` c
struct class *class_create (struct module *owner, const char *name)
```

卸载驱动程序的时候需要删除掉类，类删除函数为 class_destroy，函数原型如下：

``` c
void class_destroy(struct class *class);
```

- **创建设备**

创建好类以后还不能实现自动创建设备节点，我们还需要在这个类下创建一个设备。使用 device_create 函数在类下面创建设备，device_create 函数原型如下：

``` c
struct device *device_create(struct class *class,
        struct device *parent,
        dev_t devt,
        void *drvdata,
        const char *fmt, ...)
```

device_create 是个可变参数函数，参数 class 就是设备要创建哪个类下面；参数 parent 是父设备，一般为 NULL，也就是没有父设备；参数 devt 是设备号；参数 drvdata 是设备可能会使用的一些数据，一般为 NULL；参数 fmt 是设备名字，如果设置 fmt=xxx 的话，就会生成/dev/xxx这个设备文件。返回值就是创建好的设备。

同样的，卸载驱动的时候需要删除掉创建的设备，设备删除函数为 device_destroy，函数原型如下：

``` c
void device_destroy(struct class *class, dev_t devt)
```

参数 classs 是要删除的设备所处的类，参数 devt 是要删除的设备号。

## **设置文件私有数据**

每个硬件设备都有一些属性，比如主设备号(dev_t)，类(class)、设备(device)、开关状态(state)等等。

对于一个设备的所有属性信息,将其做成一个结构体。编写驱动 open 函数的时候将设备结构体作为私有数据添加到设备文件中。

``` c
/* 设备结构体 */
struct test_dev{
 dev_t devid; /* 设备号 */
 struct cdev cdev; /* cdev */
 struct class *class; /* 类 */
 struct device *device; /* 设备 */
 int major; /* 主设备号 */
 int minor; /* 次设备号 */
};
struct test_dev testdev;

/* open 函数 */
static int test_open(struct inode *inode, struct file *filp)
{
 filp->private_data = &testdev; /* 设置私有数据 */
 return 0;
}
```

## 示例程序

详细示例程序可以查看[代码](character_device_drivers_demo/led_newchrdev.c)
