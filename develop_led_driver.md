# LED驱动开发

## 前言

Linux内核中gpio是最简单，最常用的资源(和 interrupt , dma , timer 一样)驱动程序，应用程序都能够通过相应的接口使用gpio，gpio使用0～MAX_INT之间的整数标识，不能使用负数,gpio与硬件体系密切相关的,不过linux有一个框架处理 gpio。

## Linux下GPIO的使用

下面来介绍Linux下的GPIO的使用，主要有两种，一种是通过sysfs方式，另一种是在内核下使用

- 通过sysfs方式控制GPIO
