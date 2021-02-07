#include "stdio.h"
#include "unistd.h"
#include "sys/types.h"
#include "sys/stat.h"
#include "fcntl.h"
#include "stdlib.h"
#include "string.h"

/*
* @description : main 主程序
* @param - argc : argv 数组元素个数
* @param - argv : 具体参数
* @return : 0 成功;其他 失败
*/
int main(int argc, char *argv[])
{
  int fd, retvalue;
  char * filename;
  unsigned char databuf[1];

  if(argc != 3)
  {
    printf("Error Usage !\r\n");
    return -1;
  }

  filename = argv[1];

  /* 打开led驱动 */
  fd = open(filename, O_RDWR);
  if(fd < 0)
  {
    printf("file %s open failed!\r\n", filename);
    return -1;
  }

  databuf[0] = atoi(argv[2]);

  /* 向/dev/led_chrdev 文件写入数据 */
  retvalue = write(fd, databuf, sizeof(databuf));
  if(retvalue < 0){
    printf("write file %s failed!\r\n", filename);
    close(fd);
    return -1;
  }

  /* 关闭设备 */
  retvalue = close(fd);
  if(retvalue < 0)
  {
    printf("Can't close file %s!\r\n", filename);
    return -1;
  }
  return 0;
}