# 学习使用gitbook

## 1. 初始化gitbook

``` shell
gitbook init
```

多出两个文件，README.md为介绍文件，SUMMARY.md为章节目录文件

![初始化gitbook](./assets/gitbook_init.png)

### 2. 编辑SUMMARY.md

![编辑SUMMARY.md](./assets/SUMMARY.png)

编辑后还需要重新`gitbook init`

### 3. 启动服务，预览

``` shell
gitbook serve
```

![gitbook serve](./assets/gitbook_serve.png)

> 打开 [http://localhost:4000](http://localhost:4000) 预览

![gitbook serve2](./assets/gitbook_serve2.png)

### 4. 生成静态网页

``` shell
gitbook build
```
