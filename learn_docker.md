# 学习使用docker

## **什么是docker？**

Docker 是一个开源的应用容器引擎，基于 Go 语言 并遵从 Apache2.0 协议开源。

Docker 可以让开发者打包他们的应用以及依赖包到一个轻量级、可移植的容器中，然后发布到任何流行的 Linux 机器上，也可以实现虚拟化。

容器是完全使用沙箱机制，相互之间不会有任何接口（类似 iPhone 的 app）,更重要的是容器性能开销极低。

`简单的说，使用docker，可以跳过环境搭建等等，可以快速上手licheepi-nano`

## **docker的基本指令**

- 镜像操作

```shell
docker push [OPTIONS] NAME[:TAG]  # 获取镜像
docker images             # 列出镜像
docker search [OPTIONS] TERM    # 查找镜像
docker rmi [OPTIONS] IMAGE [IMAGE...] #删除镜像
```

- 容器操作

```shell
docker run [OPTIONS] IMAGE[:TAG|@DIGEST] [COMMAND] [ARG...] # 通过镜像启动一个新的容器
docker ps -a # 查看所有容器
docker docker start [OPTIONS] CONTAINER [CONTAINER...] # 启动停止的一个容器
docker stop [OPTIONS] CONTAINER [CONTAINER...] # 停止容器
docker rm [OPTIONS] CONTAINER [CONTAINER...] # 删除容器
```

> 访问主机USB或串行设备的方法:
> docker run -t -i --device=/dev/ttyUSB0 ubuntu bash
> docker run -t -i --privileged -v /dev/bus/usb:/dev/bus/usb ubuntu bash

- 仓库操作

```shell
docker login [OPTIONS] [SERVER] # 登录docker hub
docker logout [OPTIONS] [SERVER] # 退出docker hub
docker search [OPTIONS] TERM # 查找镜像
docker pull [OPTIONS] NAME[:TAG|@DIGEST] # 拉取镜像
docker commit [OPTIONS] CONTAINER [REPOSITORY[:TAG]] # 打包容器为镜像
docker push [OPTIONS] NAME[:TAG] # 推送镜像
```
