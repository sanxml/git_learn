# 树莓派4 ros的安装和使用

- [树莓派4 ros的安装和使用](#树莓派4-ros的安装和使用)
  - [ros 安装和配置ros环境](#ros-安装和配置ros环境)
    - [安装 ros](#安装-ros)
    - [管理环境](#管理环境)
    - [创建 ros 工作空间](#创建-ros-工作空间)
  - [ros 文件系统](#ros-文件系统)
  - [创建和编译 ros 软件包](#创建和编译-ros-软件包)
    - [创建 ros 软件包](#创建-ros-软件包)
    - [编译 ros 软件包](#编译-ros-软件包)
    - [依赖关系](#依赖关系)
  - [报错及解决办法](#报错及解决办法)
    - [Error: the rosdep view is empty: call 'sudo rosdep init' and 'rosdep update' 学习 ROS 时,运行 rospack depends1 XXX 报错的解决办法](#error-the-rosdep-view-is-empty-call-sudo-rosdep-init-and-rosdep-update-学习-ros-时运行-rospack-depends1-xxx-报错的解决办法)

## ros 安装和配置ros环境

### 安装 ros

这里使用树莓派4, 安装前需要安装 ubuntu 系统, 使用 apt 安装 ros Noetic Ninjemys

```shell
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' #设置sources.list以安装来自packages.ros.org的软件
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 # 设置密钥
sudo apt update #更新软件包
sudo apt install ros-noetic-desktop-full # 完整桌面版安装
```

### 管理环境

```shell
source /opt/ros/noetic/setup.bash # 环境设置
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc #每次打开激活脚本
source ~/.bashrc
printenv | grep ROS # 检查环境是否安装完成
```

### 创建 ros 工作空间

```shell
# 创建和构建一个 catkin 工作空间
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
# 确定 ros_PACKAGE_PATH 环境变量包含你当前的工作空间目录
echo $ROS_PACKAGE_PATH
```

## ros 文件系统

程序代码散落在许多ROS包中。使用Linux内置命令行工具（如ls和cd）来进行查找和导航可能非常繁琐，因此ROS提供了专门的命令工具来简化这些操作。这里介绍三个指令,分别是 `rospack`, `roscd`, `rosls`,其中,都支持 tab 补全功能.

- 使用 `rospack`, 用法: ```rospack find [package_name]```

    将会返回软件包的所在路径, 例子:

    ```shell
    rospack find roscpp
    ```

- 使用 `roscd`, 用法: ```roscd [locationname[/subdir]]```

    允许你直接切换目录（cd）到某个软件包或者软件包集当中, 例子:

    ```shell
    roscd roscpp/cmake
    ```

- 使用 `rosls`, 用法: ```rosls [locationname[/subdir]]```

    允许你直接按软件包的名称执行 ls 命令（而不必输入绝对路径）, 例子:

    ```shell
    rosls roscpp_tutorials
    ```

## 创建和编译 ros 软件包

### 创建 ros 软件包

在 `创建 ros 工作空间` 这一章节中,我们已经介绍了创建一个空白 catkin 工作空间, 现在来介绍如何创建 catkin 软件包.

使用 `catkin_create_pkg` 命令创建一个名为 beginner_tutorials 的新软件包，这个软件包依赖于 std_msgs、roscpp 和 rospy

``` shell
cd ~/catkin_ws/src #进入 catkin 工作空间源文件目录
# catkin_create_pkg <package_name> [depend1] [depend2] [depend3]
catkin_create_pkg beginner_tutorials std_msgs rospy roscpp
```

### 编译 ros 软件包

catkin_make 是一个命令行工具，它简化了标准catkin工作流程。你可以认为catkin_make是在标准CMake工作流程中依次调用了cmake和make。

接着前面的,现在开始构建软件包

```shell
cd ~/catkin_ws #查看这些一级依赖包
ls src #可以看到现在目录下有名为beginner_tutorials的目录,使用catkin_make来构建它.
# catkin_make [make_targets] [-DCMAKE_VARIABLES=...]
catkin_make #在catkin工作区中构建软件包
source ~/catkin_ws/devel/setup.bash #激活配置文件
```

构建结束后,可以看到 `beginner_tutorials` 目录下新建几个文件

### 依赖关系

我们可以使用 `rospack` 命令工具查看软件包的依赖关系,依赖关系存储在 `package.xml` 文件

```shell
rospack depends1 beginner_tutorials #查看这些一级依赖包
rospack depends beginner_tutorials #递归检测出所有嵌套的依赖包
```

## 报错及解决办法

### Error: the rosdep view is empty: call 'sudo rosdep init' and 'rosdep update' 学习 ROS 时,运行 rospack depends1 XXX 报错的解决办法

学习 ros 官网的 wiki 时, 使用 `rospack depends1 XXX` 报错:

Error: the rosdep view is empty: call 'sudo rosdep init' and 'rosdep update'

我使用的是树莓派的 ubuntu 20.04 系统, 安装的 ROS 版本为 `ROS Noetic Ninjemys`, 以下为报错的解决办法.

```shell
sudo apt install python3-rosdep2 # 安装rosdep
sudo rm -rf $HOME/.ros/rosdep && sudo rm -rf /etc/.ros/rosdep # 删除 rosdep 原有文件
```

```shell
sudo vim /etc/hosts # 修改 `hosts` 文件
```

修改 `/etc/hosts` 文件,在末尾添加:

```shell
151.101.84.133  raw.githubusercontent.com
```

接着按照提示运行,注意要先运行 `sudo rosdep init` ,后再运行 `rosdep update`

```shell
sudo rosdep init
rosdep update
```

以上指令没有错误再运行,可能出现的错误下面会接着介绍

``` shell
sudo apt install ros-noetic-desktop-full # 重新安装 ROS
```

**注意**: 运行 `sudo rosdep init` 可能会出现以下提示,只需要删除`/etc/ros/rosdep/sources.list.d/20-default.list`文件即可

```shell
ERROR: default sources list file already exists:
        /etc/ros/rosdep/sources.list.d/20-default.list
Please delete if you wish to re-initialize
```

删除文件后重新运行 `sudo rosdep init`, 正确输出应该如下所示

```shell
Wrote /etc/ros/rosdep/sources.list.d/20-default.list
Recommended: please run

        rosdep update
```

以下为运行 `rosdep update` 的正确输出

```shell
reading in sources list data from /etc/ros/rosdep/sources.list.d
Hit https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/osx-homebrew.yaml
Hit https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/base.yaml
Hit https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/python.yaml
Hit https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/ruby.yaml
Hit https://raw.githubusercontent.com/ros/rosdistro/master/releases/fuerte.yaml
Query rosdistro index https://raw.githubusercontent.com/ros/rosdistro/master/index-v4.yaml
Skip end-of-life distro "ardent"
Skip end-of-life distro "bouncy"
Skip end-of-life distro "crystal"
Add distro "dashing"
Skip end-of-life distro "eloquent"
Add distro "foxy"
Add distro "galactic"
Skip end-of-life distro "groovy"
Skip end-of-life distro "hydro"
Skip end-of-life distro "indigo"
Skip end-of-life distro "jade"
Add distro "kinetic"
Skip end-of-life distro "lunar"
Add distro "melodic"
Add distro "noetic"
Add distro "rolling"
updated cache in /home/ubuntu/.ros/rosdep/sources.cache
```
