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
  - [ros 节点](#ros-节点)
    - [图概念速览](#图概念速览)
    - [节点](#节点)
    - [客户端库](#客户端库)
    - [roscore](#roscore)
    - [rosnode](#rosnode)
    - [rosrun](#rosrun)
  - [ros 话题](#ros-话题)
    - [rqt_graph](#rqt_graph)
    - [rostopic](#rostopic)
    - [rqt_plot](#rqt_plot)
  - [报错及解决办法](#报错及解决办法)
    - [Error: the rosdep view is empty: call 'sudo rosdep init' and 'rosdep update' 学习 ROS 时,运行 rospack depends1 XXX 报错的解决办法](#error-the-rosdep-view-is-empty-call-sudo-rosdep-init-and-rosdep-update-学习-ros-时运行-rospack-depends1-xxx-报错的解决办法)
    - [RuntimeError: No usable plot type found. Install at least one of: PyQtGraph, MatPlotLib (at least 1.4.0) or Python-Qwt5. 运行 rosrun rqt_plot rqt_plot 报错的解决办法](#runtimeerror-no-usable-plot-type-found-install-at-least-one-of-pyqtgraph-matplotlib-at-least-140-or-python-qwt5-运行-rosrun-rqt_plot-rqt_plot-报错的解决办法)

## ros 安装和配置ros环境

### 安装 ros

这里使用树莓派4, 安装前需要安装 ubuntu 系统, 使用 apt 安装 ros Noetic Ninjemys

```shell
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' # 设置sources.list以安装来自packages.ros.org的软件
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 # 设置密钥
sudo apt update # 更新软件包
sudo apt install ros-noetic-desktop-full # 完整桌面版安装
```

### 管理环境

```shell
source /opt/ros/noetic/setup.bash # 环境设置
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc # 每次打开激活脚本
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
cd ~/catkin_ws/src # 进入 catkin 工作空间源文件目录
# catkin_create_pkg <package_name> [depend1] [depend2] [depend3]
catkin_create_pkg beginner_tutorials std_msgs rospy roscpp
```

### 编译 ros 软件包

catkin_make 是一个命令行工具，它简化了标准catkin工作流程。你可以认为catkin_make是在标准CMake工作流程中依次调用了cmake和make。

接着前面的,现在开始构建软件包

```shell
cd ~/catkin_ws # 查看这些一级依赖包
ls src # 可以看到现在目录下有名为beginner_tutorials的目录,使用catkin_make来构建它.
# catkin_make [make_targets] [-DCMAKE_VARIABLES=...]
catkin_make # 在catkin工作区中构建软件包
source ~/catkin_ws/devel/setup.bash # 激活配置文件
```

构建结束后,可以看到 `beginner_tutorials` 目录下新建几个文件

### 依赖关系

我们可以使用 `rospack` 命令工具查看软件包的依赖关系,依赖关系存储在 `package.xml` 文件

```shell
rospack depends1 beginner_tutorials # 查看这些一级依赖包
rospack depends beginner_tutorials # 递归检测出所有嵌套的依赖包
```

## ros 节点

### 图概念速览

计算图（Computation Graph）是一个由 ros 进程组成的点对点网络，它们能够共同处理数据。ros 的基本计算图概念有节点（Nodes）、主节点（Master）、参数服务器（Parameter Server）、消息（Messages）、服务（Services）、话题（Topics）和袋（Bags），它们都以不同的方式向图（Graph）提供数据。

| 概念             | 说明                                                        |
| :--------------- | :---------------------------------------------------------- |
| 节点（Nodes）    | 节点是一个可执行文件，它可以通过 ros 来与其他节点进行通信。 |
| 消息（Messages） | 订阅或发布话题时所使用的 ros 数据类型。                     |
| 话题（Topics）   | 节点可以将消息发布到话题，或通过订阅话题来接收消息。        |
| 主节点（Master） | ros 的命名服务，例如帮助节点发现彼此。                      |
| rosout           | 在 ros 中相当于stdout/stderr（标准输出/标准错误）。         |
| roscore          | 主节点 + rosout + 参数服务器                                |

### 节点

节点实际上只不过是ROS软件包中的一个可执行文件。ROS节点使用ROS客户端库与其他节点通信。节点可以发布或订阅话题，也可以提供或使用服务。

### 客户端库

ROS客户端库可以让用不同编程语言编写的节点进行相互通信：

- rospy = python 客户端库
- roscpp = c++ 客户端库
- rosjs = javascripts客户端库
- rosjava = java客户端库

### roscore

roscore是你在运行所有ROS程序前首先要运行的命令。

```shell
roscore
```

> 如果roscore运行后没有初始化，很有可能是网络配置的问题

### rosnode

rosnode 显示当前正在运行的ROS节点信息。

```shell
rosnode list # 列出运行的节点
rosnode info /rosout # 返回的是某个指定节点的信息
rosnode ping /rosout # 测试节点是否正常
```

### rosrun

rosrun 可以让你用包名直接运行软件包内的节点（而不需要知道包的路径）。

```shell
# rosrun [package_name] [node_name]
rosrun turtlesim turtlesim_node
rosrun turtlesim turtlesim_node __name:=my_turtle # 重新命名节点
```

![turtlesim_node](./assets/raspberry_ros/turtlesim.png)

## ros 话题

接着前一节,先做一些准备工作

```shell
# 新建终端,执行
rosrun turtlesim turtlesim_node
# 新建终端,执行
rosrun turtlesim turtle_teleop_key # 通过键盘遥控turtle
```

turtlesim_node节点和turtle_teleop_key节点之间是通过一个ROS话题来相互通信的。turtle_teleop_key在话题上发布键盘按下的消息，turtlesim则订阅该话题以接收消息。

![turtle_teleop_key](./assets/raspberry_ros/turtlesim_and_key.png)

### rqt_graph

rqt_graph用动态的图显示了系统中正在发生的事情。

```shell
# 新建终端,执行
rosrun rqt_graph rqt_graph
```

![rqt_graph](assets/raspberry_ros/rqt_graph.png)

### rostopic

rostopic命令工具能让你获取ROS话题的信息

```shell
rostopic echo /turtle1/cmd_vel # 显示在某个话题上发布的数据。
```

通过按下键盘方向键让 turtle_teleop_key 节点发布数据,控制乌龟的运动,可以在 rostopic 的终端窗口捕获到运动信息

![rostopic_echo](assets/raspberry_ros/rostopic_echo.png)

现在让我们再看一下rqt_graph

![rostopic_echo_graph](assets/raspberry_ros/rostopic_echo_graph.png)

rostopic list 能够列出当前已被订阅和发布的所有话题。

```shell
rostopic list -v # 列出所有发布和订阅的主题及其类型的详细信息。
```

rostopic type 命令用来查看所发布话题的消息类型。

```shell
rostopic type /turtle1/cmd_vel
rosmsg show geometry_msgs/Twist # rosmsg查看消息的详细信息
```

rostopic pub 可以把数据发布到当前某个正在广播的话题上。

```shell
# 用法: rostopic pub [topic] [msg_type] [args]
# 发送一条消息给turtlesim，告诉它以2.0大小的线速度和1.8大小的角速度移动。
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'
```

参数分析: `-1` 让rostopic只发布一条消息，然后退出. 两个破折号 `--` 用来告诉选项解析器，表明之后的参数都不是选项

![rostopic_pub](assets/raspberry_ros/rostopic_pub.png)

```shell
rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, -1.8]'
```

rostopic pub -r 命令来发布源源不断的命令, 这里的 `1` 代表以 1 Hz 的速度发布

![rostopic_pub1](assets/raspberry_ros/rostopic_pub1.png)

我们再看一下rqt_graph,可以看出键盘和 `rostopic pub` 都在发送指令给小乌龟和 `rostopic echo` .

![rostopic_pub_graph](assets/raspberry_ros/rostopic_pub_graph.png)

rostopic hz 报告数据发布的速率。

```shell
# 用法: rostopic hz [topic]
rostopic hz /turtle1/pose
```

### rqt_plot

rqt_plot 命令可以在滚动时间图上显示发布到某个话题上的数据, 新建终端输入:

```shell
rosrun rqt_plot rqt_plot
```

在弹出的窗口文本框添加 `/turtle1/pose/x`, 点击 `+` 后, 再添加 `/turtle1/pose/y`, 现在你会在图中看到小乌龟的 x 和 y 位置.

![rqt_plot](assets/raspberry_ros/rqt_plot.png)

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

### RuntimeError: No usable plot type found. Install at least one of: PyQtGraph, MatPlotLib (at least 1.4.0) or Python-Qwt5. 运行 rosrun rqt_plot rqt_plot 报错的解决办法

```shell
pip install PyQtGraph
```
