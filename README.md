# 空地无人集群虚实结合实验平台

## 仿真环境配置

### 前言

对于新装的ubuntu系统基本首次安装成功率在90%以上，已经使用一段时间的系统可能存在依赖项或驱动的异常，解决冲突反而容易导致失败

### 准备工作

1. 安装操作系统：Ubuntu 18.04
   - 如果系统层面还存在网卡驱动、显卡驱动的问题，建议先解决。
   - 内核非必要原因，不建议升级

2. 安装ROS Melodic，安装教程：[https://www.ros.org/](https://www.ros.org/)
   - 请使用官方教程进行安装
   - 如果安装过程中遇到一些网络错误，如rosdep init失败，请百度解决（方法很多，试了才知道哪个适合你）
3. 依赖项安装（基本不会出错）

```
## 安装Octomap
sudo apt-get install ros-melodic-octomap-*
## Turtlebot3
sudo apt-get install ros-melodic-turtlebot3-*
```

### 安装PX4仿真环境

Prometheus项目中的Gazebo仿真模块依赖PX4固件及sitl_gazebo包，因此需先配置PX4编译环境

- **下载PX4固件代码**，此处请使用阿木实验室的Prometheus项目专用的PX4仓库：[prometheus_px4](https://gitee.com/amovlab/prometheus_px4.git )，安装方法如下

  ```
  git clone https://gitee.com/amovlab/prometheus_px4.git
  
  ### 安装编译环境（若之前安装过PX4环境，可跳过此步骤）
  cd prometheus_px4/Tools/setup
  ## 请在运行此脚本时注意终端的打印信息，保证每一项均顺利安装
  ## 若有某一项失败或一直卡住，请重新运行即可！
  source ./ubuntu.sh
  
  ### 安装子模块
  cd prometheus_px4
  git submodule update --init --recursive
  pip3 install --user toml empy jinja2 packaging
  
  ## 编译，如果编译通过，但是gazebo启动有问题，请再次执行该语句！
  make amovlab_sitl_default gazebo
  ```


**说明**：

- 如果在更新子模块时遇到问题,可尝试先make_distclean,再git submodule update --init --recursive

- 在安装PX4编译环境时，比较常遇到的问题就是arm-none-eabi-gcc安装失败或者版本不对，请注意以下几点

  - **版本问题**：一定使用prometheus_px4/Tools/setup目录下的ubuntu.sh进行安装，如果使用PX4官方master分支下的对应文件则会安装高级版本的arm-none-eabi-gcc，会使得编译prometheus_px4出错

  - 可以通过如下指令查看arm-none-eabi-gcc版本

    ``` c
     $arm-none-eabi-gcc --version
    
     arm-none-eabi-gcc (GNU Tools for Arm Embedded Processors 7-2017-q4-major) 7.2.1 20170904 (release) [ARM/embedded-7-branch revision 255204]
     Copyright (C) 2017 Free Software Foundation, Inc.
     This is free software; see the source for copying conditions.  There is NO
     warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
    ```

    prometheus_px4对应的版本是**gcc version 7.2.1 20170904**，如果之前已经安装过最新的arm-none-eabi-gcc，请重新运行ubuntu.sh文件后，重启电脑。

  - 安装失败：一般是由于网络原因导致，在运行ubuntu.sh文件时，请耐心查看安装记录，下载arm-none-eabi-gcc有时会因为网络原因而自动放弃下载，此时，也只需要反复运行ubuntu.sh文件直至安装成功。

- PX4环境配置可参考[PX4手册 - getting_started](https://dev.px4.io/v1.10/en/setup/getting_started.html)，请选择1.11分支。

- **建议同时安装PX4官方仓库和本仓库，如果官方仓库能正常编译，而本仓库不行，可提问。否则请先解决官方仓库编译问题**

- 若使用官方PX4仓库，Prometheus部分功能会失效，需要修改后方能使用（暂无详细说明，需自行解决）

- **此处安装成功的标志为：PX4固件能够编译，并能运行其自带的Gazebo仿真，即运行`make px4_sitl gazebo`能够正常运行Gazebo仿真**

- **此处安装及编译出现任何问题，请前往PX4 Firmware的issue区寻找答案（但大部分情况是没有问题，请保证PX4环境配置正确）**或者前往本仓库的issue区提问，网址：https://gitee.com/amovlab/prometheus_px4/issues

- 对PX4固件代码进行任何修改或者执行过`git pull`都需要重新运行`make px4_sitl gazebo`

**报错解决**:

- 缺少gstreamer：`sudo apt-get install libgstreamer1.0-0 gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-doc gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio`

## 环境变量配置

- 打开终端，并输入如下指令打开`bashrc`文件

  ```
  sudo gedit ~/.bashrc 
  ```

- 在打开的文件中手动添加如下指令（以下若存在已添加过的命令，请勿重复添加），其中`${your kongdijiqun path}`为kongdijiqun项目路径，`${your px4 path}`为安装PX4固件的路径。

  ```c
  source ${your kongdijiqun path}/Kongdijiqun/devel/setup.bash
  export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:${your kongdijiqun path}/Kongdijiqun/devel/lib
  export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:${your kongdijiqun path}/Kongdijiqun/Simulator/gazebo_simulator/models
  export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:${your kongdijiqun path}/Kongdijiqun/Simulator/gazebo_simulator/amov_models
  source ${your px4 path}/prometheus_px4/Tools/setup_gazebo.bash ${your px4 path}/prometheus_px4 ${your px4 path}/prometheus_px4/build/amovlab_sitl_default
  export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${your px4 path}/prometheus_px4
  export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${your px4 path}/prometheus_px4/Tools/sitl_gazebo
  ```

**备注**：

- 添加以上环境变量后，每次打开终端会出现配置好的路径，忽略即可。
- 此步骤经常容易出错，请再三检查。（每一个路径都是有实际含义，请确保电脑中有该路径存在）

### 安装仿真代码

- **下载项目代码**，使用`crtl+alt+T`打开一个新的终端

  ```
  git clone https://github.com/SYSU-Unmaned-System-Team/Kongdijiqun
  cd Kongdijiqun
  sudo chmod 777 ./compile_all.sh (第一次运行才需要执行此赋权命令)
  ## 编译
  ./compile_all.sh
  ```

- **环境变量配置**，打开一个新终端，输入`gedit .bashrc`并回车，在打开的`bashrc.txt`文件中添加 `source /home/$(your computer name)/Kongdijiqun/devel/setup.bash`，或者使用如下命令（**需编译后才会出现该文件**！）

  ```
  echo "source (Path To Kongdijiqun)/devel/setup.bash" >> ~/.bashrc
  source ~/.bashrc
  ```



### 仿真测试

- 与本次项目相关的launch文件均放置于Simulator/gazebo_simulator/launch_cxy

  ```
  ### 执行下述语句，看到飞机，且能控制飞机起飞即代表成果！
  roslaunch prometheus_gazebo sitl_cxy_control.launch
  ```

  
