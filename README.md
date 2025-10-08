# Hikvision Camera ROS2 Driver (Task4) 海康威视相机 ROS2 驱动程序（Task4）

该项目目标是让任何ROS2开发者可以轻松地在项目中使用海康相机，获取图像数据并控制相机基础参数。


## 编译

使用 colcon 工具进行构建，命令如下：



```
colcon build
```

## 运行

### 环境变量配置

目前存在环境变量依赖问题，临时解决方案如下（终端中执行）：



```
export LD\_LIBRARY\_PATH=/opt/MVS/lib/64:\$LD\_LIBRARY\_PATH
```

### 启动节点

编译完成后，加载环境并启动相机节点与 RViz 可视化节点：



```
\# 加载环境变量

source install/setup.zsh

\# 启动相机与RViz

ros2 launch hik\_camera camera.launch.py
```

## 动态调整参数

支持在运行时动态调整相机参数，操作如下（需新建终端）：



```
\# 调整曝光时间（示例：设置为8000.0μs）

ros2 param set /hik\_camera\_node exposure\_time 8000.0

\# 调整增益（示例：设置为5.0）

ros2 param set /hik\_camera\_node gain 5.0

\# 调整帧率（示例：设置为25.0）

ros2 param set /hik\_camera\_node frame\_rate 25.0

\# 查看所有可配置参数

ros2 param list

\# 查看当前曝光时间

ros2 param get /hik\_camera\_node exposure\_time
```

## 开发常见问题及解决方案



1. **SDK 安装包路径缺失 include 目录**

   解决方案：重新下载并安装 SDK。

2. **运行 launch 文件时出现 error**

   解决方案：终端中执行以下命令（临时修复环境变量依赖）：



```
export LD\_LIBRARY\_PATH=/opt/MVS/lib/64:\$LD\_LIBRARY\_PATH
```

## 话题列表

驱动节点会发布以下 ROS2 话题，供后续模块订阅使用：



* `/image_raw` (sensor\_msgs/Image)：原始图像数据

* `/camera_info` (sensor\_msgs/CameraInfo)：相机内参等标定信息

## VSCode 配置

若使用 VSCode 开发，可添加以下配置（`.vscode/settings.json`）指定 CMake 源目录：



```
{

&#x20;   "cmake.sourceDirectory": "/home/qixuan_lu/Task4/src/hik\_camera"

}
```

