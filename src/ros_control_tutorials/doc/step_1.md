# ROS 控制教程：创建第一个节点
## 1. 创建 ROS 功能包
进入工作空间的src目录，创建功能包：

bash

#进入工作空间src目录

cd path/to/${workspace}/src

#创建功能包，依赖roscpp

catkin_create_pkg ros_control_tutorials roscpp

## 2. 配置 VSCode 头文件索引
编辑工作空间下的.vscode/c_cpp_properties.json文件：
{

  "configurations": [

    {
      "name": "ROS",
      "includePath": [
        "${workspaceFolder}/**",
        "${workspaceFolder}/src/ros_control_tutorials/include/ros_control_tutorials/**",
        "/opt/ros/noetic/include/**"  // 根据实际ROS版本调整
      ],
      "defines": [],
      "compilerPath": "/usr/bin/gcc",
      "cStandard": "gnu17",
      "cppStandard": "gnu++14",
      "intelliSenseMode": "linux-gcc-x64"
    }
  ],
  "version": 4
}

## 3. 编写头文件
### 3.1 创建头文件
bash

#进入功能包的include目录
cd path/to/ros_control_tutorials/include/ros_control_tutorials/

#创建并编辑hello.h

vim hello.h

### 3.2 头文件内容
cpp
运行

#ifndef HELLO_H
#define HELLO_H

#include <iostream>
#include <ros/ros.h>

using namespace std;

// 声明hello函数

void hello(void);

#endif // HELLO_H


保存退出：按Esc键，输入:wq，按Enter键。

## 4. 编写源文件
### 4.1 创建源文件
bash

#进入功能包的src目录

cd path/to/ros_control_tutorials/src/

#创建并编辑hello.cpp

vim hello.cpp

### 4.2 源文件内容
cpp
运行

#include "ros_control_tutorials/hello.h"

int main(int argc, char *argv[])
{
    // 初始化ROS节点
    ros::init(argc, argv, "hello_node");
    ros::NodeHandle nh;  // 创建节点句柄

    // 调用hello函数
    hello();

    // 循环等待回调
    ros::spin();
    // 关闭节点
    ros::shutdown();
    return 0;
}

// 实现hello函数
void hello(void)
{
    cout << "hello world!" << endl;
}


保存退出：按Esc键，输入:wq，按Enter键。

## 5. 配置 CMakeLists.txt

编辑${workspace}/src/ros_control_tutorials/CMakeLists.txt，添加以下内容：

cmake

#包含头文件目录

include_directories(

  include

  ${catkin_INCLUDE_DIRS}

)

#生成可执行文件

add_executable(hello src/hello.cpp)

#链接库

target_link_libraries(hello

  ${catkin_LIBRARIES}

)

## 6. 编译与运行
### 6.1 编译工作空间
bash

#回到工作空间根目录
cd path/to/${workspace}

#编译
catkin build

#刷新环境变量
source devel/setup.bash

### 6.2 运行节点

    第一个终端启动 ROS 核心：


bash

roscore



    第二个终端运行节点：


bash

rosrun ros_control_tutorials hello



运行成功后，将输出：hello world!