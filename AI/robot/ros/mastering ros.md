#### 使用 pluginlib 创建给计算器应用常见插件

结构图

> 每一个插件都需要继承 calc_functions 类

![](imgs/26.png)

目录

![](imgs/27.png)

##### 第一步 创建  calculator_base 头文件

> calc_functions 里边封装了一些公用的方法，插件必须继承这个类，calculator_base  命名空间还可以加入一些其它的类

```c++
#ifndef PLUGINLIB_CALCULATOR_CALCULTOR_BASE_H_
#define PLUGINLIB_CALCULATOR_CALCULTOR_BASE_H_

namespace calculator_base {
  class calc_functions {
    public:
      virtual void get_numbers(double number1, double number2) = 0;
      virtual double operation() = 0;
      virtual ~calc_functions(){}

    protected:
      calc_functions(){}
  };
};
#endif
```

##### 第二步 创建 calculator_plugins 头文件

>继承 calc_functions 类，下面的代码只展示了 Add 类，其他的比如乘法和除法是一样的操作

```c++
#ifndef PLUGINLIB_CALCULATOR_CALCULTOR_PLUGINS_H_
#define PLUGINLIB_CALCULATOR_CALCULTOR_PLUGINS_H_
#include <pluginlib_calculator/calculator_base.h>
#include <cmath>

namespace calculator_plugins {
  class Add : public calculator_base::calc_functions {
    public:
	Add() {
		number1_ = 0;
		number2_ = 0;
	}

	void get_numbers(double number1, double number2) {
		try {
			number1_ = number1;
			number2_ = number2;
		} catch(int e) {
			std::cerr<<"Exception while inputting numbers"<<std::endl;
		}
	}	

	double operation() {
		return(number1_+number2_);
	}

    private:
      double number1_;
      double number2_;
  };
};
#endif

```

##### 第三步 使用 calculator_plugins.cpp 导出插件

> 为了使插件能动态加载，我们需要使用一个叫做 PLUGINLIB_EXPORT_CLASS 的特殊宏去导出每一个插件。PLUGINLIB_EXPORT_CLASS 需要提供插件类和基础类名称。

```c++
#include <pluginlib_calculator/calculator_base.h>
#include <pluginlib_calculator/calculator_plugins.h>

PLUGINLIB_EXPORT_CLASS(calculator_plugins::Add, calculator_base::calc_functions);
PLUGINLIB_EXPORT_CLASS(calculator_plugins::Sub, calculator_base::calc_functions);
PLUGINLIB_EXPORT_CLASS(calculator_plugins::Mul, calculator_base::calc_functions);
PLUGINLIB_EXPORT_CLASS(calculator_plugins::Div, calculator_base::calc_functions);
```

#####第四步  calculator_loader.cpp 实现插件加载器

> 使用 calculator base 作为参数创建类加载器，创建子类实例。

```c++
#include <boost/shared_ptr.hpp>
#include <pluginlib/class_loader.h>
#include <pluginlib_calculator/calculator_base.h>

int main(int argc, char** argv) {
    // 使用 calculator base 作为参数
    pluginlib::ClassLoader<calculator_base::calc_functions> 	  
      calc_loader("pluginlib_calculator", "calculator_base::calc_functions");
    
    try {
	  // 加载子类，多态，会创建一个 Add 类的实例
      boost::shared_ptr<calculator_base::calc_functions> add =  
        	calc_loader.createInstance("pluginlib_calculator/Add");
      add->get_numbers(10.0,10.0);
      double result = add->operation();
      ROS_INFO("Triangle area: %.2f", result);
  } catch(pluginlib::PluginlibException& ex) {
      ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
  }
}
```

#####第五步 创建插件描述  calculator_plugins.xml

> 这个配置文件是很重要的，因为它可以让 ROS 系统取发现、加载插件，且可以添加每个插件的描述信息.下面是 Add 插件的描述。定义了插件的  library 路径、插件的名称、插件的基础类、插件的类型和插件的描述

```xml
<library path="lib/libpluginlib_calculator">
 	<class name="pluginlib_calculator/Add" type="calculator_plugins::Add" 
		base_class_type="calculator_base::calc_functions">
 		<description>This is a add plugin.</description>
 	</class>
</library>
```

##### 第六步 注册插件

> 在 packages.xml 中添加如下几行

```xml
<!--导出第五步创建的 xml 文件，不然 ROS 系统找不到该文件-->
<export>
 <pluginlib_calculator plugin="${prefix}/calculator_plugins.xml"/>
</export>

<!--添加依赖-->
<build_depend>pluginlib_calculator</build_depend>
<run_depend>pluginlib_calculator</run_depend>
```

##### 第七步 编辑 CMakeLists.txt 文件

```cmake
......
# pluginlib_calculator library
add_library(pluginlib_calculator src/calculator_plugins.cpp)
target_link_libraries(pluginlib_calculator ${catkin_LIBRARIES})

# calculator_loader executable
add_executable(calculator_loader src/calculator_loader.cpp)
target_link_libraries(calculator_loader ${catkin_LIBRARIES})
......
```

执行 `catkin_make`

##### 第八步 查询包中插件

```
$ rospack plugins --attrib=plugin pluginlib_calculator
```

#####第九步 加载插件

```
$ roscore
$ rosrun pluginlib_calculator calculator_loader
```



#### 理解 ROS  nodelets

> nodelets 可以在一个进程中同时运行多个结点 ，每个结点使用一个线程，这样进程内部的数据是可以共用的，同时这个内部结点也可以和外部交流。nodelets 在数据传输量很大的结点间使用多，比如 3D 数据的传输。

创建一个 nodelet， 订阅 msg_in， 输出到 msg_out

目录结构

![](imgs/28.png)

#####第一步 创建包

 $ catkin_create_pkg nodelet_hello_world  nodelet  roscpp  std_msgs

#####第二步 创建  hello_world.cpp 文件

##### 第三步 理解  hello_world.cpp

```c++
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <stdio.h>

namespace nodelet_hello_world {
class Hello : public nodelet::Nodelet {
    private:
        virtual void onInit() {
            ros::NodeHandle& private_nh = getPrivateNodeHandle();
            NODELET_DEBUG("Initialized the Nodelet");
            pub = private_nh.advertise<std_msgs::String>("msg_out",5);
            sub = private_nh.subscribe("msg_in",5, &Hello::callback, this);
        }

        void callback(const std_msgs::StringConstPtr input) {
            std_msgs::String output;
            output.data = input->data;
            NODELET_DEBUG("Message data = %s", output.data.c_str());
            ROS_INFO("Message data = %s", output.data.c_str());
            pub.publish(output);
        }

        ros::Publisher pub;
        ros::Subscriber sub;
    };
}

// 导出作为插件可以动态加载，书上换个代码里边有差别

// 代码
// PLUGINLIB_DECLARE_CLASS(nodelet_hello_world, Hello, nodelet_hello_world::Hello, //nodelet::Nodelet);

// 书上
PLUGINLIB_DECLARE_CLASS(nodelet_hello_world,Hello,nodelet_hello_world::Hello, nodelet::Nodelet);
```

#####第四步 创建插件描述文件

```xml
<library path="libnodelet_hello_world">
  <class name="nodelet_hello_world/Hello" type="nodelet_hello_world::Hello" 
		base_class_type="nodelet::Nodelet">
  	<description>
  		A node to republish a message
  	</description>
  </class>
</library>
```

##### 第五步 在 package.xml 中导出

```xml
<export>
 	<nodelet plugin="${prefix}/hello_world.xml"/>
</export>

<build_depend>nodelet_hello_world</build_depend>
<run_depend>nodelet_hello_world</run_depend>
```

#####第六步 编辑 Cmake 文件

```cmake
......
## Declare a cpp library
add_library(nodelet_hello_world src/hello_world.cpp)
  
## Specify libraries to link a library or executable target against
target_link_libraries(nodelet_hello_world ${catkin_LIBRARIES})
......
```

##### 第七步 编译和运行

执行 `catkin_make`，会生成一个 libnodelet_hello_world.so 文件，这就是一个插件。nodelet_manager 是一个可执行的 c++ 程序，监听 ROS  services 并且动态加载 nodelets，启动 nodelet_manager 可以从命令行和配置文件中启动

```
$ roscore

# 启动 nodelet_manager
$ rosrun nodelet nodelet manager __name:=nodelet_manager

# 使用 nodelet_manager 启动 nodelet
$ rosrun nodelet nodelet load nodelet_hello_world/Hello 
nodelet_manager __name:=nodelet1

# 发送消息
$ rostopic pub /nodelet1/msg_in std_msgs/String "Hello"

# 打印接收消息
$ rostopic echo /nodelet1/msg_out
```

#####第八步 创建 launch files

> 启动多个 nodelet

```xml
<launch>
   <!-- Started nodelet manager -->
   <node pkg="nodelet" type="nodelet" name="standalone_nodelet" 
       args="manager" output="screen"/>

   <!-- Starting first nodelet -->
   <node pkg="nodelet" type="nodelet" name="test1"
 	   args="load nodelet_hello_world/Hello standalone_nodelet" output="screen">
   </node> 

   <!-- Starting second nodelet -->
   <node pkg="nodelet" type="nodelet" name="test2" 
       args="load nodelet_hello_world/Hello standalone_nodelet"  output="screen">
   </node>
</launch>
```

执行 `roslaunch nodelet_hello_world hello_world.launch`

输出

`$rosrun rqt_gui rqt_gui`

 依次选择 Plugins > Introspection > Node Graph 就可以输出下面的图

![](imgs/29.png)



## Gazebo  插件

> Gazebo 插件帮助我们控制机器人的模型、传感器、环境属性、甚至是路径。Gazebo  模拟器可以动态加载这些插件。这些插件是独立的，不使用 ROS 也可以使用这些插件，主要分为以下几个类别。
>
> - 环境属性插件，可以修改物理引擎、光照等属性
>
> - 模型插件，模型插件依附于具体的模型，可以控制模型的连接状态等
> - 系统插件，在 Gazebo 启动的时候伴随着启动，可以控制系统相关的功能
> - 可视化插件，可视化属性可以通过它控制

创建一个基本的 world plugin

```
$ mkdir ~/gazebo_basic_world_plugin
$ cd ~/gazebo_basic_world_plugin
$ nano hello_world.cc
```

hello_world.cc 内容

> 头文件
>
> - gazebo/gazebo.hh： 包含 Gazebo 的一些基本的方法
>
> - gazebo/physics/physics.hh: 物理引擎
> -  gazebo/rendering/rendering.hh:处理渲染参数
> -  gazebo/sensors/sensors.hh: 处理传感器
>
> 插件导出的宏命令
>
> - GZ_REGISTER_MODEL_PLUGIN：模型插件导出
> - GZ_REGISTER_SENSOR_PLUGIN： 传感器插件导出
> -  GZ_REGISTER_SYSTEM_PLUGIN：系统插件导出
> -  GZ_REGISTER_VISUAL_PLUGIN：可视化插件导出
> - GZ_REGISTER_WORLD_PLUGIN：导出为世界插件

```c++
#include <gazebo/gazebo.hh>

// 所有的 gazebo 插件都需要命名空间
namespace gazebo {
    // WorldpluginTutorials 实现 worldPlugin 标准类。每一个插件都需要实现于标准类
    class WorldPluginTutorial : public WorldPlugin {
        public: WorldPluginTutorial() : WorldPlugin() {
            printf("Hello World!\n");
        }

        // The Load function can receive the SDF elements 
        public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) {
        }
    };
	
    // 注册这个插件到模拟器
 	GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)
}
```

修改  CMakeLists.txt 文件

```cmake
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
find_package(Boost REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
	pkg_check_modules(GAZEBO gazebo)
endif()
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
add_library(hello_world SHARED hello_world.cc)
target_link_libraries(hello_world ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
```

build

```
$ mkdir ~/gazebo_basic_world_plugin/build
$ cd ~/gazebo_basic_world_plugin/build
$ cmake ../
$ make

# 设置插件 libhello_world.so 的路径到环境变量
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/gazebo_basic_world_plugin/build
```

这样就可以在 SDF 和 URDF 文件中使用该插件了，例如：

$ nano ~/gazebo_basic_world_plugin/hello.world

```xml
<?xml version="1.0"?>
<sdf version="1.4">
	<world name="default">
        <plugin name="hello_world" filename="libhello_world.so"/>
    </world>
</sdf> 
```

加载文件

```
$ cd ~ /gazebo_basic_world_plugin
$ gzserver hello.world --verbose
```

https://bitbucket.org/osrf/gazebo，可以获取各种 Gazebo 插件





