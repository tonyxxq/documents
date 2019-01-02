## 安装 gazebo 模拟器

安装和启动

```apt-get install ros-kinetic-turtlebot-gazebo 
$ apt-get install ros-kinetic-turtlebot-*

# 启动模拟器
$ roslaunch turtlebot_gazebo turtlebot_world.launch 

# 启动键盘控制（只能在终端界面用键盘控制turtlebot移动，无法在 Gazebo 界面控制）
$ roslaunch turtlebot_teleop keyboard_teleop.launch

# 观察摄像机采集的数据
$ roslaunch turtlebot_rviz_launchers view_robot.launch
```

遇到的问题和解决办法

1. 打开一直处于如下状态，进不去，原因是没下载 models

   解决办法：

   ``` cd  ~/.gazebo/
   $ mkdir -p models
   $ cd  ~/.gazebo/models/
   $ wget http://file.ncnynl.com/ros/gazebo_models.txt
   $ wget -i gazebo_models.txt
   $ ls model.tar.g* | xargs -n1 tar xzvf
   ```

2. 错误：*Invalid tag: environment variable ‘TURTLEBOT_GAZEBO_WORLD_FILE’ is not set. Arg xml is arg default=”$(env TURTLEBOT_GAZEBO_WORLD_FILE)” name=”world_file”The traceback for the exception was written to the log file.*

   解决方法：重新配置环境

   ```
   $ sudo rosdep init
   $ rosdep update
   $ echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
   ```

   $ source ~/.bashrc
   --------------------- 
   作者：GJXS2017 
   来源：CSDN 
   原文：https://blog.csdn.net/GJXS2017/article/details/80198346 
   版权声明：本文为博主原创文章，转载请附上博文链接！

## 人脸检测

####结构图

![](imgs/2.png)

####创建 ROS 包

> mac 不能调用摄像头问题,解决方法： https://github.com/patjak/bcwc_pcie/wiki/Get-Started
>
> v4l-utils：实时从 webcam 抓取视频
>
> usb_cam：获取 v4l 的视频流，并且发布为 ROS 图像消息

```
# 进入空间
$ cd ros_project_dependencies_ws/src/

# 下载包 web_cam，用于读取摄像头的图像，可以放到一个第三方包空间中
$ git clone https://github.com/bosch-ros-pkg/usb_cam.git
$ cd ros_project_dependencies_ws
$ catkin_make

# 安装 v4l-utils
sudo apt-get install v4l-utils

# 创建 face_tracker_pkg 包
$ catkin_create_pkg face_tracker_pkg roscpp rospy cv_bridge dynamixel_controllers 	   message_generation

# 创建 face_tracker_control 包
$ catkin_create_pkg face_tracker_control roscpp rospy std_msgs dynamixel_controllers message_generation
```

> 1. opencv 通过 vision_opencv 这个包集成在 ROS 中，在安装 ROS（全部安装） 的时候就已经安装了，有两个包
>
>    cv_bridge：把 OpenCV  的图像类型转为 ROS 的图像消息( sensor_msgs/Image.msg )，或把摄像头获取的图像转换为 OpenCV 支持的类型
>
>    image_geometry：提供了一系列关于图像几何处理的方法
>
> 2. image_transport 
>
>    该包使用图像压缩技术减少传递的带宽，在安装 ROS（全部安装） 的时候就已经安装了

1. 编写 face_tracker_pkg

   ![](imgs/3.png)

包结构:

![](imgs/4.png)

face_tracker_node.cpp

```c++
// ROS headers
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// Open-CV headers
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/objdetect.hpp"

// Centroid message headers
#include <face_tracker_pkg/centroid.h>

// OpenCV window name
static const std::string OPENCV_WINDOW = "raw_image_window";
static const std::string OPENCV_WINDOW_1 = "face_detector";

using namespace std;
using namespace cv;

class Face_Detector {
  ros::NodeHandle nh_;
  
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
 
  ros::Publisher face_centroid_pub;

  face_tracker_pkg::centroid face_centroid;

  string input_image_topic, output_image_topic, haar_file_face;
  int face_tracking, display_original_image, display_tracking_image, center_offset,      screenmaxx;

  
public:
  
  Face_Detector(): it_(nh_) {
  
  // 默认值，使用 track.yaml 文件的内容进行覆盖
  input_image_topic = "/usb_cam/image_raw";
  output_image_topic = "/face_detector/raw_image";
  haar_file_face = "/home/robot/face.xml";
  face_tracking = 1;
  display_original_image = 1;
  display_tracking_image = 1;
  screenmaxx = 640;
  center_offset = 100;

  // 加载 track.yaml 问价文件中配置的参数
  try {
  	nh_.getParam("image_input_topic", input_image_topic);
  	nh_.getParam("face_detected_image_topic", output_image_topic);
  	nh_.getParam("haar_file_face", haar_file_face);
  	nh_.getParam("face_tracking", face_tracking);
  	nh_.getParam("display_original_image", display_original_image);
  	nh_.getParam("display_tracking_image", display_tracking_image);
  	nh_.getParam("center_offset", center_offset);
  	nh_.getParam("screenmaxx", screenmaxx);

  	ROS_INFO("Successfully Loaded tracking parameters");
  } catch(int e) {
      ROS_WARN("Parameters are not properly loaded from file, loading defaults");
  }

   // 订阅 input_image_topic，回调函数 imageCb, 发布到 output_image_topic 和 face_centroid
   image_sub_ = it_.subscribe(input_image_topic, 1, &Face_Detector::imageCb, this);
   image_pub_ = it_.advertise(output_image_topic, 1);
   face_centroid_pub = nh_.advertise<face_tracker_pkg::centroid>("/face_centroid", 10);
  }

  ~Face_Detector() {
    if( display_original_image == 1 or display_tracking_image == 1)
    	cv::destroyWindow(OPENCV_WINDOW);
  }

  // 订阅的回调函数
  void imageCb(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    namespace enc = sensor_msgs::image_encodings;

    // 把获取的 ROS 图像数据转换为 OpenCV 的格式
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // 加载分类器
	string cascadeName = haar_file_face;
    CascadeClassifier cascade;
	if(!cascade.load(cascadeName)) {
		cerr << "ERROR: Could not load classifier cascade" << endl;
	}

	if (display_original_image == 1) {
		imshow("Original Image", cv_ptr->image);
	}
    
    // 使用分类器在图像上找出人脸，并且画出来
    detectAndDraw( cv_ptr->image, cascade );
    
    // 发布
    image_pub_.publish(cv_ptr->toImageMsg());
    waitKey(30);
}

// 检测图像并在图像上画出检测出的人脸
void detectAndDraw( Mat& img, CascadeClassifier& cascade)
{
    double t = 0;
    double scale = 1;
    vector<Rect> faces, faces2;
    const static Scalar colors[] = {
        Scalar(255,0,0),
        Scalar(255,128,0),
        Scalar(255,255,0),
        Scalar(0,255,0),
        Scalar(0,128,255),
        Scalar(0,255,255),
        Scalar(0,0,255),
        Scalar(255,0,255)
    };
    Mat gray, smallImg;

    // 对图像转换位灰度图、resize、直方图均衡化
    cvtColor( img, gray, COLOR_BGR2GRAY );
    double fx = 1 / scale ;
    resize( gray, smallImg, Size(), fx, fx, INTER_LINEAR );
    equalizeHist(smallImg, smallImg);
	
  	// 检测人脸
    t = (double)cvGetTickCount();
    cascade.detectMultiScale(smallImg, faces, 1.1, 15, 0 | CASCADE_SCALE_IMAGE,     Size(30, 30));
    t = (double)cvGetTickCount() - t;
  
    // 对检测的人脸进行
    for ( size_t i = 0; i < faces.size(); i++ ) {
        Rect r = faces[i];
        Mat smallImgROI;
        vector<Rect> nestedObjects;
        Point center;
        Scalar color = colors[i%8];
        int radius;

        double aspect_ratio = (double)r.width / r.height;
        if( 0.75 < aspect_ratio && aspect_ratio < 1.3 ) {
            center.x = cvRound((r.x + r.width*0.5)*scale);
            center.y = cvRound((r.y + r.height*0.5)*scale);
            radius = cvRound((r.width + r.height)*0.25*scale);
            circle( img, center, radius, color, 3, 8, 0 );

   	    	face_centroid.x = center.x;
   	    	face_centroid.y = center.y;
  
            // 发布计算出的检测到的人脸中心
  	    	face_centroid_pub.publish(face_centroid);
        } else {
            rectangle( img, cvPoint(cvRound(r.x*scale), cvRound(r.y*scale)),
                       cvPoint(cvRound((r.x + r.width-1)*scale), cvRound((r.y +	 	r.height-1)*scale)), color, 3, 8, 0);
        }
    }

    // 添加左右和中心线,固定的
    Point pt1, pt2 ,pt3, pt4, pt5, pt6;

    // 中心点
    pt1.x = screenmaxx / 2;
    pt1.y = 0;
    pt2.x = screenmaxx / 2;
    pt2.y = 480;

    // 左边点
    pt3.x = (screenmaxx / 2) - center_offset;
    pt3.y = 0;
    pt4.x = (screenmaxx / 2) - center_offset;
    pt4.y = 480;

    // 右边点
    pt5.x = (screenmaxx / 2) + center_offset;
    pt5.y = 0;
    pt6.x = (screenmaxx / 2) + center_offset;
    pt6.y = 480;

    // 画出这三条线并添加文字
    line(img,  pt1,  pt2, Scalar(0, 0, 255),0.2);
    line(img,  pt3,  pt4, Scalar(0, 255, 0),0.2);
    line(img,  pt5,  pt6, Scalar(0, 255, 0),0.2);

    putText(img, "Left", cvPoint(50,240), FONT_HERSHEY_SIMPLEX, 1, cvScalar(255,0,0), 2, CV_AA);
    putText(img, "Center", cvPoint(280,240), FONT_HERSHEY_SIMPLEX, 1, cvScalar(0,0,255), 2, CV_AA);
    putText(img, "Right", cvPoint(480,240), FONT_HERSHEY_SIMPLEX, 1, cvScalar(255,0,0), 2, CV_AA);

    if (display_tracking_image == 1) {
    	imshow( "Face tracker", img );
     }
}
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "Face tracker");
  Face_Detector ic;
  ros::spin();
  return 0;
}
```

track.yml

> haar_file_face 并不是只用人脸，同时可以检测眼睛等，下载配置文件替换就可以了，下载地址：https://github.com/opencv/opencv/tree/master/data

```
image_input_topic: "/usb_cam/image_raw"
face_detected_image_topic: "/face_detector/raw_image"
haar_file_face: "/home/tony/ros_robotics_projects_ws/src/face_tracker_pkg/data/face.xml"
face_tracking: 1
display_original_image: 1
display_tracking_image: 1
```

start_usb_cam.launch

```yaml
<launch>
	<node name="usb_cam" pkg="usb_cam" type="usb_cam_node" output="screen" >
		<param name="video_device" value="/dev/video0" />
		<param name="image_width" value="640" />
		<param name="image_height" value="480" />
		<param name="pixel_format" value="yuyv" />
		<param name="camera_frame_id" value="usb_cam" />
		<param name="auto_focus" value="false" />
		<param name="io_method" value="mmap"/>
	</node>
</launch>
```

start_tracking.launch

> 使用 **find** 可以加载别的包下的 lauch 文件和配置文件

```yaml
<launch>
	<!-- Launching USB CAM launch files and Dynamixel controllers -->
	<include file="$(find face_tracker_pkg)/launch/start_usb_cam.launch"/>
	
	<!-- Starting face tracker node -->
	<rosparam file="$(find face_tracker_pkg)/config/track.yaml" command="load"/>
	
	<node name="face_tracker" pkg="face_tracker_pkg" type="face_tracker_node" 
	output="screen"/>
</launch>
```

## 聊天机器人

####AIML 数据格式

> AIML 是一种基于 XML 语言的数据存储格式，能很好的进行数据存储和检索

示例

category：输入和输出放在里边

pattern：输入的内容

template：输入内容

\*：匹配任何内容

star：n 表示匹配到的第几个

```xml
<aiml version="1.0.1" encoding="UTF-8">
	<category>
		<pattern> MY NAME IS * </pattern>
		<template>
			NICE TO SEE YOU <star index="1"/>
		</template>
		</category>
	<category>
		<pattern> MEET OUR ROBOTS * AND * </pattern>
		<template>
			NICE TO SEE <star index="1"/>AND <star index="2"/>.
		</template>
	</category>
</aiml>
```

输入：You: MY NAME IS LENTIN 

回答： Robot: NICE TO SEE YOU LENTIN

输入：You: MEET OUR ROBOTS ROBIN AND TURTLEBOT
回答：Robot: NICE TO SEE ROBIN AND TURTLEBOT    

####PyAIML interpreter

> PyAIML interpreter 可以加载 AIML 数据并生成一棵树，使用深度优先搜索查询数据。

安装：

```
$ sudo apt-get install python-aiml
```

简单使用：

```python
import aiml

bot = aiml.Kernel()
bot.setBotPredicate("name", ROBIN)
#　加载指定的 AIML 文件
bot.learn('sample.aiml")
print bot.respond("MY NAME IS LENTIN")
```

加载多个 AIML 文件：

```xml
<aiml version="1.0">
<category>
	<pattern>LOAD AIML B</pattern>
		<template>
			<!-- 可以加载指定目录下的所有 AIML 文件 -->
			<learn>*.aiml</learn>
		</template>
	</category>
</aiml>
```

示例代码：

```python
import aiml
import sys
import os

#　设置 AIML 文件夹的路径
os.chdir('/home/robot/Desktop/aiml/aiml_data_files') 
bot =　aiml.Kernel()

# 判断该目录下是否有　brainFile，有就直接加载　brainFile，没有生成　brainFile，这样就不用程序执行的
# 时候都生成一遍
initialize using bootstrap() method
if os.path.isfile("standard.brn"): 
  bot.bootstrap(brainFile =　"standard.brn") 
else:
　brain bot.bootstrap(learnFiles = "startup.xml", commands = "load　aiml b") 　bot.saveBrain("standard.brn")

# 输入数据获得输出
while True: 
  print bot.respond(raw_input("Enter input >"))
```

####搭建聊天机器人

结构图：

![](imgs/5.png)AIML ROS 包

结点之间的关系图：

![](imgs/6.png)

安装　ROS sound_play 包

```
# 安装依赖项
$ sudo apt-get install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0 gstreamer1.0-plugins-base gstreamer1.0-plugins-good　gstreamer1.0-plugins-ugly python-gi festival

＃　安装　sound_play
ros_project_dependencies_ws/src$ git clone https://github.com/ros-drivers/audio_common

ros_project_dependencies_ws$ catkin_make

# 能成功进入则表示安装成功
$ roscd sound_play
```

创建　ROS AIML 包：

```
catkin_create_pkg ros_aiml rospy std_msgs sound_play
```

目录结构位：

![](imgs/7.png)

代码：。。。

启动结点：

    $ sudo chmod +x *.launch
    $ roslaunch ros_aiml start_chat.launch
    $ roslaunch ros_aiml start_speech_chat.launch
    
    # 下载这个可以进行语音识别,还没试过
    $ roslaunch pocketsphinx robotcup.launch
## 目标检测

####find_object 包

> find_object_2d 实现了 SURF, SIFT, FAST,and BRIEF 特征提取方式和描述子，且可以标注出物体进行保存，作为之后的物检测，且可以发布检测到的目标

安装

```
# 直接安装
$ sudo apt-get install ros-kinetic-find-object-2d
$ catkin_make

# 或通过克隆下载安装
$ git clone https://github.com/introlab/find-object.git src/find_object_2d
$ catkin_make
```

执行 2D 目标检测：

```
$ roscore

# 需要先安装 usb_cam，启动
$ roslaunch usb_cam usb_cam-test.launch

# 使用 /usb_cam/image_raw 主题数据作为 find_object_2d 的输入
$ rosrun find_object_2d find_object_2d image:=/usb_cam/image_raw

# 打印检测出的物体的位置
$ rosrun find_object_2d print_objects_detected

# /objects 主题下可以获取检测到目标的详细信息，包括检测出物体的宽、高、单应矩阵
$ rostopic echo /objects
```

![](imgs/8.png)

> 在左边空白处点击鼠标右键，可以选择 add objects from screen 和 add objects from files，这里选择第一个，然后选择　 take  picture，　

![](imgs/9.png)

> 选择区域或关键点

![](imgs/10.png)

> 选择　End ，可以对指定的物体进行检测了

![](imgs/11.png)

这时 /objects 主题下也有数据了

![](imgs/12.png)

使用深度传感器，执行 3D 目标检测：

```
# 下面装的依赖和前面的装的 usb_cam 作用是一样的，就是获取摄像头数据
# Kinect 一代需要的依赖
$ sudo apt-get install ros-kinetic-openni-launch

# Kinect 二代需要自己克隆安装依赖
$ git clone https://github.com/code-iai/iai_kinect2

# 使用 Kinect 一代, 启动，操作之后的操作和之前 2D 的操作是一样的
$ roslaunch openni_launch openni.launch depth_registration:=true

# 启动 find_object_3d
$ roslaunch find_object_2d find_object_3d.launch
```

####  3D 物体识别， Object Recognition Kitchen ( ORK)

安装：

```
$ sudo apt-get install ros-kinetic-object-recognition-*

# 可以下载 ork 教程
$ git clone https://github.com/wg-perception/ork_tutorials

# 
```



## ROS 和深度学习的集成

安装 tensorflow

> 在安装过程中遇到如下问题：
>
> １．　提示 pip升级，升级完之后出现　cannot from pip import main　问题。
>
> ​	解决办法：
>
> ​	sudo  gedit /usr/bin/pip
>
> ​	把　from pip import main　修改为　from pip._internal import main
>
> ２．无法下载　tensorflow,　pip 源的问题
>
> ​	解决办法：
>
> ​	修改 ~/.pip/pip.conf (没有就创建一个文件夹及文件，文件夹要加"."，表示是隐藏文件夹)，内容如下：
> ​	[global]
> ​	index-url = https://pypi.tuna.tsinghua.edu.cn/simple
> ​	install]
>
> 　　trusted-host=mirrors.aliyun.com	

```
# 安装 pip
$ sudo apt-get install python-pip python-dev

# tensorflow 二进制　URL（cpu）
$ export TF_BINARY_URL=https://storage.googleapis.com/tensorflow/linux/cpu/tensorflow-0.11.0-cp27-none-linux_x86_64.whl

# tensorflow 二进制　URL（gpu）
$ export TF_BINARY_URL=https://storage.googleapis.com/tensorflow/linux/gpu/tensorflow-0.11.0-cp27-none-linux_x86_64.whl

# 安装　tensorflow
$ sudo pip install --upgrade $TF_BINARY_URL
```

安装成功：

![](imgs/13.png)

使用 tensorflow 进行图像识别：

> 遇到问题
>
> 在执行 `python image_recognition.py image:=/cv_camera/image_raw `  过程中出现　
>
> ```
> TA protocol message was rejected because it was too big (more than 67108864 bytes).
> ```
>
> 解决方法：
>
> 设置换进变量　export  PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python

```
# 安装 cv_bridge
$ sudo apt-get install ros-kinetic-cv-bridge ros-kinetic-cv-camera

$ roscore

# 开启摄像头
$ rosrun cv_camera cv_camera_node

# 执行图像识别，设置图像输入
$ python image_recognition.py image:=/cv_camera/image_raw

# 识别出的物体通过 /result 主题进行输出 
$ rostopic echo /result

#　image_view 查看图像，　默认选择全部安装 ros 时会自动安装 image_view, 如果没有，则安装　
# sudo apt-get install ros-kinetic-image-view
$ rosrun image_view image_view image:= /cv_camera/image_raw
```

image_recognition.py 代码：

```python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import tensorflow as tf
from tensorflow.models.image.imagenet import classify_image


class RosTensorFlow():
    def __init__(self):
        # 下载tensorflow 模型，下载地址为 /tmp/imagenet
        classify_image.maybe_download_and_extract()
        
        self._session = tf.Session()
        classify_image.create_graph()
        self._cv_bridge = CvBridge()

        self._sub = rospy.Subscriber('image', Image, self.callback, queue_size=1)
        self._pub = rospy.Publisher('result', String, queue_size=1)
        
        #　评分阈值和显示数量
        self.score_threshold = rospy.get_param('~score_threshold', 0.1)
        self.use_top_k = rospy.get_param('~use_top_k', 5)

    def callback(self, image_msg):
        cv_image = self._cv_bridge.imgmsg_to_cv2(image_msg, "bgr8")
        
        # copy from https://github.com/tensorflow/tensorflow/blob/master/tensorflow
        # /models/image/imagenet/classify_image.py
        image_data = cv2.imencode('.jpg', cv_image)[1].tostring()
        
        # Creates graph from saved GraphDef.softmax_tensor　是计算图中的一个tensor,　长度是　
        # 1000，即能识别出 1000 种类型
        softmax_tensor = self._session.graph.get_tensor_by_name('softmax:0')
        predictions = self._session.run(
            softmax_tensor, {'DecodeJpeg/contents:0': image_data})
        predictions = np.squeeze(predictions)
        
        # Creates node ID --> English string lookup.
        node_lookup = classify_image.NodeLookup()
        top_k = predictions.argsort()[-self.use_top_k:][::-1]
        for node_id in top_k:
            human_string = node_lookup.id_to_string(node_id)
            score = predictions[node_id]
            if score > self.score_threshold:
                rospy.loginfo('%s (score = %.5f)' % (human_string, score))
                # 识别出物体，发布
                self._pub.publish(human_string)

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('rostensorflow')
    tensor = RosTensorFlow()
    tensor.main()
```

很遗憾的是上面的识别任务只能识别出图像的物体，并没有标出物体的位置。

#### 使用 darknet_ros　

安装

https://github.com/leggedrobotics/darknet_ros

##　制作一个可以自动移动的机器人







## 使用 VR 头盔和体感器远程操控机器人

####安装 leap motion SDK 和 驱动

1. 下载  leap motion SDK ( https://www.leapmotion.com/setup/linux )，下载得到两个 deb 文件

2. 安装这两个驱动

   ```
   # 64 系统
   $ sudo dpkg -install Leap-*-x64.deb

   # 32 位系统
   $ sudo dpkg -install Leap-*-x86.deb
   ```

####可视化 leap motion 数据

 leap motion 插入 USB 接口，在终端执行 `dmesg`,  

```
# 判断是否连接成功
$ dmesg

#　打开控制面板 
$ sudo LeapControlPanel

# 只打开驱动
$ sudo leapd

＃ 重启驱动
$ sudo service leapd stop

# 启动 leap_motion 的 ros 驱动　
$ roslaunch leap_motion sensor_sender.launch

$ rostopic list
$ rostopic echo /leapmotion/data

# 启动可视化 ROS 可视化,订阅 leap_motion 的数据,把数据转换为 Rviz 支持的格式
$ roslaunch leap_client leap_client.launch

# 打开 rviz 选择 leap_client/launch/leap_client.rviz
```









#### 

##　使用 web 控制机器人

####rosbridge_suite

> rosbridge_suite 相当于 web 和 ros 的中间，负责数据转换（web 端需要的是 json 数据，ros 端需要的是主题数据），负责转换的结点名称为　rosbridge_server，消息类型位 service。

结构图：

![](imgs/14.png)

> rosbridge_suite 包含下面三个包：
>
> 1. rosbridge_library：包含 Python API 把 JSON 消息转换为 ROS 消息
> 2. rosbridge_server：使用 WebSocket 实现了 rosbridge 库，通过这个包实现和  web 端的数据通信
> 3. rosapi ：可以通过这个包获取 ROS 的主题列表、参数等元数据 

安装 rosbridge_suite

```
$ sudo apt-get update
$ sudo apt-get install ros-kinetic-rosbridge-suite

# 或使用源码安装
$ git clone https://github.com/RobotWebTools/rosbridge_suite
$ catkin_make
```

#### roslibjs, ros2djs, and ros3djs

> 这几个 js 库文件是在网页端使用的 rosbridge 客户端
>
> roslibjs： (http://wiki.ros.org/roslibjs ), 可以用 js 实现一些基本的 ROS 方法 ，比如 ROS topics, services, actionlib, TF support, URDF 等
>
> The ros2djs (http://wiki.ros.org/ros2djs ) , 该库基于 roslibjs, 提供了 ROS 的 2 维可视化，可以使用它在浏览器中可视化 2维 地图
>
> ros3djs (http://wiki.ros.org/ros3djs ) ：可以在浏览器中可视化三维数据。比如：URDF, TF, interactive markers, and maps，可以创建一个基于 web 的 Rviz 实例

下载这几个库文件，因为这几个库文件是网页端使用的，所以不需要想其他的 ROS 包一样，下载下来就可以了，不需要 catkin_make 编译

```
$ git clone https://github.com/RobotWebTools/roslibjs.git
$ git clone https://github.com/RobotWebTools/ros2djs
$ git clone https://github.com/RobotWebTools/ros3djs
```

roslibjs APIs: http://robotwebtools.org/jsdoc/roslibjs/current/ 

ros2djs APIs: http://robotwebtools.org/jsdoc/ros2djs/current/

 ros3djs APIs: http://robotwebtools.org/jsdoc/ros3djs/current/

#### tf2_web_republisher

安装

```
$ git clone https://github.com/RobotWebTools/tf2_web_republisher
$ sudo apt-get install ros-kinetic-tf2-ros
```

####远程操控机器人并可视化（简单）

结构：

![](imgs/16.png)

使用 keyboardteleopjs (http://wiki.ros.org/keyboardteleopjs ),根据按键发送消息

代码展示

```html
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">

 <!--Ｊquery　相关库文件-->
<link rel="stylesheet" type="text/css"
  href="http://ajax.googleapis.com/ajax/libs/jqueryui/1.8/themes/base/jquery-ui.css" />
<script src="https://ajax.googleapis.com/ajax/libs/jquery/1.8.0/jquery.min.js"></script>
<script src="https://ajax.googleapis.com/ajax/libs/jqueryui/1.8.23/jquery-ui.min.js"></script>

<!---->
<script src="http://cdn.robotwebtools.org/threejs/current/three.js"></script>
<script src="http://cdn.robotwebtools.org/threejs/current/ColladaLoader.js"></script>
<script src="http://cdn.robotwebtools.org/threejs/current/STLLoader.js"></script>
<script src="http://cdn.robotwebtools.org/ColladaAnimationCompress/current/ColladaLoader2.js"></script>

<script src="http://cdn.robotwebtools.org/EventEmitter2/current/eventemitter2.min.js"></script>

 <!--rosbridge 相关库文件-->
<script src="http://cdn.robotwebtools.org/roslibjs/current/roslib.js"></script>
<script src="http://cdn.robotwebtools.org/ros3djs/current/ros3d.min.js"></script>

<!--keyboardteleop-->
<script src="http://cdn.robotwebtools.org/keyboardteleopjs/current/keyboardteleop.js"></script>

<script>
  /**
   * Setup all GUI elements when the page is loaded. 
   */
 var teleop_topic = '/cmd_vel_mux/input/teleop'
 var base_frame = 'odom';
 var init_flag = false;

　function submit_values()　{
  　teleop_topic = document.getElementById("tele_topic").value;
  　base_frame = document.getElementById("base_frame_name").value;
  　init_flag = true;
 　 init();
　}

  function init() {
    if(init_flag == true)
    {
    // Connecting to ROS.
    var ros = new ROSLIB.Ros({
      url : 'ws://localhost:9090'
    });
    
    // Initialize the teleop.
    var teleop = new KEYBOARDTELEOP.Teleop({
      ros : ros,
      topic : teleop_topic
    });

////////////////////////////////////////////////////////////////////////////////////////////////////////
    var viewer = new ROS3D.Viewer({
      background : 000,
      divID : 'urdf',
      width : 1280,
      height : 600,
      antialias : true
    });
 
    // Add a grid.
    viewer.addObject(new ROS3D.Grid());

    // Setup a client to listen to TFs.
    var tfClient = new ROSLIB.TFClient({
      ros : ros,
      fixedFrame : base_frame,
      angularThres : 0.01,
      transThres : 0.01,
      rate : 10.0
    });

    // Setup the URDF client.
    var urdfClient = new ROS3D.UrdfClient({
      ros : ros,
      tfClient : tfClient,
      path : 'http://resources.robotwebtools.org/',
      rootObject : viewer.scene,
      loader : ROS3D.COLLADA_LOADER
    });


///////////////////////////////////////////////////////////////////////////////////////////////////

    // Create a UI slider using JQuery UI.
    $('#speed-slider').slider({
      range : 'min',
      min : 0,
      max : 100,
      value : 90,
      slide : function(event, ui) {
        // Change the speed label.
        $('#speed-label').html('Speed: ' + ui.value + '%');
        // Scale the speed.
        teleop.scale = (ui.value / 100.0);
      }
    });

    // Set the initial speed .
    $('#speed-label').html('Speed: ' + ($('#speed-slider').slider('value')) + '%');
    	teleop.scale = ($('#speed-slider').slider('value') / 100.0);
    	init_flag = false;
  	}
  }
</script>
</head>
<body onload="init()">
  <h1>Web-browser keyboard teleoperation</h1>
　<form >
  <!--主题-->
  Teleop topic:<br>
  <input type="text" name="Teleop Topic" id='tele_topic' value="/cmd_vel_mux/input/teleop">
  <br>
  
  Base frame:<br>
  <input type="text" name="Base frame" id='base_frame_name' value="/odom">
  <br>

 <input type="button" onmousedown="submit_values()" value="Submit"> 

　</form> 
  <p>Run the following commands in the terminal then refresh this page. Check the JavaScript console for the output.</p>
  <ol>
    <li><tt>roslaunch turtlebot_gazebo turtlebot_world.launch </tt></li>
    <li><tt>roslaunch rosbridge_server rosbridge_websocket.launch</tt></li>
    <li>Use your arrow keys on your keyboard to move the robot (must have this browser   window focused).</li>
  </ol>
  <div id="speed-label"></div>
  <div id="speed-slider"></div>
  <div id="urdf"></div>
</body>
</html>
```

启动

> 很多的 js 不能用了，需要自己下载 

```
# 设置只在浏览器展示可视化界面
$ rosparam set use_gui true

# 打开模拟器
$ roslaunch turtlebot3_gazebo turtlebot3_world.launch

# 打开 tf2_web_republisher 结点
$ rosrun tf2_web_republisher tf2_web_republisher

# 打开 websocket 连接
$ roslaunch rosbridge_server rosbridge_websocket.launch

# 打开刚才建的页面
$ google-chrome keyboardteleop.html
```

页面打开之后：

![](imgs/15.png)

点击 Submit，可以使用键盘控制机器人

####通过浏览器控制机器人连接处

结构图：

![](imgs/17.png)

安装 joint_state_publisher_js

```
# 下载包文件
git clone https://github.com/DLu/joint_state_publisher_js

# 编译
catkin_make
```

> 在 build 目录下可以看到两个 js 文件

![](imgs/18.png)

代码

```html
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8" />

<script src="http://cdn.robotwebtools.org/threejs/current/three.min.js"></script>
<script src="http://cdn.robotwebtools.org/ColladaAnimationCompress/current/ColladaLoader2.min.js"></script>
<script src="http://cdn.robotwebtools.org/threejs/r61/STLLoader.min.js"></script>
<script src="http://cdn.robotwebtools.org/EventEmitter2/current/eventemitter2.min.js"></script>
<script src="http://cdn.robotwebtools.org/roslibjs/current/roslib.min.js"></script>
<script src="http://cdn.robotwebtools.org/ros3djs/current/ros3d.min.js"></script>
<script src="../build/jointstatepublisher.js"></script>

<script>
  var topic;
  /**
   * Setup all visualization elements when the page is loaded.
   */
  function init() {
    // Connect to ROS.
    var ros = new ROSLIB.Ros({
      url : 'ws://localhost:9090'
    });

    // Create the main viewer.
    var viewer = new ROS3D.Viewer({
      divID : 'urdf',
      width : 1280,
      height : 720,
      antialias : true
    });

    // Add a grid.
    viewer.addObject(new ROS3D.Grid());

    // Setup a client to listen to TFs.
    var tfClient = new ROSLIB.TFClient({
      ros : ros,
      angularThres : 0.01,
      transThres : 0.01,
      rate : 10.0,
      fixedFrame : '/base_link'
    });

    // Setup the URDF client.
    var urdfClient = new ROS3D.UrdfClient({
      ros : ros,
      tfClient : tfClient,
      rootObject : viewer.scene,
      path : 'http://resources.robotwebtools.org/'
    });
        
    var jsp = new JOINTSTATEPUBLISHER.JointStatePublisher({
        ros : ros,
        divID : 'sliders'
    });
  }
</script>
</head>
<body onload="init()">
  <h1>Web based joint state controller for Robot</h1>
  <p>Run the following commands in the terminal then refresh this page.</p>
  <ol>
    <li><tt>roslaunch pr2_description upload_pr2.launch </tt></li>
    <li><tt>rosparam set use_gui true</tt></li>
    <li><tt>roslaunch joint_state_publisher_js core.launch</tt></li>
  </ol>
  <div id="sliders" style="float: right"></div>
  <div id="urdf"></div>
</body>
</html>
```

启动

```
# 安装 ros-kinetic-pr2-description
$ sudo apt-get install ros-kinetic-pr2-description

$ roslaunch pr2_description upload_pr2.launch
$ rosparam set use_gui true
$ roslaunch joint_state_publisher_js core.launch
```

#### 声控机器人的移动

安装

```
$ sudo apt-get install apache2
```

使用 chrome 的浏览器的语音识别

> 使用了 webkitSpeechRecognition API
>
> - SpeechRecognition.lang：设置识别的是什么语言，cmn-Hans-CN 代表普通话
> - SpeechRecognition.interimResults：定义 speech recognition 系统要不要返回临时结果(interim results)，还是只返回最终结果。
> - SpeechRecognition.maxAlternatives ：定义每次结果返回的可能匹配值的数量
> - recognition.continuous：是否等说完了再开始识别

```html
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8" />
<script>
 if (!('webkitSpeechRecognition' in window)) {
    console.log("not supported");
} else { 
    var recognition = new webkitSpeechRecognition();
    recognition.continuous = true;
    recognition.interimResults = true;
    
    recognition.lang = "en-US";
    recognition.maxAlternatives = 1;

    recognition.onstart = function() {
    	console.log('started');
    };
   
    recognition.onend = function() {
       console.log('ended');
    };

    recognition.onresult = function(event) {
       // no result
       if (typeof(event.results) === 'undefined') {
           recognition.stop();
	   console.log('no result');
           return;
       }
       
       // result
       for (var i = event.resultIndex; i < event.results.length; ++i) {
	    　　// 可以获得识别的信度　event.results[i][0].confidence
         　if (event.results[i].isFinal) {
	        　　console.log("final results: " + event.results[i][0].transcript);
	    　　} else {
	        　　console.log("interim results: " + event.results[i][0].transcript);
	    　　}
	　　}
    }

    recognition.onerror = function(event) {
    　　console.log("error");
	　　console.log(event);
    }

    function startButton(event) {
	　　recognition.start();
    }
}
</script>
</head>
<body>
  <button onclick="startButton(event);">start</button>
</body>
</html>
```

启动

```
# 在 speech_commands 是前面编写的页面，在 apache2 可以直接访问
$ sudo cp -r speech_commands /var/www/html

$ roslaunch turtlebot_gazebo turtlebot_world.launch
$ roslaunch rosbridge_server rosbridge_websocket.launch
```





https://blog.csdn.net/u010853356/article/details/79226764