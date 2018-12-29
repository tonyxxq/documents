## 人脸检测

1. 结构图

   ![](imgs/2.png)

   2. 创建 ROS 包

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

   3. 编写 face_tracker_pkg

      ![](imgs/3.png)

   包结构:

   ![](imgs/4.png)

   ​

   ​

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
/objects
```

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



