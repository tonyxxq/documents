## ROS

- 结构

  - [Nodes](http://wiki.ros.org/Nodes):节点,一个节点即为一个可执行文件，它可以通过ROS与其它节点进行通信。
  - [Messages](http://wiki.ros.org/Messages):消息，消息是一种ROS数据类型，用于订阅或发布到一个话题。
  - [Topics](http://wiki.ros.org/Topics):话题,节点可以发布消息到话题，也可以订阅话题以接收消息。
  - [Master](http://wiki.ros.org/Master):节点管理器，ROS名称服务 (比如帮助节点找到彼此)。
  - [rosout](http://wiki.ros.org/rosout): ROS中相当于stdout/stderr。
  - [roscore](http://wiki.ros.org/roscore): Master + rosout + 参数服务器。

- 添加环境变量

  ```
  $ echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
  $ source ~/.bashrc
  
  # 查看环境变量是否配置正确
  $ export | grep ros
  
  # 程序首次编译（catkin_make）之后需要添加环境变量才能找到各个包的位置
  $ source devel/setup.sh
  ```

- 常用命令

  ```
  # 开启主要进程
  $ roscore
  
  # 查看所有作为全局变量包的路径
  echo $ROS_PACKAGE_PATH
  
  # 查找包信息 
  rospack find [包名称]
  
  # 切换到指定包目录（可以添加子目录）下
  roscd [本地包名称[/子目录]]
  
  # ros 日志文件的目录
  $ roscd log
  
  # rosls，列出指定包（或子目录）下的文件和文件夹
  rosls [本地包名称[/子目录]]
  
  # roscp 文件复制
  $ roscp rospy_tutorials AddTwoInts.srv srv/AddTwoInts.srv
  ```

- 创建工作空间

  ```
  $ mkdir -p ~/catkin_ws/src
  $ cd ~/catkin_ws/src
  $ catkin_init_workspace
  $ ls -l
  $ cd ~/catkin_ws
  
  # 默认会编译在当前目录下的 src 文件夹下的包，也可以指定源码的路径 catkin_make --source my_src
  $ catkin_make
  # 如果之前编译过，但是后来加入了新包，可以添加 --force-cmake 把新包加入到已编译过的二进制文件
  $ catkin_make --force-cmake
  
  # 安装
  catkin_make install 或 catkin_make install --source my_src
  
  $ ls
  ```

- 创建包

  > 创建包格式：
  >
  >  $ catkin_create_pkg <your_package_name> [dependency1 dependency2 …]
  >
  > 每个程序包下必须包含两个文件：CMakeLists.txt 和 package.xml， 且一个目录下只能有一个程序包，不能嵌套。

  ```
  $ cd ~/catkin_ws/src
  
  ＃ 创建包，且添加 std_msgs rospy roscpp 两个依赖
  $ catkin_create_pkg first_package std_msgs rospy roscpp
  
  # 一级依赖， 即上面创建包时填写的参数 std_msgs rospy roscpp，（一级依赖）
  $ rospack depends1 beginner_tutorials 
  
  # 所有依赖，会递归查询出 beginner_tutorials 及其依赖包的所有依赖
  $ rospack depends beginner_tutorials
  ```

  package.xml 文件：

  ```xml
  <!--该程序包的描述-->
  <description>The beginner_tutorials package</description>
  
  <!--维护者标签-->
  <maintainer email="user@todo.todo">user</maintainer>
  
  <!--许可-->
  <license>BSD</license>
  
  <!--构建工具-->
  <buildtool_depend>catkin</buildtool_depend>
  
  <!--编译依赖-->
  <build_depend>roscpp</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>
  
  <!--运行依赖-->
  <exec_depend>roscpp</exec_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  ```

  普通的包结构：

  - scripts (python executables)
  - src (C++ source files)
  - msg (for custom message definitions)
  - srv (for service message definitions)
  - include -> headers/libraries that are needed as dependencies
  - config -> configuration files
  - launch -> provide a more automated way of starting nodes
  - urdf (Universal Robot Description Files)
  - meshes (CAD files in .dae (Collada) or .stl (STereoLithography) format)
  - worlds (XML like files that are used for Gazebo simulation environments)

  或者使用开源包并编译

  ```
  $ cd ~/catkin_ws/src
  $ git clone https://github.com/udacity/simple_arm_01.git 
  $ cd ~/catkin_ws
  $ catkin_make
  ```

- 节点

  ```
  # 查看所有结点
  $ rosnode list
  
  # 查看特定节点
  $ rosnode info /rosout
  
  # 运行节点, 第一个参数是包名，第二个参数是结点名
  rosrun turtlesim turtlesim_node
  rosrun turtlesim turtle_teleop_key
  
  # 测试 节点是否连接通
  rosnode ping turtlesim_node
  
  # 清除所有节点
  rosnode cleanup
  ```

- 主题

  > 查看 rostopic -h：
  >
  > ```
  > rostopic bw     display bandwidth used by topic
  > rostopic echo   print messages to screen
  > rostopic hz     display publishing rate of topic
  > rostopic list   print information about active topics
  > rostopic pub    publish data to topic
  > rostopic type   print topic type
  > ```

  ```
  # 列出所有的主题
  rostopic list
  
  # 查看指定主题的消息类型
  rostopic type /turtlel/cmd_vel
  
  # 查询具体的某个主题
  rostopic info /turtlel/cmd_vel
  
  # 会自动新建一个节点，订阅指定话题，当有消息接收的时候，把数据打印到页面上
  rostopic echo /turtle1/command_velocity
  
  # 查看主题图
  rosrun rqt_graph rqt_graph
  
  # 查看主题传送的消息类型
  rostopic type /turtle1/command_velocity
  
  # 发送消息到指定主题, -1 表示发布消息后马上退出，--：表示接下来的是参数
  # 这条命令只发布了一条消息就退出了
  $ rostopic pub -1 /turtle1/command_velocity turtlesim/Velocity  -- 2.0  1.8
  
  # 发布稳定的消息流，且按照指定的频率， -r 1表示频率为 1， 即 1 秒发送一条消息
  $ rostopic pub /turtle1/command_velocity turtlesim/Velocity -r 1  -- 2.0  -1.8
  
  # 查看某个主题消息发布的频率
  $ rostopic hz /turtle1/pose
  ```

- rqt_plot

  > rqt_plot 命令可以实时显示一个发布到某个话题上的数据变化图形。

  ```
  $ rosrun rqt_plot rqt_plot
  ```

- 消息

  > 消息的类型
  >
  > int8, int16, int32, int64 (plus uint*)，float32, float64，string
  >
  > time, duration
  >
  > other msg files
  >
  > variable-length array[] and fixed-length array[C]
  >
  > rosmsg -h:
  >
  > ```
  > rosmsg show     Show message description
  > rosmsg users    Find files that use message
  > rosmsg md5      Display message md5sum
  > rosmsg package  列出指定包中的所有消息类型
  > rosmsg packages 列表包含消息的包名
  > ```

  ```
  # 创建消息类型，如下 Num.msg 只创建了一行数据，可以创建多行
  $ echo "int64 num" > msg/Num.msg行
  
  # 在 package.xml 中添加如下两行，为了确保 msg 文件被转换成为C++，Python和其他语言的源代码
  <build_depend>message_generation</build_depend>
  <run_depend>message_runtime</run_depend>
  
  # 在 CMakeLists.txt 文件中，利用find_packag函数，增加对message_generation的依赖
  find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation)
  
  # 添加运行依赖
  catkin_package(
    ...
    CATKIN_DEPENDS message_runtime ...
    ...)
    
  # 修改 add_message_files
  add_message_files(
    FILES
    Num.msg
  )
  
  # 查看消息的具体信息，前面的包名可以省略
  $ resmsg info geometry_msgs/Twist
  
  # 查看消息格式，前面的包名可以省略
  $ resmsg show geometry_msgs/Twist
  
  # 编辑消息格式
  $ rosed geometry_msgs Twist.msg
  ```

- Roslaunch

  > 之前的结点是通过 ros 命令启动结点，但是 ROS 结点一般会很多，如果手动运行所有结点会非常麻烦。所以就出现了 Roslaunch。
  >
  > roslaunch功能：
  >
  > 1. 启动 master 和 nodes 
  > 2. 设置默认参数
  > 3. 自动关闭服务已经死掉的进程

  使用 roslaunch 之前需要先确保已经编译过工作空间，且刷新了 steup.bash

  ```
  $ cd ~/catkin_ws
  $ catkin_make
  $ source devel/setup.bash
  $ roslaunch simple_arm robot_spawn.launch
  ```

  > Roslaunch 配置文件的标签
  >
  > ####node 标签
  >
  > 示例：
  >
  > ```<node name =“listener1” pkg =“rospy_tutorials” type =“listener.py” args =“ -  test” respawn =“true”/>```
  >
  > 必选项：
  >
  > - `pkg = "pkg" ` 包名
  >
  > - `type = “type” ` 节点类型，**必须有一个具有相同名称的相应可执行文件**
  >
  > - `name = “nodename”`，节点基名称。**注意：name 不能包含命名空间。请改用`ns`属性**
  >
  > 可选项：
  >
  > - `args = “arg1 arg2 arg3” `，传递参数到节点，示例：args="-resolution  $(arg resolution)"，在代码的main 方法上应该能取到该值
  >
  > - `machine = “machine-name” `在指定机器上启动节点
  > - `respawn = “true” ` 如果节点退出，则自动重新启动节点
  >
  > - `respawn_delay =“30” `*（默认为 0）*如果`respawn`为`true`，等到指定秒再重启
  >
  > - `required = “true” ` 如果节点死亡，杀死整个 roslaunch
  >
  > - `ns = “foo” `在指定命名空间中启动节点
  >
  > - `clear_params = “true | false” `在启动前删除节点的私有命名空间中的所有参数
  >
  > - `output =“log | screen” ` 如果'screen'，stdout / stderr 从节点将被发送到屏幕；如果是 “log”，stdout / stderr 输出将被发送到$ ROS_HOME/ log 中的日志文件，stderr 将继续发送到屏幕。默认值为  “log”
  >
  > - `cwd = “ROS_HOME | node” ` 如果为 “node”，则节点的工作目录将设置为与节点的可执行文件相同的目录
  >
  > - `launch-prefix = “prefix arguments” ` 用于预先添加到节点的启动参数的命令/参数。这是一个强大的功能，使您能够启用`gdb`，`valgrind`，`xterm`，`漂亮`或其他方便的工具
  >
  > ####rosparam
  >
  > 可从 yaml 文件读取参数
  >
  > 示例：
  >
  > <rosparam command="load" file="$(find rosparam)/example.yaml" />
  > <rosparam command="delete" param="my/param" />
  >
  > #### param
  >
  > 可用于直接设置参数
  >
  > 示例
  >
  > <param name="pixel_format" value="yuyv"/>
  >
  > ####include
  >
  > 导入别的 launch 文件
  >
  > <include file="$(find face_tracker_pkg)/launch/start_usb_cam.launch"/>
  >
  > #### arg 
  >
  > 用于给launch文件内部设置参数
  >
  > included.launch 文件
  >
  > ```xml
  > <launch>
  >   	<!--定义该参数是传进来的，也可以定义 value，但是 default 可以被覆盖，value 不能-->
  >   	<arg name="hoge" default="default"/> 
  >   
  >     <!--使用该参数-->
  >     <param name="param" value="$(arg hoge)"/>
  > </launch>
  > ```
  >
  > my_file.launch 文件
  >
  > ```xml
  > <include file="included.launch">
  >     <!--传递参数-->
  >     <arg name="hoge" value="fuga" />
  > </include>
  > ```
  >
  > 也可以通过命令行传递参数
  >
  > ```
  > $ roslaunch my_file.launch hoge:=my_value 
  > ```
  >
  > #### remap
  >
  > 给结点设置重新映射
  >
  > ```xml
  > <!--将当且节点话题 chatter 映射成 hello-->
  > <remap from="chatter" to="hello"/>
  > ```

- 查看和安装依赖（Rosdep）

  > Rosdep 检查或安装包中缺失的依赖

  ```
  # 检查包中缺失的依赖
  rosdep check simple_arm
  
  # 安装包中缺失的依赖
  rosdep install -i simple_arm
  
  # 或使用 apt-get 安装
  sudo apt-get install ros-kinetic-gazebo-ros-control
  ```

- ROS 服务

  > rosservice  -h:
  >
  > rosservice  list           输出可用服务的信息
  > rosservice  call          调用带参数的服务
  > rosservice  type        输出服务类型
  > rosservice  find         依据类型寻找服务
  > rosservice  uri           输出服务的 ROSRPC uri

  srv文件夹下保存是服务的类型分为请求和响应两部分，由'---'分隔。下面是srv的一个样例：

  ```
  int64 A
  int64 B
  ---
  int64 Sum
  ```

  ```
  # 查看服务类型，并输出相信信息
  $ rosservice type spawn| rossrv show
  ```

  创建服务：

  ```
  # 创建或复制一个文件
  $ roscp rospy_tutorials AddTwoInts.srv srv/AddTwoInts.srv
  
  # 其他跟 message 一致修改package.xml 和 CmakeLists.txt（不一样的是在 add_service_files 下添加文件名称）
  
  # 查看服务类型，可以不指定包名
  $ rossrv show beginner_tutorials/AddTwoInts
  $ rossrv info beginner_tutorials/AddTwoInts
  
  # 修改文件
  $ rosed beginner_tutorials AddTwoInts.srv
  ```

- rosparam 可以对参数服务器上的参数进行操作，有以下几种方式

  > 1. 命令行
  >
  >    ```
  >    rosparam  -h：
  >    rosparam  set     [参数名]     [参数值]    设置参数
  >    rosparam  get     [参数名]                获取参数
  >    rosparam  load    [文件地址]              从文件读取参数，
  >    rosparam  dump    [文件地址]              向文件中写入参数
  >    rosparam  delete  [参数名]                删除参数
  >    rosparam  list                           列出参数名
  >    ```
  >
  > 2. 配置文件
  >
  >     roslaunch 配置文件中 node 标签中添加，参考上面讲解的 roslaunch
  >
  > 3. 代码
  >
  >    *param  和  getParam 都能获取参数值，区别是 param 可以设置默认值*
  >
  >    ```c++
  >    ros::init(argc, argv, "param_demo");
  >    
  >    // n 使用的是全局空间，pn 使用的是 ~my_namespce 空间
  >    // 所以 “string_param” 是全局的参数，“int_param” 是在命名空间 my_namespace下的参数
  >    ros::NodeHandle n;
  >    ros::NodeHandle pn("~my_namespce");
  >    
  >    td::string s;
  >    int num;
  >    
  >    // 初始化参数值
  >    n.param<std::string>("string_param", s, "haha");
  >    pn.param<int>("int_param", num, 666);
  >    
  >    // 输出被初始化后的变量值
  >    ROS_INFO("string_param_init: %s", s.c_str());
  >    ROS_INFO("int_param_init: %d", num);
  >    
  >    // 设置参数的值
  >    n.setParam("string_param", "hehe");
  >    pn.setParam("int_param", 222);
  >        
  >    //获取参数的值；
  >    n.getParam("string_param", s);
  >    pn.getParam("int_param", num)；
  >    ```

- rqt_console 和 rqt_logger_level

  > `rqt_console`属于ROS日志框架(logging framework)的一部分，用来显示节点的输出信息。`rqt_logger_level`允许我们修改节点运行时输出信息的日志等级（logger levels）（包括 DEBUG、WARN、INFO和ERROR）。

  ```
  # 启动 rqt_console
  $ rosrun rqt_console rqt_console
  
  # 启动 rqt_logger_lev
  $ rosrun rqt_logger_level rqt_logger_level
  ```

- 发布和订阅消息例子

  发布者：

  ```python
  #!/usr/bin/env python
  # license removed for brevity
  import rospy
  from std_msgs.msg import String
  
  def talker():
      # 创建发布者
      pub = rospy.Publisher('chatter', String, queue_size=10)
      # 创建节点，且节点名称是惟一的
      rospy.init_node('talker', anonymous=True)
      # 输出频率
      rate = rospy.Rate(10) # 10hz
      while not rospy.is_shutdown():
          hello_str = "hello world %s" % rospy.get_time()
          # 输出到日志文件 /rosout 和 屏幕
          # 定义复杂的消息类型
          # msg = String()
  		# msg.data = str
          # 或 String(data=str)
          # 顺序和消息类型定义的一致
          rospy.loginfo(hello_str)
          # 发布消息
          pub.publish(hello_str)
          # 根据频率进行睡眠
          rate.sleep()
  
  if __name__ == '__main__':
      try:
          talker()
      except rospy.ROSInterruptException:
          pass
  ```

  ```
  # 添加执行权限
  $ chmod +x talker.py
  ```

  订阅者：

  ```python
  #!/usr/bin/env python
  import rospy
  from std_msgs.msg import String
  
  def callback(data):
      # 输出消息到屏幕 和 rosout 
      rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
      
  def listener():
      # In ROS, nodes are uniquely named. If two nodes with the same
      # node are launched, the previous one is kicked off. The
      # anonymous=True flag means that rospy will choose a unique
      # name for our 'listener' node so that multiple listeners can
      # run simultaneously.
      # 注册 listener 节点
      rospy.init_node('listener', anonymous=True)
  
  	# 订阅 chatter 节点
      rospy.Subscriber("chatter", String, callback)
  
      # spin() simply keeps python from exiting until this node is stopped
      rospy.spin()
  
  if __name__ == '__main__':
      listener()
  ```

  ```
  # 添加执行权限
  $ chmod +x listener.py
  ```

  测试：

  ```
  $ catkin_make
  $ rosrun beginner_tutorials talker.py
  $ rosrun beginner_tutorials listener.py
  ```

- 服务例子

  服务器：

  ```python
  #!/usr/bin/env python
  
  from beginner_tutorials.srv import *
  import rospy
  
  def handle_add_two_ints(req):
      print "Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b))
      return AddTwoIntsResponse(req.a + req.b)
  
  def add_two_ints_server():
      rospy.init_node('add_two_ints_server')
      s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
      print "Ready to add two ints."
      rospy.spin()
  
  if __name__ == "__main__":
      add_two_ints_server()
  ```

  客户端：

  ```python
  #!/usr/bin/env python
  
  import sys
  import rospy
  from beginner_tutorials.srv import *
  
  def add_two_ints_client(x, y):
      rospy.wait_for_service('add_two_ints')
      try:
          add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
          resp1 = add_two_ints(x, y)
          return resp1.sum
      except rospy.ServiceException, e:
          print "Service call failed: %s"%e
  
  def usage():
      return "%s [x y]"%sys.argv[0]
  
  if __name__ == "__main__":
      if len(sys.argv) == 3:
          x = int(sys.argv[1])
          y = int(sys.argv[2])
      else:
          print usage()
          sys.exit(1)
      print "Requesting %s+%s"%(x, y)
      print "%s + %s = %s"%(x, y, add_two_ints_client(x, y))
  ```

- roswtf

  > [roswtf](http://wiki.ros.org/roswtf) 可以检查 ROS 系统并尝试发现问题

  ```
  $ roswtf
  ```

- 录制与回放数据

  ```
  $ mkdir ~/bagfiles
  $ cd ~/bagfiles
  
  # -a 表示所有话题数据都录制
  $ rosbag record -a
  
  # 查看录制数据的描述信息
  $ rosbag info <your bagfile>
  
  # 回放， -r 2 表示以两倍速率回放
  $ rosbag play -r 2 <your bagfile>
  
  # 录制数据子集 -O subset 表示录制数据保存到 subset.bag 文件中 /turtle1/command_velocity 
  # /turtle1/pose 表示要录制的话题
  $ rosbag record -O subset /turtle1/command_velocity /turtle1/pose
  ```



参考：http://wiki.ros.org

​	   http://docs.ros.org/kinetic/api/rospy/html/

​	   http://wiki.ros.org/rospy_tutorials/Tutorials/WritingPublisherSubscriber