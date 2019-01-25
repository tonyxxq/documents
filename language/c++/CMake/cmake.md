#####常用命令

```cmake
# cmake 最小版本需求
cmake_minimum_required(VERSION xxx) 

# 设置项目名称
project("project name") 

# 设置变量 var_name 的值为 var_value
set(var_name var_value)

# 设置 c++ 编译器，-std=c++11 包含 C++11 特性，如果是 CMAKE_C_FLAGS 就是 C 编译器
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")	

# 类似 echo 打印消息
message(STATUS, "MSG")

# option 提供选项让用户选择是 ON 或者 OFF，如果没有提供初始化值，使用OFF
option(var_name "help string describing option" ON)

# 调用系统环境变量, $ENV{NAME} ，使用 set 可以设置值,比如：
SET($ENV{变量名} 值)

# 找到后面需要库和头文件的包
find_package(OpenCV 2.4.3 REQUIRED)

# 把 src 目录下的所有源文件名赋值给 DIR_SRCS
aux_source_directory(src DIR_SRCS)

# 添加 include 路径
include_directories(xxx) 

# 添加库路径
link_directories(xxx)

# 调用 xxx 子目录的 CMakeLists.txt 并执行
add_subdirectory(xxx) 

# 生成可执行文件 target，后面填写的是生成此可执行文件所依赖的源文件列表
add_executable(target target_source_codes) 

# 和 add_executable 类似，生成库文件，SHARED 代表动态库，STATIC 代表静态库，最后一个参数代表此库的# 源文件列表
add_library(lib_name SHARED lib_source_code)

# 给目标添加依赖库
target_link_libraries(target_name lib_name ...)
```

#####预定义的一些变量

```cmake
# 工程的根目录
PROJECT_SOURCE_DIR 

# 返回通过 PROJECT 指令定义的项目名称
PROJECT_NAME

# 编译类型，可以设置为 DEBUG 和 RELEASE, DEBUG 模式可以使用 GDB 进行调试 
CMAKE_BUILE_TYPE 

# 运行 cmake 命令的目录, 通常是 ${PROJECT_SOURCE_DIR}/build
PROJECT_BINARY_DIR 

# 环境变量，非 cmake 变量
CMAKE_INCLUDE_PATH

# 环境变量
CMAKE_LIBRARY_PATH

# 当前 CMakeLists.txt 所在的路径
CMAKE_CURRENT_SOURCE_DIR

# 编译目录
CMAKE_CURRENT_BINARY_DIR

# 目标二进制可执行文件的存放位置
EXECUTABLE_OUTPUT_PATH 

# 链接库文件的存放位置
LIBRARY_OUTPUT_PATH 

# 系統全名，如 “Linux-2.4.22″，”FreeBSD-5.4-RELEASE” 或 “Windows 5.1
CMAKE_SYSTEM

# 系統名称，如 “Linux”, “FreeBSD” or “Windows”，注意大小写
CMAKE_SYSTEM_NAME

# 只显示系统全名中的版本部分
CMAKE_SYSTEM_VERSION

# CPU 名称
CMAKE_SYSTEM_PROCESSOR
```

#####对条件判断的理解：

> 注意：后面必须跟 endif(expression)

表达式的使用方法如下:

```cmake
# 如果变量不是：空, 0, N, NO, OFF, FALSE, NOTFOUND 或 <var>_NOTFOUND时，表达式为真
# 比如定义 WIN32 没有定义的时候为假，可以用来判断系统
if(var)

# 与上述条件相反
if(NOT var)

# 当两个变量都为真是为真
if(var1	AND var2)

# 当两个变量其中一个为真时为真。
if(var1	OR var2)

# 当给定的 cmd 确实是命令并可以调用是为真
if(COMM	AND	cmd)

# 当目录或者文件存在时为真
if(EXISTS dir) or IF(EXISTS file)

# 当 file1 比 file2 新时为真，文件名请使用完整路径
if(file1 IS_NEWER_THAN file2)

# 当 dirname 是目录时为真
if(IS_DIRECTORY dirname)

# 当给定的变量或者字符串能够匹配正则表达式regex时为真
if(string MATCHES regex)

##### 例如 #####
if("hello" MATCHES "hello")
    message("true")
endif("hello" MATCHES "hello")
###############

# 数字比较
if(variable LESS number)
if(variable GREATER number)
if(variable EQUAL number)

# 字母排序进行比较
if(variable STRLESS string)
if(variable STRGREATER string)
if(variable STREQUAL string)

# 判断是哪个平台执行不同的操作
if(WIN32)
   #dosomething related to WIN32
elseif(UNIX)
   #dosomething related to UNIX
elseif(APPLE)
   #dosomething related to APPLE
endif(WIN32)
```

##### 对 while 的理解

```cmake
while(condition)
    …
endwhile(condition)
```

#####对 foreach 的理解

三种方式

1. 列表

   例子：

   ```cmake
   AUX_SOURCE_DIRECTORY(. SRC_LIST)
   FOREACH(F ${SRC_LIST})
       MESSAGE(${F})
   ENDFOREACH(F)
   ```

2. 范围

   例子：从 0 开始到 10 结束，步长为 1

   ```cmake
   
   FOREACH(VAR RANGE 10)
       MESSAGE(${VAR})
   ENDFOREACH(VAR)
   ```

3. 范围和步长

   例子：从 5  开始 15 结束，步长为 3

   ```cmake
   FOREACH(A RANGE 5 15 3)
       MESSAGE(${A})
   ENDFOREACH(A)
   ```

#####对find_package 的理解：

我们使用第三方库时需要引用别的包，比如 Sophus，可能会用到像下面代码 ：

```cmake
# 查找 Sophus 包
find_package(Sophus REQUIRED)

# 添加头文件目录
include_directories(${Sophus_INCLUDE_DIRS})

# 添加可执行文件
add_executable(useSophus useSophus.cpp)　　

# 添加库文件目录
target_link_libraries(useSophus ${Sophus_LIBRARIES})
```

在执行 find_package 之后，如找到，就会提供头文件和库文件所在目录的变量，比如上边的：Sophus_INCLUDE_DIRS 和 Sophus_LIBRARIES

find_package 查找有两种方式，下面的 XXX 代表包名：

- Module 模式：搜索 CMAKE_MODULE_PATH 指定路径下的 FindXXX.cmake 文件，执行该文件从而找到 XXX 库。

- Config 模式：搜索 XXX_DIR 指定路径下的 XXXConfig.cmake 文件，执行该文件从而找到 XXX 库。

所以经常看见像下面的代码

```cmake
# 添加 OpenCVConfig.cmake 的搜索路径
set(OpenCV_DIR  ~/ssd/software/opencv3.3.1/build)
find_package( OpenCV 3 REQUIRED )
```
#####对 File 指令的理解

```cmake
file(WRITEfilename "message to write"... )
file(APPENDfilename "message to write"... )
file(READfilename variable)
file(GLOBvariable [RELATIVE path] [globbing expression_r_rs]...)
file(GLOB_RECURSEvariable [RELATIVE path] [globbing expression_r_rs]...)
file(REMOVE[directory]...)
file(REMOVE_RECURSE[directory]...)
file(MAKE_DIRECTORY[directory]...)
file(RELATIVE_PATHvariable directory file)
file(TO_CMAKE_PATHpath result)
file(TO_NATIVE_PATHpath result)
```

#####对 find 指令的理解

```cmake
FIND_FILE(<VAR> name1 path1 path2 …)   # VAR 变量代表找到的文件全路径,包含文件名
FIND_LIBRARY(<VAR> name1 path1 path2 …)# VAR 变量表示找到的库全路径,包含库文件名
FIND_PATH(<VAR> name1 path1 path2 …)   # VAR 变量代表包含这个文件的路径
FIND_PROGRAM(<VAR> name1 path1 path2 …)# VAR 变量代表包含这个程序的全路径
FIND_PACKAGE(<name>[major.minor] [QUIET] [NO_MODULE] [[REQUIRED|COMPONENTS][componets...]]) # 用来调用预定义在CMAKE_MODULE_PATH下的Find<name>.cmake模块，也可以自己定义Find<name>模块，通过SET(CMAKE_MODULE_PATH dir)将其放入工程的某个目录中供工程使用
```

例子：

```cmake
FIND_LIBRARY(libX X11 /usr/lib)
IF(NOT libX)
    MESSAGE(FATAL_ERROR "libX not found")
ENDIF(NOT libX)
```







参考：https://www.hahack.com/codes/cmake/





https://github.com/askme765cs/Wine-QQ-TIM