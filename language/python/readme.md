- map 函数 

  接收一个函数和一个或多个 list 返回一个 list (没有去掉元素)

  ```python
  # 输出 ['1', '2', '3', '4', '5', '6', '7', '8', '9']
  list(map(str, [1, 2, 3, 4, 5, 6, 7, 8, 9]))
  
  # 输出 [1, 2, 3, 4, 5]，字符串的每个字符当做一个元素处理
  list(map(lambda x: int(x), "12345"))
  ```

- filter函数

  接收一个函数和一个或多个 list 返回一个 list (去掉不满足条件的元素)

- reduce 函数

  ```python
  # python3 中需要导入 reduce
  from functools import reduce
  
  def fn(x, y):
      return 10 * x + y
  
  t = reduce(fn, [1, 2, 3, 4])
  
  print(t)
  ```

- capitalize  把字符串的第一个字符大写，其它字符都小写

  ```python
  # 输出 Hello world
  print("hello world".capitalize())
  ```

- random.choice 方法返回一个列表，元组或字符串的随机项。

  ```python
  import random
  
  print(random.choice([1, 2, 3]))
  print(random.choice('A String'))
  ```

- python 包，目录下有\_\_init\_\_.py文件

  \_\_init\_\_.py文件的作用

  1. python 中 package 的标识，不能删除

  2. 在内部定义 \_\_all\_\_ 用来模糊导入 (import *)

     ```python
     __all__ = ["Pack1Class", "Pack1Class1"]
     ```

- python 模块是一个 Python 文件，以 .py 结尾

- help() 函数    

  ```python
  help(max)
  ```

- dir() 函数（可带参数不带参数）

  ```python
  # 不到参数返回当前范围内的所有变量
  # 下方输出 ['__builtins__', '__cached__', '__doc__', '__file__', '__loader__', 
  # '__name__', '__package__', '__spec__']
  print(dir())
  
  # 带参数返回当前对象的所有的方法和属性列表，参数可以是 对象、变量、类型
  # z 下方输出 ['__add__', '__class__', '__contains__', '__delattr__', '__delitem__', #'__delslice__', '__doc__', '__eq__', '__format__', '__ge__', '__getattribute__', #'__getitem__', '__getslice__', '__gt__', '__hash__', '__iadd__', '__imul__', #'__init__', '__iter__', '__le__', '__len__', '__lt__', '__mul__', '__ne__', #'__new__', '__reduce__', '__reduce_ex__', '__repr__', '__reversed__', '__rmul__', #'__setattr__', '__setitem__', '__setslice__', '__sizeof__', '__str__', #'__subclasshook__', 'append', 'count', 'extend', 'index', 'insert', 'pop', 'remove', #'reverse', 'sort']
  l = [1, 2, 3, 4, 5]
  print(dir(l))
  ```

- \_\_doc\_\_函数，查看模块的文档

  ```python
  print(int.__doc__)
  ```

- \_\_file\_\_ 输出文件的所在路径

  ```python
  import random
  
  # 输出 C:\ProgramData\Anaconda2\envs\carnd-term1\lib\random.py
  print(random.__file__)
  ```

- environ，获取当前系统的环境变量

  ```python
  import os
  
  # 输出 F:\software\jdk1.8.0_45\jdk1.8.0_45
  print(os.environ['JAVA_HOME'])
  ```

- globals() 和 locals()

  >python 的全局名字空间存储在一个叫 globals() 的 dict 对象中；局部名字空间存储在一个叫 locals()的dict 对象中。全局名字空间：在模块内部，局部名字空间：在类或函数内部

  ```python
  # 查看所有全局和局部变量
  print(globals())
  print(locals())
  
  # 查看指定名称的全局和局部变量
  print(globals()["name"])
  print(locals()["name2"])
  ```

- eval 执行一个字符串表达式，并返回表达式的值

  > 这个方法比较好用，tensorflow 里边用得比较多
  >
  > 同时也可以把字符串转换为列表、字典、元组
  >
  > 参数 globals 表示全局变量搜索变量
  >
  > 参数 locals 表示在局部搜索变量

  ```python
  n = 81
  print(eval("n + 4"))
  
  # 输出 3
  print(eval("x + y", {'x': 1, 'y': 2}))
  
  # 字符串转换成列表，输出 [1,2]
  a = "[[1,2], [3,4], [5,6], [7,8], [9,0]]"
  b = eval(a)
  print(b[0])
  
  # 字符串转换成字典，输出 a
  a = "{1: 'a', 2: 'b'}"
  b = eval(a)
  print(b[1])
  
  # 字符串转换成元组，输出 [1,2]
  a = "([1,2], [3,4], [5,6], [7,8], (9,0))"
  b = eval(a)
  print(b[0])
  
  # 优先使用全局变量
  print(eval("x+y", globals()))
  ```

- 把变量放置到 scope （globals, loacals）作用域内

  ```python
  x = 1
  exec("x=3", globals())
  # 输出 3
  print(x)
  ```

- pip 安装时可以设置默认的时间大一点

   ```
  pip --default-timeout=100 install -U scikit-learn
   ```

- \_\_repr\_\_和\_\_str\_\_这两个方法都是用于显示的，\_\_str\_\_是面向用户的，而\_\_repr\_\_面向程序员

  > 类似 java 中的 toString
  >
  > \_\_repr\_\_ 和 \_\_str\_\_的不同是在命令行可以不用输入 print，直接输入对象名就能打印对象内容

  ```python
  class Car:
      def __init__(self):
          self.name = "jiaoche"
          self.age = 10
  
      def __str__(self):
          return self.name + str(self.age)
  
      def __repr__(self):
          return self.name + str(self.age)
  
  car = Car()
  
  # 输出 jiaoche10
  print(car)
  ```

- 生成迭代器的几种方式

  > 迭代器在处理数据量比较大的迭代的时候比较不好用，不会一次性把数据加入内存
  >
  > 注意：列表、自定、元组等他们虽然可迭代但不是迭代器
  >
  > 迭代器对象必须有 \_\_iter\_\_( )方法和 \_\_next\_\_( )方法
  >
  > 对象只要有 \_\_iter\_\_( ) 或 \_\_getitem\_\_ 方法就是可迭代的

  1. 类方式

     ```python
     class Data:
         def __init__(self, data):
             self.data = data
     
         # 只要存在 __iter__ 就表示可迭代的
         def __iter__(self):
             return self
     	
         # 迭代返回值，必须抛出 StopIteration 异常
         def __next__(self):
             if self.data > 2:
                 raise StopIteration
             else:
                 self.data += 1
                 return self.data
     
     data = Data(1)
     # 输出 
     # 2
     # 3
     for i in data:
         print(i)
     ```

  2. 生成器函数

     >与普通的函数的区别是多了一个或多个 yield
     >
     >普通函数在执行之后直接退出当前函数，但是生成器函数执行之后并没有退出，生成器函数其实是返回的一个对象

     ```python
     def xx():
         for i in range(100):
             yield i
     
     for i in xx():
         print(i)
     ```

- 判断当前文件夹下的文件或文件夹是否存在

  ```python
  import os
  
  # 判断文件（文件夹）是否存在
  os.path.exists("test_file.txt")
  
  # 判断文件是否存在，不包括文件夹
  os.path.isfile("test-data")
  
  # 使用 try except 语句,判断文件文件是否存在，不包括文件夹子
  # 但是当 open 的第二个参数包含 a 或 w 时会自动创建文件不会抛出异常
  try:
      f =open("file.txt")
      f.close()
  except IOError:
      print（"File is not accessible."）
  
  # 使用 pathlib
  import pathlib
  
  path = pathlib.Path("path/file")
  path.exists()
  path.is_file()
  ```

- 写文件

  ```python
  """
  'r'：读
  'w'：写
  'a'：追加
  'r+' == r+w（可读可写，文件若不存在就报错(IOError)）
  'w+' == w+r（可读可写，文件若不存在就创建）
  'a+' == a+r（可追加可写，文件若不存在就创建）
  对应的，如果是二进制文件，就都加一个b就好啦：
  'rb' 'wb' 'ab' 'rb+' 'wb+' 'ab+'
  """
  
  file = r'D:\test.txt'
  with open(file, 'w+') as f:
      f.write(mobile)
  ```

- 读取文件

  ```python
  # read 按字节读取,参数为字节数，不传表示读取所有
  f = open('go.txt')
      print(f.read())
      f.close()
  
  # readline
  file = open("sample.txt") 
  while 1:
    line = file.readline()
    if not line:
      break
    pass # do something
  file.close()
  
  # readlines， 读取文件的所有行放到列表中
  file = open("sample.txt") 
  while 1:
    lines = file.readlines()
    if not lines:
      break
    for line in lines:
      pass # do something
  file.close()
  
  # 迭代器，当读取大文件的时候按这种方式
  file = open("sample.txt") 
  for line in file:
    pass # do something
  file.close()
  ```

- 字符串去掉空格的方法

  ```
  strip()：把头和尾的空格去掉
  lstrip()：把左边的空格去掉
  rstrip()：把右边的空格去掉
  replace(‘c1’, ‘c2’)：把字符串里的 c1 替换成 c2。故可以用replace(’ ‘,’’)来去掉字符串里的所有空格
  split()：通过指定分隔符对字符串进行切片，如果参数 num 有指定值，则仅分隔 num 个子字符串
  ```

- 定时任务

  ```python
  from threading import Timer
  
  def printHello(): 
    # 每隔两秒打印一下 hello world，这也是个递归调用
    t = Timer(2, printHello) 
    t.start() 
    print("Hello World")
   
  if __name__ == "__main__": 
    printHello() 
  ```

- time

  ```python
  # 当前时间戳
  t = time.time()
  
  # 返回当前时间
  time.ctime()
  
  # localtime 获取本地时间，可以转为 struct_time
  t = time.localtime()
  
  # 可以代码获取年、月、日等信息
  t = time.struct_time(t)
  
  # 时间格式化， struct_time 作为参数
  t = time.strftime("%Y%m%d", t)
  
  # 时间字符串转为 struct_time
  t = time.strptime("19年1月1日", "%y年%m月%d日")
  
  # struct_time 转为时间戳
  t = time.mktime(t)
  
  # 返回系统运行时间，在测试程序执行时间时比较有用
  time.perf_counter()
  ```

  各种时间之间的转换关系，可以看出 struct_time 是比较重要的时间格式，作为纽带存在

  ![](imgs/1.png)

- Python 中 \*args 和 \*\*kwargs 的区别

  > \*args 用来将参数打包成 tuple 给函数体调用

  ```python
  def function(x, y, *args):
      print(x, y, args)
  
  function(1, 2, 3, 4, 5)
  
  # 输出 1 2 (3, 4, 5)
  ```

  >  \*\*kwargs 打包关键字参数成 dict 给函数体调用

  ```python
  def function(**kwargs):
      print(kwargs)
  
  function(a=1, b=2, c=3)
  # 输出 {'b': 2, 'a': 1, 'c': 3}
  ```

  **注意： 参数arg、\*args、\*\*kwargs三个参数的位置必须是一定的。必须是(arg, *args, \*\*kwargs)这个顺序，否则程序会报错。**

- 多线程

  ```python
  import time
  import threading
  
  def say_hello(xx2, *name, **xx):
      for i in range(10):
          print(xx2)
          print(name, "hello")
          print(xx['a'])
          time.sleep(1)
  
  
  def say_yes(name):
      for i in range(10):
          print(name, "yes")
          time.sleep(1)
  
  t1 = threading.Thread(target=say_hello, args=("xiaoqiang", "xiaoqing2"), kwargs={'a': 1})
  t2 = threading.Thread(target=say_yes, args=("xiaoming",))
  
  t1.start()
  t2.start()
  ```