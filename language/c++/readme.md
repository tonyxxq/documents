## c++ 技术文档



- 简介：C++ 是一种静态类型的、编译式的、通用的、大小写敏感的、不规则的编程语言，支持过程化编程、面向对象编程和泛型编程。

- 安装c++

- c++ 预处理器

  > 预处理器是一些指令，指示编译器在实际编译之前所需完成的预处理。

  \#define

  预处理指令用于创建符号常量，该符号常量通常称为**宏**

  ```c++
  #include <iostream>
  using namespace std;
   
  #define PI 3.14159
   
  int main (){
      // PI 被替换为 3.14159
      cout << "Value of PI :" << PI << endl; 
      return 0;
  }
  ```

  带有参数的宏

  ```c++
  #include <iostream>
  using namespace std;
  
  // 下面所有调用的 MIN(a,b) 都将被替换为 (a<b ? a : b) ,其中a，b 是参数
  #define MIN(a,b) (a<b ? a : b)
   
  int main ()
  {
     int i, j;
     i = 100;
     j = 30;
     cout <<"较小的值为：" << endl;
   
      return 0;
  }
  ```

  条件编译

  ```c++
  // 结构
  #ifndef NULL
     #define NULL 0
  #endif
  
  // 如果 DEBUG 在前面进行了定义，执行中间的代码
  #ifdef DEBUG
     cerr <<"Variable x = " << x << endl;
  #endif
  
  // 中间的代码不执行, 可以使用此方法注释代码
  #if 0
     不进行编译的代码
  #endif
  ```

  \# 和 \#\# 运算符

  \# : 运算符会把 replacement-text 转换为用引号引起来的字符串。 

  ```c++
  #include <iostream>
  using namespace std;
   
  #define MKSTR( x ) #x
   
  int main (){
      cout << MKSTR(HELLO C++) << endl;
      return 0;
  }
  
  // 输出 HELLO C++
  ```

  \#\#：运算符用于连接两个令牌

  ```c++
  #include <iostream>
  using namespace std;
  
  #define concat(a, b) a ## b
  int main() {
     int xy = 100;
     cout << concat(x, y);
     return 0;
  }
  // cout << concat(x, y); 转换为了 cout << xy;
  ```

  预定义宏：

  \_\_LINE__	：这会在程序编译时包含当前行号。
  \_\_FILE__	：这会在程序编译时包含当前文件名。
  \_\_DATE__ ：这会包含一个形式为 month/day/year 的字符串，它表示把源文件转换为目标代码的日期。
  \_\_TIME__ ：这会包含一个形式为 hour:minute:second 的字符串，它表示程序被编译的时间。

  ```c++
  #include <iostream>
  using namespace std;
   
  int main ()
  {
     cout << __LINE__ << endl;
     cout << __DATE__ << endl;
     cout << __TIME__<<  endl;
     cout << __FILE__<<  endl;
  }
  ```

- 模板

  > 在模板中typename 和 class 是等价的

  1. 函数模板

     ```c++
     #include <iostream>
     #include <string>
      
     using namespace std;
     
     // typename 定义变量的类型
     template <typename T1, typename T2>
     inline T1 const& Max (T1 const& a, T2 const& b) 
     { 
         return a < b ? b:a; 
     } 
     
     int main ()
     {
      
         int i = 39;
         int j = 20;
         cout << "Max(i, j): " << Max(i, j) << endl; 
      
         double f1 = 13.5; 
         double f2 = 20.7; 
         cout << "Max(f1, f2): " << Max(f1, f2) << endl; 
      
         string s1 = "Hello"; 
         string s2 = "World"; 
         cout << "Max(s1, s2): " << Max(s1, s2) << endl; 
      
        return 0;
     }
     ```

  2. 类模板

     ```c++
     #include <iostream>
     #include <vector>
     #include <cstdlib>
     #include <string>
     #include <stdexcept>
      
     using namespace std;
      
     template <class T>
     class Stack { 
       private: 
         vector<T> elems;     // 元素 
      
       public: 
         void push(T const&);  // 入栈
         void pop();               // 出栈
         T top() const;            // 返回栈顶元素
         bool empty() const{       // 如果为空则返回真。
             return elems.empty(); 
         } 
     }; 
      
     template <class T>
     void Stack<T>::push (T const& elem) { 
         // 追加传入元素的副本
         elems.push_back(elem);    
     } 
      
     template <class T>
     void Stack<T>::pop () { 
         if (elems.empty()) { 
             throw out_of_range("Stack<>::pop(): empty stack"); 
         }
         // 删除最后一个元素
         elems.pop_back();         
     } 
      
     template <class T>
     T Stack<T>::top () const { 
         if (elems.empty()) { 
             throw out_of_range("Stack<>::top(): empty stack"); 
         }
         // 返回最后一个元素的副本 
         return elems.back();      
     } 
      
     int main() { 
         try { 
             Stack<int>         intStack;  // int 类型的栈 
             Stack<string> stringStack;    // string 类型的栈 
      
             // 操作 int 类型的栈 
             intStack.push(7); 
             cout << intStack.top() <<endl; 
      
             // 操作 string 类型的栈 
             stringStack.push("hello"); 
             cout << stringStack.top() << std::endl; 
             stringStack.pop(); 
             stringStack.pop(); 
         } 
         catch (exception const& ex) { 
             cerr << "Exception: " << ex.what() <<endl; 
             return -1;
         } 
     }
     ```

- STL（标准模板库）

  核心的三个组件：容器、算法、迭代

- 标准库

  C++ 标准库可以分为两部分：

  - 标准函数库： 这个库是由通用的、独立的、不属于任何类的函数组成的。函数库继承自 C 语言。
  - 面向对象类库： 这个库是类及其相关函数的集合。





