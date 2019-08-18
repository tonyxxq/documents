- lambda 表达式

  1. 编写一个函数式接口

     ```java
     public class test {
         public static void main(String[] args) {
             // 定义表达式
             Operation o1 = (a, b) -> a + b;
             Operation o2 = (a, b) -> a * b;
     
             System.out.println(o1.operate(1, 2));
             System.out.println(o2.operate(1, 2));
         }
     	
         // 定义一个接口和内部方法
         interface Operation {
             int operate(int a, int b);
         }
     }
     ```

     输出：

     ```
     3
     2
     ```

     