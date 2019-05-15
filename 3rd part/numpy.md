- np.random.choice()

  > 通过给定的一维数组数据产生随机采样
  >
  > 参数说明：
  >
  > a：一维数组或者 int 型变量，如果是 int 则 使用 np.arrange(a) 生成数组
  >
  > size : 决定了输出的 shape
  >
  > replace : 采样中是否有重复值布尔参数，可选参数 
  >
  > p :每个采样点被选中的概率，可选参数 

  ```python
  xx = np.random.choice([1, 2, 3, 4, 5, 6], size=(2, 2), replace=True, p=[0, 0, 0.7, 0.1, 0.1, 0.1])
  print(xx)
  
  xx = np.random.choice(6, size=(2, 2), replace=True, p=[0, 0, 0.7, 0.1, 0.1, 0.1])
  print(xx)
  ```

  输出：

  ```
  [[3 3]
   [5 6]]
   
   [[3 2]
   [5 2]]
  ```

  