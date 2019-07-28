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

- dtype

  ```python
  student = np.dtype([('name','U20'), ('age', 'i1'), ('marks', 'f')]) 
  a = np.array([('abc', 21, 50.225001), ('xyz', 18, 75)], dtype = student) 
  
  # 追加元素
  a = np.append(a, np.array(('zxl', 18, 75), dtype=student))
  
  # 按列名取数据
  print(a['name'])
  
  # 按行取
  print(a[0])
  
  # 按行号和列名取数据
  print(a[0]['name'])
  
  # 按行号和列号取数据
  print(a[0][0])
  
  print(a.ndim)
  
  print(a.itemsize)
  ```

  ```
  ['abc' 'xyz' 'zxl']
  ('abc', 21,  50.22500229)
  abc
  abc
  1
  85
  ```

- frombuffer

  ```python
  s =  b'Hello World' 
  a = np.frombuffer(s, dtype = 'S1')  
  print (a)
  ```

  输出：

  ```
  [b'H' b'e' b'l' b'l' b'o' b' ' b'W' b'o' b'r' b'l' b'd']
  ```

- 使用迭代器创建 ndarray 

  ```python
  # 使用迭代器创建 ndarray 
  it = (i for i in range(10))
  x = np.fromiter(it, dtype=float)
  print(x)
  ```

  输出：

  ```
  [ 0.  1.  2.  3.  4.  5.  6.  7.  8.  9.]
  ```

- 创建等比数列

  ```python
  # np.logspace(start, stop, num=50, endpoint=True, base=10.0, dtype=None)
  # 默认底数是 10
  a = np.logspace(1.0,  2.0, num = 10)  
  print(a)
  ```

  输出：

  ```
  [  10.           12.91549665   16.68100537   21.5443469    27.82559402
     35.93813664   46.41588834   59.94842503   77.42636827  100.        ]
  ```

- 使用省略号，表示满选

  ```python
  # 使用省略号，表示满选
  a = np.array([[1, 2, 3], [3, 4, 5], [4, 5, 6]])  
  print (a[..., 1])   # 第2列元素
  print (a[1, ...])   # 第2行元素
  print (a[..., 1:])  # 第2列及剩下的所有元素
  ```

  输出：

  ```
  [2 4 5]
  [3 4 5]
  [[2 3]
   [4 5]
   [5 6]]
  ```

- 数组索引

  ```python
  # 数组索引
  x = np.array([[1,  2],  [3,  4],  [5,  6]]) 
  y = x[[0, 1, 2], [0, 1, 0]]
  print(y)
  
  # 使用数组索引获取四个对角线的元素
  z = np.array([[0, 1, 2], [3, 4, 5], [6, 7, 8], [9, 10, 11]])
  a = z[[[0, 0], [3, 3]], [[0, 2], [0, 2]]]
  print(a)
  ```

  输出：

  ```
  [1 4 5]
  [[ 0  2]
   [ 9 11]]
  ```

- 布尔索引

  ```python
  x = np.arange(0, 12).reshape([4, 3])
  print(x > 5)
  print(x[x > 5]) # 为什么变成一维了？？？
  ```

  输出：

  ```
  [[ 0  1  2]
   [ 3  4  5]
   [ 6  7  8]
   [ 9 10 11]]
  [[False False False]
   [False False False]
   [ True  True  True]
   [ True  True  True]]
  [ 6  7  8  9 10 11]
  ```

- 取补运算符 ～， 类似：not 的用法

  ```python
  a = np.array([np.nan,  1, 2, np.nan, 3, 4, 5])  
  print(a[~np.isnan(a)])
  ```

  输出：

  ```
  [ 1.  2.  3.  4.  5.]
  ```

- 迭代数组

  ```python
  a = np.arange(6).reshape(2,3)
  print(a)
  for x in np.nditer(a):
      print(x, end=", ")
      
  # 修改数组中的元素，必须指定 op_flags=['readwrite']
  # 发现修改之后的数组维度没有变化
  a = np.arange(0, 60, 5)
  a = a.reshape(3, 4)
  print (a)
  for x in np.nditer(a, op_flags=['readwrite']): 
      x[...] = 2 * x
  print (a)
  
  # 广播迭代，第一个数组维度为 3X4，第二个数组维度为 1x4
  # 第二个数组被广播
  a = np.arange(0, 60, 5) 
  a = a.reshape(3, 4)
  print(a)
  b = np.array([1,  2,  3,  4], dtype = int)  
  print (b)
  for x,y in np.nditer([a,b]): 
      print ("%d:%d"  %  (x, y), end=", " )
  ```

  输出：

  ```
  [[0 1 2]
   [3 4 5]]
  0, 1, 2, 3, 4, 5, [[ 0  5 10 15]
   [20 25 30 35]
   [40 45 50 55]]
  [[  0  10  20  30]
   [ 40  50  60  70]
   [ 80  90 100 110]]
  [[ 0  5 10 15]
   [20 25 30 35]
   [40 45 50 55]]
  [1 2 3 4]
  0:1, 5:2, 10:3, 15:4, 20:1, 25:2, 30:3, 35:4, 40:1, 45:2, 50:3, 55:4, 
  ```

- flatten 和 ravel

  ```python
  a = np.arange(6).reshape(2, 3)
  
  # flatten, 转换为一维数组, 修改不影响原始数组
  f = a.flatten()
  print(f)
  f[0] = 100
  print(a)
  
  # ravel, 转换为一维数组, 修改影响原始数组
  r = a.ravel()
  print(r)
  r[0] = 100
  print(a)
  ```

  输出：

  ```
  [0 1 2 3 4 5]
  [[0 1 2]
   [3 4 5]]
  [0 1 2 3 4 5]
  [[100   1   2]
   [  3   4   5]]
  ```

- squeeze 删除维度

  ```python
  # 增加维度 numpy.expand_dims(arr, axis)
  a = np.arange(0, 12).reshape(3, 4)
  a = np.expand_dims(a, axis=2)
  print(a)
  
  # 删除维度，只能是维度为一的才能删除 numpy.squeeze(arr, axis)
  a = np.squeeze(a, axis=2)
  print(a)
  ```

  输出：

  ```
  [[[ 0]
    [ 1]
    [ 2]
    [ 3]]
  
   [[ 4]
    [ 5]
    [ 6]
    [ 7]]
  
   [[ 8]
    [ 9]
    [10]
    [11]]]
  [[ 0  1  2  3]
   [ 4  5  6  7]
   [ 8  9 10 11]]
  ```

- concatenate 合并数组

  ```python
  # 不会增加维度， numpy.concatenate((a1, a2, ...), axis)
  a = np.arange(0, 12).reshape(3, 4)
  b = np.arange(0, 12).reshape(3, 4)
  print(a)
  np.concatenate((a, b), axis=0)
  ```

  输出：

  ```
  [[ 0  1  2  3]
   [ 4  5  6  7]
   [ 8  9 10 11]]
   
   array([[ 0,  1,  2,  3],
         [ 4,  5,  6,  7],
         [ 8,  9, 10, 11],
         [ 0,  1,  2,  3],
         [ 4,  5,  6,  7],
         [ 8,  9, 10, 11]])
  ```

- stack,  指定维度的数据合并成新的维度

  ```python
  # 会增加维度，把指定维度的数据合并成新的维度， numpy.stack(arrays, axis)
  a = np.arange(0, 12).reshape(3, 4)
  b = np.arange(0, 12).reshape(3, 4)
  print(a)
  np.stack((a, b), axis=0)
  ```

  输出：

  ```
  [[ 0  1  2  3]
   [ 4  5  6  7]
   [ 8  9 10 11]]
   
   array([[[ 0,  1,  2,  3],
          [ 4,  5,  6,  7],
          [ 8,  9, 10, 11]],
  
         [[ 0,  1,  2,  3],
          [ 4,  5,  6,  7],
          [ 8,  9, 10, 11]]])
  ```

- numpy.vstack(arrays, axis) , 垂直堆叠，维度不变（类似 concatenate） 

  ```python
  a = np.arange(0, 12).reshape(3, 4)
  b = np.arange(0, 12).reshape(3, 4)
  print(a)
  np.vstack((a, b))
  ```

  输出：

  ```
  [[ 0  1  2  3]
   [ 4  5  6  7]
   [ 8  9 10 11]]
   
   array([[ 0,  1,  2,  3],
         [ 4,  5,  6,  7],
         [ 8,  9, 10, 11],
         [ 0,  1,  2,  3],
         [ 4,  5,  6,  7],
         [ 8,  9, 10, 11]])
  ```

- numpy.vstack(arrays, axis)，水平堆叠，维度不变（类似 concatenate）

  ```python
  a = np.arange(0, 12).reshape(3, 4)
  b = np.arange(0, 12).reshape(3, 4)
  print(a)
  np.hstack((a, b))
  ```

  输出：

  ```
  [[ 0  1  2  3]
   [ 4  5  6  7]
   [ 8  9 10 11]]
   
   array([[ 0,  1,  2,  3,  0,  1,  2,  3],
         [ 4,  5,  6,  7,  4,  5,  6,  7],
         [ 8,  9, 10, 11,  8,  9, 10, 11]])
  ```

- 分割数组 numpy.split(ary, indices_or_sections, axis)

  ```python
  a = np.arange(0, 12).reshape(3, 4)
  print(a)
  
  print(np.split(a, 3)) # 分成 3 个子数组
  print(np.split(a, [1, 2])) # 确定分割点
  
  print(np.split(a, 2, axis=1)) # 水平分割
  print(np.hsplit(a, 2))
  
  print(np.split(a, 3, axis=0)) # 垂直分割
  print(np.vsplit(a, 3))
  ```

  输出：

  ```
  [[ 0  1  2  3]
   [ 4  5  6  7]
   [ 8  9 10 11]]
  [array([[0, 1, 2, 3]]), array([[4, 5, 6, 7]]), array([[ 8,  9, 10, 11]])]
  [array([[0, 1, 2, 3]]), array([[4, 5, 6, 7]]), array([[ 8,  9, 10, 11]])]
  [array([[0, 1],
         [4, 5],
         [8, 9]]), array([[ 2,  3],
         [ 6,  7],
         [10, 11]])]
  [array([[0, 1],
         [4, 5],
         [8, 9]]), array([[ 2,  3],
         [ 6,  7],
         [10, 11]])]
  [array([[0, 1, 2, 3]]), array([[4, 5, 6, 7]]), array([[ 8,  9, 10, 11]])]
  [array([[0, 1, 2, 3]]), array([[4, 5, 6, 7]]), array([[ 8,  9, 10, 11]])]
  ```

- resize

  ```python
  # 如果新数组大小大于原始大小，则包含原始数组中的元素的副本， 和 reshape 的区别
  a = np.array([[1,2,3],[4,5,6]])
  b = np.resize(a, (3, 3))
  print(a)
  print(b)
  ```

  输出：

  ```
  [[1 2 3]
   [4 5 6]]
  [[1 2 3]
   [4 5 6]
   [1 2 3]]
  ```

- append

  ```python
  # 注意： append 会生成新的数组且如果不指定 axis，会变成一维度数组
  a = np.array([[1,2,3], [4,5, 6]])
  print(np.append(a, [[1,2,3]]))
  print(np.append(a, [[5,5,5], [7,8,9]], axis = 1))
  print(np.append(a, [[5,5,5], [7,8,9]], axis = 0))
  ```

  输出：

  ```
  [1 2 3 4 5 6 1 2 3]
  [[1 2 3 5 5 5]
   [4 5 6 7 8 9]]
  [[1 2 3]
   [4 5 6]
   [5 5 5]
   [7 8 9]]
  ```

- insert

  ```python
  # numpy.insert(arr, obj, values, axis)
  # 注意：会生成新的数组且如果不指定 axis，会变成一维度数组， 有广播机制
  a = np.array([[1,2],[3,4],[5,6]])
  print (a)
  print (np.insert(a, 3, [11,12]))
  print (np.insert(a, 1, [11], axis = 0))
  print (np.insert(a, 1, 11, axis = 1))
  ```

  输出：

  ```
  [[1 2]
   [3 4]
   [5 6]]
  [ 1  2  3 11 12  4  5  6]
  [[ 1  2]
   [11 11]
   [ 3  4]
   [ 5  6]]
  [[ 1 11  2]
   [ 3 11  4]
   [ 5 11  6]]
  ```

- delete

  ```python
  # Numpy.delete(arr, obj, axis)
  # 注意：会生成新的数组且如果不指定 axis，会变成一维度数组 和 insert 类似
  a = np.array([[1,2],[3,4],[5,6]])
  print (a)
  print (np.delete(a, 3))
  print (np.delete(a, 1, axis = 0))
  print (np.delete(a, 1, axis = 1))
  ```

  输出：

  ```
  [[1 2]
   [3 4]
   [5 6]]
  [1 2 3 5 6]
  [[1 2]
   [5 6]]
  [[1]
   [3]
   [5]]
  ```

- unique

  ```python
  # numpy.unique(arr, return_index, return_inverse, return_counts)
  a = np.array([5,2,6,2,7,5,6,8,2,9])
  print (a)
   
  u = np.unique(a)
  print (u, '\n')
  
  # return_index 新数组在旧数组的下标
  u, indices = np.unique(a, return_index = True)
  print (u)
  print (indices, '\n')
  
  # return_inverse 旧数组在新数组的下标
  u, indices = np.unique(a, return_inverse = True)
  print (indices)
  print (u[indices], '\n') # 还原旧数组
  
  # 统计每个元素个数
  u, indices = np.unique(a, return_counts = True)
  print (u)
  print (indices)
  ```

  输出：

  ```
  [5 2 6 2 7 5 6 8 2 9]
  [2 5 6 7 8 9] 
  
  [2 5 6 7 8 9]
  [1 0 2 4 7 9] 
  
  [1 0 2 0 3 1 2 4 0 5]
  [5 2 6 2 7 5 6 8 2 9] 
  
  [2 5 6 7 8 9]
  [3 2 2 1 1 1]
  ```

  