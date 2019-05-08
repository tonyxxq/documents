- tf.InteractiveSession

  > 可以在定义了  InteractiveSession 之后再定义一些操作，而 Session 操作必须在之前全部定义完成。
  >
  > InteractiveSession  位置比较随意，在 IPython 里比较好用。

  ```python
  sess = tf.InteractiveSession()
  
  op = tf.get_variable("a", shape=(2,), initializer=tf.truncated_normal_initializer())
  
  tf.global_variables_initializer().run()
  print(sess.run(op))
  sess.close()
  ```

- 关于 max 函数 axis 参数的理解

  > 当我们指定了函数按某一轴来计算时，函数的输出数组的 shape 就是去掉当前轴的 shape
  >
  > 在指定的维度下有多个并列的矩阵，把并列的矩阵合并成一个矩阵，注意：这个并列指的是相同层级的
  >
  > 例如如下的矩阵：
  >
  > 当 axis =  0 时有两个并列的矩阵：
  >
  > 1. [[2, 3, 4, 8],
  >      [3, 1, 4, 1],
  >      [6, 3, 2, 6]],
  > 2. [[10, 2, 45, 2],
  >       [2, 4, 5, 10],
  >       [22, 4, 4, 1]]
  >
  > 取 max 的时候找到对应的元素取最大值
  >
  > [[10  3 45  8]
  >  [ 3  4  5 10]
  >  [22  4  4  6]]

  ```python
  import numpy as np
  
  arr = np.array([[[2, 3, 4, 8],
                   [3, 1, 4, 1],
                   [6, 3, 2, 6]],
                  [[10, 2, 45, 2],
                   [2, 4, 5, 10],
                   [22, 4, 4, 1]]])
  
  print(arr.shape)
  
  print(np.max(arr, axis=0).shape)
  print(np.max(arr, axis=1).shape)
  print(np.max(arr, axis=2).shape)
  
  print(np.max(arr, axis=0))
  print(np.max(arr, axis=1))
  print(np.max(arr, axis=2))
  ```

  输出数据为

  ```
  (2, 3, 4)
  
  (3, 4)
  (2, 4)
  (2, 3)
  
  [[10  3 45  8]
   [ 3  4  5 10]
   [22  4  4  6]]
  
  [[ 6  3  4  8]
   [22  4 45 10]]
  
  [[ 8  4  6]
   [45 10 22]]
  ```

- tf.strided_slice 函数

  ```python
  import tensorflow as tf
  
  """
  第一个参数为 input
  第二个参数为 start
  第三个参数为 end （不包含）
  第四个参数为 stride
  
  start 和 end 的第一个元素代表第一维度，最后一个元素代表最后维度
  从外到内依次依次取出需要的元素，返回数据的维度和原始维度一致
  start 和 end 的维度数量可以任意指定
  
  例如 x：默认步长是 1
  第一维 0， 1 
  [[[1, 1, 1], [2, 2, 2]]]
  
  第二维 1， 2 
  [[[2, 2, 2]]]
  
  第三维：0, 1:
  [[[2]]]
  """
  data = [[[1, 1, 1], [2, 2, 2]],
          [[3, 3, 3], [4, 4, 4]],
          [[5, 5, 5], [6, 6, 6]]]
  
  # 只取到第一维度的第一行数据，[[[1, 1, 1], [2, 2, 2]]]
  a = tf.strided_slice(data, [0], [1])
  x = tf.strided_slice(data, [0, 1, 0], [1, 2, 1])
  y = tf.strided_slice(data, [0, 0, 0], [2, 2, 2], [1, 1, 1])
  z = tf.strided_slice(data, [0, 0, 0], [2, 2, 2], [1, 2, 1])
  
  with tf.Session() as sess:
      print(sess.run(a))
      print(sess.run(x))
      print(sess.run(y))
      print(sess.run(z))
  ```

  输出数据为

  ```
  [[[1 1 1]
    [2 2 2]]]
  
  [[[2]]]
  
  [[[1 1]
    [2 2]]
    [[3 3]
    [4 4]]]
  
  [[[1 1]]
   [[3 3]]]
  ```

- tf.tile

  > 矩阵进行自身进行复制的功能，比如按行进行复制，或是按列进行复制
  > 当然也可以是多维向量，因为是复制，所以最终结果元素的数量只增加不减少

  ```python
  import tensorflow as tf
  
  a = tf.constant([[1, 2], [2, 3], [3, 4]], dtype=tf.float32)
  tile_a_1 = tf.tile(a, [1, 2])
  tile_a_2 = tf.tile(a, [2, 1])
  tile_a_3 = tf.tile(a, [2, 2])
  
  with tf.Session() as sess:
      print(sess.run(tile_a_1))
      print(sess.run(tile_a_2))
      print(sess.run(tile_a_3))
  ```

  输出为：

  ```
  [[ 1.  2.  1.  2.]
   [ 2.  3.  2.  3.]
   [ 3.  4.  3.  4.]]
  [[ 1.  2.]
   [ 2.  3.]
   [ 3.  4.]
   [ 1.  2.]
   [ 2.  3.]
   [ 3.  4.]]
  [[ 1.  2.  1.  2.]
   [ 2.  3.  2.  3.]
   [ 3.  4.  3.  4.]
   [ 1.  2.  1.  2.]
   [ 2.  3.  2.  3.]
   [ 3.  4.  3.  4.]]
  ```

- tf.sequence_mask

  > 第一个参数表示第 True/1 的个数
  >
  > 第二个参数表示生成的序列的长度
  >
  > dtype 默认是 bool 可以设为其他的类型

  ```python
  mask = tf.sequence_mask([1, 3, 2], 5, dtype=tf.bool)
  
  with tf.Session() as sess:
      tf.global_variables_initializer().run()
      print(mask.eval())
  ```

  输出为：

  ```
  [[True, False, False, False, False],
  [True, True, True, False, False],
  [True, True, False, False, False]]
  ```

- tf.expand_dims

  >找到对应的维度在外面添加 []， 0 表示在最外层添加 []

  ```python
  c1 = tf.constant([[1, 2, 3], [4, 5, 6]])
  c2 = tf.expand_dims(c1, axis=0)
  c3 = tf.expand_dims(c1, axis=1)
  c4 = tf.expand_dims(c1, axis=2)
  
  with tf.Session() as sess:
      tf.global_variables_initializer().run()
      print(c1.eval())
      print(c2.eval())
      print(c3.eval())
      print(c4.eval())
  ```

  输出为：

  ```
  [[1 2 3]
   [4 5 6]]
  
  [[[1 2 3]
    [4 5 6]]]
  
  [[[1 2 3]]
   [[4 5 6]]]
  
  [[[1] [2] [3]]
  [[4]  [5] [6]]]
  ```

- tf.stack

  ```python
  x0 = tf.stack([[1, 2, 4], [2, 5, 8]], axis=0)
  x1 = tf.stack([[1, 2, 4], [2, 5, 8]], axis=1)
  
  with tf.Session() as sess:
      print(x0.eval())
      print(x1.eval())
  ```

  输出为：

  ```
  [[1 2 4]
   [2 5 8]]
  [[1 2]
   [2 5]
   [4 8]]
  ```

- tf.random_shuffle

  > 沿着 value 的第一维进行随机重新排列

- tf.variable_scope

  > 为变量添加命名域

  ```python
  with tf.variable_scope("foo"):
      v = tf.get_variable("v", [1])
      print(v)
  ```

  输出为：

  ```
  <tf.Variable 'foo/v:0' shape=(1,) dtype=float32_ref>
  ```

- tf.assign

  > 更新模型中变量的值

  ```python
  a = tf.Variable(0, dtype=tf.float32)
  
  with tf.Session() as sess:
      tf.global_variables_initializer().run()
      print(a.eval())
      tf.assign(a, 10).eval()
      print(a.eval())
  ```

  输出为：

  ```
  0.0
  10.0
  ```

- tf.linspace 和 tf.range

  > 产生等级数列

  ```python
  # 前两个参数为开始和结束（必须为浮点数）， 第三个参数表示产生元素的个数
  a = tf.linspace(1.0, 10.0, 10)
  
  # 和 python 原生的 range 函数类似
  b = tf.range(1, 10, 1)
  
  sess = tf.InteractiveSession()
  
  print(sess.run(a))
  print(sess.run(b))
  ```

  输出为：

  ```
  [  1.   2.   3.   4.   5.   6.   7.   8.   9.  10.]
  [1 2 3 4 5 6 7 8 9]
  ```

- tf.nn.dropout

  按概率来将x中的一些元素值置零，并将其他的值放大。用于进行dropout操作，一定程度上可以防止过拟合 

  没有清零的，每个元素需要乘以 1/keep_prob，目的是为了保持x的整体期望值不变

  ```python
  sess = tf.InteractiveSession()
  a = tf.get_variable('a', shape=[2, 5])
  a_drop = tf.nn.dropout(a, 0.8)
  
  sess.run(tf.initialize_all_variables())
  
  print(sess.run(a))
  # [[-0.35163319 -0.15617168  0.37421417  0.55973208  0.42068231]
  # [-0.29390228  0.5910778   0.15247107  0.51066864  0.15389156]]
  
  print(sess.run(a_drop))
  # [[-0.43954149 -0.1952146   0.          0.69966507  0.52585286]
  # [-0.36737785  0.73884726  0.19058883  0.63833576  0.        ]]
  ```

- 滑动平均模型

  > decay：实数类型，衰减率。
  >
  > num_updates：可选，设置这个参数之后，将会通过min(decay, (1 + num_updates) / (10 + num_updates))函数，从中选择最小值做为衰减率。
  >
  > 影子变量：shadow_variable = decay * shadow_variable + (1-decay) * variable
  >
  > 1. 训练阶段：为每个可训练的权重维护影子变量，并随着迭代的进行更新
  > 2.  预测阶段：使用影子变量替代真实变量值，进行预测
  >
  > 预测和训练阶段分别是两个前向传播网络

  ```python
  # 定义一个变量用于计算滑动平均，变量的初始值为 0，变量的类型必须是实数
  v1 = tf.Variable(5, dtype=tf.float32)
  
  # 定义一个迭代轮数的变量，动态控制衰减率,并设置为不可训练，这个地方的 step 没有递增
  step = tf.Variable(1, trainable=False)
  
  # 定义一个滑动平均类，初始化衰减率为 0.99 和衰减率的变量 step，这是一个类不是 tensor，里边包括了影子变量
  ema = tf.train.ExponentialMovingAverage(0.99, step)
  
  # 定义每次滑动平均所更新的列表
  maintain_average_op = ema.apply([v1])
  
  # 初始化上下文会话
  with tf.Session() as sess:
      # 初始化所有变量
      init = tf.initialize_all_variables()
      sess.run(init)
  
      # 更新 v1 的滑动平均值
      '''
      衰减率为 min(0.99,(1+step)/(10+step)=0.1}=0.1
      '''
      sess.run(maintain_average_op)
  
      # [5.0, 5.0]
      # 0.1 * 5 + 0.9 * 5 = 5
      print(sess.run([v1, ema.average(v1)]))
  
      sess.run(tf.assign(v1, 4))
      sess.run(maintain_average_op)
  
      # [4.0, 4.5500002], 5*(2/11) + 4 * (9/11)，averge 里边存的就是影子变量
      print(sess.run([v1, ema.average(v1)]))
  ```

- tf.control_dependencies

  > 用来控制计算流图的，给图中的某些计算指定顺序。比如：我们想要获取参数更新后的值，那么我们可以这么组织我们的代码。
  >
  > **需要注意的是，除了 对于Variable 的 identity op 会在 control dependence 后重新计算，其它 op 都不会重新计算**
  >
  > tf.identity 是添加的一个 op

  ```python
  opt = tf.train.Optimizer().minize(loss)
  
  with tf.control_dependencies([opt]):
    updated_weight = tf.identity(weight)
  
  with tf.Session() as sess:
    tf.global_variables_initializer().run()
    sess.run(updated_weight, feed_dict={...}) # 这样每次得到的都是更新后的weight
  ```

- 学习率衰减

  > 常用的学习率衰减方式，参考
  >
  > https://blog.csdn.net/zxyhhjs2017/article/details/82383723
  >
  > 其中指数衰减公式：
  >
  > decayed_learning_rate = learning_rate * decay_rate ^ (global_step / decay_steps)  
  >
  > global_step：训练的步数，递增
  >
  > decay_steps：指定步数更新学习率
  >
  > decay_rate ：学习率衰减的底数
  >
  > staircase：更新是否连续，当为 True 的时候每一步都更新，否则得等 decay_steps 步才更新

  ```python
  learning_rate = tf.train.exponential_decay(
      0.001,
      global_step,
      200,
      0.99,
      staircase=True
  )
  ```

- tf.clip_by_global_norm

  > 在前向传播与反向传播之后，我们会得到每个权重的梯度 diff，权重可能会很大（梯度爆炸）。求得所有权重的和的平方根 global_norm 和 clip_norm 比较，如果 global_norm > clip_norm , 则让每一个权重乘以 clip_norm / global_norm，这个数是大于 0 小于 1 的，**这样就把梯度变化很大的权重缩小，降低了梯度爆炸的可能**

  ```python
  # clip_norm 表示截取的比率
  tf.clip_by_global_norm(t_list, clip_norm, use_norm=None, name=None) 
  ```

  

  

  

  