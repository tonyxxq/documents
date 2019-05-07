- tf.strided_slice 函数

  ```python
  import tensorflow as tf
  
  data = [[[1, 1, 1], [2, 2, 2]],
          [[3, 3, 3], [4, 4, 4]],
          [[5, 5, 5], [6, 6, 6]]]
  
  """
  第一个参数为 input
  第二个参数为 start
  第三个参数为 end （不包含）
  第四个参数为 stride
  
  start 和 end 的 第一个元素代表第一维度，最后一个元素代表最后维度
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

  >添加一个维度， 0 表示在最外层添加 []

  ```python
  c1 = tf.get_variable(name="c1", shape=(2,), initializer=tf.truncated_normal_initializer())
  c2 = tf.expand_dims(c1, 0)
  c3 = tf.expand_dims(c1, 1)
  
  with tf.Session() as sess:
      tf.global_variables_initializer().run()
      print(c1.eval())
      print(c2.eval())
      print(c3.eval())
  ```

  输出为：

  ```
  [ 1.40756035  1.46115756]
  [[ 1.40756035  1.46115756]]
  [[ 1.40756035]
   [ 1.46115756]]
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

  > 沿着value的第一维进行随机重新排列

- tf.clip_by_global_norm

  > 在前向传播与反向传播之后，我们会得到每个权重的梯度 diff，权重可能会很大（梯度爆炸）。求得所有权重的和的平方根 global_norm 和 clip_norm 比较，如果 global_norm > clip_norm , 则让每一个权重乘以 clip_norm / global_norm，这个数是大于 0 小于 1 的，**这样就把梯度变化很大的权重缩小，降低了梯度爆炸的可能**

  ```python
  # clip_norm 表示截取的比率
  tf.clip_by_global_norm(t_list, clip_norm, use_norm=None, name=None) 
  ```

  

  

  

  