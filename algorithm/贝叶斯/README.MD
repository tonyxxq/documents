- 贝叶斯概率是“逆向”概率，由观察结果去推测原始概率分布

- 理解了贝叶斯概率中几个重要的概念，就理解了贝叶斯算法

  > 可以把观察值的概率分布想成是和先验概率有相同的随机变量（比如分类任务中的类别）但是不同的概率值的分布，这样出现观察值时，每一个类别出现的概率，每一个类别分别乘以各自的先验概率，最终归一化处理。

  先验概率：已知概率分布。

  似然：获取的观察值在在某一个分布点上的概率。

  全概率：观察值在所有数据点上的概率和，为了使结果归一化。

  后验概率：根据观察值和先验分布获取的预测值。

  ​