## RDD 及其常用操作

RDD 支持两种类型的操作：

> RDD 可以理解为一个大的集合，将所有数据都加载到内存中，方便进行多次重用。
>
> 第一，它是分布式的，可以分布在多台机器上，进行计算。
>
> 第二，它是弹性的，在计算处理过程中，机器的内存不够时，它会和硬盘进行数据交换，某种程度上会减低性能，但是可以确保计算得以继续进行。

- `变换（Transformation）`变换的**返回值是一个新的 RDD 集合**，而不是单个值。调用一个变换方法，不会有任何求值计算，它只获取一个 RDD 作为参数，然后返回一个新的 RDD。 变换函数包括：map，filter，flatMap，groupByKey，reduceByKey，aggregateByKey，pipe 和 coalesce。Transformations 一般都是 lazy 的，直到 action 执行后才会被执行。
- `行动（Action）`行动操作计算并返回一个新的**值**。当在一个 RDD 对象上调用行动函数时，会在这一时刻计算全部的数据处理查询并返回结果值。 行动操作包括：reduce，collect，count，first，take，countByKey 以及 foreach。

###Transformation

- `map(func):` 对调用 map 的 RDD 数据集中的每个 element 都使用 func，然后返回一个新的 RDD，这个返回的数据集是分布式的数据集。
- `filter(func):` 对调用 filter 的 RDD 数据集中的每个元素都使用 func，然后返回一个包含使 func 为 true 的元素构成的 RDD。
- `flatMap(func):` 和 map 差不多，但是 flatMap 生成的是多个结果。
- `mapPartitions(func):` 和 map 很像，但是 map 是每个 element，而 mapPartitions 是每个 partition。
- `mapPartitionsWithSplit(func):` 和 mapPartitions 很像，但是 func 作用的是其中一个 split 上，所以 func 中应该有 index。
- `sample(withReplacement,faction,seed):`抽样。
- `union(otherDataset)：` 返回一个新的 dataset，包含源 dataset 和给定 dataset 的元素的集合。
- `distinct([numTasks]):` 返回一个新的 dataset，这个 dataset 含有的是源 dataset 中的 distinct 的 element。
- `join(otherDataset,[numTasks]):` 当有两个 KV 的 dataset(K,V) 和 (K,W)，返回的是 (K,(V,W)) 的 dataset,numTasks 为并发的任务数。
- `cogroup(otherDataset,[numTasks]):` 当有两个 KV 的 dataset(K,V)和(K,W)，返回的是 (K,Seq[V],Seq[W])的 dataset，numTasks 为并发的任务数。
- `cartesian(otherDataset)：` 笛卡尔积简单说就是 m*n。
- `groupByKey(numTasks):` 返回(K,Seq[V])，也就是 hadoop 中 reduce 函数接受的 key-valuelist。
- `reduceByKey(func,[numTasks]):` 就是用一个给定的 reduce func再作用在groupByKey产生的(K,Seq[V]),比如求和，求平均数。
- `sortByKey([ascending],[numTasks]):` 按照 key 来进行排序，是升序还是降序，ascending 是 boolean 类型。
- `leftOuterJoin:` leftOuterJoin 类似于 SQL 中的左外关联 left outer join，返回结果以前面的 RDD 为主，关联不上的记录为空。只能用于两个 RDD 之间的关联，如果要多个 RDD 关联，多关联几次即可。

### Actions

- `count():` 返回的是 dataset 中的 element 的个数。
- `first():` 返回的是 dataset 中的第一个元素。
- `take(n):` 返回前 n 个 elements，这个是driver program 返回的。
- `takeSample(withReplacement，num，seed)：` 抽样返回一个 dataset 中的 num 个元素，随机种子 seed。
- `reduce(func)：` 说白了就是聚集，但是传入的函数是两个参数输入返回一个值，这个函数必须是满足交换律和结合律的。
- `collect()：` 一般在 filter 或者足够小的结果的时候，再用 collect 封装返回一个数组。
- `saveAsTextFile（path）：` 把 dataset 写到一个 text file 中，或者 hdfs，或者 hdfs 支持的文件系统中，spark 把每条记录都转换为一行记录，然后写到 file 中。
- `saveAsSequenceFile(path):` 只能用在 key-value 对上，然后生成 SequenceFile 写到本地或者 hadoop 文件系统。
- `countByKey()：` 返回的是 key 对应的个数的一个 map，作用于一个 RDD。
- `foreach(func):` 对 dataset 中的每个元素都使用 func。

