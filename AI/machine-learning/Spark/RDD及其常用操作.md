## RDD 及其常用操作

RDD 支持两种类型的操作：

- `变换（Transformation）`变换的返回值是一个新的 **RDD 集合**，而不是单个值。调用一个变换方法，不会有任何求值计算，它只获取一个 RDD 作为参数，然后返回一个新的 RDD。 变换函数包括：map，filter，flatMap，groupByKey，reduceByKey，aggregateByKey，pipe 和 coalesce。
- `行动（Action）`行动操作计算并返回一个新的**值**。当在一个 RDD 对象上调用行动函数时，会在这一时刻计算全部的数据处理查询并返回结果值。 行动操作包括：reduce，collect，count，first，take，countByKey 以及 foreach。

