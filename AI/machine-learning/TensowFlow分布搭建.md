## TensorFlow 分布式搭建

tensorflow集群包含两类job: ps server 和 worker server, 每类job 可以包含多个 task, 每个task 是1个server，而每个server 包含两部分： master 和 worker, master用于创建session, worker用于执行具体的计算。











参考：https://blog.csdn.net/u011026329/article/details/79190537