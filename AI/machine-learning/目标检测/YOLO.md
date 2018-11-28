## YOLO（You Only Look Once）

####YOLO v1

![](../imgs/88.png)



算法步骤

1. 图像 resize 成指定大小

2. 图像分割成 S X S 的格子（grid cell），真实目标中心所在的格子负责预测目标类别（如上图中：红点所在的格子负责预测红框，即狗的目标）， 每个格子生成指定数量的 Bounding Box（有一定的规则，可以根据论文设置），每个 Bounding Box 有 5 个参数（x,y： 相对格子的偏移，w,h:宽高，  Confidence：置信度，表示该Bounding Box 有物体的概率）， 每个格子除了指定数量的 Bounding Boxes 还有每个类别的概率。最终输出数据的大小为：S\*S\*(5*B + C)

   >**Bounding Box 置信度的计算方式**：如果 grid cell 里面没有object，confidence 就是 0，如果有，则 confidence score 等于预测的 box 和 ground truth 的 IOU 值
   >
   >​	公式：![](../imgs/89.png)P的取值： 当前 Bounding Box 所在的 grid cell 有物体为1，否则 0
   >
   >grid cell 的 C 值 ：是一个向量，表示该 grid cell 属于哪个类别

3. 进行 NMS（非极大值抑制）进行 Bounding Box 的冗余裁剪，处理掉大批的冗余，得到最后的预测结果。

   **grid cell score 的计算方式**：利用上上一步已经计算出的所有 Bounding Box 的置信度乘以该 grid cell 的score（是一个向量，表示类别的每个类别的分数）得到 Bounding Box 的 score 矩阵（比如：12 x 20, 12 表示 Bounding Box 数量， 20 表示每个类别的分数 ），在某个类别中，即矩阵某列，把小于阈值的分数（比如：0.2）设置 0，使用 NMS算法去掉指定类别中重复的 Bounding Box 的分数设为 0，最终从每个 Bounding BOX 中选出大于 0 且最大分数值的那个作为当前 Bounding Box 的类别， 如果最大分数时小等于 0，直接忽略该 Bounding Box, 表示没有检测到物体。

   **NMS 计算方法**：指定某一类别，选择一个 Bounding Box 计算和其他所有 Bounding Box 的 IOU 值如果大于 0.5 ，分数值设为 0， 否则保持不变，这样迭代所有类别

![](../imgs/90.png)



注意：理解最终的输出结果的构成是理解该算法的关键



#### YOLO v2











论文：http://arxiv.org/abs/1506.02640