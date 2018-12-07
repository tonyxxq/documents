## openpose

文章的核心是提出一种利用 Part Affinity Fields（PAFs）的自下而上的人体姿态估计算法。研究自下而上算法（得到关键点位置再获得骨架）而不是自上而下算法（先检测人，再回归关键点），是因为后者运算时间会随着图像中人的个数而显著增加，而自下而上所需计算时间基本不变。 





github 地址：<https://github.com/CMU-Perceptual-Computing-Lab/openpose>

论文：https://arxiv.org/abs/1611.08050