#### A* 实现 google map 简单路径规划

```python
import math
from helpers import Map, load_map
from collections import deque 

# 输入为有 40 个节点的地图， 节点之间有些可以连接
map_40 = load_map('map-40.pickle')

def shortest_path(M, start, goal):
    # 初始化 open 和 close 列表
    g = 0
    h = cal_dist(M.intersections[start], M.intersections[goal])
    f = g + h
    open_path = [[f, g, h, start]]
    
    closed_path = [0 for i in M.intersections]
    closed_path[start] = 1
    
    # 为了从终点回溯找到最短路径
    action = [-1 for i in M.intersections]
    
    # 循环直到找到路径或找寻路径失败
    success = True 
    while True:
        if len(open_path) == 0:
            print("路线规划失败！")
            success = False
            break
        
        # 从 open 列表中移除 f 值最小的点
        open_path.sort()
        open_path.reverse()
        f, g, h, x = open_path.pop()
        
        # 判断是不是目的地
        if x == goal:
            break
       
        # 遍历该点附近能连接的地点, 计算 f 值，并放入 open 和 close 列表
        roads = M.roads[x]
        for x2 in roads:
            if closed_path[x2] == 0:
                g2 = g + cal_dist(M.intersections[x], M.intersections[x2])
                h2 = cal_dist(M.intersections[x2], M.intersections[goal])
                f2 = h2 + g2
                open_path.append([f2, g2, h2, x2])
                closed_path[x2] = 1
                action[x2] = x # 表示 x2 的上一步是 x
        
    return success, action            

    
# 计算两点之间的欧式距离，作为启发函数值
def cal_dist(p1, p2):
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1]- p2[1])**2)


start = 5
goal = 34
success, action = shortest_path(map_40, 5, 34)


# 找到路径了，从后往前回溯，找到路径，并打印
if success:
    path = deque([goal])
    x = goal
    while x != start:
        x = action[x]
        path.appendleft(x)
        
    print(path)    
```

输出：

```
deque([5, 16, 37, 12, 34])
```



#### A* 算法实现走迷宫

```python
# -*- encoding:utf-8 -*-

grid = [[0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0]]
heuristic = [[9, 8, 7, 6, 5, 4],
             [8, 7, 6, 5, 4, 3],
             [7, 6, 5, 4, 3, 2],
             [6, 5, 4, 3, 2, 1],
             [5, 4, 3, 2, 1, 0]]

init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1

delta = [[-1, 0 ], # go up
         [ 0, -1], # go left
         [ 1, 0 ], # go down
         [ 0, 1 ]] # go right

delta_name = ['^', '<', 'v', '>']

def search(grid,init,goal,cost,heuristic):
    closed = [[0 for col in range(len(grid[0]))] for row in range(len(grid))]
    closed[init[0]][init[1]] = 1

    expand = [[-1 for col in range(len(grid[0]))] for row in range(len(grid))]
    action = [[-1 for col in range(len(grid[0]))] for row in range(len(grid))]

    x = init[0]
    y = init[1]
    g = 0
    h = heuristic[x][y]
    f = h + g
    
    open = [[f, g, h, x, y]]

    found = False  # flag that is set when search is complete
    resign = False # flag set if we can't find expand
    count = 0
    
    while not found and not resign:
        if len(open) == 0:
            resign = True
            return "Fail"
        else:
            open.sort() # 默认使用第一个元素排序， 这里是 f
            open.reverse()
            next = open.pop()
            x = next[3]
            y = next[4]
            f = next[0]
            g = next[1]
            h = next[2]
            expand[x][y] = count
            count += 1
            
            if x == goal[0] and y == goal[1]:
                found = True
            else:
                for i in range(len(delta)):
                    x2 = x + delta[i][0]
                    y2 = y + delta[i][1]
                    if x2 >= 0 and x2 < len(grid) and y2 >=0 and y2 < len(grid[0]):
                        if closed[x2][y2] == 0 and grid[x2][y2] == 0:
                            h2 = heuristic[x2][y2]
                            g2 = g + cost
                            f2 = h2 + g2
                            open.append([f2, g2, h2, x2, y2])
                            closed[x2][y2] = 1
                            action[x2][y2] = i

    return expand, action

# 输出 expand
expand, action =  search(grid,init,goal,cost,heuristic)

for i in expand:
    print(i)
    
# 输出 policy
policy = [[' ' for col in grid[0]] for row in grid]
x = goal[0]
y = goal[1]
policy[x][y] = '*' 

while x != init[0] or y != init[1]:
    x2 = x - delta[action[x][y]][0]
    y2 = y - delta[action[x][y]][1]
    policy[x2][y2] = delta_name[action[x][y]]
    x = x2
    y = y2
    
for i in policy:
    print(i)
```

输出为：

```
[0, -1, -1, -1, -1, -1]
[1, -1, -1, -1, -1, -1]
[2, -1, -1, -1, -1, -1]
[3, -1, 8, 9, 10, 11]
[4, 5, 6, 7, -1, 12]
['v', ' ', ' ', ' ', ' ', ' ']
['v', ' ', ' ', ' ', ' ', ' ']
['v', ' ', ' ', ' ', ' ', ' ']
['v', ' ', ' ', '>', '>', 'v']
['>', '>', '>', '^', ' ', '*']
```



#### 动态规划实现走迷宫

```python
grid = [[0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0]]
init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1 # the cost associated with moving from a cell to an adjacent one

delta = [[-1, 0 ], # go up
         [ 0, -1], # go left
         [ 1, 0 ], # go down
         [ 0, 1 ]] # go right

delta_name = ['^', '<', 'v', '>']

def optimum_policy(grid,goal,cost):
    value = [[99 for row in range(len(grid[0]))] for col in range(len(grid))]
    policy = [[' ' for row in range(len(grid[0]))] for col in range(len(grid))]
    
    change = True
    while change:
        change = False
		# 遍历每一个元素
        for x in range(len(grid)):
            for y in range(len(grid[0])):
                # 如果是目标点更新值为 0
                if goal[0] == x and goal[1] == y:
                    if value[x][y] > 0:
                        value[x][y] = 0
                        policy[x][y] = '*'
                        change = True
				# 否则更新 value 值为临近点的最小值 + cost
                elif grid[x][y] == 0:
                    for a in range(len(delta)):
                        x2 = x + delta[a][0]
                        y2 = y + delta[a][1]

                        if x2 >= 0 and x2 < len(grid) and y2 >= 0 and y2 < len(grid[0]) and grid[x2][y2] == 0:
                            v2 = value[x2][y2] + cost
                            if v2 < value[x][y]:
                                change = True
                                value[x][y] = v2
                                policy[x][y] = delta_name[a]
    
	return policy
    

policy = optimum_policy(grid,goal,cost)

for i in policy:
    print(i)
```

输出为：

从结果可看出和 A* 算法的区别是：DP 的可以处理随机环境，出发点为可通行的所有点

```
['v', ' ', 'v', 'v', 'v', 'v']
['v', ' ', 'v', 'v', 'v', 'v']
['v', ' ', 'v', 'v', 'v', 'v']
['v', ' ', '>', '>', '>', 'v']
['>', '>', '^', '^', ' ', '*']
```

