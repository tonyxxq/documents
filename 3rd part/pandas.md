#### 生成数据表

```python
# 读取外部文件 excel、csv、json、html 等, 返回数据是 DataFrame 格式
# df = pd.read_excel("name.xls")
# df = pd.read_csv("name.csv", encoding = "gbk")
# df = pd.read_json("name.json", encoding = "utf-8")
# df = pd.read_html("name.html", encoding = "utf-8") # 爬虫

# 使用 pandas 创建数据表
df = pd.DataFrame({"id": [1001, 1002, 1003, 1004, 1005, 1006],
                   "date": pd.date_range('201301021220', periods=6, freq='2D'),
                   "city": ['Beijing ', 'SH', ' guangzhou ', 'Shenzhen', 'shanghai', 'BEIJING '],
                   "age": [23, 44, 54, 32, 34, 32],
                   "category": ['100-A', '100-B', '110-A', '110-C', '210-A', '130-F'],
                   "price": [1200, np.nan, 2133, 5433, np.nan, 4432]},
                  columns=['id', 'date', 'city', 'category', 'age', 'price'])

print(df)
```

输出为：

```
     id                date         city category  age   price
0  1001 2013-01-02 12:20:00     Beijing     100-A   23  1200.0
1  1002 2013-01-04 12:20:00           SH    100-B   44     NaN
2  1003 2013-01-06 12:20:00   guangzhou     110-A   54  2133.0
3  1004 2013-01-08 12:20:00     Shenzhen    110-C   32  5433.0
4  1005 2013-01-10 12:20:00     shanghai    210-A   34     NaN
5  1006 2013-01-12 12:20:00     BEIJING     130-F   32  4432.0
```

#### 数据表信息查看

> 注意：使用  .values 可以把  pandas 的数据类型转化为 numpy 的 ndarray 数据类型
> 例如：df.values , df['price'].values , df.columns.values
>
> 使用 .values.tolist()  可以转换为 list

```python
# 查看前 10 行后 10 行数据
df.head()

# 查看列名
df.columns
df.columns.values # 类型为 ndarray

# 基本信息（维度、列名称、数据格式、所占空间等）
df.info()

# 维度
df.shape

# 每一列的数据类型和指定列的数据类型
df.dtypes
df['date'].dtype

# 查看某一列的值
# df[date] 或 df.date

# 判断所有列或指定列是否为空值，为空返回 True ,否则为 False，返回类型为 DataFrame 或 Series
# isna/notna 是 isnull/notnull 的别名，所以用法一致，建议使用 isna/notna
df.isnull()
df.notnull()
df['price'].isnull()
df['price'].notnull()

# 指定列的唯一值（去重），类型为 ndarray
df["price"].unique()

# 判断数据元素是否在指定的范围内，返回一个 series
df['city'].isin(['beijing', 'shanghai'])
```

#### 数据表清洗和数据预处理

> 注意：pandas 中的修改操作会生成一个新的 DataFrame 或 Series，不是在原来的基础上修改，所以需要重新赋值，这点和 numpy 不一样

- 常用功能

  ```python
  # 空值进行赋值
  # df = df.fillna(0)
  df['price'] = df['price'].fillna(df['price'].mean())
  
  # 修改数据类型
  # df['price'] = df['price'].astype('int') 或 使用 apply
  # apply 可以使用函数或 lambda 表达式，且如果是 DataFrame 可指定 axis
  df['price'] = df['price'].apply(int)
  
  # 字符串处理
  df['city'] = df['city'].str.lower()
  df['city'] = df['city'].str.strip()
  df['city'] = df['city'].str.replace("beijing", "shanghai")
  
  # 更改列名
  df = df.rename(columns={'category': 'category-size'})
  
  # 删除后出现的重复值/删除先出现的重复值
  # df['city'].drop_duplicates()
  df['city'].drop_duplicates(keep='last')
  
  # 把某一列作为索引, （如果不指定索引值，默认为行号）
  df = df.set_index('id')
  
  # 重置索引值（索引值为行号）
  df = df.reset_index()
  
  # 按照特定列的值排序，优先级由高到低
  df = df.sort_values(by=['price', 'id'])
  
  # 按照索引列排序
  df = df.sort_index()
  
  # 增加一列 group， 如果 m-point 列的值 >= 20，group 列显示 high，否则显示 low
  df['group'] = np.where(df['price'] >= 20, 'high', 'low')
  
  # 对复合多个条件的数据进行分组标记，当不存在 sign 这一列的时候会创建一列且赋值
  df.loc[(df['age'] == '30') & (df['price'] >= 20), 'sign'] = 1
  ```

- 数据合并（新建两个 DataFrame）

  ```python
  df1 = pd.DataFrame({"id": [1001, 1002, 1003, 1004, 1005, 1006, 1007, 1008],
                      "gender": ['male', 'female', 'male', 'female', 'male', 'female', 'male', 'female'],
                      "pay": ['Y', 'N', 'Y', 'Y', 'N', 'Y', 'N', 'Y', ],
                      "m-point": [10, 12, 20, 40, 40, 40, 30, 20]})
  
  
  df2 = pd.DataFrame({"id": [1001, 1002, 1003, 1004, 1005, 1006, 1007, 1008],
                      "gender": ['male', 'female', 'male', 'female', 'male', 'female', 'male', 'female'],
                      "pay": ['Y', 'N', 'Y', 'Y', 'N', 'Y', 'N', 'Y', ],
                      "m-point": [10, 12, 20, 40, 40, 40, 30, 20]}, index=[1001, 1002, 1003, 1004, 1005, 1006, 1007, 1008])
  ```

  1. merge

     > merge 和数据库的连接查询一致
     >
     >  参数说明：
     >
     > how：指定连接方式
     >
     > on：指定连接的列名称，如果列名不一样时可以使用 left_on 和 right_on

     ```python
     df_inner = pd.merge(df, df1, left_on=['id', "price"], right_on=['id', "m-point"], how='inner')  # 匹配合并，交集
     df_left = pd.merge(df, df1, how='left')
     df_right = pd.merge(df, df1, how='right')
     df_outer = pd.merge(df, df1, how='outer') 
     ```

  2. join

     > join 是通过索引进行连接
     >
     > df1 的列名和 对方 df2 的索引名一致可以使用 on 参数指定列名称进行关联查询

     ```python
     df = df1.join(df2, on='id', lsuffix='_left', rsuffix="_right")
     ```

  3. append

     > 上下进行拼接

     ```python
     result = df1.append(df2)
     ```

  4. concat

     > 上下进行拼接, 可拼接多个 DataFrame 

     ```python
     df = pd.concat([df1, df2])
     ```

#### 数据提取

> 主要用到三个函数：loc，iloc 和 ix，loc 函数按标签值进行提取，iloc 按位置进行提取，ix 可以同时按标签和位置进行提取（ix 已经过时，不用再讨论）
> **注意：pandas 范围提取的时候 loc 包括结束位置， iloc 不包括 **

```python
# loc 按照 index 的标签值进行提取, 如果标签不存在会有警告信息
df.loc[1001]
df.loc[1001:1002]
df.loc[0:2, ['price', 'id']] # 指定行标签和指定列标签

# iloc 按照位置进行提取和 numpy 的提取类似
df.iloc[0]
df.iloc[0:2]
df.iloc[0:2, :2] # 指定行号和列号
df.iloc[[0,2,5],[4,5]] # 提取第0、2、5行，4、5列
```

#### 数据筛选

> 使用与、或、非三个条件配合大于、小于、等于对数据进行筛选
>
> 使用 query 方函数进行筛选

```python
# 先选出满足条件的行，再选出指定的列
df.loc[(df['age'] > 25) & (df['city'].str.strip() == 'Beijing'), ['id', 'price', 'city']]
df.loc[(df['age'] > 25) | (df['city'].str.strip() == 'Beijing'), ['id', 'price', 'city']]

# query 函数内部指定列需要满足的条件，下面等价于条件 city in ['beijing', 'shanghai']
df.query("city == ['beijing', 'shanghai' and  price > 30]")
```

#### 数据分组

> 主要函数是 groupby
>
> groupby 后面需要加上聚合函数，才能得出各个分组的结果，类似 sql 查询语句

```python
# 按照 city 分组，并统计每个分组中列元素的个数
# 注意：为 nan 的位置不进入个数统计
print(df.groupby('city').count())
```

输出为：

```
city          id  date  category  age   price  group
guangzhou     1     1         1    1      1      1
Beijing       2     2         2    2      2      2
SH            1     1         1    1      0      1
Shenzhen      1     1         1    1      1      1
shanghai      1     1         1    1      0      1
```

```python
# 也可按多个列进行分组，统计指定列的个数，
# 注意：分组中有 nan 的元素，该分组被自动去掉
print(df.groupby(['city', 'price'])['id'].count())

# 按 city 进行分组，统计 price 的 len sum mean
print(df.groupby('city')['price'].agg([len, np.sum, np.mean]))
```

输出为：

```
city         price 
guangzhou    30.0      1
Beijing      1200.0    1
             4432.0    1
Shenzhen     5433.0    1
Name: id, dtype: int64
             
city         len     sum    mean                           
 guangzhou   1.0    30.0    30.0
Beijing      2.0  5632.0  2816.0
SH           1.0     0.0     NaN
Shenzhen     1.0  5433.0  5433.0
shanghai     1.0     0.0     NaN
```

#### 数据统计

- 采样

  > 默认 replace  为 false，不放回采样

  ```python
  df.sample(n=3) # 简单采样，每一行取到的概率一致
  df.sample(n=3, weights=[0, 0, 0, 0, 0.5, 0.5]) # 设置每一行取到的概率
  df.sample(n=6, replace=False) # 不放回采样
  df.sample(n=6, replace=True) # 放回采样
  ```

- 标准差、协方差、相关性

  > 注意：统计的数据列只能是 int 或 float  类型，其他的列自动排除掉不进入统计

  ```python
  # 数据表描述性统计，round函数设置显示小数位，T表示转置
  df.describe().round(2).T
  
  # 计算列的标准差
  df['price'].std()
  
  # 计算两个字段间的协方差，如果两个字段一致则是计算方差
  df['price'].cov(df1['m-point'])
  
  # 数据表中所有字段间的协方差
  df.cov()
  
  # 两个字段的相关性分析
  # 相关系数在-1到1之间，接近1为正相关，接近 -1 为负相关，0 为不相关
  df['price'].corr(df1['m-point'])
  
  # 数据表的相关性分析
  df.corr()
  ```

  输出为：

  ```
         count     mean      std   min    25%     50%      75%     max
  age      6.0    36.50    10.88  23.0   32.0    33.0    41.50    54.0
  price    4.0  2773.75  2570.75  30.0  907.5  2816.0  4682.25  5433.0
  
  2570.749355732682
  
  6608752.25
                  age         price
  age      118.300000 -1.539958e+04
  price -15399.583333  6.608752e+06
  0.8824707496482248
  
              age     price
  age    1.000000 -0.453798
  price -0.453798  1.000000
  ```

#### 保存数据

```python
df.to_excel('result.xlsx', sheet_name='result')
df.to_csv('result.csv')
df.to_json('result.json')
```







