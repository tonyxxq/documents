## MongoDB

### 基本概念

#### 什么是 NoSQL

NoSQL（Not Only SQL），指的是非关系型的数据库，NoSQL 用于超大规模数据的存储。这些类型的数据存储不需要固定的模式，无需多余操作就可以横向扩展。

#### 为什么使用 mongoDB

现实中有不少的非结构化数据，需要用到非关系型数据库

特点如下：

- MongoDB 是一个面向文档存储的数据库，操作起来比较简单和容易
- 你可以在 MongoDB 记录中设置任何属性的索引 (如：FirstName="Sameer",Address="8 Gandhi Road")来实现更快的排序
- 你可以通过本地或者网络创建数据镜像，这使得 MongoDB 有更强的扩展性
- 如果负载的增加（需要更多的存储空间和更强的处理能力） ，它可以分布在计算机网络中的其他节点上这就是所谓的分片
- Mongo支持丰富的查询表达式。查询指令使用 JSON 形式的标记，可轻易查询文档中内嵌的对象及数组
- MongoDb 使用 update() 命令可以实现替换完成的文档（数据）或者一些指定的数据字段 
- Mongodb中的 Map/reduce 主要是用来对数据进行批量处理和聚合操作
- Map 和 Reduce。Map函数调用 emit(key,value) 遍历集合中所有的记录，将 key 与 value 传给 Reduce函数进行处理
- Map 函数和 Reduce 函数是使用 Javascript 编写的，并可以通过 db.runCommand 或 mapreduce 命令来执行 MapReduce 操作
- GridFS 是 MongoDB 中的一个内置功能，可以用于存放大量小文件
- MongoDB 允许在服务端执行脚本，可以用 Javascript 编写某个函数，直接在服务端执行，也可以把函数的定义存储在服务端，下次直接调用即可
- MongoDB支持各种编程语言:RUBY，PYTHON，JAVA，C++，PHP，C# 等多种语言
- MongoDB 安装简单

- 安装

  参考：https://www.runoob.com/mongodb/mongodb-linux-install.html

#### 常用术语

| SQL术语/概念 | MongoDB术语/概念 | 解释/说明                           |
| :----------- | :--------------- | :---------------------------------- |
| database     | database         | 数据库                              |
| table        | collection       | 数据库表/集合                       |
| row          | document         | 数据记录行/文档                     |
| column       | field            | 数据字段/域                         |
| index        | index            | 索引                                |
| table joins  |                  | 表连接,MongoDB不支持                |
| primary key  | primary key      | 主键,MongoDB自动将_id字段设置为主键 |

数据类型

| 数据类型           | 描述                                                         |
| :----------------- | :----------------------------------------------------------- |
| String             | 字符串。存储数据常用的数据类型。在 MongoDB 中，UTF-8 编码的字符串才是合法的。 |
| Integer            | 整型数值。用于存储数值。根据你所采用的服务器，可分为 32 位或 64 位。 |
| Boolean            | 布尔值。用于存储布尔值（真/假）。                            |
| Double             | 双精度浮点值。用于存储浮点值。                               |
| Min/Max keys       | 将一个值与 BSON（二进制的 JSON）元素的最低值和最高值相对比。 |
| Array              | 用于将数组或列表或多个值存储为一个键。                       |
| Timestamp          | 时间戳。记录文档修改或添加的具体时间。                       |
| Object             | 用于内嵌文档。                                               |
| Null               | 用于创建空值。                                               |
| Symbol             | 符号。该数据类型基本上等同于字符串类型，但不同的是，它一般用于采用特殊符号类型的语言。 |
| Date               | 日期时间。用 UNIX 时间格式来存储当前日期或时间。你可以指定自己的日期时间：创建 Date 对象，传入年月日信息。 |
| Object ID          | 对象 ID。用于创建文档的 ID。                                 |
| Binary Data        | 二进制数据。用于存储二进制数据。                             |
| Code               | 代码类型。用于在文档中存储 JavaScript 代码。                 |
| Regular expression | 正则表达式类型。用于存储正则表达式。                         |

![](imgs/285.png)

ObjectId 类似唯一主键，可以很快的去生成和排序，包含 12 bytes，含义是：

- 前 4 个字节表示创建 **unix** 时间戳，格林尼治时间 **UTC** 时间，比北京时间晚了 8 个小时

- 接下来的 3 个字节是机器标识码

- 紧接的两个字节由进程 id 组成 PID

- 最后三个字节是随机数

### 常用功能

> 文档是一组键值 (key-value) 对 (即 BSON)。MongoDB 的文档不需要设置相同的字段，并且相同的字段不需要相同的数据类型，这与关系型数据库有很大的区别，也是 MongoDB 非常突出的特点。

```bash
# 连接
mongodb://[username:password@]host1[:port1][,host2[:port2],...[,hostN[:portN]]][/[database][?options]]

show dbs

# 查看当前的数据库
db

# 切换数据库，不存在则创建
use local

# 删除当前数据库
db.dropDatabase()

# 创建集合，可以显示创建也可以在插入数据的时候会自动创建
# db.createCollection("mydocs")
db.mydocs.insert({'name':'tony', 'age':28})

# 删除集合
db.mydocs.drop()

# 插入文档
db.mydocs.insert({"name":'tony',age:20})
# 通过数组的方式一次性插入多条
#var arr = [];
#for(var i=1 ; i<=20000 ; i++){
#    arr.push({num:i});
#}
#db.mydocs.insert(arr);

# 更新文档
# 第一个参数查询条件，相当于 sql 中的 where
# 第二个参数，更新的内容，相当于 sql 中的 set
# 第三个参数：是否在没有查询到记录的时候，插入查询条件的文档
# 第四个参数：是否查询多行，默认为 false
db.mydocs.update({'age':{$lt:0}}, {$set:{'age':10000}}, true, true)

# 删除文档,remove 方法已经过时了
# remove 方法删除之后不会释放空间
db.mydocs.deleteMany({name:'tony'})
db.mydocs.deleteOne({name:'tony'})

# 查询文档
db.mydocs.find().pretty()
# 查询条件中的值大小比较 $lt，$gt, $lte, $gte，$ne（不等于）
db.mydocs.find({'age':{$lt:80}}).pretty()
# and or
db.mydocs.find({$or:[{'age':20}, {'name':'gary'}]}).pretty()
db.mydocs.find({$and:[{'age':20}, {'name':'tony2'}]}).pretty()
db.mydocs.find({'sex':'male', $or:[{'age':20}, {'name':'tony2'}]}).pretty()

# 查询或删除指定类型的数据
db.mydocs.find({age:{$type: 'string'}})

# limit 查询结果个数
db.mydocs.find().limit(2).pretty()
# skip 跳过前面指定个数，即从第几个开始
db.mydocs.find().skip(2).pretty()
db.mydocs.find().skip(2).limit(2).pretty()

# sort 排序，1 升许，-1 降序
db.mydocs.find().sort({'age':1})

# 索引
# 创建、删除
db.mydocs.createIndex({"name":1})
db.mydocs.createIndex({"name":1, 'age':1}, {'background': true, 'unique': true, 'name':'name_index'})
db.col.dropIndexes()
db.mydocs.dropIndex("name_index")
```

聚合

> 管道，前面的数据传给后面继续进行处理，aggregate 接收一个数组，前面的处理结果传给后面
>
> - $project：修改输入文档的结构。可以用来重命名、增加或删除域，也可以用于创建计算结果以及嵌套文档。
> - $match：用于过滤数据，只输出符合条件的文档。$match使用MongoDB的标准查询操作
> - $limit：用来限制MongoDB聚合管道返回的文档数
> - $skip：在聚合管道中跳过指定数量的文档，并返回余下的文档
> - $unwind：将文档中的某一个数组类型字段拆分成多条，每条包含数组中的一个值
> - $group：将集合中的文档分组，可用于统计结果
> - $sort：将输入文档排序后输出
> - $geoNear：输出接近某一地理位置的有序文档

```bash
# 聚合 aggregate
# _id 表示需要分组的字段，后面是自定义的字段
db.mydocs.aggregate([{$group:{_id: '$name', num: {$avg: '$age'}}}])

# 统计次数（类似: select name, count(*) from mydocs group by name）
db.mydocs.aggregate([{$group:{_id: '$name', num: {$sum: 1}}}])

# 管道进行组合
db.mydocs.aggregate([{$group:{_id: '$name', num: {$sum: 1}}}, {$project: {'num': 'xxq'}}])

 db.mydocs.aggregate([{$group:{_id: '$name', num: {$sum: 1}}}, {$project: {'num': 'xxq'}}, {$limit: 2}, {$sort: {'xxq':1}}]）
```

### 副本与分片

主节点记录在其上的所有操作 oplog，**从节点定期轮询主节点**获取这些操作，然后对自己的数据副本执行这些操作，从而保证从节点的数据与主节点一致

.....

### JAVA 客户端

添加依赖

```xml
<dependency>
	<groupId>org.mongodb</groupId>
	<artifactId>mongodb-driver</artifactId>
	<version>3.11.0</version>
</dependency>
```

代码

```java
public class Mongo {

    public static void main(String[] args) {
        // 连接到 mongodb 服务
        MongoClient mongoClient = new MongoClient("localhost", 27017);

        // 连接（创建）数据库
        MongoDatabase mydb = mongoClient.getDatabase("mydb");

        // 获得集合，若没有会自动创建
        MongoCollection<Document> mycol = mydb.getCollection("mycol");

        // 插入数据
        // 创建文档 org.bson.Document 参数为key-value的格式，value 可以为多种格式
        Document document = new Document("name", "xxq")
                .append("age", 28)
                .append("birthday", new Date())
                .append("other", new Document());
        mycol.insertOne(document);

        // 查询文档
        Document doc = new Document().append("name", "xxq");

        FindIterable<Document> findIterable = mycol.find(doc);

        MongoCursor<Document> mongoCursor = findIterable.iterator();
        while (mongoCursor.hasNext()) {
            System.out.println(mongoCursor.next());
        }

        // 更新文档，Document 或 Filter 作为条件
        mycol.updateMany(Filters.eq("name", "xxq"), new Document("$set", new Document("name", "tony")));

        // 删除文档，Document 或 Filter 作为条件
        mycol.deleteMany(Filters.eq("name", "tony"));
        // mycol.deleteMany(new Document().append("name", "tony"));
    }
}
```

### 整合 Spring

添加依赖

```

```

















































