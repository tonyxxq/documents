## DataFrame

```scala
val spark = SparkSession.builder().appName("Spark_SQL_FIRST").getOrCreate()

// 加入隐式变换，将RDD转换为DataFrame
import spark.implicits._

// 读取 json 文件
val df = spark.read.json("person1.json")
df.show()

// 打印表结构
df.printSchema()

// 执行查询出操作
df.select($"name", $"age" + 1).show()
df.filter($"name" > 21).show()
df.groupBy("age").count().show()

// 根据 df 创建名称为 people 的临时表，并执行查询操作
df.createOrReplaceTempView("people")
spark.sql("SELECT * FROM people").show
spark.sql("SELECT * FROM people WHERE age<20").show

// 常用的 DataFrame 的 Action 操作

// 使用collect 以 Array 形式返回 DataFrame 的所有Rows
println("rows", df.collect())

// 使用 count 返回 DataFrame 的所有 Rows 数目
println("rows count", df.count())

// 返回第一行
println("第一行", df.first())
println("第一行", df.head())

// 返回前 N 行
println("前两行", df.take(2))

// columns 查询列名, 参数为列的索引
println("第一列名", df.columns(0))

// dtyps， 返回所有列名和数据结构， 不指定索引表示所有，以数组返回
println("所有的列", df.dtypes)
println("第一列", df.dtypes(0))

// explain 打印执行计划
println(df.explain())

// persist 数据持久话
df.persist()

// 打印树形结构的schema
df.printSchema()

// 类似于 SQL 函数
df.filter($"age">20).show()
df.sort("age").show
df.limit(2).show

// intersect 与另外一个 DataFrame 交集
val df2 = spark.read.json("person2.json")
df.intersect(df2).show


// RDD 转换为 DataFrame

// 1. 以反射机制推断 RDD 模式

// 将一个 RDD 隐式转换为一个 DataFrame
import spark.implicits._

// 从文本文件中创建一个 RDD，并且将其转换为 DataFrame
val peopleDF = spark.sparkContext.textFile("people.txt").map(_.split(",")).map(attr => Person(attr(0), attr(1).trim.toInt)).toDF()

// 注册一张临时表，表名为 people
peopleDF.createOrReplaceTempView("people")

// 使用 SQL 语句查询 13 到 19 岁之间的 people，对查询出的数据进行操作，索引和字段名称都可以
val teenagersDF = spark.sql("SELECT name, age FROM people WHERE age BETWEEN 13 AND 19")
teenagersDF.map(person => "Name: " + person(0)).show()
teenagersDF.map(person => "Name: " + person.getAs[String]("name")).show()

// 使用隐式转换 ncoder[Map[String, Any]] = ExpressionEncoder()
implicit val mapEncoder = org.apache.spark.sql.Encoders.kryo[Map[String, Any]]

// 将所有列按照 list 指定的元素放入一个 map 中
println(teenagersDF.map(person => person.getValuesMap[Any](List("name", "age"))).collect())


// 2. 使用编程方式定义 RDD 模式
// 2.1 从原始 RDD 中创建一个 Rows 的 RDD
// 2.2 创建一个表示为 StructType 类型的 Shema，匹配在第一步创建的 RDD 的 Rows 的结构。
// 2.3 通过 SparkSession 提供的 createDataFrame 方法，应用 Schema 到 Rows 的 RDD。

import org.apache.spark.sql.types._
import org.apache.spark.sql.Row

// 创建一个 RDD
val peopleRDD = spark.sparkContext.textFile("people.txt")

// 创建一个包含 Schema 的字符串
val schemaString = "name age"

// 创建一个 StructType 类型的 Schema
val fields = schemaString.split(" ").map(fieldName => StructField(fieldName, StringType, nullable = true))
val schema = StructType(fields)

// 将 RDD 转换为 Row
val rowRDD = peopleRDD.map(_.split(",")).map(attributes => Row(attributes(0), attributes(1).trim))

// 使用 Row 和 Schema 创建一个 DataFrame
val peopleDF2 = spark.createDataFrame(rowRDD, schema)

// 创建一张临时表
peopleDF2.createOrReplaceTempView("people")
val results = spark.sql("SELECT name FROM people")

// 展示查询的第一个字段的结果
results.map(attributes => "Name: " + attributes.getAs[String]("name")).show()


// DataFrame 转换为 DataSet
val personDS = spark.read.json("person1.json").as[Person]
personDS.show()
```

上面用到的样例类

```scala
case class Person(name:String, age:Int)
```

```scala
df.printSchema()

// 修改列类型, 可以封装成一个函数
val df_1 = df.withColumnRenamed("Year","oldYear")
val df_2 = df_1.withColumn("Year",df_1.col("oldYear").cast("int")).drop("oldYear")
```

