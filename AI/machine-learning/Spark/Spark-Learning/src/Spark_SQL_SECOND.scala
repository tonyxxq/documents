import org.apache.spark.sql.SparkSession

/**
  * Created by Administrator on 2018/11/2 0002.
  */
object Spark_SQL_SECOND {

  // 创建两个样例类进行匹配文本文件
  case class Emp(empId: Int, name: String, age: Int, depId: String)

  case class Dep(depId: String, name: String)

  def main(args: Array[String]): Unit = {

    val spark = SparkSession.builder().appName("Spark_SQL_SECOND").getOrCreate()

    import spark.implicits._

    val emp = spark.read.text("emp.txt").map(_.toString().split(" ")).map(attr => Emp(attr(0).replace("[", "").toInt, attr(1), attr(2).toInt, attr(3).replace("]", "")))
    val dep = spark.read.text("dep.txt").map(_.toString().split(" ")).map(attr => Dep(attr(0).replace("[", ""), attr(1).replace("]", "")))

    // 对两张表进行 join
    emp.join(dep, "depId").show()

    //

  }
}
