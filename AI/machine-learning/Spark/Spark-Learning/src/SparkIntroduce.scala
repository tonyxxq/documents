import org.apache.spark.{SparkConf, SparkContext, SparkFiles}

import scala.util.Properties

/**
  * Created by Administrator on 2018/11/22 0022.
  */
object SparkIntroduce {
  def main(args: Array[String]): Unit = {
    // 连接到远端 spark 集群， 使用 Properties 可以获取当前的环境变量
    // val conf = new SparkConf().setAppName("Spark Pi").setMaster("spark://192.168.10.197:7077")
    val master = Properties.envOrElse("MASTER", "spark://192.168.10.197:7077")
    val sc = new SparkContext(master, "Spark Pi")

    // 把文件添加到 spark 的每一个结点
    sc.addFile("person1.txt")

    // 加载文件，从 addFile 里边或直接从本地取
    val text = sc.textFile(SparkFiles.get("person1.txt"))
    // val text = sc.textFile("person1.txt")
    print(text)

    // 停止 SparkContext
    sc.stop()
  }
}
