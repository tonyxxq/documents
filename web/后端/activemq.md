## ActiveMQ

### 一、JMS 介绍

- JMS 是什么？

  是消息发送和接收的一套规范

- JMS 的角色

  生产者、消费者、MOM 消息中间件（比如：ActiveMQ）

- JMS 作用

  异步通讯，完成程序之间的解耦

- JMS 消息模型

  点对点模型：只能一个取，消息的生产和消费没有时间上的相关性（消费者在消息产生之后能也获取消息）

  发布订阅模型：可以多个人取，类似广播模式，消息的生产和消费有时间上的相关性（消费者在消息产生之后不能获取消息）

- JMS 消息域

  Queue Region 和 Topic Region

- JMS 目的地

  Queue 目的地、Topic 目的地

- JMS 消息正文

  Stream、Map、Text、Byte、Object

### 二、下载与安装 ActiveMQ

> 前提条件，需要先安装 jdk

```bash
# 下载
wget https://mirrors.tuna.tsinghua.edu.cn/apache//activemq/5.15.9/apache-activemq-5.15.9-bin.tar.gz

# 解压缩
tar -zxvf apache-activemq-5.15.9-bin.tar.gz

# 进入 bin 目录，启动
./activemq start

ip:8161/admin

用户名都是 admin
```

通过浏览器访问控制台（IP 地址更换为 ActiveMQ 服务器所在的 IP ，访问不到可以关掉防火墙）
http://192.168.101.104:8161/admin



### 三、ActiveMQ常用接口

生产者：

```java
public class Producer {

    public static void main(String[] args) throws Exception {
        String broker = "tcp://192.168.101.104:61616";
        Connection connection = null;
        Session session = null;
        try {
            // 1. 创建 Connection  工厂用于连接 broker
            ConnectionFactory connectionFactory = new ActiveMQConnectionFactory(broker);

            // 2. 创建并且启动连接
            connection = connectionFactory.createConnection();
            connection.start();

            // 3. 创建 session
            // 第一个参数表示是否开启事务，第一个参数为 true, 第二个参数无效
            // 第二个参数表示指定的应答模式
            session = connection.createSession(false, Session.AUTO_ACKNOWLEDGE);

            // 4. 通过 session 创建 destination
            Queue queue = session.createQueue("kkb_queue");
			// Topic topic = session.createTopic("kkb_queue");
            
            // 5. 创建 producer
            MessageProducer producer = session.createProducer(queue);

            // 6. 创建 message，并发送
            Message message = session.createTextMessage("你有男朋友吗？");
            producer.send(message);
        } catch (Exception e) {
            e.printStackTrace();
        } finally {
            connection.close();
            session.close();
        }
    }
}
```

消费者：

```java
public class Consumer {
    public static void main(String[] args) throws Exception {
        String broker = "tcp://192.168.101.104:61616";
        Connection connection = null;
        Session session = null;
        try {
            // 1. 创建 Connection  工厂用于连接 broker
            ConnectionFactory connectionFactory = new ActiveMQConnectionFactory(broker);

            // 2. 创建并且启动连接
            connection = connectionFactory.createConnection();
            connection.start();

            // 3. 创建 session
            // 第一个参数表示是否开启事务，第一个参数为 true, 第二个参数无效
            // 第二个参数表示指定的应答模式
            session = connection.createSession(false, Session.AUTO_ACKNOWLEDGE);

            // 4. 通过 session 创建 destination
            Queue queue = session.createQueue("kkb_queue");
			// Topic topic = session.createTopic("kkb_queue");
            
            // 5. 创建 producer
            MessageConsumer consumer = session.createConsumer(queue);

            // 6. 接收消息
            // 一种是使用 receive 接收, 同步消息，没有收到消息会阻塞
            // Message message = consumer.receive(1000);
            // 一种是使用监听器，异步消息，没有收到消息不会阻塞
            consumer.setMessageListener(new MessageListener() {
                @Override
                public void onMessage(Message message) {
                    if (message instanceof TextMessage) {
                        TextMessage tm = (TextMessage) message;
                        try {
                            System.out.println(tm.getText());
                        } catch (JMSException e) {
                            e.printStackTrace();
                        }
                    }
                }
            });
        } catch (Exception e) {
            e.printStackTrace();
        } finally {
            connection.close();
            session.close();
        }
    }
}
```

也可以自己新建一个 broker，那么这个 broker 就和 远程服务器ActiveMQ 没有关系了，可以使用消费者和生产者连接这个 broker

```java
public class Broker {
    public static void main(String[] args) {
        try {
            BrokerService service = new BrokerService();
            service.setUseJmx(true);
            service.addConnector("tcp://localhost:61616");
            service.start();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
```

### 四、JMS 事务

![](imgs/126.png)

生产者发送应答消息给 Broker 是为了数据持久化

消费则发送应答消息给 Broker 是为了删除数据

应答消息分为四种：

指定事务（1种）：

SESSION_TRANSACTED：开启事务之后，使用该应用模式，需要执行 commit 进行消息应答

不指定事务（3种）：
AUTO_ACKNOWLEDGE：调用 send 方法之后，自动完成消息应答

CLIENT_ACKNOWLEDGE：需要生产者或消费者使用 acknowledge 完成手动应答

DUPS_OK_ACKNOWLEDGE：当接收消息数量到达一定阈值之后，通过一个ACK指令把它们全部确认

