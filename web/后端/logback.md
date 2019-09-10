## logback

#### 介绍

- logback 是 java 的日志开源组件，是 log4j 创始人写的，性能比 log4 j要好，目前主要分为 3 个模块

  1. logback-core：核心代码模块

  2. logback-classic：log4j 的一个改良版本，同时实现了 slf4j 的接口

  3. logback-access：访问模块与 servlet 容器集成提供通过 http 来访问日志的功能

- logback 依赖于 slf4j，关于他们的区别

  log4j 和 logback 是真正实现日志功能的产品，slf4j 是一个适配器，不是具体的实现，我们通过调用 slf4 j的日志方法统一打印我们的日志

#### 实现

1. 添加依赖

   ```xml
   <!--这个依赖直接包含了 logback-core 以及 slf4j-api的依赖-->
   <dependency>
        <groupId>ch.qos.logback</groupId>
        <artifactId>logback-classic</artifactId>
        <version>1.2.3</version>
   </dependency>
   ```

2. 添加配置文件

   > configuration 主要配置三个节点的信息， appender、logger、root\
   >
   > 关于日志输出格式解析
   >
   > ```xml
   > <!--
   > 	%d：表示日期,默认是 yyyy-MM-dd，后面添加 {}， 可以自定义格式
   > 	%thread： 表示线程名
   > 	%-5level：级别从左显示5个字符宽度
   > 	%logger：表示 logger 的名称，默认一般为类名
   > 	%msg：日志消息
   > 	%n：换行符
   > -->
   > 
   > %d{yyyy-MM-dd HH:mm:ss.SSS} [%thread] %-5level %logger{50} - %msg%n
   > ```

   详细的配置文件解析

   ```xml
   <?xml version="1.0" encoding="UTF-8"?>
   <!--
   scan: 当此属性设置为true时，配置文件如果发生改变，将会被重新加载，默认值为true
   scanPeriod: 当 scan 为 true 时，此属性生效，默认的时间间隔为1分钟
   debug: 当此属性设置为true时，将打印出logback内部日志信息，实时查看logback运行状态。默认值为false
   -->
   <configuration scan="true" scanPeriod="60 seconds" debug="false">
       <!--定义常量，后面可以使用 ${} 直接引用-->
       <!--定义日志文件的存储地址 勿在 LogBack 的配置中使用相对路径-->
       <property name="LOG_HOME" value="D:\\logs" />
       <property name="APP_NAME" value="kkb" />
   
   
       <!-- 输出到控制台 -->
       <appender name="STDOUT" class="ch.qos.logback.core.ConsoleAppender">
           <!-- 日志输出格式 -->
           <encoder class="ch.qos.logback.classic.encoder.PatternLayoutEncoder">
               <pattern>
                   %d{yyyy-MM-dd HH:mm:ss.SSS} [%thread] %-5level %logger{50} - %msg%n
               </pattern>
           </encoder>
       </appender>
   
   
       <!--每天生成一个日志文件-->
       <appender name="ERROR" class="ch.qos.logback.core.rolling.RollingFileAppender">
           <file>${LOG_HOME}/${APP_NAME}/error.log</file>
   
           <!--过滤日志的级别，只显示 ERROR 级别及其以上的-->
           <filter class="ch.qos.logback.classic.filter.ThresholdFilter">
               <level>ERROR</level>
           </filter>
   
           <!--设置日志输出格式-->
           <encoder>
               <Pattern>
                   [%d{yyyy-MM-dd HH:mm:ss.SSS}] [%5level] [%thread] %logger{0} %msg%n
               </Pattern>
               <charset>UTF-8</charset>
           </encoder>
   
           <!--设置输出文件的格式-->
           <rollingPolicy class="ch.qos.logback.core.rolling.TimeBasedRollingPolicy">
               <!--日志文件输出的文件名-->
               <FileNamePattern>
                   ${LOG_HOME}/${APP_NAME}/error_%d{yyyy-MM-dd}.log
               </FileNamePattern>
               <!--只保留过去 30 天的日志数据-->
               <MaxHistory>30</MaxHistory>
               <!--总日志文件的最大限制为 2GB，超过会自动删除最久的日志记录-->
               <totalSizeCap>2G</totalSizeCap>
           </rollingPolicy>
       </appender>
   
       
       <!-- 每天可以生成多个日志文件（指定日志文件的大小） -->
       <appender name="FILE"  class="ch.qos.logback.core.rolling.RollingFileAppender">
           <file>${LOG_HOME}/${APP_NAME}/info.log</file>
   
           <!--过滤日志的级别，只显示 INFO 级别及其以上的-->
           <filter class="ch.qos.logback.classic.filter.ThresholdFilter">
               <level>INFO</level>
           </filter>
   
           <!--设置日志输出格式-->
           <encoder class="ch.qos.logback.classic.encoder.PatternLayoutEncoder">
               <pattern>
                   %d{yyyy-MM-dd HH:mm:ss.SSS} [%thread] %-5level %logger{50} - %msg%n
               </pattern>
           </encoder>
   
           <!--设置每个文件的最大 100MB-->
           <rollingPolicy class="ch.qos.logback.core.rolling.SizeAndTimeBasedRollingPolicy">
               <FileNamePattern>
                   ${LOG_HOME}/${APP_NAME}/info_%d{yyyy-MM-dd}_%i.log
               </FileNamePattern>
               <!--每个文件最大存储空间-->
               <maxFileSize>20MB</maxFileSize>
               <!--只保留过去 30 天的日志数据-->
               <MaxHistory>30</MaxHistory>
               <!--总日志文件的最大限制为 2GB，超过会自动删除最久的日志记录-->
               <totalSizeCap>2G</totalSizeCap>
           </rollingPolicy>
       </appender>
   
       
       <!-- 日志输出级别 -->
       <root level="INFO">
           <appender-ref ref="STDOUT" />
           <appender-ref ref="FILE" />
           <appender-ref ref="ERROR"/>
       </root>
   
   
       <!-- 定义指定的包或者类的日志文件输出
   	   - name：表示包名或类名的全限定名，拦截指定的包或类的日志输出
          - level: 设置日志输出级别
          - additivity：是否向上级传递，如果设置为true，会打印两次
          - appender-ref：指定了名字为"STDOUT"的appender。
       -->
       <!--<logger name="com.kkb.service" additivity="false" level="error">-->
           <!--<appender-ref ref="ERROR"></appender-ref>-->
       <!--</logger>-->
   </configuration>
   ```
   
3. 添加测试代码

   ```java
   // 这是slf4j的接口，由于我们引入了 logback-classic 依赖，所以底层实现是 logback
   private static final Logger logger = LoggerFactory.getLogger(Test.class);
   
   public static void main(String[] args) throws InterruptedException {
       logger.info("hello world");
   }
   ```

   