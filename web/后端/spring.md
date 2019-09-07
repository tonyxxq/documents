# Spring

#### IoC 原理

​	什么是 IOC

​	什么是 Spring 容器

​	如何创建 Spring 容器

​	Spring 容器如何初始化？（如何创建 bean 实例）

#### DI 介绍

#### IoC 使用



## Spring 整合 Junit

1. 添加依赖包

   ```xml
   <!-- 单元测试 Junit -->
   <dependency>
   	<groupId>junit</groupId>
   	<artifactId>junit</artifactId>
   	<version>4.12</version>
   </dependency>
   
   <!--spring-test-->
   <dependency>
   	<groupId>org.springframework</groupId>
   	<artifactId>spring-test</artifactId>
   	<version>5.0.7.RELEASE</version>
   </dependency>
   ```

2. 通过 @RunWith 注解，指定 spring 的运行器，使用 @ContextConfiguration 注解指定配置文件或配置类的位置

   ```java
   @RunWith(SpringJUnit4ClassRunner.class) // 替换运行期为 Spring 自己提供的运行器
   // @ContextConfiguration(classes = Configuration.class) // 纯注解，指定注解配置
   @ContextConfiguration(locations = "classpath:application-context.xml") // 配置文件
   public class MainTest {
   
       @Autowired
       UserService userService;
   
       @Test
       public void test(){
           userService.saveUser();
       }
   }
   ```



## Spring AOP 原理分析

> OOP 的继承是纵向抽取，AOP 是横向抽取
>
> 将业务逻辑和系统处理的代码（性能检测，日志记录，事务处理等）解耦，是对业务逻辑的增强

相关术语：

1. 连接点

2. 切入点

   ....



#### 两种动态代理技术（ jdk 的方式和 cglib 方式）

```java
public class MyProxyUtils {


    /**
     * 使用 JDK 的方式动态代理（基于接口）
     */
    public static UserService getProxy(final UserService userService) {
        // 第一个参数：目标对象类加载器
        // 第二个参数：目标对象接口
        // 第三个参数：代理对象的执行处理器
        UserService proxy = (UserService) Proxy.newProxyInstance(
                userService.getClass().getClassLoader(),
                userService.getClass().getInterfaces(),
                // 监控目标对象的行为，当调用方法时执行如下的代码
                new InvocationHandler() {
                    public Object invoke(Object proxy, Method method, Object[] args) throws Throwable {
                        // 增加增强代码
                        System.out.println("记录日志-开始.............");

                        // 该行代码调用的实际是目标对象的方法
                        Object object = method.invoke(userService, args);

                        // 增加增强代码
                        System.out.println("记录日志-结束.............");

                        return object;
                    }
                });

        return proxy;
    }

    /**
     * 使用 Cglib 的方式动态代理（继承的方式）
     */
    public static UserService getCglibProxy(UserService userService) {
        // 创建增强对象
        Enhancer enhancer = new Enhancer();

        // 设置需要增强的类对象
        enhancer.setSuperclass(userService.getClass());

        // 设置回调函数
        enhancer.setCallback(new MethodInterceptor() {
            public Object intercept(Object object, Method method, Object[] args, MethodProxy methodProxy) throws Throwable {

                System.out.println("记录开始时间：" + System.currentTimeMillis());

                // 该行代码实际调用的是目标对象的方法
                Object object2 = methodProxy.invokeSuper(object, args);

                System.out.println("记录结束时间：" + System.currentTimeMillis());

                return object2;
            }
        });

        // 获取增强之后的类
        return (UserService) enhancer.create();
    }
}
```



#### Spring AOP 的使用(Aspect)

- xml 的方式

  1. 添加依赖

     ```xml
     <!-- 基于 AspectJ 的 aop 依赖 -->
     <dependency>
         <groupId>org.springframework</groupId>
         <artifactId>spring-aspects</artifactId>
         <version>5.0.7.RELEASE</version>
     </dependency>
     
     <dependency>
         <groupId>aopalliance</groupId>
         <artifactId>aopalliance</artifactId>
         <version>1.0</version>
     </dependency>
     ```

  2. 编写**通知**（增强类，一个普通的类，不需要继任何类和实现任何接口）

     ```java
     public class MyAdvice {
         public void log() {
             System.out.println("写日志了。。。。。。。。。。");
         }
     
         /**
          * 环绕通知（没有返回值可以不返回）
          */
         public Object log2(ProceedingJoinPoint joinPoint) throws Throwable {
             System.out.println("开始记录日志");
     
             Object object = joinPoint.proceed();
     
             System.out.println("结束记录日志");
     
             return object;
         }
     }
     ```

  3. 配置切面，将通知类交给spring IoC容器管理

     > **execution([修饰符] 返回值类型 包名.类名.方法名(参数))**
     >
     > 修饰符：可省略
     >
     > 返回值类型：必须要，但是可以使用 \* 通配符
     >
     > 包名：多级包之间使用.分割，包名可以使用  \* 代替，多级包名可以使用多个 \* 代替， 如果想省略          中间的包名可以使用 ..
     >
     > 类名： 可以使用 \* 代替， *也可以写成  \*DaoImpl
     >
     > 方法名： 可以使用 \* 好代替，也可以写成类似 add*    
     >
     > 参数：    参数使用 \* 代替，  如果有多个参数，可以使用  .. 代替             

     ```xml
     <bean id="myAdvice" class="com.kkb.advice.MyAdvice"></bean>
     
     <aop:config>
         <!--切面， ref 指定通知-->
         <aop:aspect ref="myAdvice">
             <!--通知中的 method 增强方法-->
             <!--pointcut 目标对象切入点-->
             <!--<aop:before method="log" pointcut="execution(void com.kkb.spring.service.UserService.saveUser())"></aop:before>-->
             <!--<aop:after method="log" pointcut="execution(void com.kkb.spring.service.UserService.saveUser())"></aop:after>-->
             <!--<aop:after-returning method="log" pointcut="execution(void com.kkb.spring.service.UserService.saveUser())"></aop:after-returning>-->
             <!--<aop:after-throwing method="log" pointcut="execution(void com.kkb.spring.service.UserService.saveUser())"></aop:after-throwing>-->
             <!--环绕通知-->
             <aop:around method="log2" pointcut="execution(void *..*.*Service.*(..))"></aop:around>
         </aop:aspect>
     </aop:config>
     ```

- 注解方式

  1. 编写切面类（注意不是通知类，因为包含切入点）

     ```java
     // 表示该类是一个切面类
     @Aspect
     // 交给 Spring IoC 容器管理
     @Component("myAspect")
     public class MyAspect {
     
         // 定义该方法是一个前置通知
         @Before(value = "execution(* *..*.*(..))")
         public void before() {
             System.out.println("注解前置通知");
         }
     
         // 定义该方法是一个环绕通知
         @Around(value = "execution(* *..*.service.save*(..))")
         public void around(ProceedingJoinPoint joinPoint) throws Throwable {
             System.out.println("注解环绕通知开始");
             joinPoint.proceed();
             System.out.println("注解环绕通知结束");
         }
     }
     ```

  2. 编写自动代理

     ```xml
     <context:component-scan base-package="com.kkb"></context:component-scan>
     <!--开启 AOP 自动代理-->
     <aop:aspectj-autoproxy/>
     ```

     或使纯注解的方式

     ```
     
     ```




### Spring 操作数据库

#### JdbcTemplate

#### JdbcDaoSupport



### Spring 事务管理

#### 编程式事务管理（不推荐）

#### 声明式事务管理（重点）

- 基于 AspectJ + xml 方式

  ```xml
  <bean id="transactionManager" class="org.springframework.jdbc.datasource.DataSourceTransactionManager">
      <property name="dataSource" ref="dataSource"></property>
  </bean>
  
  <!--处理器就是 TransactionInterceptor 类 实现了 MethodInterceptor 接口-->
  <tx:advice id="txAdvice" transaction-manager="transactionManager">
      <!--设置事务管理信息-->
      <tx:attributes>
          <!--增删改使用 REQUIRED 事务行为-->
          <!--查询使用 READ-ONLY-->
          <tx:method name="transfer" propagation="REQUIRED"/>
      </tx:attributes>
  </tx:advice>
  
  <aop:config>
      <!--Spring 已经实现了该增强功能， Spring 使用的是 MethodInterceptor 接口方式实现的-->
      <aop:advisor advice-ref="txAdvice" pointcut="execution(* *..*.*ServiceImpl.*(..))"></aop:advisor>
  </aop:config>
  ```

  

- 基于 AspectJ + 注解 方式

  1. 配置

     ```xml
     <bean id="transactionManager" class="org.springframework.jdbc.datasource.DataSourceTransactionManager">
         <property name="dataSource" ref="dataSource"></property>
     </bean>
     
     <tx:annotation-driven transaction-manager="transactionManager"/>
     ```

     

  > @Transactional

  

  