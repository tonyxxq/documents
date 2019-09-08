# Maven

#### Maven 依赖常见的范围（sope）

- compile：编译依赖，**默认的依赖方式**，在编译（编译项目和编译测试用例），运行测试用例，运行（项目实际运行）三个阶段都有效，典型地有 spring-core 等 jar
- test: 测试依赖，只在编译测试用例和运行测试用例有效，典型地有 JUnit
- provided: 对于编译和测试有效，不会打包进发布包中，典型的例子为servlet-api, 一般的 web 工程运行时都使用容器的 servlet-api
- runtime: 编译时不需要，在运行测试用例和实际运行时有效。这种主要是指代码里并没有直接引用而是根据配置文件在运行时动态加载并实例化的情况。例如：DBCP 连接池

#### maven 的生命周期

> Maven 拥有三套相互独立的生命周期: clean、default 和 site, 而每个生命周期包含一些 phase 阶段, 阶段是有顺序的, 并且后面的阶段依赖于前面的阶段. 而三套生命周期相互之间却并没有前后依赖关系
>
> **生命周期的每个阶段 （phase） 的作用有点类似于 Java 语言中的接口，只协商了一个契约，但并没有定义具体的动作**
>
> 之所以能执行一些动作是因为 maven 在某些 phase **默认**已经绑定了一些具体的动作（plugin）

clean 生命周期：

```
pre-clean：执行一些需要在clean之前完成的工作
clean：移除所有上一次构建生成的文件
post-clean：执行一些需要在clean之后立刻完成的工作
```

default 生命周期：

```
validate： 用于验证项目的有效性和其项目所需要的内容是否具备
initialize：初始化操作，比如创建一些构建所需要的目录等。
generate-sources：用于生成一些源代码，这些源代码在compile phase中需要使用到
process-sources：对源代码进行一些操作，例如过滤一些源代码
generate-resources：生成资源文件（这些文件将被包含在最后的输入文件中）
process-resources：对资源文件进行处理
compile：对源代码进行编译
process-classes：对编译生成的文件进行处理
generate-test-sources：生成测试用的源代码
process-test-sources：对生成的测试源代码进行处理
generate-test-resources：生成测试用的资源文件
process-test-resources：对测试用的资源文件进行处理
test-compile：对测试用的源代码进行编译
process-test-classes：对测试源代码编译后的文件进行处理
test：进行单元测试
prepare-package：打包前置操作
package：打包
pre-integration-test：集成测试前置操作   
integration-test：集成测试
post-integration-test：集成测试后置操作
install：将打包产物安装到本地maven仓库
deploy：将打包产物安装到远程仓库
```

site 生命周期：

```
pre-site：执行一些需要在生成站点文档之前完成的工作
site：生成项目的站点文档
post-site： 执行一些需要在生成站点文档之后完成的工作，并且为部署做准备
site-deploy：将生成的站点文档部署到特定的服务器上
```

绑定插件 goal 到指定的 phase 示例：

```xml
<build>
    <plugins>
        <plugin>
            <groupId>org.apache.maven.plugins</groupId>
            <artifactId>maven-source-plugin</artifactId>
            <version>3.0.0</version>
            <executions>
                <!--将 jar-no-fork 绑定到 verify 阶段-->
                <execution>
                    <id>attach-sources</id>
                    <phase>verify</phase>
                    <goals>
                        <goal>jar-no-fork</goal>
                    </goals>
                </execution>
            </executions>
        </plugin>
    </plugins>
</build>
```

执行单独的 goal

> 不用指定 phase，只执行 goal

```bash
# 执行 tree 这个 goal
mvn org.apache.maven.plugins:maven-dependency-plugin:<版本号信息>:tree

# 可以简写（若 groupId 为 org.apache.maven.plugins 则可以使用上述的简写形式）
mvn dependency:tree
```

#### 聚合

> 父项目在执行构建操作的时候，会为下面的子项目分别进行构建操作，这样一条命令可以完成所有子项目的构建，提升效率
>
> 使用 IDEA ，在父项目下创建子项目会自动在父项目的 POM 文件中添加子模块

```xml
<project xmlns="http://maven.apache.org/POM/4.0.0"
         xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
         xsi:schemaLocation="http://maven.apache.org/POM/4.0.0
         http://maven.apache.org/xsd/maven-4.0.0.xsd">

    <modelVersion>4.0.0</modelVersion>

    <groupId>com.vdian.feedcenter</groupId>
    <artifactId>feedcenter-push</artifactId>
    
    <!--packaging 类型定义为 pom-->
    <packaging>pom</packaging>
    <version>1.0.0.SNAPSHOT</version>
	
    <!--子项目模块-->
    <modules>
        <module>feedcenter-push-client</module>
        <module>feedcenter-push-core</module>
        <module>feedcenter-push-web</module>
    </modules>
</project>
```

#### 继承

> dependencyManagement 和 pluginManagement定义在父 pom，子 pom 使用 parent 继承， 子 pom 想选择性继承且可以不指定版本号（也可指定），版本号由父 pom 统一管理

- dependencyManagement: 能让子 POM 继承父POM的配置的同时, 又能够保证子模块的灵活性: 在父POM dependencyManagement 元素配置的依赖声明不会实际引入子模块中, 但能够约束子模块 dependencies 下的依赖的使用 (子模块只需配置groupId与artifactId)，可以参考： spring-boot 
- pluginManagement: 与dependencyManagement类似,  配置的插件不会造成实际插件的调用行为, 只有当子POM 中配置了相关 plugin 元素, 才会影响实际的插件行为

示例：

父 pom：

```xml
<project xmlns="http://maven.apache.org/POM/4.0.0"
         xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
         xsi:schemaLocation="http://maven.apache.org/POM/4.0.0
         http://maven.apache.org/xsd/maven-4.0.0.xsd">

    <modelVersion>4.0.0</modelVersion>

    <groupId>com.vdian.feedcenter</groupId>
    <artifactId>feedcenter-push</artifactId>
    <packaging>pom</packaging>
    <version>1.0.0.SNAPSHOT</version>

    <!--定义一些可以公用的属性-->
    <properties>
        <finalName>feedcenter-push</finalName>
        <warName>${finalName}.war</warName>
        <spring.version>4.0.6.RELEASE</spring.version>
        <junit.version>4.12</junit.version>
        <project.build.sourceEncoding>UTF-8</project.build.sourceEncoding>
        <warExplodedDirectory>exploded/${warName}</warExplodedDirectory>
    </properties>

    <dependencyManagement>
        <dependencies>
            <dependency>
                <groupId>org.springframework</groupId>
                <artifactId>spring-core</artifactId>
                <version>${spring.version}</version>
            </dependency>
            <dependency>
                <groupId>org.springframework</groupId>
                <artifactId>spring-beans</artifactId>
                <version>${spring.version}</version>
            </dependency>
            <dependency>
                <groupId>org.springframework</groupId>
                <artifactId>spring-context</artifactId>
                <version>${spring.version}</version>
            </dependency>

           <dependency>
                <groupId>junit</groupId>
                <artifactId>junit</artifactId>
                <version>${junit.version}</version>
                <scope>test</scope>
            </dependency>
        </dependencies>
    </dependencyManagement>

    <build>
        <pluginManagement>
            <plugins>
                <plugin>
                    <groupId>org.apache.maven.plugins</groupId>
                    <artifactId>maven-source-plugin</artifactId>
                    <version>3.0.0</version>
                    <executions>
                        <execution>
                            <id>attach-sources</id>
                            <phase>verify</phase>
                            <goals>
                                <goal>jar-no-fork</goal>
                            </goals>
                        </execution>
                    </executions>
                </plugin>
            </plugins>
        </pluginManagement>
    </build>
</project>
```

子 pom:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<project xmlns="http://maven.apache.org/POM/4.0.0"
         xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
         xsi:schemaLocation="http://maven.apache.org/POM/4.0.0
         http://maven.apache.org/xsd/maven-4.0.0.xsd">
	
    <!--指定父 pom--->
    <parent>
        <groupId>com.vdian.feedcenter</groupId>
        <artifactId>feedcenter-push</artifactId>
        <version>1.0.0.SNAPSHOT</version>
    </parent>
    <modelVersion>4.0.0</modelVersion>
    <artifactId>feedcenter-push-client</artifactId>

    <dependencies>
        <dependency>
            <groupId>org.springframework</groupId>
            <artifactId>spring-core</artifactId>
        </dependency>
        <dependency>
            <groupId>org.springframework</groupId>
            <artifactId>spring-context</artifactId>
        </dependency>
        <dependency>
            <groupId>org.springframework</groupId>
            <artifactId>spring-beans</artifactId>
        </dependency>

        <dependency>
            <groupId>junit</groupId>
            <artifactId>junit</artifactId>
        </dependency>
    </dependencies>

    <build>
        <plugins>
            <plugin>
                <groupId>org.apache.maven.plugins</groupId>
                <artifactId>maven-source-plugin</artifactId>
            </plugin>
            <plugin>
                <groupId>org.apache.maven.plugins</groupId>
                <artifactId>maven-compiler-plugin</artifactId>
            </plugin>
        </plugins>
    </build>
</project>
```

#### 超级 POM

任何一个Maven项目都隐式地继承自超级POM, 因此超级POM的大量配置都会被所有的Maven项目继承, 这些配置也成为了Maven所提倡的约定.

位置为：%maven_home%\lib\maven-model-builder-3.3.9\org\apache\maven\model\pom-4.0.0.xml

```xml
!-- START SNIPPET: superpom -->
<project>
  <modelVersion>4.0.0</modelVersion>

  <!-- 定义了中央仓库以及插件仓库, 均为:https://repo.maven.apache.org/maven2 -->
  <repositories>
    <repository>
      <id>central</id>
      <name>Central Repository</name>
      <url>https://repo.maven.apache.org/maven2</url>
      <layout>default</layout>
      <snapshots>
        <enabled>false</enabled>
      </snapshots>
    </repository>
  </repositories>

  <pluginRepositories>
    <pluginRepository>
      <id>central</id>
      <name>Central Repository</name>
      <url>https://repo.maven.apache.org/maven2</url>
      <layout>default</layout>
      <snapshots>
        <enabled>false</enabled>
      </snapshots>
      <releases>
        <updatePolicy>never</updatePolicy>
      </releases>
    </pluginRepository>
  </pluginRepositories>

  <!-- 依次定义了各类代码、资源、输出目录及最终构件名称格式, 这就是Maven项目结构的约定 -->
  <build>
    <directory>${project.basedir}/target</directory>
    <outputDirectory>${project.build.directory}/classes</outputDirectory>
    <finalName>${project.artifactId}-${project.version}</finalName>
    <testOutputDirectory>${project.build.directory}/test-classes</testOutputDirectory>
    <sourceDirectory>${project.basedir}/src/main/java</sourceDirectory>
    <scriptSourceDirectory>${project.basedir}/src/main/scripts</scriptSourceDirectory>
    <testSourceDirectory>${project.basedir}/src/test/java</testSourceDirectory>
    <resources>
      <resource>
        <directory>${project.basedir}/src/main/resources</directory>
      </resource>
    </resources>
    <testResources>
      <testResource>
        <directory>${project.basedir}/src/test/resources</directory>
      </testResource>
    </testResources>

    <!-- 为核心插件设定版本 -->
    <pluginManagement>
      <!-- NOTE: These plugins will be removed from future versions of the super POM -->
      <!-- They are kept for the moment as they are very unlikely to conflict with lifecycle mappings (MNG-4453) -->
      <plugins>
        <plugin>
          <artifactId>maven-antrun-plugin</artifactId>
          <version>1.3</version>
        </plugin>
        <plugin>
          <artifactId>maven-assembly-plugin</artifactId>
          <version>2.2-beta-5</version>
        </plugin>
        <plugin>
          <artifactId>maven-dependency-plugin</artifactId>
          <version>2.8</version>
        </plugin>
        <plugin>
          <artifactId>maven-release-plugin</artifactId>
          <version>2.3.2</version>
        </plugin>
      </plugins>
    </pluginManagement>
  </build>

  <!-- 定义项目报告输出路径 -->
  <reporting>
    <outputDirectory>${project.build.directory}/site</outputDirectory>
  </reporting>

  <!-- 定义release-profile, 为构件附上源码与文档 -->
  <profiles>
    <!-- NOTE: The release profile will be removed from future versions of the super POM -->
    <profile>
      <id>release-profile</id>

      <activation>
        <property>
          <name>performRelease</name>
          <value>true</value>
        </property>
      </activation>

      <build>
        <plugins>
          <plugin>
            <inherited>true</inherited>
            <artifactId>maven-source-plugin</artifactId>
            <executions>
              <execution>
                <id>attach-sources</id>
                <goals>
                  <goal>jar</goal>
                </goals>
              </execution>
            </executions>
          </plugin>
          <plugin>
            <inherited>true</inherited>
            <artifactId>maven-javadoc-plugin</artifactId>
            <executions>
              <execution>
                <id>attach-javadocs</id>
                <goals>
                  <goal>jar</goal>
                </goals>
              </execution>
            </executions>
          </plugin>
          <plugin>
            <inherited>true</inherited>
            <artifactId>maven-deploy-plugin</artifactId>
            <configuration>
              <updateReleaseInfo>true</updateReleaseInfo>
            </configuration>
          </plugin>
        </plugins>
      </build>
    </profile>
  </profiles>

</project>
```

#### 自定义插件

参考：<https://www.jianshu.com/p/b07e7dbd8e4c>

#### 添加 tomcat 插件

```xml
<build>
  	<plugins>
	  <!-- tomcat插件控制 -->
		<plugin>
		    <groupId>org.apache.tomcat.maven</groupId>
		    <artifactId>tomcat7-maven-plugin</artifactId>
		    <version>2.2</version>
		    <configuration>
                <--端口控制-->
				<port>8080</port>
                <--项目路径控制意味着http://localhost:8080/abc-->
				<path>/abc</path>
                <--编码-->
				<uriEncoding>UTF-8</uriEncoding>
			</configuration>
		</plugin>
	  
      <!-- maven插件控制 -->
      <plugin>
            <groupId>org.apache.maven.plugins</groupId>
            <artifactId>maven-compiler-plugin</artifactId>
            <version>3.1</version>
            <configuration>
                <source>1.8</source>
                <target>1.8</target>
                <encoding>utf-8</encoding>
            </configuration>
        </plugin>
  	</plugins>
</build>
```







