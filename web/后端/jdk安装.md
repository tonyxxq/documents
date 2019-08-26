

```bash
# 下载（可以选择版本，这里下载的是 jdk11）
wget https://repo.huaweicloud.com/java/jdk/11.0.2+7/jdk-11.0.2_linux-x64_bin.tar.gz

# 解压到指定目录
mkdir /usr/java
tar -zxvf jdk-11.0.2_linux-x64_bin.tar.gz  -C /usr/java

# 配置环境变量
vim /etc/profile
# 添加如下内容
JAVA_HOME=/usr/java/jdk-11.0.2
export PATH=$JAVA_HOME/bin:$PATH

# 更新环境变量
source /etc/profile

# 测试
java -version
```



