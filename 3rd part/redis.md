#### 安装 redis 和修改配置文件

1. 安装 redis

   ```
   sudo apt-get install redis-server
   ```


2. 修改 redis 配置文件,允许远程访问且修改密码

   ```
   sudo gedit /etc/redis/redis.conf

   # 注释掉,允许远程访问
   bind 127.0.0.1

   # 修改密码,默认为 foobared
   requirepass 123456

   # 重启 redis
   sudo service redis-server restart

   # 连接 redis
   redis-cli -a 123456
   ```

#### 安装 mongodb

1. ​

#### 爬虫常用库的安装

1. urllib 和 re

   安装 python 的时候默认就已经安装

2. requests

   ```
   pip install requests
   ```

3. selenium

   ```
   pip install selenium
   ```

4. chromedriver

   在网站下载

   https://npm.taobao.org/mirrors/chromedriver/

   解压之后放到 python 的 bin 目录下,或其他环境变量的目录下,以便系统能访问

   ```python
   # 执行,下面代码可以打开 chrome 浏览器
   from selenium import webdriver
   driver = webdriver.Chrome()

   # 打开百度
   driver.get("http://www.baidu.com")
     
   # 查看网页的源码
   driver.page_source
   ```

5. pantomjs

   >  chromedriver 会打开浏览器,如果不需要浏览器使用 pantomjs

   下载

   http://phantomjs.org/download.html

   解压并添加环境变量

   ```
   bzip2 -d  phantomjs-2.1.1-linux-x86_64.tar.bz2
   tar -xvf phantomjs-2.1.1-linux-x86_64.tar
   sudo mv phantomjs-2.1.1-linux-x86_64 /usr/local/src
   sudo gedit /etc/environment
   ```

   测试

   ```python
   from selenium import webdriver
   driver = driver.PhantomJS()
   driver.get("http://www.baidu.com")
   ```

   提示 selenium 不支持 pantomjs,原因是 selenium 版本太高,或使用 chrome 加上 headless

6. lxml

   > xpath 的一些解析方式

   安装

   ```
   pip install lxml
   ```

7. beautifulsoup

   依赖 lxml, 所以需要先安装 lxml

   ```
   pip install beautifulsoup
   ```

   测试

   ```
   from bs4 import BeautifulSoup

   soup = BeautifulSoup("<html></html>", 'lxml')
   ```

8. pyquery

   > 基本和 JQuery 类似

   安装

   ```
   pip install pyquery
   ```

   测试

   ```python
   from pyquery import PyQuery as pq

   doc = pq("<div>xx</div>")
   doc('div').text()
   ```

9. pymysql

   安装

   ```
   pip install pymysql
   ```

   测试:

   .....

10. pymongo

    安装

    ```
    pip install pymongo
    ```

    测试

    ...

11. redis

    安装

    ```
    pip install redis
    ```

12. flask

    安装

    ```
    pip install flask
    ```

13. django

    安装

    ```
    pip install django
    ```

14. jupyter

    使用 Anaconda 或 pip install jupyter



#### 什么是爬虫















​	