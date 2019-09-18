

## Nginx

#### nginx 概述

1. 代理服务器

   正向代理（架设在客户端的，比如：翻墙）和反向代理（架设在的服务端）

2. Nginx 的 特点

   - 高并发

     并发：同一时间端内运行程序的个数

     tomcat 默认的并发量（ QPS） 为 150， nginx 默认为 1000， 如果配置一下的话为 5-10万

   - 低消耗

   - 热部署

     修改配置文件之后，若重新启动，会平滑重启，服务不会间断	

   - 高扩展

   - 高可用

3. Nginx 的 web 请求处理机制

   并行请求处理方式：多进程方式，多线程方式，异步非阻塞方式

   ![](imgs/28.png)

#### 安装 Nginx

```bash
# 安装依赖
yum install -y  gcc-c++ pcre-devel  openssl-devel

# 下载 nginx 并解压
wget http://nginx.org/download/nginx-1.1.10.tar.gz
tar -zxvf nginx-1.1.10.tar.gz

# 进入 nginx 目录
cd nginx-1.1.10

# 执行安装， --prefix 指定安装到的目录，--with-http_gzip_static_module 安装 gzip 模块
# ./configure  --help 可以查看所有可以安装的模块，也可以安装外部扩展模块
./configure  \
	--prefix=/kkb/server/nginx \
	--with-http_gzip_static_module \

make && make install

# 进入 /kkb/server/nginx/sbin,查看模块是否安装成功
cd /kkb/server/nginx/sbin
./nginx -V

# 添加软链接到 /usr/local/sbin
ln -n /usr/local/nginx/sbin/nginx /usr/local/sbin

# 启动 nginx，使用  nginx -h  查看可以使用的命令
nginx

# 查看端口
netstat  -tunlp

# 浏览器输入nginx 服务器的 ip 进行访问
```

#### nginx 核心功能

1. 请求定位

   > **语法规则**：location     [=|~|~*|^~]  /uri/     {...}
   >
   > - =    开头表示精准匹配
   > - ~    开头表示区分大小写的正则匹配
   > - ~*  开头表示不区分大小写的正则匹配
   > - !~   和 !~*  开头表示匹配的反面
   > - /     通用匹配，任何请求都会匹配到
   >
   > 
   >
   > 匹配顺序：首先匹配 = ，其次匹配 ^~， 匹配内容最长的，最后 / 通用匹配，匹配到其中一个就返回结果

   ```perl
   # root 和 alias 的区别
   
   # 匹配为 /myhtml/h
   location /h {
       root   myhtml;
       index  default.html index.htm;
   }
   
   # 匹配为 /myhtml，且 alias 最后必须带 /
   location /h {
       alias   myhtml/;
       index  default.html index.htm;
   }
   
   # root 和 alias 可以匹配相对目录（相对 nginx_home）也可以匹配绝对目录
   ```

2. 静态代理

   nginx 对静态资源的处理比 Tomcat 性能更好，效率更高，且为了减少对 Tomcat 服务器的压力，使用 Nginx 作为静态代理服务器。

   

   扩展名拦截：

   > ~ ：开始
   >
   > $ ：结束
   >
   > .：任意字符
   >
   > *：任意个数
   >
   > \\ .： .小数点 . 因为 . 是特殊字符进行一下转义
   >
   > (jpg|js|css|html)：匹配其中任意一个

   ```perl
   location  ~.*\.(jpg|js|css|html)$ {
   	root   static;
   }
   ```

   

   目录名拦截：

   > 只要访问路径中包含 jpg|js|css|html 任意一个都会被拦截
   >
   > .+：表示一个或多个字符
   >
   > 所以下面的定位肯定是目录，不会是扩展包

   ```perl
   location  ~.*(jpg|js|css|html).+ {
   	root   static;
   }
   ```

3. 负载均衡

   ![](imgs/31.png)

   搭建一个 web 应用，分别部署到不同的 tomcat 服务器，设置 nginx 的负载均衡比例

   ![](imgs/32.png)

   设定 upstream（和 server 同一层级）

   ```perl
   # 名称可以任意给定	
   upstream tomcat.kaikeba.com {
       server 192.168.131.17:8080 weight=1;
       server 192.168.131.18:8080 weight=1;
   }
   ```

   在 location 添加  proxy_pass 属性（转发）

   ```perl
   location / {
   	proxy_pass http://tomcat.kaikeba.com;
   }
   ```

4. 动静分离

   > 搭建如下的服务端架构一共需要 5 台服务器

   ![](imgs/33.png)

   负载均衡服务器相关配置

   ```perl
   upstream tomcat.kaikeba.com {
   	server 192.168.131.17:8080 weight=1;
   	server 192.168.131.18:8080 weight=1;
   }
   
   upstream static.kaikeba.com {
   	server 192.168.131.21 weight=1;
   	server 192.168.131.22 weight=1;
   }
   
   server {
       listen       80;
   	location / {
   	    proxy_pass http://tomcat.kaikeba.com;
       }
   
   	location ~.*(jpg|js|css|html)$ {
   	    proxy_pass http://static.kaikeba.com;
      }
   }
   ```

   静态代理服务器相关配置（两个服务器一样）

   ```
   server {
       listen       80;
   
   	location ~.*(jpg|js|css|html)$ {
   	    root   images;
      }
   }
   ```

5. 虚拟主机

   ![](imgs/34.png)

   ![](imgs/35.png)

   

   

   ​		![](imgs/36.png)

   ​		配置 3 个Server,端口一致，主机名不一致

   ​		![](imgs/37.png)

6. Nginx 性能调优

    <https://blog.csdn.net/lamp_yang_3533/article/details/80383039>

#### rewrite

> 语法 rewrite regex replacement [flag];
>
> **rewrite**的含义：该指令是实现URL重写的指令
> **regex**的含义：用于匹配URI的正则表达式
> **replacement**：将regex正则匹配到的内容替换成 replacement
> **flag: flag**标记
>
> flag有如下值：
>
> **last:** 本条规则匹配完成后，继续向下匹配新的location URI 规则。(不常用)
> **break:** 本条规则匹配完成即终止，不再匹配后面的任何规则(不常用)
> **redirect:** 返回302临时重定向，浏览器地址会显示跳转新的URL地址
> **permanent:** 返回301永久重定向。浏览器地址会显示跳转新的URL地址

rewrite 可以在 server 和 location 块中使用

1. 在 server 块中

   > 域名跳转需要加 http://

   ```perl
   # 浏览器访问当前主机都跳转到为 www.baidu.com
   rewrite ^/(.*)$ http://www.baidu.com/$1 redirect;
   
   # 如果是移动端跳转到移动端页面
   if ($http_user_agent ~* "(Android)|(iPhone)|(Mobile)|(WAP)|(UCWEB)" ){
       rewrite ^(.*)$    http://www.storeage.com/    permanent;
       # 带上域名后面的信息
       # rewrite ^/(.*)$    http://www.storeage.com/$1  permanent;
   }
   ```

2. 在 location 块中

   ```perl
   # $n 和 $t 是前面定义的变量
   rewrite ^/group1/(.*)$ /group1/$n.$t 是前面定义的变量  break;
   ```

#### if

> 语法: `if (condition) {...}`
>
> - 变量名: 如果变量值为0, 或''(空字符串) 则为false
> - 使用 = 和 != 运算符比较变量和字符串
> - 使用 ~ (区分大小写的匹配) 和 `~*` (不区分大小写) 运算符，（可以理解为 like，模糊匹配）
> - -f: 文件是否存在
> - -d: 目录是否存在
> - -e: 检查文件, 目录, 符号链接 是否存在
> - -x: 检查可执行文件



















































