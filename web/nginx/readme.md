- nginx的安装和卸载

  ```
  # 安装
  sudo apt-get install nginx

  # 卸载
  sudo apt-get purge nginx
  ```

- 启动nginx

  ```
  sudo /etc/init.d/nginx start

  sudo service nginx start
  ```

- 停止nginx

  ```
  sudo /etc/init.d/nginx stop

  sudo service nginx stop
  ```

- 重启nginx

  ```
  sudo /etc/init.d/nginx restart

  sudo service nginx restart

  ```

- 测试配置文件

  ```
  sudo nginx -t 

  # 设置配置文件
  sudo vim /etc/nginx/sites-avaiable/default 
  ```

- 查看运行状态

  ```
  sudo service nginx status
  ```


- 反向代理和正向代理的区别

  正向代理 是一个位于客户端和原始服务器(origin server)之间的服务器，为了从原始服务器取得内容，客户端向代理发送一个请求并指定目标(原始服务器)，然后代理向原始服务器转交请求并将获得的内容返回给客户端。客户端必须要进行一些特别的设置才能使用正向代理。
   反向代理（Reverse Proxy）实际运行方式是指以代理服务器来接受internet上的连接请求，然后将请求转发给内部网络上的服务器，并将从服务器上得到的结果返回给internet上请求连接的客户端，此时代理服务器对外就表现为一个服务器。

- 不使用service命令，直接在软件内部启动，需要加上-c 表示配置文件的位置， 其他的类似

  ```
  ./nginx -c  /usr/local/edu/nginx/conf/nginx.conf
  ```

  ?





