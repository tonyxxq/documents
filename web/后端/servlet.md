# Servlet

- servlet 的两个 map

  > 当用户发请求过来的时候，tomcat 容器去掉了 域名、端口、上下文，发送到指定的应用，在应用里边有两个 Map(在项目启动的时候生成)，通过 urlpattern（正则匹配）查询第一个 Map 的 Servlet ，在开始的时候是空的，然后通过 urlpattern（正则匹配） 查询第二 Map,构建实例（单例），实例的引用放入Map,下次访问的时候就直接访问第一个Map
  >
  > 在 Sprin MVC 中配置的 load-on-startup 表示的就是一次加载所有的 Servlet

  ![](imgs/1.png)