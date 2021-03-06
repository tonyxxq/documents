# ElasticSearch

## ElasticSearch 概述

ElasticSearch 是一个基于 Lucene 的搜索服务器，它提供了一个分布式多用户能力的全文搜索引擎，基于 RESTful web 接口

ElasticSearch 是用 java 开发的，并作为 Apach 许可条款下的开源发布，是当前流行的企业级搜索引擎，设计用于云计算中，能够达到实时搜索，稳定、可靠、快速安装使用方便

构建在全文搜索开源软件 Lucene 之上的 ElasticSearch，不仅能对海量规模的数据完成分布式索引与检索，还能提供数据聚合分析

目前为排名第一的搜索引擎类应用

优点：

- 速度快，近实时查询：最迟 1 秒
- 索引和检索的数据量大：支持 TB 级数据
- 高扩展，高可用

概括：

基于 RESTful 标准的高扩展高可用的实时数据分析的全文搜索工具

跟 Solr 一样也是使用文档来存储数据

### 基本概念

| 关系数据库      | ElasticSearch |
| --------------- | ------------- |
| 数据库 Database | 索引 Index    |
| 表 Table        | 类型 Type     |
| 数据行 Row      | 文档 Document |
| 数据列 Column   | 字段 Field    |
| 表结构 Schema   | 映像 Mapping  |

## 安装

![](imgs/219.png)

### 安装单机版

>  JDK 不要安装在 root 目录下，因为启动的时候不能使用 root 账号启动

```bash
# 安装 jdk 
# 略

# 下载 elasticsearch，解压
wget https://artifacts.elastic.co/downloads/elasticsearch/elasticsearch-6.5.3.tar.gz
tar -zxvf elasticsearch-6.5.3.tar.gz -C  /usr/local/kkb

# 配置远程访问
vim /usr/local/kkb/elasticsearch/config/elasticsearch.yml
# 设置如下属性(或为本地 ip)
network.port: 0.0.0.0

# 设置进程可以拥有的 VMA(虚拟内存区域的)数量（？）
vim /etc/sysctl.conf
# 添加如下内容
vm.max_map_count=655360
# 生效配置文件
sysctl -p

# 修改允许打开的最大文件描述符常量（？）
vim /etc/security/limits.conf
# 添加以下内容
esuser soft nofile 65536
esuser hard nofile 65536
esuser soft nproc 4096
esuser hard nproc 4096

# 创建用户和组， 不能使用
groupadd esgroup
useradd esuser -g esgroup -p 123456
# 更改 elasticsearch 文件的所属用户和组
chown -R esuser:esgroup /usr/local/kkb/elasticsearch-6.5.4
# 切换到 esuser
su esuser

# 因为 es 有缓存机制，比较吃内存，启动之前根据服务器大小，修改一下 jvm.options，默认为 1g
vim /usr/local/kkb/elasticsearch-6.5.4/config/jvm.options
# 修改如下配置
-Xms512m
-Xmx512m

# 前端启动，ctrl c 就关闭了
/usr/local/kkb/elasticsearch-6.5.4/bin/elasticsearch
# 后端启动（后端运行，推荐）
/usr/local/kkb/elasticsearch-6.5.4/bin/elasticsearch -d
```

访问（IP 为服务器的 IP）

![](imgs/261.png)

### 集群版安装

> 关于配置的详细描述：https://blog.csdn.net/sd4015700/article/details/20736177					

配置每台服务器的 elasticsearch.conf

```yaml
# 集群名称，唯一
cluster.name: kkb-es

# 节点名称不同
node.name: node-135

# 是否可以参与竞选 master
node.master: true

# 是否是数据节点
node.data: true

# 节点将绑定到此主机名或 IP 地址，设置成 0.0.0.0 为当前主机，允许 ip 映射访问
network.host: 0.0.0.0

# http 访问端口
http.port: 9200

# tcp 访问端口（集群服务器之间是通过 tcp 访问的）
transport.tcp.port: 9300

# 自动发现的路由节点
discovery.zen.ping.unicast.hosts:
["192.168.1.107:9300", "192.168.1.108:9300", "192.168.1.109:9300"]

# 集群中最小主机节点数，防止脑裂
discovery.zen.minimum_master_nodes: 2

# head 跨域访问
http.cors.enabled: true
http.cors.allow-origin: "*"

# 设置一台机子能运行的节点数目，一般采用默认的 1 即可，因为我们一般也只在一台机子上部署一个节点
node.max_local_storage_nodes: 1
```

启动服务

```bash
/usr/local/kkb/elasticsearch-6.5.4/bin/elasticsearch -d
```

> 查看集群状态

```bash
curl -i -XGET -H 'Content-Type: application/json' 'http://192.168.1.107:9200/_cat/health?v' 
```

![](imgs/262.png)

### 安装 head 插件

> 可以进行独立安装在一台服务器上或集群中一台机器上

```bash
# 安装 npm，添加淘宝镜像
yum install -y epel-release
yum install -y nodejs npm
npm install -g cnpm --registry=https://registry.npm.taobao.org

# 下载 git
yum install -y git
git clone git://github.com/mobz/elasticsearch-head.git

# 安装 head
cd /usr/local/kkb/elasticsearch-head
cnpm install

# 将 grunt 加入环境变量
cnpm install -g grunt-cli
```

修改  elastic-head/Gruntfile.js 配置文件

![](imgs/263.png)

修改  elastic-head/_sites/app.js 配置文件

> 地址修改为了集群中的其中一台服务器的地址

![](imgs/264.png)

启动

```bash
nohup grunt server &
```

http://192.168.1.108:9200

![](imgs/265.png)



![](imgs/236.png)

![](imgs/237.png)





​		![](imgs/238.png)





![](imgs/243.png)

### 安装 ik-analyzer

> 每台机器上都需要装
>
> 注意：ik 的版本要和 es 的版本一致

```bash
# 下载
wget https://github.com/medcl/elasticsearch-analysis-ik/releases/download/v6.5.4/elasticsearch-analysis-ik-6.5.4.zip

# 解压到 es 的 plugins 目录下
unzip elasticsearch-analysis-ik-6.5.4.zip  -d  /usr/local/kkb/elasticsearch-6.5.4/plugins/ik

# 重启 es
su esuser
../bin/elasticsearch -d
```

在浏览器访问

https://192.168.1.107:9200

### Kibana 安装

> Kibana 是一个开源的分析与可视化平台，设计出来用于和Elasticsearch一起使用的。你可以用kibana搜索、查看存放在Elasticsearch中的数据。Kibana与Elasticsearch的交互方式是各种不同的图表、表格、地图等，直观的展示数据，从而达到高级的数据分析与可视化的目的。
> Elasticsearch、Logstash和Kibana这三个技术就是我们常说的ELK技术栈，可以说这三个技术的组合是大数据领域中一个很巧妙的设计。一种很典型的 MVC 思想，模型持久层，视图层和控制层。Logstash 担任控制层的角色，负责搜集和过滤数据。Elasticsearch 担任数据持久层的角色，负责储存数据。而我们这章的主题Kibana担任视图层角色，拥有各种维度的查询和分析，并使用图形化的界面展示存放在 Elasticsearch 中的数据

```bash
# 下载并解压
wget https://artifacts.elastic.co/downloads/kibana/kibana-6.5.4-linux-x86_64.tar.gz
tar -zxvf kibana-6.5.4-linux-x86_64.tar.gz /usr/local/kkb/

# 编辑 kibana.yml 文件
# 配置如下（修改成服务器所在 ip,集群选其中一个服务器 IP）
server.host: "192.168.1.107"
elasticsearch.url: "http://192.168.1.107：9200"

# 启动 kibana
nohup /usr/local/kkb/kibana-6.5.4-linux-x86_64/bin/kibana &
```

通过浏览器访问

http://192.168.1.107:5601

​					![](imgs/247.png)



### Java 客户端



​			

## ElasticSearch 原理

### 集群的节点

#### 三种节点角色

master 节点

这个集群只会有一个 master 节点，它将负责管理集群范围内的所有变更，例如：增加、删除索引；或增加、删除节点等。而 master 节点并不需要涉及到文档级别的变更和搜索等操作，所以当集群只拥有一个 master 节点的情况下，即使流量的增加它也不会成为瓶颈。



master 节点需要从众多候选 master 节点中选择一个

#### master 节点的作用

负责集群节点上下线，shard分片的重新分配。

创建、删除索引 。



![](imgs/249.png)

节点配置选择

![](imgs/250.png)



![](imgs/253.png)

![](imgs/255.png)

![](imgs/254.png)

![](imgs/257.png)

![](imgs/258.png)



![](imgs/259.png)



![](imgs/260.png)







脑裂及其解决方案：

![](imgs/252.png)

​			![](imgs/251.png)

项目使用：

导入依赖：

```groovy
"org.springframework.data:spring-data-elasticsearch:${versions.elasticsearch}"
```

实体类配置

```java
@Getter
@Setter
@Document(indexName = "question", type = "question")
@Accessors(chain = true)
public class Question {
    @Id
    @Field(store = true)
    private Long id;  // 问题ID

    @Field(type = FieldType.Integer, store = true)
    private int type;  // 问题类型 0：投票， 1：问题

    @NotBlank(message = "问题标题不能为空")
    @Field(type = FieldType.Text, store = true, searchAnalyzer = "ik_max_word", analyzer = "ik_max_word")
    private String title; // 问题标题

    @Field(type = FieldType.Object, store = true)
    private RichText content; // 问题内容

    @Field(store = true, type = FieldType.Long)
    private Long userId;  // 提问人 ID

    @Field(store = true, type = FieldType.Text)
    private String nickname; // 提问人昵称

    @Field(store = true, type = FieldType.Boolean)
    private boolean classic; // 是否是精选问题

    @Field(store = true, type = FieldType.Boolean)
    private boolean top; // 是否置顶

    @Field(store = true, type = FieldType.Boolean)
    private boolean answersVisible; // 是否回答问题之后回答列表才可见

    @Field(store = true, type = FieldType.Long)
    private Long clazzId; // 班级 ID

    @Field(store = true, type = FieldType.Long)
    private Long teacherId; // 老师 ID

    @Field(store = true, type = FieldType.Text)
    private String subject; // 科目名称

    @Field(store = true, type = FieldType.Text)
    private String clazzName; // 班级名称

    @Field(store = true, type = FieldType.Integer)
    private Long answerCount; // 回答人数

    @Field(store = true, type = FieldType.Integer)
    private Long zanCount; // 点赞人数

    @Field(store = true, type = FieldType.Integer)
    private Long viewCount; // 浏览人数

    @Field(store = true, type = FieldType.Date)
    private Date createdTime; // 创建时间

    // 投票
    private boolean multiSelect;   // 是否多选
    private boolean voteAnonymous; // 是否匿名
    private int maxChoiceCount;    // 最多选几个

    // 冗余属性
    private Long[] clazzIds;                  // 勾选的班级
    private List<Map<String, String>> choices;// 投票选项，创建投票时使用
    private List<QuestionChoice> choiceList;  // 投票选项，查询投票时使用
    private boolean voted;                    // 是否已投票
    private int myZanStatus;// 点赞状态
    private int order;      // 排序规则

    @Field(store = true, type = FieldType.Text)
    private String avatar;                    // 头像URL
}
```

 service

```java
@Service
public class ElasticSearchService implements InitializingBean{

    @Autowired
    ElasticsearchTemplate elasticsearchTemplate;

    /**
     * 插入或更新问题到索引库
     *
     * @param id     对象 ID
     * @param object 需要插入或更新的对象
     * @return true：成功 false：失败
     */
    public boolean createOrUpdateIndex(Object object, Long id) {
        try {
            IndexQuery indexQuery = new IndexQueryBuilder()
                    .withIndexName("question")
                    .withType("question").withId(id + "")
                    .withObject(object).build();
            elasticsearchTemplate.index(indexQuery);
            return true;
        } catch (Exception e) {
            return false;
        }
    }

    /**
     * 删除指定 ID 对象的索引
     *
     * @param id   对象 ID
     * @param type 类型
     * @return true：成功 false：失败
     */
    public <T> boolean deleteById(String id, Class<T> type) {
        try {
            elasticsearchTemplate.delete(type, id);
            return true;
        } catch (Exception e) {
            return false;
        }
    }

    /**
     * 根据条件查询问题列表
     *
     * @param question 问题对象
     * @return 问题列表
     */
    public List<Question> findQuestions(Question question, int pageNumber, int pageSize) {
        BoolQueryBuilder boolQueryBuilder = QueryBuilders.boolQuery();
        if (question.getClazzId() != null) {
            boolQueryBuilder.must(QueryBuilders.termQuery("clazzId", question.getClazzId()));
        }
        if (question.getTeacherId() != null) {
            boolQueryBuilder.must(QueryBuilders.termQuery("teacherId", question.getTeacherId()));
        }
        if (question.isClassic()) {
            boolQueryBuilder.must(QueryBuilders.termQuery("classic", question.isClassic()));
        }
        if (question.getUserId() != null) {
            boolQueryBuilder.must(QueryBuilders.termQuery("userId", question.getUserId()));
        }
        if (question.getAnswerCount() != null) {
            boolQueryBuilder.must(QueryBuilders.termQuery("answerCount", question.getAnswerCount()));
        }
        boolQueryBuilder.must(QueryBuilders.multiMatchQuery(question.getTitle(), "title", "content.html"));

        SearchQuery searchQuery = new NativeSearchQueryBuilder()
                .withTypes("question")
                .withQuery(boolQueryBuilder)
                .withPageable(PageRequest.of(pageNumber, pageSize))
                .build();

        return elasticsearchTemplate.queryForList(searchQuery, Question.class);
    }

   @Override
    public void afterPropertiesSet() throws Exception {
       if (!elasticsearchTemplate.indexExists(Question.class)) {
           elasticsearchTemplate.createIndex(Question.class);
       }
       elasticsearchTemplate.putMapping(Question.class);
    }
}
```

