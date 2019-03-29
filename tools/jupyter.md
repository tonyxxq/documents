1. nbviewer，在线查看jupyter，比如可以指定github url查看

   网址：http://nbviewer.jupyter.org/

2. .ipynb只是大型的json文件

3. 代码的执行流程

   user->browser->notebook server->kernel
   由于代码和内核分离，所以内核可以变化，不一定是python代码，且notebook是可以远程访问的

   ```
   # 可以在环境中执行下列命令，安装R kernel
   conda install -c r r-essentials
   ```

4. 安装和启动

   conda install jupyter notebook 或 pip install jupyter notebook

   启动：
   在命令行进入指定目录，jupyter notebook
   在浏览器键入，http://localhost:8888
   如果使用命令行启动多个notebook，端口会递增代码单元格

5. 单元格

   从Markdown切换到单元格，按 Y 。从代码切换到Markdown，按M键盘。
   h键，显示所有快捷键
   A：上方插入单元格
   B：下方插入单元格

   l ： 打开左边的行号（code）
   s：保存
   dd：删除

   shitf + tab：显示函数帮助

   tab：自动补全

   cmd/ctrl + shift + p： 显示命令搜索框

   esc + f：查找和替换代码

   esc + o：打开代码块输出

   shift + down ： 向下选择多个cell

   shift + up：向上选择多个cell

   shift + m：合并多个cell

   ?sum()：可以找到某个方法的使用说明，或则再help下找寻帮助文档

   alt + 选中的文本 + 鼠标拖拽：可以使代码拖到指定的位置，不同单元格复制，相同单元格剪切


6. magic关键字

   Magic 命令的前面带有一个或两个百分号（% 或 %%），分别对应行Magic命令和单元格Magic命令。
   %timeit numpy.random.normal(size=100)：预测函数的运行时间
   %%time ：预测整个单元格的运行时间
   %matplotlib inline 
   在分辨率较高的屏幕（例如 Retina 显示屏）上，notebook 中的默认图像可能会显得模糊。可以在 %matplotlib inline 之后使用 %config InlineBackend.figure_format = 'retina' 来呈现分辨率较高的图像。
   %pdb 开启交互式调试器，退出输入“q”

   %matplotlib notebook : 可以和图片有一些交互，比如放大，拖动，导出等功能

   %env：列出所有的环境变量

   %env  OMP_NUM_THREADS=4：设置环境变量

   %run ./LinearRegression.ipynb：执行其他的notebook

   %run ./LinearRegression.py：执行其他的文件

   ！ls：在notebook中执行shell

7. 创建幻灯片：

   - 击“View”（视图）>“Cell Toolbar”（单元格工具栏）>“Slideshow”（幻灯片）
   - Slides（幻灯片）是你从左向右移动的完整幻灯片。按向上或向下的箭头时，Sub-slides（子幻灯片）会出现在幻灯片中。Fragments（片段）最初是隐藏的，在你按下按钮时会出现。选择 Skip（忽略）会忽略幻灯片中的单元格，而选择 Notes（备注）会将为演讲者保留备注。
   - jupyter nbconvert notebook.ipynb --to slides
   - jupyter nbconvert notebook.ipynb --to slides --post serve

8. 在环境中安装和指定kernel

   - pip install ipykernel // 确保当前的python环境中已经安装了ipkernel,不然会提示找不到ipykernel模块
   - python -m ipykernel install // 设置当前环境为kernel，-n可以设置该内核的名称
   - jupyter kernelspec remove python3 // 删除kernel，python3表示删除的kernel名称
   - jupyter kernelspec list // 查看已经安装的kernel列表

   ​    一般在环境中新建kernel流程为：

   ​    前提：环境需先安装python，否则取anaconda默认的python。

   1.  进入环境,source activate  env-name
   2. conda install ipykernel
   3. python -m ipykernel install --user --name dlnd --display-name "Python [dlnd]" //--name表示环境的名        称，--display-name:表示在notebook中显示的名称

9. 设置允许外网访问

   jupyter notebook --ip=0.0.0.0 --no-browser

   通过访问服务器的ip，就能通过外网访问。

   http://ip:8888

10. 去掉\添加jupyter notebook密码输入

  ```
  # 重新生成配置文件，注意会覆盖之前得内容
  jupyter notebook --generate-config
  # 会提示生成的配置文件得地址，例如：
  C:\Users\Administrator\.jupyter\jupyter_notebook_config.py
  # 修改配置文件中得内容为如下，这样就不会有token了
  #c.NotebookApp.token = ''

  # 给jupyter notebbo设置密码
  jupyter notebook password
  ```

11. 设置默认的浏览器

   ```python
   # 设置默认打开chrome浏览器,其他类似
   # 在jupyter_notebook_config.py文件中"#c.NotebookApp.browser=u''"的替换为如下
   import webbrowser
   webbrowser.register("chrome", None, webbrowser.GenericBrowser("C:\Users\Administrator\AppData\Local\Google\Chrome\Application\chrome.exe"))
   c.NotebookApp.browser = 'chrome'
   ```

12. 修改默认打开的工作目录，否则为当前终端所在的目录

   ```python
   # 在jupyter_notebook_config.py文件中"#c.NotebookApp.notebook_dir=u''"的替换为如下
   c.NotebookApp.notebook_dir = 'F:\\temp'
   # 如果目录中含中文
   c.NotebookApp.notebook_dir = 'F:\\我的'.decode('utf-8')
   ```

13. 使用jupyter进行输出

   ```python
   import os
   from IPython.display import display, Image
   names = [f for f in os.listdir('../images/') if f.endswith('.png')]
   for name in names[:5]:
       display(Image('../images/' + name, width=100))
   ```

   ​

14. 对于大规模数据样本的查询/处理也有一些解决方案

   ```
   # 地址：https://github.com/ipython/ipyparallel
   ipyparallel(以前叫ipython cluster)是使用 Python 进行简单 map-reduce 操作的一个很好的选择。
   # 地址：http://www.cloudera.com/documentation/enterprise/5-5-x/topics/spark_ipython.html
   pyspark
   # 地址：https://github.com/jupyter-incubator/sparkmagic
   spark-sql 魔法 %%sql
   ```

   ​

