1. 把当前环境的依赖包导出到文件（如果没有安装conda） 

   ```
   pip freeze > requirements.txt
   ```


2. 安装Anaconda

​        网址： https://www.continuum.io/downloads 

3. 更换镜像（清华）

   ```
   conda config --add channels https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/free/
   conda config --set show_channel_urls yes
   此时C:\Users<你的用户名> 下就会生成配置文件.condarc 
   打开.condarc ，去掉第三行（default），保存
   conda info // 查看是否修改成功
   ```


4. 管理包

   ```
   conda upgrade --all // 更新所有的包
   conda install numpy // 安装包
   conda install numpy=1.0 // 安装包且指定版本
   conda install numpy pandas // 同时安装多个包
   conda remove numpy // 卸载包
   conda update numpy // 更新包
   conda list // 列出所有conda存在的包
   conda search numpy // 查询包
   ```

5. 管理环境(管理包和上面一致，只不过是在当前环境下)

   ```
   conda env remove -n env_name // 删除环境
   conda create -n env_name list of packages // 创建环境，env_name代表环境名称
   conda create -n env_name python=3.5 // 指定python版本的环境
   source activate my_env // 进入环境（linux/mac）
   activate my_env // 进入环境（windows）
   source deactivate // 离开环境（linux/mac）
   deactivate my_env // 离开环境（windows）
   conda env export > environment.yaml // 输出当前环境中包的名称
   conda env create -f environment.yaml // 通过环境文件创建环境
   conda env list// 列出所有的环境
   ```










