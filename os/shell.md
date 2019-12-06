### Shell

#### 文件头

> 表示是一个 shell 脚本文件

```bash
#!/bin/bash
```

#### 命令替换

在bash中，\$() 与 \`\`（反引号）都是用来作命令替换的。**先完成引号里的命令行**，然后将其结果替换出来，再重组成新的命令行，\$() 比 \`\` 更直观。

```bash
echo  $(echo today is $(date "+%Y-%m-%d"))
echo today is `date "+%Y-%m-%d"
```

输出为

```
today is 2019-12-03
today is 2019-12-03
```

#### 变量

> \${} 和 \$ 变量替换，效果一样只是 \${} 更直观

```shell
# 定义变量，注意：等号前后没有空格
name="xiexq"

# 使用变量
echo #{name}
# 或　echo $name

# 删除变量
unset name

# 设置位只读
readonly name

# 单引号和双引号的区别
# 双引号内部可以有变量，单引号内部的任何内容只会原样输出
```

#### 取路径/文件名/后缀名

> 记忆方法：
>
> \# 是去掉左边(在键盘上 # 在 $ 之左边)
>
> % 是去掉右边(在键盘上 % 在 $ 之右边)
>
> 单一符号是最小匹配;两个符号是最大匹配
> \* 是用来匹配不要的字符，也就是想要去掉的那部分
> 还有指定字符分隔号，与*配合，决定取哪部分

```bash
file=/dir1/dir2/dir3/my.file.txt

${file#*/}    # 拿掉第一条 / 及其左边的字符串    dir1/dir2/dir3/my.file.txt
${file##*/}   # 拿掉最后一条 / 及其左边的字符串    my.file.txt
${file#*.}    # 拿掉第一个 . 及其左边的字符串    file.txt
${file##*.}   # 拿掉最后一个 . 及其左边的字符串    txt
${file%/*}    # 拿掉最后一条 / 及其右边的字符串    /dir1/dir2/dir3
${file%%/*}   # 拿掉第一条 / 及其右边的字符串    (空值)
${file%.*}    # 拿掉最后一个 . 及其右边的字符串    /dir1/dir2/dir3/my.file
${file%%.*}   # 拿掉第一个 . 及其右边的字符串    /dir1/dir2/dir3/my￼
```

#### 字符串

```bash
# 字符串拼接
echo "his name is "$name

# 获取字符串长度
echo ${#name}

# 提取子字符串,指定开始位置和结束位置
echo ${name:1:3}

# 查找字符出现的位置，x 和 q 哪个字符最先出现，下标从 1 开始
echo `expr index $name xq`　# 输出 １

# 判断字符串是否相等，不相等使用 !=
a="good"
b="good"

if [ $a = $b ]
then 
	echo "相等"
else
	echo  "不相等"
fi

# 检测长度是否为 0， 为 0 返回 true，注意：默认去掉前后空格
c="  "
if [ -z $c ]
then 
	echo "长度为空"
else
	echo  "长度不为空"
fi

# 将第一个 dir 提换为 path
${file/dir/path}

# 将全部 dir 提换为 path　　
${file//dir/path}

# 
```

#### 数组

```bash
# 定义数组，使用空格进行分割
names=(tony cindy killy tommy)

# 读取数组中的单个元素
echo ${names[0]}

# 读取数组中的所有元素
echo ${names[@]}
# 或 echo ${names[*]}

# 遍历数组中的元素　＠ 可以使用　*
for name in ${names[@]}
do 
	echo $name
done

# 获取数组的长度
echo ${#names[@]}

# 获取数组中单个元素的长度
echo ${#names[1]}
```

#### 注释

```bash
# 单行注释
# echo $name

# 多行注释（还有别的方式，这种方式比较简单）
# 注意：冒号和单引号之间是有空格的
: '
注释1...
注释2...
注释3...
'
```

#### 参数传递

```shell
echo "总共的参数个数为： $#"
echo "第一个参数为： $1"
echo "第二个参数为： $2"
echo "第三个参数为： $3"
echo "所有的参数为：$*"
echo "所有的参数为：$@"
```

执行

```bash
chmod +x hello.sh
./hello.sh first second third
```

输出

```
总共的参数个数为： 3
第一个参数为： first
第二个参数为： second
第三个参数为： third
所有的参数为：first second third
所有的参数为：first second third
```

注意：\$@ 和 \$\* 的区别，**在加上引号的时候**，\$\* 表示一个字符串，\$@  是一个数组，不加引号没有区别

如下：

```bash
for i in "$*"
do
	echo $i
done

for j in "$@"
do
	echo $j
done
```

输出为：

```
first second third
first
second
third
```

#### 进制转换

> $((N#xx))，将指定禁止的数值转换为 10 进制
>
> 其中，N为进制，xx为该进制下某个数值，命令执行后可以得到该进制数转成十进制后的值。

```bash
echo $((２#10)) # 二进制下的 10 转换为十进制，结果为 ２，　其他进制类似
```

#### 运算符

1. 算数运算符

   >  原生 bash 不支持简单的数学运算，但是可以通过其他命令来实现，例如 awk 和 expr，expr 最常用。

   ```bash
   a=10
   b=20

   echo "两数之和为 :`expr $a + $b`"
   echo "两数之差为 :`expr $a - $b`"
   echo "两数之积为 :`expr $a \* $b`"
   echo "两数之商为 :`expr $a / $b`"

   # 或
   echo "两数之和为 :$[a + b]"
   echo "两数之差为 :$[a - b]"
   echo "两数之积为 :$[a * b]"
   echo "两数之商为 :$[a / b]"

   # 或
   echo "两数之和为 :$((a + b))"
   echo "两数之差为 :$((a - b))"
   echo "两数之积为 :$((a * b))"
   echo "两数之商为 :$((a / b))"
   ```

   注意：使用 "expr" 运算符前后必须有空格，乘法 \* 需要进行转义

2. 关系运算符

   ```bash
   -eq
   -ne
   -le
   -lt
   -ge
   -gt
   ```

3. 布尔（逻辑）运算符

   ```bash
   -o # 或 ||
   -a # 或 &&
   !  # 非

   ```

   示例

   ```bash
   a=10

   # 表达式前加上 ！ 就表示取反
   if test ! $a -gt 10
   then
   echo "不大于 10"
   else 
   echo "不小于 10"
   fi
   ```

4. 文件测试运算符

   ```bash
   -d file	# 检测文件是否是目录，如果是，则返回 true
   -f file	# 检测文件是否是普通文件（既不是目录，也不是设备文件），如果是，则返回 true
   -r file	# 检测文件是否可读，如果是，则返回 true
   -w file	# 检测文件是否可写，如果是，则返回 true
   -x file	# 检测文件是否可执行，如果是，则返回 true
   -s file	# 检测文件是否为空（文件大小是否大于0），不为空返回 true
   -e file	# 检测文件（包括目录）是否存在，如果是，则返回 true
   -b file	# 检测文件是否是块设备文件，如果是，则返回 true
   -c file	# 检测文件是否是字符设备文件，如果是，则返回 true
   -g file	# 检测文件是否设置了 SGID 位，如果是，则返回 true
   -k file	# 检测文件是否设置了粘着位(Sticky Bit)，如果是，则返回 true
   -p file	# 检测文件是否是有名管道，如果是，则返回 true
   -u file	# 检测文件是否设置了 SUID 位，如果是，则返回 true
   ```

   例如：

   ```bash
   file="/var/www/runoob/test.sh"

   if [ -f $file ]
   then
      echo "文件为普通文件"
   else
      echo "文件为特殊文件"
   fi
   if [ -d $file ]
   then
      echo "文件是个目录"
   else
      echo "文件不是个目录"
   fi
   if [ -s $file ]
   then
      echo "文件不为空"
   else
      echo "文件为空"
   fi
   if [ -e $file ]
   then
      echo "文件存在"
   else
      echo "文件不存在"
   fi
   ```

   注意：[] 的前后必须有空格

#### printf

> 格式化输出字符串，printf 末尾不会自带换行符，echo 末尾会带换行符

```bash
# %s %c %d %f 都是格式替代符
# %-10s 指一个宽度为10个字符（-表示左对齐，没有则表示右对齐），任何字符都会被显示在10个字符宽的字符内，# 如果不足则自动以空格填充，超过也会将内容全部显示出来。
# %-4.2f 指格式化为小数，其中.2指保留2位小数。
printf "%-10s%-8s%-4s\n" "姓名"   "年龄"  "收入"
printf "%-10s%-8d%-4.1f\n" "tony"   28     2000.0
```

#### test

> 测试，和 [] 类似

```bash
num1=100
num2=100
if test $[num1] -eq $[num2]
then
    echo '两个数相等！'
else
    echo '两个数不相等！'
fi
```

#### 函数

```bash
# 不带参数
fun1(){
    echo "函数调用成功"
}
# 调用函数
fun1

# 带参数
fun2(){
    echo "参数 1： $1"
    echo "参数 2： $2"
    echo "参数个数： $#" 
    echo "所有的参数  $*"
    echo "函数调用成功"
}
fun2 1 2

# 带返回值 使用 $? 获取
fun3(){
    echo "参数 1： $1"
    echo "参数 2： $2"
    echo "函数调用成功"
    return `expr $1 + $2`
}
fun3 1 2
echo "输入的两个数字之和为： $?"
```

注意：当获取的参数个数大于等于 10 时，使用 ${n} 表示

####  流程控制

1. if

   格式

   ```
   if condition
   then
       command1 
       command2
       ...
       commandN 
   fi
   ```

   示例：

   ```bash
   a=10
   b=20

   # if
   if test $a -gt $b
   then 
   	echo "a 大于 b"
   fi

   # if else
   if test $a -ge $b
   then 
   	echo "a 大于等于 b"
   else
   	echo "a 小于 b"
   fi

   # if else-if else
   if test $a -gt $b
   then 
   	echo "a 大于 b"
   elif test $a -eq $b
   then
   	echo "a 等于 b"
   else
   	echo "a 小于 b"
   ```

2. for

   格式

   ```
   for var in item1 item2 ... itemN
   do
       command1
       command2
       ...
       commandN
   done
   ```

   示例：

   ```bash
   # 遍历文件夹下的文件包名称
   for name in `ls /`
   do
   	echo $name
   done

   # 遍历数组
   arr=(apple pear peach)

   for name in ${arr[*]}
   do
   	echo $name
   done

   # 遍历数组带下标
   for((i=0;i<${#arr[@]};i++)) 
   do
   	echo ${arr[i]}
   done
   ```

3. while 

   格式

   ```
   while condition
   do
       command
   done
   ```

   示例

   ```bash
   a=10

   while test $a -gt 0
   do 
   	echo $a
   	a=$[a-1]
   done

   # 遍历数组
   arr=(apple pear peach)
   i=0  
   while [ $i -lt ${#arr[@]} ]  
   do  
       echo ${ arr[$i] }
       let i++  
   done
   ```

4. case

   格式

   ```
   case 值 in
   模式1)
       command1
       command2
       ...
       commandN
       ;;
   模式2）
       command1
       command2
       ...
       commandN
       ;;
   esac
   ```

   示例：

   ```bash
   # 示例 1
   read n1
   case $n1 in
       1) echo "input 1"
       ;;
       2) echo "input 2"
       ;;
       3) echo "input 3"
       ;;
       4) echo "input 4"
       ;;
       *) echo "input other"
       ;;
   esac

   # 示例 2
   read n2
   case $n2 in 
      1|2|3|4|5) echo "input num in 1-5"
       ;;
       *) echo "game over"
          break
       ;;
   esac
   ```

5. break 和 continue

   ```bash
   a=10

   while test $a -gt 0
   do 
   	let "a--"
   	echo $a
   	break
   done
   ```

#### 文件重定向

> 0 是标准输入（STDIN）
>
> 1 是标准输出（STDOUT）
>
> 2 是标准错误输出（STDERR）

1. 输出

   ```bash
   command > file # 追加
   command >> file #　覆盖

   # stderr 重定向到 file
   command 2 > file

   # stdout 和 stderr 合并后重定向到 file
   command > file 2>&1
   # 或
   command > file １>&2
   ```


2. 输入

   ```bash
   command < file
   ```

   例如：

   test.txt 文件内容

   ```
   我
   爱
   北京天安门
   ```

   统计行数

   ```bash
   #　统计　test.txt 文件的行数
   wc -l < test.txt
   ```

   结果：

   ```
   3
   ```


3. Here Document

   > 可以添加分隔符

   ```bash
   # 如下的 EOF 作为分隔符，可以任意指定分隔符
   $ wc -l << EOF
       欢迎来到
       菜鸟教程
       www.runoob.com
   EOF
   3          # 输出结果为 3 行
   ```

4. 不输出

   > 默认在控制台输出，如果不显示，输出到　/dev/null
   >
   > /dev/null 是一个特殊的文件，写入到它的内容都会被丢弃；如果尝试从该文件读取内容，那么什么也读不到
   >
   > 可以起到禁止输出的效果

   ```bash
   command > /dev/null

   # 标准输出和错误输出到文件　/dev/null
   command > /dev/null 2>&1
   ```

