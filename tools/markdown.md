1. 链接

   要在 Markdown 中添加链接，请在文本两侧加上方括号，并在 URL 两侧加上圆括号，例如：[Udacity's home page](https://www.udacity.com) 表示指向 Udacity's home page 的链接。

2. 强调效果

   可以在文本两侧使用一对星号或下划线（* 或 __）来表示斜体。

   在文本两侧使用两对星号或下划线（* 或 __）来表示粗体。

   在一行的行首添加-，可以在一行的前面添加点符号

3. 代码

   可以通过两种不同的方式显示代码，一种是与文本内联，另一种是将代码块与文本分离。要将代码变为内联格式，请在文本两侧加上反撇号。例如，`string.punctuation` 会呈现为 string.punctuation。

   要创建代码块，请另起一行并用三个反撇号（一般在键盘数字1左边）将文本包起来：

   ```
   import requests
   response = requests.get('https://www.udacity.com')
   ```

   或者将代码块的每一行都缩进四个空格。
   import requests
   response = requests.get('https://www.udacity.com')

4. 数学表达式

   在 Markdown 单元格中，可以使用 LaTeX 符号创建数学表达式。notebook 使用 MathJax 将 LaTeX 符号呈现为数学符号。要启动数学模式，请在 LaTeX 符号两侧加上美元符号（例如 $y = mx + b$），以创建内联的数学表达式。对于数学符号块，请使用两个美元符号：
   $$
   y = \frac{a}{b+c}
   $$
   此功能的确很有用，因此，如果你没有用过 LaTeX，请阅读这篇入门文档（http://data-blog.udacity.com/posts/2016/10/latex-primer/），它介绍了如何使用 LaTeX 来创建数学表达式。

5. 在文字前边添加 > 可以添加竖线

   > ​

