# 打开`Markdown`的正确姿势

0. 这篇文章的目的

   希望大家都能学会`Markdown`来记录实验记录，实验报告等，**希望大家投稿的时候都尽量用`Markdown`**

1. 什么是`Markdown`

   `Markdown`是一种轻量级的标记性语言，也是写文档用到的最主流的语言。与写论文用到的`LaTeX`复杂排版系统不同，`Markdown`提供了一种快速排版与标记的语法，例如，多达6级的标题

   ## 二级标题

   ### 三级标题

   #### 四级标题

   ##### 五级标题

   ##### 六级标题

   再比如可以方便地插入代码块(而不用像word那样要去别的网站复制粘贴格式)

   ```clike
   return 0;
   ```

   也可以优美地使用$L^AT_EX$公式(word自带的公式块渲染非常麻烦，更不用说墨迹公式了)
   $$
   \oint_\gamma f(z)dz=2\pi \sum^n_{k=1} I(\gamma,a_k)Res(f,a_k)
   $$
   等等（类似博客园，知乎等网站里面的博客都是使用这套排版系统）

   本站的所有内容都是基于`Markdown`完成的

2. `Markdown`是怎么工作的

   首先`Markdown`编辑器会将`Markdown`语法的源文件编译为`html`格式的代码，然后进行渲染，得到各种不同的排版效果，所以如果想要实现更多复杂的效果，你可以直接在`.md`文件里面写`html`。

3. 如何使用`Markdown`

   `Markdown`文件都是以`.md`结尾，你可以通过直接新建文本文件然后把`.txt`后缀直接改成`.md`，然后使用对应的`Markdown`编辑器打开即可。

   主流的`Markdown`编辑器推荐两个：一是[Typora](https://typoraio.cn/)(付费)、二是[MarkText](https://github.com/marktext/marktext/releases)(开源但是访问这个链接记得科学上网)

   *下附一些基本的`Markdown`语法，详情访问[markdownguide](https://www.markdownguide.org/basic-syntax)网站*

   ### 多级标题

   <table class="table table-bordered">
     <thead class="thead-light">
       <tr>
         <th>Markdown</th>
         <th>HTML</th>
         <th>Rendered Output</th>
       </tr>
     </thead>
     <tbody>
       <tr>
         <td><code class="highlighter-rouge"># Heading level 1</code></td>
         <td><code class="highlighter-rouge">&lt;h1&gt;Heading level 1&lt;/h1&gt;</code></td>
         <td><h1 class="no-anchor" data-toc-skip="" id="heading-level-1">Heading level 1</h1></td>
       </tr>
       <tr>
         <td><code class="highlighter-rouge">## Heading level 2</code></td>
         <td><code class="highlighter-rouge">&lt;h2&gt;Heading level 2&lt;/h2&gt;</code></td>
         <td><h2 class="no-anchor" data-toc-skip="" id="heading-level-2">Heading level 2</h2></td>
       </tr>
       <tr>
         <td><code class="highlighter-rouge">### Heading level 3</code></td>
         <td><code class="highlighter-rouge">&lt;h3&gt;Heading level 3&lt;/h3&gt;</code></td>
         <td><h3 class="no-anchor" data-toc-skip="" id="heading-level-3">Heading level 3</h3></td>
       </tr>
       <tr>
         <td><code class="highlighter-rouge">#### Heading level 4</code></td>
         <td><code class="highlighter-rouge">&lt;h4&gt;Heading level  4&lt;/h4&gt;</code></td>
         <td><h4 class="no-anchor" id="heading-level-4">Heading level 4</h4></td>
       </tr>
       <tr>
         <td><code class="highlighter-rouge">##### Heading level 5</code></td>
         <td><code class="highlighter-rouge">&lt;h5&gt;Heading level 5&lt;/h5&gt;</code></td>
         <td><h5 class="no-anchor" id="heading-level-5">Heading level 5</h5></td>
       </tr>
       <tr>
         <td><code class="highlighter-rouge">###### Heading level 6</code></td>
         <td><code class="highlighter-rouge">&lt;h6&gt;Heading level 6&lt;/h6&gt;</code></td>
         <td><h6 class="no-anchor">Heading level 6</h6></td>
       </tr>
     </tbody>
   </table>


   ### 分段

   使用空行分割不同段落即可

   <table class="table table-bordered">
     <thead class="thead-light">
       <tr>
         <th>Markdown</th>
         <th>Rendered Output</th>
       </tr>
     </thead>
     <tbody>
       <tr>
         <td>
           <code class="highlighter-rouge">
             I really like using Markdown.<br><br>
             I think I'll use it to format all of my documents from now on.
           </code>
         </td>
         <td>
           <p>I really like using Markdown.</p>
           <p>I think I'll use it to format all of my documents from now on.</p>
         </td>
       </tr>
     </tbody>
   </table>


   注意：`Markdown`没有所谓的缩进分段的概念，不要在每段开始打四个空格

   ### 粗体与斜体

   <table class="table table-bordered">
     <thead class="thead-light">
       <tr>
         <th>Markdown</th>
         <th>HTML</th>
         <th>Rendered Output</th>
       </tr>
     </thead>
     <tbody>
       <tr>
         <td><code class="highlighter-rouge">I just love **bold text**.</code></td>
         <td><code class="highlighter-rouge">I just love &lt;strong&gt;bold text&lt;/strong&gt;.</code></td>
         <td>I just love <strong>bold text</strong>.</td>
       </tr>
       <tr>
         <td><code class="highlighter-rouge">I just love __bold text__.</code></td>
         <td><code class="highlighter-rouge">I just love &lt;strong&gt;bold text&lt;/strong&gt;.</code></td>
         <td>I just love <strong>bold text</strong>.</td>
       </tr>
       <tr>
         <td><code class="highlighter-rouge">Love**is**bold</code></td> <td><code class="highlighter-rouge">Love&lt;strong&gt;is&lt;/strong&gt;bold</code></td>
         <td>Love<strong>is</strong>bold</td>
       </tr>
     </tbody>
   </table>


   <table class="table table-bordered">
     <thead class="thead-light">
       <tr>
         <th>Markdown</th>
         <th>HTML</th>
         <th>Rendered Output</th>
       </tr>
     </thead>
     <tbody>
       <tr>
         <td><code class="highlighter-rouge">Italicized text is the *cat's meow*.</code></td>
         <td><code class="highlighter-rouge">Italicized text is the &lt;em&gt;cat's meow&lt;/em&gt;.</code></td>
         <td>Italicized text is the <em>cat’s meow</em>.</td>
       </tr>
       <tr>
         <td><code class="highlighter-rouge">Italicized text is the _cat's meow_.</code></td>
         <td><code class="highlighter-rouge">Italicized text is the &lt;em&gt;cat's meow&lt;/em&gt;.</code></td>
         <td>Italicized text is the <em>cat’s meow</em>.</td>
       </tr>
       <tr>
         <td><code class="highlighter-rouge">A*cat*meow</code></td>
         <td><code class="highlighter-rouge">A&lt;em&gt;cat&lt;/em&gt;meow</code></td>
         <td>A<em>cat</em>meow</td>
       </tr>
     </tbody>
   </table>


   ### 引用

   <p>To create a blockquote, add a <code class="language-plaintext highlighter-rouge">&gt;</code> in front of a paragraph.</p>

   <div class="language-plaintext highlighter-rouge"><div class="highlight"><pre class="highlight"><code>&gt; Dorothy followed her through many of the beautiful rooms in her castle.
   </code></pre></div></div>


   <p>The rendered output looks like this:</p>

   <blockquote>
     <p>Dorothy followed her through many of the beautiful rooms in her castle.</p>
   </blockquote>


   也可以这样子用

   <div class="language-plaintext highlighter-rouge"><div class="highlight"><pre class="highlight"><code>&gt; #### The quarterly results look great!
   &gt;
   &gt; - Revenue was off the chart.
   &gt; - Profits were higher than ever.
   &gt;
   &gt;  *Everything* is going according to **plan**.
   </code></pre></div></div>


   对应的渲染为

   <blockquote>
     <h4 class="no-anchor" id="the-quarterly-results-look-great">The quarterly results look great!</h4>
     <ul>
       <li>Revenue was off the chart.</li>
       <li>Profits were higher than ever.</li>
     </ul>
     <p><em>Everything</em> is going according to <strong>plan</strong>.</p>
   </blockquote>

   ### 有序列表

   <table class="table table-bordered">
     <thead class="thead-light">
       <tr>
         <th>Markdown</th>
         <th>HTML</th>
         <th>Rendered Output</th>
       </tr>
     </thead>
     <tbody>
     <tr>
       <td>
         <code class="highlighter-rouge">
           1. First item<br>
           2. Second item<br>
           3. Third item<br>
           4. Fourth item
         </code>
       </td>
       <td>
         <code class="highlighter-rouge">
           &lt;ol&gt;<br>
             &nbsp;&nbsp;&lt;li&gt;First item&lt;/li&gt;<br>
             &nbsp;&nbsp;&lt;li&gt;Second item&lt;/li&gt;<br>
             &nbsp;&nbsp;&lt;li&gt;Third item&lt;/li&gt;<br>
             &nbsp;&nbsp;&lt;li&gt;Fourth item&lt;/li&gt;<br>
           &lt;/ol&gt;
         </code>
       </td>
       <td>
         <ol>
           <li>First item</li>
           <li>Second item</li>
           <li>Third item</li>
           <li>Fourth item</li>
         </ol>
       </td>
       </tr>
       <tr>
         <td>
           <code class="highlighter-rouge">
             1. First item<br>
             1. Second item<br>
             1. Third item<br>
             1. Fourth item
           </code>
         </td>
         <td>
           <code class="highlighter-rouge">
             &lt;ol&gt;<br>
               &nbsp;&nbsp;&lt;li&gt;First item&lt;/li&gt;<br>
               &nbsp;&nbsp;&lt;li&gt;Second item&lt;/li&gt;<br>
               &nbsp;&nbsp;&lt;li&gt;Third item&lt;/li&gt;<br>
               &nbsp;&nbsp;&lt;li&gt;Fourth item&lt;/li&gt;<br>
             &lt;/ol&gt;
           </code>
         </td>
         <td>
           <ol>
             <li>First item</li>
             <li>Second item</li>
             <li>Third item</li>
             <li>Fourth item</li>
           </ol>
         </td>
       </tr>
       <tr>
         <td>
           <code class="highlighter-rouge">
             1. First item<br>
             8. Second item<br>
             3. Third item<br>
             5. Fourth item
           </code>
         </td>
         <td>
           <code class="highlighter-rouge">
             &lt;ol&gt;<br>
               &nbsp;&nbsp;&lt;li&gt;First item&lt;/li&gt;<br>
               &nbsp;&nbsp;&lt;li&gt;Second item&lt;/li&gt;<br>
               &nbsp;&nbsp;&lt;li&gt;Third item&lt;/li&gt;<br>
               &nbsp;&nbsp;&lt;li&gt;Fourth item&lt;/li&gt;<br>
             &lt;/ol&gt;
           </code>
         </td>
         <td>
           <ol>
             <li>First item</li>
             <li>Second item</li>
             <li>Third item</li>
             <li>Fourth item</li>
           </ol>
         </td>
       </tr>
       <tr>
         <td>
           <code class="highlighter-rouge">
             1. First item<br>
             2. Second item<br>
             3. Third item<br>
             &nbsp;&nbsp;&nbsp;&nbsp;1. Indented item<br>
             &nbsp;&nbsp;&nbsp;&nbsp;2. Indented item<br>
             4. Fourth item
           </code>
         </td>
         <td>
           <code class="highlighter-rouge">
             &lt;ol&gt;<br>
               &nbsp;&nbsp;&lt;li&gt;First item&lt;/li&gt;<br>
               &nbsp;&nbsp;&lt;li&gt;Second item&lt;/li&gt;<br>
               &nbsp;&nbsp;&lt;li&gt;Third item<br>
                 &nbsp;&nbsp;&nbsp;&nbsp;&lt;ol&gt;<br>
                   &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&lt;li&gt;Indented item&lt;/li&gt;<br>
                   &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&lt;li&gt;Indented item&lt;/li&gt;<br>
                 &nbsp;&nbsp;&nbsp;&nbsp;&lt;/ol&gt;<br>
               &nbsp;&nbsp;&lt;/li&gt;<br>
               &nbsp;&nbsp;&lt;li&gt;Fourth item&lt;/li&gt;<br>
             &lt;/ol&gt;
           </code>
         </td>
         <td>
           <ol>
             <li>First item</li>
             <li>Second item</li>
             <li>Third item
               <ol>
                 <li>Indented item</li>
                 <li>Indented item</li>
               </ol>
             </li>
             <li>Fourth item</li>
           </ol>
         </td>
       </tr>
     </tbody>
   </table>

   ### 无序列表

   <table class="table table-bordered">
     <thead class="thead-light">
       <tr>
         <th>Markdown</th>
         <th>HTML</th>
         <th>Rendered Output</th>
       </tr>
     </thead>
     <tbody>
       <tr>
         <td>
           <code class="highlighter-rouge">
             - First item<br>
             - Second item<br>
             - Third item<br>
             - Fourth item
           </code>
         </td>
         <td>
           <code class="highlighter-rouge">
             &lt;ul&gt;<br>
               &nbsp;&nbsp;&lt;li&gt;First item&lt;/li&gt;<br>
               &nbsp;&nbsp;&lt;li&gt;Second item&lt;/li&gt;<br>
               &nbsp;&nbsp;&lt;li&gt;Third item&lt;/li&gt;<br>
               &nbsp;&nbsp;&lt;li&gt;Fourth item&lt;/li&gt;<br>
             &lt;/ul&gt;
           </code>
         </td>
         <td>
           <ul>
             <li>First item</li>
             <li>Second item</li>
             <li>Third item</li>
             <li>Fourth item</li>
           </ul>
         </td>
       </tr>
       <tr>
         <td>
           <code class="highlighter-rouge">
             * First item<br>
             * Second item<br>
             * Third item<br>
             * Fourth item
           </code>
         </td>
         <td>
           <code class="highlighter-rouge">
             &lt;ul&gt;<br>
               &nbsp;&nbsp;&lt;li&gt;First item&lt;/li&gt;<br>
               &nbsp;&nbsp;&lt;li&gt;Second item&lt;/li&gt;<br>
               &nbsp;&nbsp;&lt;li&gt;Third item&lt;/li&gt;<br>
               &nbsp;&nbsp;&lt;li&gt;Fourth item&lt;/li&gt;<br>
             &lt;/ul&gt;
           </code>
         </td>
         <td>
           <ul>
             <li>First item</li>
             <li>Second item</li>
             <li>Third item</li>
             <li>Fourth item</li>
           </ul>
         </td>
       </tr>
       <tr>
         <td>
           <code class="highlighter-rouge">
             + First item<br>
             + Second item<br>
             + Third item<br>
             + Fourth item
           </code>
         </td>
         <td>
           <code class="highlighter-rouge">
             &lt;ul&gt;<br>
               &nbsp;&nbsp;&lt;li&gt;First item&lt;/li&gt;<br>
               &nbsp;&nbsp;&lt;li&gt;Second item&lt;/li&gt;<br>
               &nbsp;&nbsp;&lt;li&gt;Third item&lt;/li&gt;<br>
               &nbsp;&nbsp;&lt;li&gt;Fourth item&lt;/li&gt;<br>
             &lt;/ul&gt;
           </code>
         </td>
         <td>
           <ul>
             <li>First item</li>
             <li>Second item</li>
             <li>Third item</li>
             <li>Fourth item</li>
           </ul>
         </td>
       </tr>
       <tr>
         <td>
           <code class="highlighter-rouge">
             - First item<br>
             - Second item<br>
             - Third item<br>
             &nbsp;&nbsp;&nbsp;&nbsp;- Indented item<br>
             &nbsp;&nbsp;&nbsp;&nbsp;- Indented item<br>
             - Fourth item
           </code>
         </td>
         <td>
           <code class="highlighter-rouge">
             &lt;ul&gt;<br>
               &nbsp;&nbsp;&lt;li&gt;First item&lt;/li&gt;<br>
               &nbsp;&nbsp;&lt;li&gt;Second item&lt;/li&gt;<br>
               &nbsp;&nbsp;&lt;li&gt;Third item<br>
                 &nbsp;&nbsp;&nbsp;&nbsp;&lt;ul&gt;<br>
                   &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&lt;li&gt;Indented item&lt;/li&gt;<br>
                   &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&lt;li&gt;Indented item&lt;/li&gt;<br>
                 &nbsp;&nbsp;&nbsp;&nbsp;&lt;/ul&gt;<br>
               &nbsp;&nbsp;&lt;/li&gt;<br>
               &nbsp;&nbsp;&lt;li&gt;Fourth item&lt;/li&gt;<br>
             &lt;/ul&gt;
           </code>
         </td>
         <td>
           <ul>
             <li>First item</li>
             <li>Second item</li>
             <li>Third item
               <ul>
                 <li>Indented item</li>
                 <li>Indented item</li>
               </ul>
             </li>
             <li>Fourth item</li>
           </ul>
         </td>
       </tr>
     </tbody>
   </table>

   ### 行内代码

   <table class="table table-bordered">
     <thead class="thead-light">
       <tr>
         <th>Markdown</th>
         <th>HTML</th>
         <th>Rendered Output</th>
       </tr>
     </thead>
     <tbody>
       <tr>
         <td><code class="highlighter-rouge">At the command prompt, type `nano`.</code></td>
         <td><code class="highlighter-rouge">At the command prompt, type &lt;code&gt;nano&lt;/code&gt;. </code></td>
         <td>At the command prompt, type <code class="highlighter-rouge">nano</code>.</td>
       </tr>
     </tbody>
   </table>

   ### 表格

   <div class="language-plaintext highlighter-rouge"><div class="highlight"><pre class="highlight"><code>| Syntax      | Description |
   | ----------- | ----------- |
   | Header      | Title       |
   | Paragraph   | Text        |
   </code></pre></div></div>

   对应的渲染为

   <table class="table table-bordered">
     <thead>
       <tr>
         <th>Syntax</th>
         <th>Description</th>
       </tr>
     </thead>
     <tbody>
       <tr>
         <td>Header</td>
         <td>Title</td>
       </tr>
       <tr>
         <td>Paragraph</td>
         <td>Text</td>
       </tr>
     </tbody>
   </table>

   ### 代码块

   <div class="language-plaintext highlighter-rouge"><div class="highlight"><pre class="highlight"><code>```javascript
   console.log("Hello, World!");
   ```
   </code></pre></div></div>

   对应的渲染为

   ```javascript
   console.log("Hello, World!");
   ```

   ### $L^AT_EX$公式

   <div class="language-plaintext highlighter-rouge"><div class="highlight"><pre class="highlight"><code>$$
   a^2 + b^2 = c^2
   $$
   </code></pre></div></div>

   对应的渲染为
   $$
   a^2+b^2=c^2
   $$
   

   有关$L^AT_EX$的更多符号使用方法，可以参见这篇[知乎](https://zhuanlan.zhihu.com/p/510451940?utm_id=0)

   更多有关`Markdown`的内容可以到[这里](https://www.markdownguide.org/)查找，本站非常欢迎各种经验攻略的`Markdown`文件投稿。*如果执意希望用其他方式比如`word`,`txt`，邮件投稿的时候向工作人员说明，我们会帮助排版内容*