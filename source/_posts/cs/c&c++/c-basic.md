---
title: c-basic
toc: true
---

> 备注：
> C语言读书摘录笔记，笔记内容绝大部分摘录整理自李春葆、李筏驰老师编著的《直击招聘——程序员面试笔试C语言深度解析》一书，少部分来自于网络博客及网上资源（尽量保留了资源原始链接）.

# 第一章 程序设计基础——变量



## 1.1 变量定义和声明

###  1.1.1定义变量就是使用内存

1.如果一个变量占用内存空间的多个内存字节，其第一个字节地址就是它的存储地址

2.简单理解为变量就是内存的一个箱子，箱子的名称就是变量名，用户可以向其中放入数据，也可以取出其中的数据。

###  1.1.2变量的作用域和在内存中的存储方式

1.变量的作用域：确定变量的作用范围

- 局部变量：在函数内部定义的变量为内部变量，只在本函数范围内有效，在该函数外不能使用这些变量。

- 全局变量：在函数之外定义的变量为外部变量，作用域从定义变量的位置开始到本源程序文件结束，全局变量保存在静态存储区。

- C程序中局部变量和全局变量重名时，局部变量会屏蔽全局变量

 

2.变量的存储类别：确定变量存放在内存的位置

- auto：自动变量，缺省情况下，编译器默认所有局部变量为自动变量，它的存储空间由系统自动分配和释放，系统不会自动初始化。【影响】若定义局部变量时不初始化值，那么此时该局部变量为无意义的辣鸡值，因为自动变量不会初始化。

- register：寄存器变量，变量值存放在CPU内部寄存器中，存取速度最快，这类变量不能进行取变量地址操作。只有局部自动变量和函数形参才可以定义为寄存器变量。

- extern：外部变量，全局变量是从作用域角度提出，而外部变量是从存储类别提出。该关键字告诉编译器存在着一个变量或函数，即使在当前源程序文件中没有看到它，也不是错误的，它可能在另外一个源程序文件中或者在当前文件的后面定义。extern的作用有两个：

- 在函数中提前使用全局变量（提前声明），即在定义之前使用

- 使用其他源文件中的全局变量

- static：静态变量，在函数内部用该关键字定义的变量称为静态局部变量，在函数外部用static关键字定义的变量称为静态全局变量。

- 静态局部变量，程序执行期间，在内存的静态存储区中占据这永久性的存储单元，即便退出函数后，该变量的生命期也不终止，下次再次进入函数时，仍使用原来的存储单元。定义时初始化的静态局部变量，初始化仅仅执行一次；对于初始化的静态局部变量，C编译系统自动给它幅值0.

- 静态全局变量，作用域只限于本源程序文件。静态全局变量和普通全局变量的区别是静态全局变量只能初始化一次，由于静态全局变量的作用域限于本源程序文件内，只能为该源文件内的函数公用，因此可以避免在其他源文件中引用而引起错误。

3.内存组织结构

![](https://silencht.oss-cn-beijing.aliyuncs.com/img/19653046.png)

- 代码段：该内存区域属于只读，区域大小在程序执行前就已经确定

- 数据段：执行程序时，BSS段会预先清空，所以存放在BSS段的变量均默认初始化为0

- 堆空间：存放进程（可简单理解为程序的一次执行）执行中被动态分配的内存段，大小不固定，可动态扩张或缩减。从堆分配的内存仅能通过指针访问。速度一般较慢，容易产生内存碎片。

- 栈空间：方便用来保存/恢复调用现场，可以看作一个存放、交换临时数据的内存区。由系统自动分配，速度较快，程序员无法控制栈空间。

4.变量静态分配和动态分配方式

- 变量静态分配：程序编译期间分配固定的存储空间的方式。该存储分配方式在变量定义时就分配存储单元并一直保持不变，直到整个程序结束。所有变量都是采用静态分配方式。静、动态分配方式主要是针对指针变量（或者数组）指向的空间而言的。

- 变量动态分配：程序执行期间根据需要动态申请堆空间的方式。C语言提供了一套机制可以在程序执行时动态分配存储空间。如malloc(),calloc()等函数。

如果程序员在程序中采用动态分配方式分配大量内存空间，用完后不及时释放，可能会消耗完应用程序的内存空间，称之为**内存泄漏**。

## 1.2 运算符和表达式

### 1.2.1 C中的运算符

1.++n、n++、--n、n--的区别

- 虽然对于int数据类型的变量进行++、--运算完全可以用n=n±1完成，但是用++、--运算符可以提高程序的执行效率，因为++、--只需要一条机器指令就可以完成，而n=n±1需要对应三条机器指令

- 自增、自减运算符的运算对象只能是简单变量，不能是常数或是带有运算符的表达式

- 编译器实现n++、n--是先创建n的一个副本，然后n自增、自减1，最后返回这个副本的值，所以n++、n--不能作为左值（因为作为左值时改变的是副本的值）；而实现++n、--n是先n自增、自减1，最后返回n的值（不是副本的值），所以++n、--n可以作为左值。故for循环递增量表达式建议采用++i，而非i++，因为++i占用空间小。

![](https://silencht.oss-cn-beijing.aliyuncs.com/img/6d9e2fb7-72aa-43f5-be9d-e54f8265daea.png)

 

2.表达式中符号的求值

符号指的是程序的一个基本组成单元，其作用相当于一个句子中的单词。在C编译器解释表达式符号时，它在移动到下一个符号之前在单个符号中包括尽可能多的字符，即为贪心法。

- `i+++j`解释为`（i++）+j`

- `a+++++b`本应解释为`（a++）++ +b`，而`a++`不能作为左值，所以应解释为`（a++）+（++b）`

- `y=x/*p`按照贪心法，会把p当作注释语句，导致编译错误，所以应该书写为`y=x/（*p）`

 

3.表达式中求值中的类型转换

- 自动转换的原则

- 若参与运算类型不同，则先转换为同一类型

- 转换按数据长度增加的方向进行，以保证精度不降低

- 所有的浮点运算都是以双精度进行的，即使仅含float单精度运算表达式，也要先转换为double型

- char和short型参与运算时必须先转换为int型

- 幅值运算中，幅值号右边量的类型先转换为左边量的类型。如果右边的数据类型长度比左边长，将丢失一部分数据，降低精度，丢失的部分按照四舍五入向前舍入。

- 隐式转换见书25页-27页

 

4.表达式求值的副作用

一个表达式在求值的过程中对使用的变量不但引用，还对它们的值加以修改，这样的表达式称为有副作用的表达式，例如：x=x++，有两个副作用，因为x值发生了两次修改.

 

> a[i]=i++;

>

> 问题是：数组下标i是引用旧值还是引用新值？对这种情况编译器的解释可能不同，并因此产生不同的结果。C语言标准对大多数这类问题有意未做具体规定。表达式何时会产生这种副作用（对变量赋值），将由编译器决定，因为最佳的求值顺序同机器结构有很大关系。（ANSI C标准明确规定了所有对参数的副作用都必须在函数调用之前生效，但这对前面介绍的printf函数调用并没有什么帮助）。

>

> 在任何一种编程语言中，如果代码的执行结果与求值顺序相关，则都是不好的程序设计风格。很自然，有必要了解哪些问题需要避免。但是，如果不知道这些问题在各种机器上是如何解决的，就最好不要常识运用某种特殊的实现方式。

>

> K&R.The C Programming Language.43页




# 第二章 数据处理——控制结构


## 2.1 选择控制结构

### 2.1.1 If 语句

1. 计算if后面的表达式，结果为0或者空字符时表示假，为非0或者非空字符时表示真。当实数变量与0值进行比较时，由于实数有精度限制，不能写成`if(f==0.0)`的形式，而应该写为`if(f＞=EPSINON && f＜=EPSINON)`，其中EPSINON是允许的误差（精度），如取值为0.000001.
1. 在if的表达式中不能将“==”写为“=”，后者为幅值表达式，总是为真。好的习惯应是写为`if(2==n)`，这样可以避免类似的错误出现。

### 2.1.2 Switch语句

1. switch后面圆括号内的“表达式”的值只能是整数或字符值，不允许是实数和布尔值，例如`switch(n==1)`是错误的
1. case后面的值必须是整形或字符型值，不允许是实数，也不允许含有运算符，例如`case 1.2：`和`case 1+2：`都是错误的
1. 同一switch语句的所有case的值必须互不相同，多个case可以共用一组执行语句

## 2.2 循环控制结构

### 2.2.1 for语句 `for(表达式1；表达式2；表达式3) 语句；`

1. 表达式1可以省略，此时应该在for语句之前给循环变量赋初值，其后的分号不能省略
1. 表达式2可以省略，即不继续判断条件，循环无终止的进行下去，需要在循环体中用break等语句退出循环，其后的分号不能省略
1. 表达式3可以省略，这样需要在循环体中让循环变量变化，以保证循环能正常的结束

### 2.2.2 break和continue语句

1. break只能用于循环语句和switch语句中，它跳出所在的那一层循环语句或者switch语句
1. continue只能用于循环语句中，它跳出所在的那一轮循环，继续下一轮循环



# 第三章 内存操作——指针

 

## 3.1 指针基础

### 3.1.1 指针变量与运算

指针变量中存放的是地址值，无论指针变量的基类型是何种数据类型，占用的内存大小都是相同的。

### 3.1.2 野指针

一个指针变量的值（地址值）为垃圾值的指针变量称为野指针。产生野指针的原因和解决方法：

- 指针变量定义时没有被初始化。解决办法是定义指针时初始化，可以是具体的地址值，也可以是NULL

- 指针p被free或者delete之后没有被置为NULL，后面还使用它。解决办法是指针指向的内存空间被释放后指针应该指向NULL

- 指针操作超越了所指变量的作用域。解决办法是在所指变量的作用域结束前释放掉变量的地址空间，并让指针指向NULL

 

## 3.2 常量和常量指针

### 3.2.1 常量

程序执行期间其值**不能被改变的量**称为**常量**，常量分为**字面常量**和**符号常量**

1. **字面常量**

字面常量只能引用不能修改，如123等，通常保存在程序符号表中，程序无法读取字面常量的地址，只有一个例外，即字符串常量。例如：`char *p="abc";`，字符串常量放在静态数据区，由p指针指向它，**不能通过p指针来修改该常量**。

 

 程序员最好采用`const char *p="abc";`定义，这样在执行`*p='x'`时会发生编译错误，以便避免bug。

 

 `int *p=123；`也是不允许的，尽管123是常量，但这里编译器认为是将123作为地址存放在指针变量p中，而123是整数，正确的做法是`int *p=（int *）123；`即将123转换为地址值赋给p，但这种做法是有危险的。

 

 又由于p指向的常量字符串不是通过malloc函数分配的，所以执行free（p）会导致程序崩溃。

1. **符号常量**

 符号常量主要又两种定义方法：

 - 第一种是用宏定义实现（即宏常量），例如`#define PI 3.14`

 - 第二种是用const定义（即const常量），const的意思是“一个不能被改变的变量”，例如`const int n=123；`

 - const修饰的常量的值不能修改，所以必须在定义时初始化

 

 两种定义方法的区别：

  - 前者是宏替换命令，不是语句，所以不以“；”结尾，后者是定义，以“；”结尾。

  - 前者在预处理时进行替换，后者定义的常量像变量一样（称为常变量），只是其值不能改变

  - const常量有数据类型，而宏常量没有数据类型。编译器对前者进行类型安全检查，对后者不进行类型安全检查。

 

### 3.2.2 const指针常量、常量指针

在定义指针时用const关键字进行修饰，称为const指针常量，有三种情况：

 

1.**常量指针**

 

用const修饰`*`时称为常量指针，表示**不能修改p指向的内容** [注意此时p指向的内容仅仅是不能通过指针p修改，其自身如果不是常量的话，可以通过其他方式修改]。例如：

 

`const char *p;`，此时不能通过p指针修改指向的内容，否则会出现编译错误。

- 也可以写作`char const *p;`，但习惯上常用第一种

- 它的本质还是一个指针，是一个指向常量的指针（变量）

- 指针本身的指向可以改变，但是指向的内容不可修改

- 通常用于**参数传递**过程中，如果被传入的参数的值在函数执行期间不希望被修改可以使用`const` 修饰已达到安全的目的。

 

2.**指针常量**

 

`char * const p；`const修饰p，**表示不能修改变量p**。指针p是一个指针常量，p的值不能再发生改变，所以必须初始化。一旦初始化，p不能指向其他数据，但可以通过指针p修改所指的内容。

- 一般指针常量用于不会发生指向变化的指针，但是用法并不常见。

 

3.**指向常量的常指针**

 

相当于常量指针和指针常量的结合，格式 `const char * const p = &num`,相当于有一个**指向不可修改的指针指向了一个不可修改的常量**，在实际coding中很少使用。

 

4.**总结**

- **const 修饰谁，谁就不变**

 

## 3.3 多级指针

### 3.3.1 `void *`和`void **`

- `void`：字面意思是**无类型**，真正发挥作用的地方在于对函数返回值的限定和对函数参数的限定；

- `void *`：**无类型指针**，可以指向任何类型的数据。

 - 既然是无类型指针，那么就不能做**解引用**与**指针算数运算**.

 - 任何类型的指针都可以直接幅值给它，无需进行强制类型转换，但这并不意味这`void *`也可以无须强制类型转换地赋给其他类型的指针。因为“无类型”可以包容“有类型”，但是“有类型”不能包容“无类型”，例如：

 

```c
void *p1;

int *p2;

double  *p3;

p3=p2;//错误，必须改为第5行，强制转换后才可编译通过

p3=(double *)p2;//正确

p1=p2;//正确，可以将任何有类型地址赋给无类型指针变量

p2=p1;//错误，不能将任何无类型地址赋给有类型指针变量

p2=(int *)p1;//正确，将无类型地址强制转换为有类型地址，这就是malloc等函数原型的返回值为void *的原因

 

```

- `void *`**的用处**：因为对于函数的通用型接口，你不知道用户的数据类型是什么，但是你必须能够处理用户的各种类型数据，因而会使用`void*`。`void*`能包容地接受各种类型的指针。也就是说，如果你期望接口能够接受任何类型的参数，你可以使用`void*`类型。但是在具体使用的时候，你必须转换为具体的指针类型。例如，你传入接口的是`int*`，那么你在使用的时候就应该按照`int*`使用。

 

- `void **`：本质上是标识一个二级指针,即**无类型指针的指针**，它指向一个放 `void*`型的地方.

 

 `(void**)&data`：把变量的地址强制转换为无类型指针的指针，即`(void**)`本质表示将`&data`强制转换类型为一个指向无类型的二级指针。举例：

 

```c
//为了使函数更加的通用，使用void**作为函数参数类型

void swap(void **a, void **b)

{

    void *t;

    t =*a;

    *a =*b;

    *b=t;

}

int main()

{

    int i = 3;

    int j = 5;

    int *p = &i;

    int *q = &j;

    char *s1="abc";

    char *s2="def";

    swap((void**)&p, (void**)&q);

    swap((void**)&s1, (void**)&s2);

}

//注意char*是字符串指针，需要改变其对应的变量必须用地址，s1就是"abc"的起始地址，是不能被改变，要想改变s1必须用他的地址也就是&s1，所以需要void**

————————————————

版权声明：本文为CSDN博主「unix21」的原创文章，遵循 CC 4.0 BY-SA 版权协议，转载请附上原文出处链接及本声明。

[原文链接](https://blog.csdn.net/unix21/article/details/8923752)

```

 

`*(void**)&data`：data是指针变量，&data取指针变量的地址， `(void **)`将这个data这个指针变量的地址类型强制转换为 指向无类型的二级指针类型，最后“翻译”最前面的`*`，作用是解引用，将 `(void **)`类型的&data变量解引用一次，即指向了data本身，但此时data本身的数据类型实际上已经是无类型的一级指针了。

 

![](https://silencht.oss-cn-beijing.aliyuncs.com/img/b1388529-d6b4-4ed3-aa38-7ec0cf449bff.jpg)

 

## 3.4 实例解析

 

### 3.4.1 （int&)a

 

代码：

 

```c
float a = 01f;

printf("%d",(int&)a);

```

 

解释：（存疑）

 

​    等价于`*(int*)(&a)`，

 

1. 首先对float型变量取地址

2. 强制类型转换为整型变量的地址（地址的值并没有变）

3. 将该地址指向的变量输出（但是由于整型和浮点型数据存储方式的不同，输出结果是不同的）




# 第4章 数据组织——数组

 

## 4.1 一维数组

 

### 4.1.1 一维数组定义

 

由于const定义的常量具有变量的性质，因此这类常量不能作为定义数组的“长度表达式”。

 

### 4.1.2 一维数组初始化

 

- 对于局部数组，若没有进行初始化，其所有元素值为垃圾值；若初始化时仅对部分元素赋了初值，其余元素取默认值（数值型为0，字符型为空字符）

 

- 除了初始化外，数组名不能作为左值，因为它是一个表示首元素地址的常量

 

- 数组名具有地址概念，是代表**数组** **首元素**的**地址常量**，因此可以将数组名赋给指针，例如`int a[10]`中`a`与`&a[0]`含义相同，“`a==&a[0]`”返回真

 

- `&a`表示**整个数组**的首地址，其与`a`的区别主要是**步长的不同**。`a+i=a+i*sizeof(int)`，其步长为`sizeof(int)`；而`&a+i=&a+i*(a数组的大小)`（这里为`10*sizeof(int)`）.因此尽管二者值相同，但表示不同的含义。因此`&a==a`是错误的比较，因为它们的数据类型并不相同（一个是`int *[10]`，一个是`int *`）

 

- 归纳起来，`int a[10],*p=a;`情况下：

 

  - `p+i`和`a+i`就是`a[i]`的地址，地址值都要进行`a+i*d`（d为步长）的运算

  - `*(p+i)`或`*(a+i)`就是`p+i`或`a+i`所指向的数组元素`a[i]`。数组元素中的“[]”是变址运算符，相当于`*(+)`，`a[i]`相当于`*(a+i)`

  - 指向数组元素的指针变量也可以带下标，所以`a[i]`、`*(a+i)`、`p[i]`、`*(p+i)`全部等价

  - 注意p和a的差别，p是指针变量，a是符号常量，a不能作为左值

 

- `*p++` 、`*++p`、 `(*p)++`、`++(*p)`区别

 

  - `*p++`

    解析：等同于`*p;  p += 1;`由于`*`和`++`的运算优先级一样，且是右结合。故`*p++`相当于`*(p++)`，p先与++结合，然后p++整体再与`*`结合。**前面陈述是一种最常见的错误**。因为++后置的时候，本身含义就是先运算后增加1（运算指的是p++作为一个整体与前面的`*`进行运算；增加1指的是p+1），所以**实际上`\*p++`符号整体对外表现的值是`\*p`的值，运算完成后p再加**1.

    【注意】是运算后p再加1，而不是p所指向的变量`*p`再加1

  - `*++p`

    解析：等同于 `p += 1; *p;`由于++在p的前面，++前置的含义是，先加1，得到一个新的p（它的值是原来p的值加1）。然后这个新的p再与前面的`*`结合.

 

  【总结】无论是`*p++`还是`*++p`，都是指针`p += 1`，即p的值+1，而不是p所指向的变量`*p`的值+1。++前置与++后置，只是决定了到底是先`p+=1`，还是先`*p`。++前置表示先`p+= 1`，再`*p`。++后置表示先`*p`，在`p+= 1`

 

  - `(*p)++`

    解析：使用()强制将`*`与p结合，只能先计算`*p`，然后复制创建`*p`副本，再对原始`*p`整体的值++，最后结果返回复制创建的`*p`副本值，因此其结果不能作为左值。

  - `++(*p)`

    解析：先`*p`取值，再前置++，该值+1后作为整个表达式的值。

 

### 4.1.3 一维数组和指针的差异

 

例如：`char *p="abc"; char s[]="abc";`

 

- 对于字符串"abc"，编译器存储的是常量"abc\0"

- 对于`char *p="abc";`

  - 定义的是一个字符指针p，没有动态分配空间来存放字符串，所以编译器把"abc"当作常量存放在静态数据区，p作为指针指向这个常量的地址。因此，最好采用`const char *p="abc";`的定义方式；

  - 该定义中，因为字符串是常量，因此不能通过指针p来修改"abc".

  - 如果希望p为**指针变量**，又要初始化"abc"，定义方式应为：

    `char *p=(char*)malloc(4sizeof(char));strcpy(p;"abc");`

  - `sizeof(p)`值为4，含义表示一个地址空间的大小

- 对于`char s[]="abc";`

  - 定义的是一个字符数组，编译器把它解析为 `char s[4]={'a','b','c','\0'};`

  - 如果数组s是已初始化的全局数组或者静态局部数组，则存放在静态数据区，如果是在函数内部定义的局部数组则存放到栈空间

  - `sizeof(s)`值为4，含义表示数组s含有4个字符元素（含结尾符）

 

### 4.1.4 运算符sizeof

 

- sizeof的三种语法格式

 

  - sizeof (object)

  - sizeof (type_name)

  - sizeof object

 

- 基本数据类型的sizeof：大小一般与系统相关

 

- 指针变量的sizeof：等于计算机内部地址总线的宽度

 

- 数组名的sizeof：等于其元素类型做sizeof的结果乘以数组元素的个数，即返回整个数组在内存中占用的内存字节数

 

- [数组名在作为函数参数传递过程中，会退化成指针，因此在函数内部使用数组名的sizeof时，返回退化为指针变量的sizeof，而非返回原数组名的sizeof.](https://blog.csdn.net/KangRoger/article/details/20653255)

 

- sizeof的副作用

 

  > sizeof(i++)之后，i的值会怎样？答案是不变。记得大一初学C语言时想研究一下sizeof与函数有什么区别，得到的结果只是一些语法上的差别；学了汇编之后看看编译器生成的代码，才发现sizeof在编译时直接给定了一个常值，而非在运行时求值。进而又分析过sizeof(表达式)的结果，清楚了类型提升原理。但我之前没有注意过表达式中出现副作用的问题，于是在sizeof(i++)的问题上犹豫了。现在经过查阅资料和实验，结论是：sizeof在大多数情况下是编译时定值的，表达式中的任何副作用（包括有副作用的运算符、函数调用等）都不会发生。这里说“大多数情况”，排除了针对C99的新特性——不定长数组（variable length array）的特例。

  > [原文出处点击此处](https://linjian.org/blog/tech/programming/c/sizeof-problems)

 

 

 

## 4.2 二维数组

 

### 4.2.1 二维数组的定义

 

- `数据类型 数组名[长度表达式1][长度表达式2]；`

  长度表达式1 指数组行大小，长度表达式2 指数组列大小，都必须为正整数

  二维数组中的所有元素在内存中按行序优先存放，即先顺序存放第1行的元素，再存放第2行的元素，以此类推。

 

### 4.2.2 多维数组的各级地址

 

- 以二维数组为例， 设二维数组 a 有3行4列，定义：`int a [3][4] = {{1,2,3,4},{5,6,7,8},{9,10,11,12}};`

  - 其中， a 是数组名，它的各元素是按行顺序存储的。a 数组有3 行，将它们看成3 个一维数

    组元素，即`a={a[O],a[1],a[2]} `，每个一维数组元素又含4 个元素。这种降维的思路可以扩

    展到三维或三维以上的数组。

  - **数组名 a 代表的是该二维数组首元素 a[0] 的首地址**，即a 与＆a[0] 的含义相同，

    `a==&a[0]`返回真，是正确的比较。因此，二维数组名是个二级地址（例如，`**a` 的结果为

    `a[0][0]` ），三维数组名是个三级地址，以此类推。

  - 二维数组 a 的一维数组元素`a[i] (0≤i≤2)`又是一级地址，例如， `a[0]`与`&a[0][0] `的含

    义相同(`&a[0][0] `中的＆运算符将其提升为一级地址）， `a[0]==&a[0][0]` 返回真，是正确的

    比较。

  - &a 是**整个二维数组的首地址**，为三级指针，所以`a==&a` 的比较是错误的，会出现

    `'==': no conversion from 'int (＊)[3][4]' to 'int (*)[4]' `的编译错误。

- **总结**：

  - 对于一维数组b, `b[j] `相当千`＊(b+j)`

  - 对千二维数组元素`a[i][j]` ，将一维数组元素`a[i] `当成 b 代入`＊(b+j)`得到`＊(a[i]+j) `，再将

    其中的`a[i]` 换成`＊(a+i)`又得到`＊(＊(a+i)+j) `，所以`a[i][j] `、 `＊(a[i]+j) `、 `＊(＊(a+i)+j)`三者相同都表示第 i 行第 j 列元素。

 

| 表达式                                  | 含义                                                         |

| :-------------------------------------- | :----------------------------------------------------------- |

| a、&a[0]                                | 分别为二维数组名和首元素a[0]的地址，两者含义相同，均为**二级地址**。 |

| a[0] 、`＊(a+O) `、＊a 、`&a[0][0]`     | 均为`a[0][0]`元素的地址，四者含义相同，均为一级地址          |

| `a[0]+1` 、`*a+1` 、`&a[0][1]`          | 均为`a[0][1]`元素的地址， 三者含义相同，均为一级地址         |

| a+1、`&a[1]`                            | 均为a[1] 的地址，两者含义相同，均为二级地址                  |

| a[1] 、`*(a+1)` 、`&a[1][0]`            | 均为`a[1][0]`元素的地址， 三者含义相同，均为一级地址         |

| a[1]+3 、`*(a+1)+3 `、`&a[1][3]`        | 均为`a[1][3]`元素的地址， 三者含义相同，均为一级地址         |

| `*(a[1]+3)` 、`*(*(a+1)+3)` 、`a[1][3]` | 均为`a[1][3] `元素， 三者含义相同                            |

 

（注意：二级地址与二级指针并非一个概念）

 

 

 

## 4.3 字符数组和字符串数组

 

 

 

### 4.3.1 字符数组

 

- 定义及初始化

  - 字符数组中的元素是字符，因此在对字符数组中的元素赋值时必须使用单引号

  - 初始化表中的初值个数可以少于数组元素的个数，这时只为数组的前几个元素赋初值，其余的元素将自动被赋以**空格符**（空格符不同于空字符，空字符的ASCII 码为0, 空格符的ASCII码为32)。如果初始化表中的初值个数多于数组元素的个数，则被当成语法错误来处理

- sizeof 运算符与strlen函数的差别：

  - strlen 函数用于**求一个字符串的实际长度**，从开始字符到遇见第1个'\0'，如果只定义没有给它赋初值，这个结果是不定的，它会从首地址一直找下去，直到遇到 '\0' 停止

  - sizeof 运算符**返回变量定义后所占内存的字节数**，不是实际长度。例如，定义char a[5], strlen(a)的结果是不定的，因为数组a没有赋初值，而sizeof(a)的结果为5

  - sizeof 可以用类型作为参数，而strlen 只能用char ＊作为参数，且必须是以“\0”结尾的

  - 数组在作为strlen 的参数时退化为指针，而作为sizeof 的参数时不退化

  - strlen 的结果要在执行时才能计算出来，是用来计算字符串的长度，不是类型占内存的大小；而sizeof 不能返回动态分配的空间大小

 

### 4.3.2 字符串数组

 

- 字符串数组的赋值操作

 

  - 初始化赋值，如：`char name[3][8]={ "Mary","Smith","GoodBye" };`

  - 使用scanf 或者gets 函数赋值，如：

 

  ```c
  scanf("%s",name[0]) ; ／／输入的字符串不能含空格

  gets(name[0]) ;       ／／输入的字符串可以含空格

  ```

 

  - 使用标准字符串函数strcpy等实现字符串的复制，如：`strcpy(name[0], "Smith");`

  - 使用一般赋值语句赋值，如：`name [0][0]='M'; name [0][1] ='a';etc...` 注：在这种情况下，编译器不会自动添加结尾符 '\0', 需要程序添加

 

 

 

## 4.4 指针数组

 

 

 

- 当多个基类型相同的指针变量集合成数组时，就形成了指针数组。指针数组是**指针的集合**，它的每个元素都是一个指针变量。其定义形式为：`数据类型 ＊指针数组名［长度表达式］；`

 

- 实例解析：`int *p[3];`

 

  由千`[]`比`*`优先级高，因此p先与[3]结合，形成p[3] 的数组形式，它有3个元素。然后再与p前面的`*`结合，表示是**指针类型的数组**，该数组的每个元素都是整型数的指针，所以每个元素都具有指针的特性。

 

 

 

## 4.5 数组指针

 

 

 

因为数组名是常量，不能像变量那样操作，为此可以设计指向数组的指针变量，以便于数组的操作。

 

- 一维数组指针，例如`int a[]={l,2,3},*p=a; `，通过指针p 访问数组a 的元素，其中a 为一级地址，p 为一级指针。

 

- 二维数组指针，定义格式：`基类型 （＊指针变量）［长度表达式］`，“长度表达式”指出二维数组中列的大小.

 

  - 例如：`int a[2][3] , (*p)[3]=a; `  解析（对比指针数组例子）：

 

    在`(*p)[3] `中，由于括号和［］的优先级相同，其结合性是从左到右的，所以“*“首先与p 结合，表示p是一个指针变量，然后再与［］结合，表示指针变量p的基类型（即这个p指向的变量的类型）是一个包含有3 个int型元素的数组，也就是说p为一个二维数组的指针变量，该数组中每列有3个元素。

 


- 二维数组名不能直接幅给二维指针的原因

 

  - **二维数组名** 指向 一个包含有已知列数量个基础类型元素的数组 的**二级地址常量**，而**二维指针** 是指向 基础类型的指针 的**二维指针变量**；例如：

 

    ```c
    
    int a[2][3];  int **pa;     pa=a;//p=a会报错
    
    int a[2][3];  int (*pb)[3]; pb=a;//正确
    
    int a[2][3];  int **pc;     pc=(int**)a;//正确
    
    ```

 


    - 因为a指向包含3个int型元素的一维数组，a逻辑上等同于&a[0]，此时a+1是a[1]的地址，所以步长为sizeof(a[0])=4×3=12 ；

 


    - 而pa是指向int *类型的二维指针，步长未知，因此两者指向对象的类型并不相同。而第二行代码中，说明了pb是一个指针变量，且pb这个指针变量指向的是一个包含有3个int型元素的数组，其步长也为12 ；这也解释了**为什么定义二维数组指针时必须要指定列的大小**：因为要“指定”你的二级地址/二维指针的步长。
    
    -  第三行代码强制转换后语法正确，但不能通过pc 来访问数组a 的元素，因为强制转换之后已经丢失步长信息。

 


  - 一旦定义了二维数组指针变量，该数组指针变量就可以像数组名一样使用，且可以在数组元素中移动.

 

- 三维数组指针

 

  - 定义：`基类型 ( (＊指针变批)[第二维长度] ) [第三维长度];`




# 第5章 数据结构II——结构体与联合体

 

## 5.1 结构体

 

### 5.1.1 结构体类型的声明

 

- 结构体类型声明语句必须以分号结尾，可以放在函数内部， 也可以放在函数外部，其作用域和变量的作用域类似

- 不同于变量可以使用extern 声明，必须先声明结构体类型，再定义其变量，如在a.c文件中定义了`struct Student `结构体之后，在b.c文件中 `extern struct Student `的提取声明将被忽略，也就是不接受结构体类型的提取声明.因为此时b.c文件只知道结构体变量的性质，并不知道其定义。简单的办法就是把结构体定义放在公共头文件中，a.c和b.c都包含该头文件。

 

### 5.1.2 结构体变量的定义

 

```c
struct Student st;//传统C语言方式

Student st;//C++方式

```

 

注：从语法角度出发，结构体变虽可以和结构体成员同名，因为它们处于不同的＂层次“

上，不会有二义性，但从软件工程角度出发建议不要这样做。

 

### 5.1.3 结构体变量的引用

 

- 引用结构体变量中的一个成员

 

  ```c
  结构体变量．成员名     // "."为结构体变量成员访问运算符

  结构体指针变量->成员名// "->"为结构体指针变量成员访问运算符

  ```

 

- 结构体类型变量的整体引用

 

  - 用户可以将一个结构体变量作为一个整体赋给另一个同结构体类型的结构体变量，其前提条件是两个结构体变量必须具有完全相同的结构体类型。

 

  - 当结构体内成员有指针变量类型时，单纯的整体幅值会造成“浅复制”，即a结构体变量幅值给b结构体变量后，b的指针变量成员同样指向了a结构体的对应成员指向的位置。因此以后对b的该成员的操作可能会直接影响到a结构体变量。

 

### 5.1.4 结构体变量的初始化

 

​    在对结构体变量赋初值时， C 编译程序按每个成员在结构体中的顺序一一对应赋初值，不允许跳过前面的成员给后面的成员赋初值；但可以只给前面的若干个成员赋初值，对于后面未赋初值的成员，对于数值型和字符型数据，系统自动赋初值零。

 

### 5.1.5 结构体变量的内存分配

 

- 结构体的内存对齐

 

  - **结构体变量的首地址是结构体中有效对齐值的整数倍**

 

    编译器在给结构体开辟空间时，首先找到结构体中**有效对齐值**。有效对齐值取得方法：

 



    1. 取结构体内所有成员数据类型中占字节空间**最大**的**自身对齐值**记为N1.【注：若有结构体类型成员，其对齐值为**该结构体成员内的所有成员中自身对齐值最大的那个值**。】
    
    2. 寻找是否有**自定义有效对齐值**，若有则记为N2.【注1：**自定义有效对齐值**是用宏命令#pragma pack(n) 自定义的，对齐值为n ，用宏命令#pragma pack() 取消自定义对齐。】【注2：自定义有效对齐值中n=1时，称为**紧凑编译**】
    
    3. 若N2不存在，则**有效对齐值**就是N1；若N2存在，则**有效对齐值**便是：min｛N1,N2｝。

 



    然后寻找内存地址能是该基本数据类型的整倍的位置，作为结构体的首地址。

 



  - **结构体每个成员相对于结构体首地址的偏移量（offset）都是当前成员大小的整数倍，如有需要编译器会在成员之间加上填充字节**

 

    为结构体的一个成员开辟空间之前，编译器首先检查预开辟空间的首地址相对于结构体首地址的偏移是否是本成员的整数倍，若是，则存放本成员，反之，则在本成员和上一个成员之间填充一定的字节，以达到整数倍的要求，也就是将预开辟空间的首地址后移几个字节。

 



  - **结构体的总大小为结构体中最宽基本数据成员的整数倍。如有需要，编译器将会在结构体末尾**（[存疑](https://blog.csdn.net/Thanksgining/article/details/42024977)）**添加填充字符**

 

- 柔性数组成员

 

  ```c
  Struct Type

  {

     int n;

     int a[];//或者 “int a[0];”

  }s;

  ```

 

  成员数组a 称为柔性数组，它的长度为0 , 所以sizeof(s) 的大小为4, 以后可以分配变长度空间。柔性数组只能作为最后一个成员。[柔性数组扩展阅读](http://luodw.cc/2015/10/22/Cplus6/)

 

 

 

## 5.2 联合体

 

### 5.2.1 定义联合体的注意事项

 

- 分配内存时，联合体变量所占内存的实际长度等千各成员中占内存最长的成员的

  长度。

- 只有最后一个存放的成员值有效，其他成员将失去原值。

 

### 5.2.2 初始化联合体的注意事项

 

- 联合体变量在定义的同时只能用第1个成员的类型的值进行初始化，对联合体变量初始化时尽管只能给第1个成员赋值，但必须用大括号括起来。

- 联合体变量和结构体变量的区别：

  - 联合体变量在定义的同时只能用第1个成员的类型的值进行初始化

  - 联合体变量中的所有成员共享一段公共存储区，所以联合体变量所占内存的字节数与其成员中占字节数最多的那个成员相等；而结构体变量中的每个成员分别占有独立的内存空间，所以结构体变量所占内存的字节数是其成员所占字节数的总和

  - 由于联合体变量中的所有成员共享内存空间，因此变量中的所有成员的首地址相同，而且变量的地址也就是该变量成员的地址

- 字节序描述数据在内存中的排列格式。在存储和加载时， CPU 必须采用硬件支待的字节序格式。字节序分为两类，即**大端( BigEndian ) **和**小端(LittleEndian ) **

  采用大端格式时，高位字节存储在第一个位置，次高位字节存储在次邻位置。采用小端格式时，低位字节存储在第一个位置，次低位字节存储在次邻位置。

  - [字节序扩展阅读1](https://zhuanlan.zhihu.com/p/44625744)

  - [字节序扩展阅读2](https://songlee24.github.io/2015/05/02/endianess/)

 

## 5.3 枚举类型

 

暂无

 

## 5.4 用户定义类型

 

### 5.4.1 有关typedef的说明

 

- 使用typedef 只能对已有的类型名重新命名，并不能产生新的数据类型，原有的类型也没有被取代，即用户定义类型只是原类型的一个别名

 

- typedef 并不是做简单的字符串替换。**typedef与#define 的区别**:

 

  - **语法格式不同**：typedef定义是语句，句尾要加上分号；而#define不是语句，不能在句尾加分号

 

  - **用法不同**： typedef用来定义一种数据类型的别名，增强程序的可读性；而#define主要用来定义常量，以及书写复杂的使用频繁的宏

 

  - **执行时间不同**：typedef是编译过程的一部分，有类型检查的功能；#define 是宏定义，是预编译的部分，其发生在编译之前，只是简单粗暴地进行字符串的替换，不进行类型的检查

 

  - **作用域不同**：typedef有作用域限定；#define不受作用域约束，只要是在define命令后的引用都正确

 

  - **对指针的操作不同**： typedef 和#define 定义的指针有很大的区别，如此例：

 

    ```c
    
    typedef char * String_t;//是语句，为char＊指定一个新类型别名String_t, 有类型检查，编译的时候处理；
    
    #define String_d  char * //是宏命令，只做简单的替换，无类型检查，预编译的时候处理，所以typedef 比＃define安全
    
    //而且定义多个变量时有区别，如：
    
    String_t a,b; /*等同于*/ char *a,*b;
    
    String_d c,d; /*替换为*/ char *c,d;  //c为char＊类型，而d为char类型。

 





    ```

 



  - [typedef与#define区别的扩展阅读](https://www.runoob.com/note/24230)

 

- 用typedef 定义类型名可嵌套进行

 

- 用typedef 定义类型名有利于程序的移植，并增加程序的可读性.

 



# 第6章 模块化——函数

 

## 6.1 函数基础

 

### 6.1.1函数的定义与调用

 

1. 函数的定义

 

   - 默认的函数类型为int

   - void型函数无返回值，不能包含带返回值的return语句；其他类型的函数至少包含一个return语句

   - 函数的定义不能嵌套，即不能在一个函数体内又包含另一个函数的定义。这就保证了**每一个函数是一个独立的、功能单一的程序单元**

   - **复合语句**（用花括号{}括起来的语句）申明的变量的作用域只在复合语句中,出了复合语句就不起作用。复合语句中的变量名和复合语句外面的变量即使同名也不是同一变量。

 

2. 函数的调用

 

   - **实参(argument)**：

 

        全称为"实际参数"，是在调用时传递给函数的参数。实参可以是常量、变量、表达式、函数等， 无论实参是何种类型的量，在进行函数调用时，它们都**必须具有确定的值**， 以便把这些值传送给形参。 因此应预先用赋值，输入等办法使实参获得确定值

 


   - **形参(parameter)**：

 

       全称为"形式参数" ，由于它不是实际存在变量，所以又称虚拟变量。是在定义函数名和函数体的时候使用的参数，目的是**用来接收调用该函数时传入的参数**。在调用函数时，实参将赋值给形参。因而，必须注意实参的个数，类型应与形参一一对应，并且实参必须要有确定的值。没有形参时，圆括号也不可省；多个参数之间应用逗号分隔。参数包括参数名和参数类型

 


     [来源：牛客网](https://www.nowcoder.com/questionTerminal/d36b0c45b4694e279490ecbfe81221d3)

 


### 6.1.2 函数的返回值与return语句

 

1. return的语句功能：返回调用函数（终止该函数的执行），并将return语句中表达式的值带给调用函数

2. return语句中表达式的类型与函数的类型不一致时则**以函数类型定义为准**，系统自动进行类型转换

3. C语言中可以使用不带表达式的语句直接返回，C++必须使用带表达式的return语句返回

4. return 语句不能返回局部变量的地址，因为该地址中存放的局部变量在函数执行完毕后被释放，但可以返回静态局部变量的地址，因为静态局部变量的空间不是在栈帧中，而是在静态数据区，即使栈帧退栈了，它仍然存在

 

### 6.1.3 函数的声明

 

1. 函数声明语句也称为**函数原型**

2. 如果调用一个函数出现在该函数的定义之前，则在调用前必须对该函数进行声明

3. 如果函数原型放在调用函数定义的内部，则该声明仅对该调用函数有效

4. fun()与fun(void)声明的区别：对于前者，编译器编译时不检查该函数调用的参数传递情况；对于后者，括号中有void，编译器编译时会严格检查该函数调用时的参数传递情况，如果带参数调用，则会编译错误或者警告

 

### 6.1.4 外部函数与内部函数

 

1. 函数默认类型是**外部函数**，其作用域是整个源程序，即：除了可被本源文件中的其他函数调用外，还可被其他源文件中的函数调用（其他源文件调用时，需要对被调用的外部函数用extern语句进行声明）

2. **内部函数**，也称为**静态函数**，使用static关键字定义，其作用域局限于定义它的源文件内部，即：只能被本源文件中的函数调用，不能被统一程序的其他源文件中的函数调用，其有以下优点：

   - 其他源文件中可以定义相同名字的函数，不会发生冲突

   - 静态函数不能被其他源文件所用，达到“隐藏”目的

 

### 6.1.5 函数间的参数传递

 

参数传递有两种方式：**传值**，**传地址**

 

1. **传值方式**

 

   一个函数调用另一个函数时直接将实参的值传递给对应的形参，这称为**传值方式**，对应的形参称为值参数。传值方式实现了把数据由调用函数传递给被调用函数。由于数据在传递方（实参方）和被传递方（形参方）占用不同的内存空间（函数的形参属于自动变量，函数执行完毕后自动释放），所以形参在被调用函数中无论如何变化都不会影响调用函数中相应实参的值，也就是说调用函数时实参和形参之间是单向的从实参到形参的值传递。

 

2. **传地址方式**

 

   如果要通过一个函数fun改变某个实参y （对应形参为x, 数据类型为Type ）的值，需要在fun 形参表定义为`Type *x`, 在调用函数的语句中指定为＆y (取y 的地址）。这样此时形参变量指向的内存地址与实参的地址相同，所以通过**解引用**形参变量指针进行的操作等同于对实参进行操作。

 

### 6.1.5 函数调用的实现原理

 

大多数CPU上的程序使用**栈空间**来支持函数调用操作。单个函数调用操作所使用的函数调用栈被称为**栈帧(stack frame) 结构**。每次函数调用时都会相应地创建一帧，保存返问地址、函数形参和局部变量值等，并将该帧压入调用栈。若在该函数返回之前又发生新的调用，则同样要将与新函数对应的一帧进栈，成为栈顶。函数一旦执行完毕，对应的帧便出栈（此时局部变量的生命周期结束），控制权交还给该函数的上层调用函数，并按照该帧中保存的返回地址确定程序中继续执行的位置。

 

- 函数调用要点

  - 栈空间中每个栈帧的大小是有限的，所以在—个函数中不要定义很大空间的数组，否则可能会导致**栈溢出**，程序崩溃。

  - 每个栈帧对应着一个未运行完的函数。栈帧中保存了该函数的返回地址和局部变量，每个函数的每次调用，都有它自己独立的一个栈帧，这个栈帧维护着函数调用所需要的各种信息。函数的返回地址和参数，保存当前函数调用前的“断点”信息，也就是函数调用前的指令位置，以便在函数返回时能够恢复到函数被调用前的代码区中继续执行指令。函数栈帧的大小并不固定，一般与其对应函数的局部变量多少有关。函数运行过程中，其栈帧大小也是在不停变化的！

    ————————————————

    [版权声明：本文为CSDN博主「YYtengjian」的原创文章，遵循 CC 4.0 BY-SA 版权协议，转载请附上原文出处链接及本声明。](https://blog.csdn.net/yaotengjian/article/details/72629735)

  - 当—个函数多次调用时，每次调用都会创建一个栈帧，为同名的局部变量分配空间，但它们的地址是不同的，它们之间也没有关系。

 

### 6.1.7 函数调用时参数的求值顺序

 

如果所有实参表达式的求值没有二义性，那么从右往左求值和从左往右求值结果是相同的。人们一般认为是从右往左顺序求值的。但，如果出现二义性，输出结果便是不确定的。例如：

 

```c
#include <stdio.h>

int main()

{

    int a=l;

    printf("%d,%d,%d\n",a++,++a,a++);

    return 0;

}

```

 

此程序在VC++中输出“2,2,1”，但在Dev C++中输出“3,4,1”，这是因为printf函数的实参存在二义性，因为在函数的所有参数赋值之后且在函数的第一条语句执行之前有一个顺序点，而参数间的逗号处没有顺序点，任意两个顺序点之间的副作用的求值次序都是不确定的，这里有3 个副作用，所以输出结果不确定。

 

- [见笔记第1章1.2.1中第4点表达式求值的副作用]((wiz://open_document?guid=5e9705cd-02ce-4f54-90f4-737d935ad1d9&kbguid=&private_kbguid=c07167b7-b007-4f38-b466-ea613199e5ae))

- [C语言参考手册：求值顺序](https://zh.cppreference.com/w/c/language/eval_order)

 

### 6.1.8 atexit()函数

 

​    即使main()函数终止以后仍然可以执行一些代码，这需要使用stdlib.h 头文件中的atexit()函数。一般来说，如果在main()中调用某个函数，程序的执行会跳转到该函数并执行它。在执行该函数后控制权又交还给main()函数。

 

​    当使用了atexit() 函数以后，进程的执行可以简单地理解为**当main()函数终止后跳转到atexit()函数，然后再也不会返回到main()函数**。atexit()函数的使用格式如下：

 

```c
atexit(函数名)；

```

 

​    **由于atexit()函数是按后进先出的方式注册这些函数的，因此最后注册的函数先调用**。

 

## 6.2 数组作为函数参数

 

以二维数组为例：

 

```c
int a[M][N]; int (*pa)[n] = a ;

void func(int a[][N]); void func(int a[M][N]);//函数声明

func(a); func(pa);//调用

```

 

## 6.3 指针数组作为函数参数

 

当指针数组作为实参时，对应的形参应当是一个指向指针的指针变量.例如以下三种形式：

 

```c
void func(int *a[]);

void func(int *a[N]);

void func(int **a);

```

 

## 6.4 指针函数和函数指针

 

### 6.4.1 **指针函数**

 

例如：`int *func(int a,float x); `

 

**解释**：

 

- 定义指针型函数时前面的`*`号与“数据类型”相结合，表示此函数是指针型函数。上例，定义func()函数时首部中的`int*` 是一个整体，表示该函数返回的是整型变量的地址

- 在程序中不要使用数组名接收指针型函数的返回值，因为数组名为地址常量，不能向它赋值

 

### 6.4.2 **函数指针**

 

函数的存储首地址又称为函数的执行入口地址， C 规定函数的首地址就是函数名。当指针变量保存函数的入口地址时它就指向了该函数，所以称这种指针变量为指向函数的指针变量，简称为**函数指针**.

 

定义函数指针的一般格式：`函数类型（＊函数指针名）（形参表）;`

 

**解释**：

 

- 在定义函数指针变量时，“函数指针名“两边的圆括号不能省略，它表示函数指针名先与`*`结合，即为指针变量，然后再与后面的”(形参表)”相结合，表示该指针变量指向函数。如果少了前面的一组括号，则变为`函数类型 ＊函数名（形参表）`它表示返回值为地址值（指针）的函数

- 函数指针变量的类型是被指向的函数类型

 

1. 给函数指针赋值格式：`函数指针名＝函数名;`

 

2. 通过函数指针调用函数格式：`(＊函数指针)(实参表);`

 

3. 函数指针的作用主要体现于在函数间传递函数，这种传递不是传递任何数据，而是传递函数的执行地址，或者说是传递函数的调用控制。**当函数在两个函数间传递时，调用函数的实参应该是被传递函数的函数名，而被调用函数的形参应该是接收函数地址的函数指针**。

 

4. 函数指针的用处：

   - 使用函数指针的目的是为了增加执行函数的通用性，特别是在可能调用的函数可变的情况下，可以动态设置内容，有灵活性。如：排序的qsort函数需要传入比较的函数指针，来确定排序是从大到小还是从小到大，如下：

 

```c
void qsort(void *base, size_t nitems, size_t size, int (*compar)(const void *, const void*));//其中参数compar——用来比较两个元素的函数，即函数指针（回调函数）

```

 

### 6.2.3 两个函数指针实例

 

1.实例一：![](https://silencht.oss-cn-beijing.aliyuncs.com/img/函数指针定义实例20200326010002.png)

 

2. 实例二：

 

```c
void ( *func(void (*p)(void *),void *x) ) (void *);

```

 

​        **解释**：

 

![](https://silencht.oss-cn-beijing.aliyuncs.com/img/函数指针实例20200326203843.jpg)

 

## 6.5 递归函数

 

- 递归函数又称自调用函数，其特点是在函数内部调用自己。C 规定不允许函数递归定义，即不允许在一个函数体中定义另一个函数，但可以递归调用。在执行递归函数时将反复调用其自身，每调用一次就进入新的一层。

- 一般地， 一个递归函数定义由两个部分组成，即递归结束情况和递推关系情况。递推关系就是把一个不能或不好直接求解的“大问题“转化成一个或儿个“小问题”来解决，再把这些“小问题”进一步分解成更小的“小问题”来解决（即递推），如此分解，直到每个“小问题”都可以直接解决（此时分解到递归结束情况）。

 

 
# 第7章 位操作——位运算和位域


## 7.1 位运算符

在C 中只能对**整型或字符型**数据进行位运算，不能对其他类型的数据进行位运算。

### 7.1.1 **按位求反`~`**

- 所有的位运算都是以**补码**形式进行的
- 位运算适合于任何类型的整数，包括各种带符号整数，对于带符号整数，符号位也参与按位求反
- `~`运算符的优先级比算术运算符、关系运算符、逻辑运算符和其他位运算符都高

### 7.1.2 **左移运算`<<`**

- 在VC++环境下，当左移位数n大于等于整数的位数m时，实际左移位数为n%m ，例如a<<36 等同于`a<<(36%32) `（这里n=36 、m=32) ，如a=1时其结果为16

### 7.1.3 **右移运算`>>`**

- 在VC++环境下，对于带符号整数，右移时高位正数补0 、负数补1， 例如int a=-2, a>>2的结果为-1。有的编译器全部补0

### 7.1.4 **按位与`&`**

与运算可以实现的功能如下：

- **清零**

  如果想将一个单元清零，也就是使其全部二进位为0, 只需与0进行按位与运算即可达到清零的目的

- **保留/取 一个数中的某些位**

  要想将哪一位（或哪些位）保留下来，就与一个数进行＆运算，此数在这些位取1。例如有一个整数a (16 位），想取a中两个字节中的高字节，只需将a 与八进制数177400 (1111 1111 0000 0000) 按位与即可。

### 7.1.5 按位异或`^`

异或运算可以实现的功能如下：

- **使特定位翻转**

  要使哪几位翻转，就将与其进行按位异或运算的那几位置为1 即可。（注：任何数与0按位异或保留原值。）

- **编码解码**

  任何数与它自己做按位异或结果为0 。例如，设k为密码，a为原始数据， a＾k 为加密结果，再做a^k^k结果即为a, 称为**解码**。

### 7.1.6 按位或`|`

- 任何位上的二进制数，只要和1进行按位或运算，该位即为1；和0进行按位或运算，该位保留原值不变。

### 7.1.7 不同长度的数据进行位运算

- 两个运算数位数不同时系统自动处理如下：
  1. 将两个运算数右端对齐
  2. 将位数较短的运算数往高位扩充，即无符号数和正整数左侧用0补全，负数左侧用1补全；然后对位数相等的这两个运算数按位进行位运算

## 7.2 位图


位图( bit-map ) 就是用一个位来标记某个元素对应的值。由于采用了位为单位来存储数据，因此可以大大节省存储空间。

### 7.2.1 在位图 a 中设置位序号为 i 的位为1

int的长度为4个字节，设SHIFT为5（2的5次方=32）, MASK 为0x1f（ 二进制数为11111) 。对于位序号为 i 的位，在位图a中设置对应位为1的过程如下：

1. **求a所在的元素**：位序号为 i 的位应该在`a[i>>SHIFT] `（即`a[i/32]`) 元素中
2. **求32位基准向量**：位序号为 i 的位在`a[i>>SHIFT]`元素中从右数第`（ i & MASK）`位，这里` i & MASK `表示取出 i 末尾的5个二进制位，相当于 i%32; 将0x00000001（或者1) 左移`（i & MASK）`位，即`1<<( i & MASK)`得到对应的32 位基准向量
3. 将`a[i>>SHIFT]`元素与求得的32位基准向量进行按位或运算，就在位图 a 中将位序号为 i 的位设置为1 ，即`a[i >> SHIFT] |= (1 <<(i & MASK)）`

- 函数代码

```C
void clr(int i)
{
    a[i>>SHIFT]|=(1<<(i&MASK));
}
```

### 7.2.2 初始化位序号为i的位的设置值为0

求出位序号为i的位在`a[i>>SHIFT]`元素中，对应的32位基准向量是`1<<(i&MASK)`，通过`a[i>>SHIFT]& = ~(1<<(i&MASK))`便位序号为i的设置值设为0.

- 函数代码

```C
void set(int i)
{
    a[i>>SHIFT]& = ~(1<<(i&MASK));
}
```

### 7.2.3 获取位图a中位序号为i的位的设置值

求出位序号为i的位在`a[i>>SHIFT]`元素中，对应的32位基准向量是`1<<(i&MASK)`，通过`a[i>>SHIFT]&(1<<(i&MASK))`取出对应的设置值，如果为0（假），表示对应位为0；如果为非0值（真），表示对应位为1.

- 函数代码

```C
int getbit(int i)
{
    return a[i>>SHIFT]&(1<<(i&MASK));
}
```

## 7.3 位段（位域）

### 7.3.1 位段定义的说明

1. 位段名省略时称作**匿名位段**。匿名位段的存储空间通常闲置不用。

   当匿名位段的**宽度被指定为0时**有特殊作用：**它使下一个位段跳过当前字节剩余空间，直接从一个新的地址开始存放**。

2. 常规结构体成员和位段可以定义在一个结构体中

3. 不能定义元素为位段结构的数组

### 7.3.2 位段的引用和赋值

1. 位段的应用形式和结构体成员相同
2. 超出位段位数范围的赋值不会报错，而是**自动截取所赋值的低位**，例如一个位段定义为`struct {……unsigned short a:1;……}var;`，若给位段a赋值3，即`var.a=3;`，3的二进制码是11，取低一位是1，所以`var.a`的值是1
3. 由于每个位段都指定了长度，所以特别要注意位段中的**位扩展**。当一个位段转换为有符号类型时按位段的最高位进行为扩展，例如：

![](https://silencht.oss-cn-beijing.aliyuncs.com/img/位域的位扩展20200401000617.png)

4. 位段成员的类型必须为无符号或者带符号整形（含char）

5. 位段可以参与算数表达式的运算，系统自动将其转换成整型数

6. 位段可以利用整型格式描述符（%d,%u,%o,%x）进行输出

7. 由于位段没有地址，所以**不能对位段求地址**，也不能通过scanf(）语句读入位段值、不能用指针指向位段，但可以对一个含位段的结构体变量中的非位段成员求地址

8. 几个位段的分配空间不一定紧挨在一起，中间可能有空着不用的空间。例如几个位段均为char 类型， 一个长度小于8 的位段不能跨两个字节。


 # 第8章 编译前的处理——预处理


## 8.1 宏定义

在C 源程序被编译之前，首先对源程序中的预处理命令进行处理，然后才对程序进行编译。编译预处理命令都是以“#“开头的，它不是C 语句，必须单独占一行，末尾不使用分号作为结束符。**类似Word中的替换功能**。常用的预处理命令包括宏定义、条件编译和文件包含等。

### 8.1.1 无参宏定义

- **格式**：`#define  标识符 字符串`

- **定义要点**：

1. 宏定义是用宏名来表示一个字符串，在宏展开时又以该字符串取代宏名，属于一种简单的代换。该字符串可含任意字符，预处理时对它不作任何检查。如有错误，只能在编译已被宏替换的源程序的过程中发现问题。
2. 当宏定义在一行中写不下， 需要在下一行继续时，只需在最后一个字符后紧接着加一个反斜线”\＂ 。
   - [扩展阅读：反斜线可能引发的bug](https://mp.weixin.qq.com/s/fxmheEE2fYbHAYcvFUwIWw)

3. 宏定义不是语句，在行末不必加分号，如加上分号则连分号一起被置换
4. 宏名在源程序中若用引号括起来， 则预处理程序不对其作宏替换,例如printf函数的参数中，双引号内的宏名不会被替换
5. 宏定义允许嵌套， 即在宏定义的字符串中可使用已经定义的宏名，并且在宏替换时由预处理程序层层代换

### 8.1.2 带参宏定义

- **格式**：`#define 标识符(形参表) 字符串`

- **调用**：`宏名(实参表)`

- **定义要点**：

1. 在带参宏的定义中宏名和形参表之间不能有空格出现，否则宏名后面的括号、形参表和字符串会被错误认为是无参宏

2. 在宏定义中形参是标识符，而宏调用中的实参可以是表达式

3. 在宏定义中字符串内的形参通常要用括号括起来，否则可能出错，这称为宏的副作用。例如，定义求正方形面积的宏如下：

   ```c
   #define  area(a) (a*a)
   ```

   当调用`area(2+3) `时宏替换成(2+3*2+3) ，那么求出的面积是11, 而不是正确的25 。所以应该改为如下：

   ```c
   #define  area(a) ((a)*(a))
   ```

4. 宏定义可用来定义多个语句，在宏调用时把这些语句又代换到源程序内，例如：

   ```c
   #define SET(a,b,c,d)  a=1;b=2;c=3;d=4;
   ```

 

- **带参宏与函数的区别**：

1. 函数调用时**先求出实参表达式的值**，然后代入函数定义中的形参；而使用带参宏只是进行**简单的字符串替换**，不进行实参的计算
2. 函数调用是**在程序执行时处理**的，分配临时的内存单元；而宏替换是**在编译之前进行**的，在宏替换时并不分配内存单元，也不进行值的传递处理，也没有“返回值＂的概念
3. 对函数中的实参和形参都要定义类型，且两者的**类型要求一致**，如不一致应进行类型转换；而宏**不存在类型**问题，宏名无类型，它的参数也无类型，只是一个符号代表，宏替换时代入指定的字符即可
4. 当使用宏次数较多时，宏替换后**使源程序变长**，而函数调用**不使源程序变长**，因此一般用宏来替换小的、可重复的代码段，对于代码行较多的应使用函数方式
5. 宏替换**不占执行时间**，只占编译预处理时间，而函数调用**占执行时间**（分配内存、保留现场、值传递、返回等）。

 

## 8.2 条件编译

一般情况下，C 源程序中所有的行都参加编译过程。但有时出于对程序代码优化的考虑，希望对其中一部分内容只是在满足一定条件时才进行编译，形成目标代码。这种对程序的一部分内容指定编译的条件称为条件编译。

- **条件编译注意要点**：

头文件中的ifndef/define/endif 有什么作用？例如：

```c
#ifndef _STDIO_H
#define _STDIO_H
```

“宏名“在理论上来说可以是自由命名的，但每个头文件的宏名都应该是唯一的，其命名规则一般是**头文件名全大写，前、后加下划线， 并把文件名中的“.”也变成下划线**，例如stdio. h 头文件的宏名是＿STDIO_H。上述条件编译命令表示若没有定义＿STDIO_H，就定义＿STDIO_H 。

其目的是**为了防止同一头文件等被重复引用**。[牛客网题目参考连接](https://www.nowcoder.com/questionTerminal/16f568295550480e8cf902864bfcbb86)

 

## 8.3 文件包含

所谓文件包含预处理，是指在一个文件中将另一个文件的全部内容包含进来的处理过程，即将另外的文件包含到本文件中。C 提供了＃include 编译预处理命令实现文件包含操作。

### 8.3.1 文件包含操作的两种格式

1. `#include ＜包含文件名＞`
2. `#include “包含文件名”`

**两者区别**：

第一种：＜包含文件名＞表示直接到指定的C/C++编译系统标准包含文件目录去寻找文件

第二种：   “包含文件名”  表示先在当前目录寻找，如找不到再到标准包含文件目录寻找

**因此**：

一般来说，前者用来包含开发环境提供的库头文件，后者用来包含自己编写的头文件。

 

### 8.3.2 文件包含操作的过程

文件包含预处理的功能是在对源程序进行编译之前用包括文件的内容取代该文件包含预处理语句。例如， a.c 文件中有文件包含命令#include "b.c", 其预处理过程如下图：

![](https://silencht.oss-cn-beijing.aliyuncs.com/img/文件包含过程20200401134102.png)

注：被包含的文件并不限于C系统所提供的头文件（如stdio.h 、string.h 等），还可以是用户自己编写的命名文件（其中包括宏、结构体名、联合体名、全局变量的定义等）和其他的要求在本文件中引用的源程序文件。

### 8.3.3 文件包含的说明

1. 一个#include 命令只能指定一个包含文件。如果要包含多个文件，则要使用多个#include 命令

2. 如果文件file1.c 要使用文件file2.c 中的内容，而文件file2.c 要用到文件file3.c 中的内容，则可以在文件file1.c 中用两个#include 命令分别包含file2.c 和file3.c, 而且文件file3.c 应出现在文件file2.c 之前，即file1.c中有：

   ```c
   #include "file3.c"
   #include "file2.c"
   ```

   这样file1.c 和file2.c 均可以用file3.c 中的内容，而在file2.c 中不必再用＃include "file3.c” 了

3. 在一个被包含文件中又可以包含另一个被包含文件，即文件包含是可以嵌套的

4. 被包含文件（如file2.c) 与其所在的文件(file1.c) 在预编译后已成为同一个文件（而不是两个文件），因此，如果file2.c 中有全局静态变量，它也在filel.c 文件中有效，不必再用extern 声明。

5. 全局变量是否可以定义在可被多个 .c 文件包含的头文件中？为什么？

   如果包含该头文件（其中定义有全局变量n) 的多个.c 文件属于同一个工程，并且希望通过该全局变量n 在这些.c 文件的函数中共享数据，这是不可以的。例如，一个工程包含三个文件：

   headfile.h 头文件：

   ```c
   int n; ／／全局变量
   ```

   file1.c 文件：

   ```c
   #include  <stdio.h>
   #include "headfile.h"
   ……
   ```

   file2.c 文件：

   ```c
   #include  <stdio.h>
   #include "headfile.h"
   ……
   ```

   在包含文件预处理之后，相当于一个程序中两次定义全局变量n，在编译时会出现变量重复定义的编译错误。

   如果将headfile.h 头文件中全局变量n的定义改为`static int n; `，程序正确执行，即可以在不同c文件中定义同名的全局变量，但变量n为静态全局变量，不能在多个.c文件的函数中共享数据。即，虽然此时每个文件中都有静态全局变量n，但每个文件中的变量n的地址都是不相同的，只是变量名看似相同，并非真正的全局变量。

   [头文件中的的static变量意味着什么？](https://www.cnblogs.com/zplutor/archive/2011/08/06/2129401.html)

   [菜鸟教程：C/C++ 中 static 的用法全局变量与局部变量](https://www.runoob.com/w3cnote/cpp-static-usage.html)



 















