---
title: cpp-thread_local
toc: true
---

## 线程局部存储

线程局部存储（TLS, thread local storage）是一个已有的概念。线程局部存储是指对象内存在线程开始后分配，线程结束时回收且每个线程有该对象自己的实例，线程局部存储的对象都是独立于各个线程的。简单地说，**所谓线程局部存储变量，就是拥有线程生命期及线程可见性的变量。**虽然C++一直没有在语言层面支持它，但是很早之前操作系统就有办法支持线程局部存储了。



## 缘由

线程局部存储实际上是由单线程程序中的全局 / 静态变量被应用到多线程程序中被线程共享而来。我们可以简单地回顾一下所谓的线程模型。通常情况下，线程会拥有自己的栈空间，但是堆空间、静态数据区（如果从可执行文件的角度来看，静态数据区对应的是可执行文件的data、bss段的数据，而从C/C++语言层面而言，则对应的是全局/静态变量）则是共享的。这样一来，全局、静态变量在这种多线程模型下就总是在线程间共享的。全局、静态变量的共享虽然会带来一些好处，尤其对一些资源性的变量（比如文件句柄）来说也是应该的，不过并不是所有的全局、静态变量都适合在多线程的情况下共享。

代码示例：

```c++
#include <pthread.h>
#include <iostream>
using namespace std;
int errorCode = 0;
void* MaySetErr(void * input)
{
    if(*(int*)input == 1)
        errorCode = 1;
    else if(*(int*)input == 2)
        errorCode = -1;
    else 
        errorCode = 0;
}
int main()
{
    int input_a = 1;
    int input_b = 2;
    pthread_t thread1,thread2;
    pthread_create(&thread1, NULL,&MaySetErr,&intput_a);
    pthread_create(&thread2, NULL,&MaySetErr,&intput_b);
    pthread_join(thread2,NULL);
    pthread_join(thread2,NULL);
}
//编译选项 g++ name.cpp -lpthread
```

在上述代码中，函数MaySetErr函数可能会根据输入值input设置全局的错误码errorCode。设想一下，一个多线程程序的线程A在某个时刻刚刚调用过一个函数，正准备获取其错误码，也正是这个时刻，另外一个线程B在执行了某个函数后修改了这个错误码，那么线程A接下来获取的错误码自然不会是它真正想要的那个。这种线程间的竞争关系破坏了errno的准确性，导致一些程序中运行的错误将会被隐藏不报。

实际上，本例中的errorCode即是POSIX标准中的错误码全局变量errno在多线程情况下遭遇的问题的一个简化。为了规避由此产生的不确定性，POSIX将errno重新定义为线程独立的变量，为了实现这个定义就需要用到线程局部存储，直到C++11之前，errno都是一个静态变量，而从C++11开始errno被修改为一个线程局部存储变量TLS。

## 定义

各个编译器公司都有自己的TLS标准。我们在g++/clang++/xlc++中可以看到如下的语法：

```c++
__thread int errCode
```

即在全局或者静态变量的声明中加上关键字`__thread`，即可将变量声明为TLS变量。每个线程将拥有独立的errCode的拷贝，一个线程中对errCode的读写并不会影响另外一个线程中的errCode的数据。

C++11对TLS标准做出了一些统一的规定。与`__thread`修饰符类似，声明一个TLS变量的语法很简单，即通过`thread_local`修饰符声明变量即可。

```c++
#一个例子
int thread_local errCode;
#另一个例子
struct X{
    thread_local static int i;
}
thread_local X a;
int main()
{
    thread_local X b;
}
```

**一旦声明一个变量为`thread_local`，在同一个线程中，一个线程局部存储对象只会初始化一次，即使在某个函数中被多次调用。这一点和单线程程序中的静态对象非常相似。相对应的，对象的销毁也只会发生一次，通常发生在线程退出的时刻。**。

`thread_local`说明符可以用来声明线程生命周期的对象，它能与`static`或`extern`结合，分别指定内部或外部链接，不过额外的`static`并不影响对象的生命周期。换句话说，**`static`并不影响其线程局部存储的属性。**



虽然TLS变量的声明很简单，使用也很直观，不过实际上TLS的实现需要涉及编译器、链接器、加载器甚至是操作系统的相互配合。**在TLS中一个常被讨论的问题就是TLS变量的静态/动态分配的问题**，即TLS变量的内存究竟是在程序一开始就被分配还是在线程开始运行时被分配。通常情况下，前者比后者更易于实现。C++11标准允许平台/编译器自行选择采用静态分配或动态分配，或者两者都支持。

---

还有一点值得注意的是，**C++11对TLS只是做了语法上的统一，而对其实现并没有做任何性能上的规定**。这可能导致thread_local声明的变量在不同平台或者不同的TLS实现上出现不同的性能（通常TLS变量的读写性能不会高于普通的全局/静态变量）。如果读者想得到最佳的平台上的TLS变量的运行性能的话，最好还是阅读代码运行平台的相关文档。

---

在了解了线程局部存储的意义之后，让我们回头仔细阅读其定义，会发现**线程局部存储只是定义了对象的生命周期，而没有定义可访问性**。也就是说，我们可以获取线程局部存储变量的地址并将其传递给其他线程，并且其他线程可以在其生命周期内自由使用变量。不过这样做除了用于诊断功能以外没有实际意义，而且其危险性过大，一旦没有掌握好目标线程的声明周期，就很可能导致内存访问异常，造成未定义的程序行为，通常情况下是程序崩溃。

值得注意的是，**使用取地址运算符&取到的线程局部存储变量的地址是运行时被计算出来的，它不是一个常量**，也就是说无法和constexpr结合：

```c++
thread_local int tv;
static int sv;
int main()
{
    constexpr int *sp = &sv;//编译成功，sv的地址在编译时确定
    constexpr int *tp = &tv;//编译失败，tv的地址在运行时确定
}
```

在上面的代码中，由于sv是一个静态变量，因此在编译时可以获取其内存常量地址，并赋值到常量表达式sp。但是tv则不同，它在线程创建时才可能确定内存地址，所以这里会产生编译错误。

---



多线程已经成为现代程序应用中不可缺少的技术环节，但是在C++11标准出现之前，C++语言标准对多线程的支持是不完善的，无法创建线程局部存储对象就是其中的一个缺陷。幸好C++11的推出挽救了这种尴尬的局面。



> - [摘改自《深入理解C++11：C++11新特性解析与应用》](https://weread.qq.com/web/bookDetail/596325a059346c59642f910)
> - [摘改自《现代C++语言特性解析》](https://weread.qq.com/web/bookDetail/22d32dd0726fa07122d86db)


