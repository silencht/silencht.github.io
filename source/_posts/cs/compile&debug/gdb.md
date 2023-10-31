---
title: gdb
toc: true
---
### 1. 启动gdb调试

#### 1. 编译程序

程序编译时需要附加调试信息-g，并关闭优化-O0

```bash
$ gcc -g -o helloworld helloworld.c
$ gdb helloworld
GNU gdb (Ubuntu 9.2-0ubuntu1~20.04.1) 9.2
Copyright (C) 2020 Free Software Foundation, Inc.
……
Reading symbols from xxx/helloworld...done.
#如果不带-g，即gcc -o helloworld helloworld.c，则输出
Reading symbols from xxx/helloworld...(no debugging symbols found)...done.
#编译命令带-g生成程序后，可以使用Linux的strip命令移除该程序的调试信息
#使用strip命令之前
-rwxr-xr-x. 1 root root 12416 Sep 8 09:45 helloworld
#使用strip命令之后，文件大小变小（12416->6312），此时再用gdb启动该程序，则会返回no debugging symbols found
$ strip helloworld
-rwxr-xr-x. 1 root root 6312 Sep 8 09:55 helloworld
```

#### 2. 启动调试

三种方式

```bash
#1. 直接启动程序进行调试
$ gdb helloworld

#2. 如果程序已经启动在运行，使用attach命令将gdb附加到该程序进程ID上（gdb attach pid）
#进程ID可以使用ps命令获取
$ ps -ef | grep helloworld
changhe   217055  216643 99 10:43 pts/2  00:00:55 ./helloworld
$ gdb attach 217055

#3. 程序Crash后调试core文件
#Linux默认不开启程序Crash产生core文件机制，可以使用ulimit -c查看是否开启，输出0则是不开启
$ ulimit -c
#开启产生core机制可以使用ulimit -c unlimited命令，unlimited意为不限制core文件大小，也可改成具体数值，比如1024（即1024k字节大小）
#ulimit -c unlimited命令只在当前终端有效，关闭终端后失效。设置永久生效的方式有两种：
#一是在/etc/security/limits.conf 中增加一行
#<domain>    <type>  <item>  <value>
*             soft    core   unlimited
#二是在/etc/proﬁle文件（所有用户）或~/.bashrc（当前用户）末尾加入该命令
ulimit -c unlimited （或 ulimit -c 1024）
#程序崩溃生成core.pid文件（需要编译时添加-g）后，使用下面命令进行调试
$ gdb filename corename
```

程序崩溃后很难知道当时它运行的PID，所以也就不知道对应哪个core文件。以下是记录程序对应core文件的两个方法

```c++
//1. 程序启动时记录自己的PID，这样就可以根据core.pid去查找对应的core文件了
void writePid()
{
uint32_t curPid = (uint32_t) getpid();
FILE* f = fopen("helloworld.pid", "w");
assert(f);
char szPid[32];
snprintf(szPid, sizeof(szPid), "%d", curPid);
fwrite(szPid, strlen(szPid), 1, f);
fclose(f);
}
//2. 自定义生成core文件的目录名称
mkdir ~/core_dump_directory
//%e是程序名，%p是PID，%t是文件生成时间
echo "~/core_dump_directory/core-%e-%p-%t" > /proc/sys/kernel/core_pattern
//最终会在 ~/core_dump_directory 目录下（需具备写入权限）生成的 test 的 core 文件名格式如下
-rw-------. 1 user user 409600 Jan 14 13:54 core-helloworld-13154-1547445291
```

### 2. 命令详解

#### run

启动调试后，只是在程序上附加了gdb，并没有启动该程序，需要输入run命令启动这个程序。

程序启动后可使用Ctrl+C终端程序，再次输入run，可以重新启动该程序。

#### continue

如果不想重新启动程序，而是想让中断的程序继续运行，可以输入continue命令

#### break、tbreak

```bash
#在名为function的函数入口出添加一个断点
(gdb) break function
#在gdb调试器所在的当前文件第num行处添加一个断点
(gdb) break num
#在filename文件第num行处添加一个断点
(gdb) break filename:num
```

break命令是添加一个永久的断点，tbreak命令是添加一个临时断点，触发一次后自动删除。

#### backtrace、frame

backtrace命令可以简写为bt，用来查看当前所在线程的调用堆栈。堆栈编号以#number表示。

如果在主线程，那么最顶层堆栈是main()函数。

切换到某堆栈，可以使用frame命令（可简写为f）

```bash
$ frame number #堆栈编号不用加“#”
```

#### info break、enable、disable、delete

```bash
#查看所有断点
(gdb) info break # 或 info b
Num     Type           Disp Enb Address            What
1       breakpoint     keep y   0x000000000004d93a in main at server.c:6147
2       breakpoint     keep y   0x0000000000041593 in dictSdsKeyCaseCompare at server.c:1269
3       breakpoint     keep y   0x0000000000043589 in beforeSleep at server.c:2364
#可见示例中一共存在3个断点，分别在6147、1269、2364行
#如果想禁用某个断点，使用 disable 断点编号 就可以禁用这个断点；同理，被禁用的断点也可以使用 enable 断点编号 重新开启
(gdb) disable 2
(gdb) info b
Num     Type           Disp Enb Address            What
1       breakpoint     keep n   0x000000000004d93a in main at server.c:6147
2       breakpoint     keep y   0x0000000000041593 in dictSdsKeyCaseCompare at server.c:1269
3       breakpoint     keep y   0x0000000000043589 in beforeSleep at server.c:2364
#可见，禁用断点2后，Enb下方的标志由y变为n. 同理，使用enable 2命令重启后，该标志又会回到y
#enable、disable命令后不加断点编号，即表示开启、禁用所有断点
#delete 断点编号，即表示删除某断点。同理，不加断点编号，即表示删除所有断点
```

#### list

该命令可以查看当前断点前后的代码（默认范围为10行），可以简写为l. 再次输入命令继续向后显示10行代码。

list +和list -分别表示向后和向前显示代码。

list 也可以显示其他文件某一行前后的代码。

```bash
list FILE:LINENUM  #to list around that line in that file,
```

更多用法输入help list

#### print、ptype

print 命令可以在调试过程中查看变量的值，也可以修改当前内存中的变量值，可以简写成 p

print 命令不仅可以输出变量值，也可以输出特定表达式计算结果值，甚至可以输出一些函数的执行结
果值。

```bash
#打印server.port变量的值 (gdb) print variable
(gdb) p server.port
$1 = 6379
#打印server.port变量的类型
(gdb) ptype server.port
type = int
#列出当前对象的各个成员变量值
(gdb) p *this
#打印三个变量的和
(gdb) p a+b+c
#打印函数执行结果
(gdb) p function()
#某个时刻，某个系统函数执行失败了，通过系统变量 errno 得到一个错误码，可以使用
(gdb) p strerror(errno) 
#将这个错误码对应的文字信息打印出来，这样就不用去 man 手册上查找这个错误码对应的错误含义
#修改变量的值
(gdb) p server.port=6400
$4 = 6400
#打印变量时可以指定输出格式 (gdb) print /format variable
(gdb) p /x server.port
$6 = 0x1900
#format常见取值
o octal 八进制显示
x hex 十六进制显示
d decimal 十进制显示
u unsigned decimal 无符号十进制显示
t binary 二进制显示
f float 浮点值显示
a address 内存地址格式显示(与十六进制相似)
i instruction 指令格式显示
s string 字符串形式显示
z hex, zero padded on the left 十六进制左侧补0显示
```

#### info、thread

- 可以使用 info threads 来查看进程当前所有线程信息

```bash
(gdb) info threads 
  Id   Target Id                                           Frame 
* 1    Thread 0x7ffff79d57c0 (LWP 230901) "redis-server"   0x00007ffff7c9046e in epoll_wait (
    epfd=5, events=0x5555557a1db0, maxevents=10128, timeout=100)
    at ../sysdeps/unix/sysv/linux/epoll_wait.c:30
  2    Thread 0x7ffff6ba6700 (LWP 230905) "bio_close_file" futex_wait_cancelable (
    private=<optimized out>, expected=0, futex_word=0x5555557073e8 <bio_newjob_cond+40>)
    at ../sysdeps/nptl/futex-internal.h:183
  3    Thread 0x7ffff63a5700 (LWP 230906) "bio_aof_fsync"  futex_wait_cancelable (
    private=<optimized out>, expected=0, futex_word=0x555555707418 <bio_newjob_cond+88>)
    at ../sysdeps/nptl/futex-internal.h:183
  4    Thread 0x7ffff5ba4700 (LWP 230907) "bio_lazy_free"  futex_wait_cancelable (
    private=<optimized out>, expected=0, futex_word=0x555555707448 <bio_newjob_cond+136>)
    at ../sysdeps/nptl/futex-internal.h:183
#一共有4个线程，当前gdb附加在1号线程（带星号）。
#【所有的线程ID在第三栏(LWP number)中。在早期的 Linux 系统的内核里面，不存在真正的线程实现
# 当时所有的线程都是用进程来实现，称之为LWP，即Light Weight Process（轻量级进程）】

#切换到其他线程，使用thread 线程编号
(gdb) thread 3
[Switching to thread 3 (Thread 0x7ffff63a5700 (LWP 231546))]
#0  futex_wait_cancelable (private=<optimized out>, expected=0, 
    futex_word=0x555555707418 <bio_newjob_cond+88>) at ../sysdeps/nptl/futex-internal.h:183
183	../sysdeps/nptl/futex-internal.h: No such file or directory.
(gdb) info threads 
  Id   Target Id                                           Frame 
  1    Thread 0x7ffff79d57c0 (LWP 231538) "redis-server"   0x00007ffff7c9046e in epoll_wait (epfd=5, 
    events=0x5555557a1db0, maxevents=10128, timeout=100) at ../sysdeps/unix/sysv/linux/epoll_wait.c:30
  2    Thread 0x7ffff6ba6700 (LWP 231545) "bio_close_file" futex_wait_cancelable (private=<optimized out>, 
    expected=0, futex_word=0x5555557073e8 <bio_newjob_cond+40>) at ../sysdeps/nptl/futex-internal.h:183
* 3    Thread 0x7ffff63a5700 (LWP 231546) "bio_aof_fsync"  futex_wait_cancelable (private=<optimized out>, 
    expected=0, futex_word=0x555555707418 <bio_newjob_cond+88>) at ../sysdeps/nptl/futex-internal.h:183
  4    Thread 0x7ffff5ba4700 (LWP 231547) "bio_lazy_free"  futex_wait_cancelable (private=<optimized out>, 
    expected=0, futex_word=0x555555707448 <bio_newjob_cond+136>) at ../sysdeps/nptl/futex-internal.h:183
```

- 如何找到main函数所在的主线程，并切换过去？

```bash
#_start 是C程序的入口函数，可以设置断点在入口函数内，然后运行程序，查看当前线程：
(gdb) break _start
Breakpoint 1 at 0x55555558bb80
(gdb) run
The program being debugged has been started already.
Start it from the beginning? (y or n) y
Starting program: /home/changhe/gdb/redis-6.2.4/src/redis-server 
[Thread debugging using libthread_db enabled]
Using host libthread_db library "/lib/x86_64-linux-gnu/libthread_db.so.1".

Breakpoint 1, 0x000055555558bb80 in _start ()
(gdb) info threads 
  Id   Target Id                                         Frame 
* 1    Thread 0x7ffff79d57c0 (LWP 231538) "redis-server" 0x000055555558bb80 in _start ()
```

- info 命令还可以用来查看当前堆栈处函数的参数值：info args

```bash
(gdb) bt
#0 0x00007ffff71e2603 in epoll_wait () from /usr/lib64/libc.so.6
#1 0x0000000000428a9e in aeApiPoll (eventLoop=0x5e5770, tvp=0x7fffffffe140) at ae_epoll.c:112
#2 0x00000000004297e2 in aeProcessEvents (eventLoop=0x5e5770, flags=27) at ae.c:447
#3 0x0000000000429ab6 in aeMain (eventLoop=0x5e5770) at ae.c:539
#4 0x00000000004372bb in main (argc=1, argv=0x7fffffffe308) at server.c:5175
(gdb) f 2 #切换到堆栈 #2,堆栈 #2 调用处的函数是 aeProcessEvents(),一共有两个参数
#2 0x00000000004297e2 in aeProcessEvents (eventLoop=0x5e5770, flags=27) at ae.c:447
447 numevents = aeApiPoll(eventLoop, tvp);
(gdb) info args #使用该命令输出当前函数的两个参数值，指针类型的参数，gdb 默认会输出该变量的指针地址值
eventLoop = 0x5e5770
flags = 27
(gdb) p *eventLoop #要想输出指针指向的对象的值，可以使用print命令，
#如果还要查看其成员值，继续使用 变量名->字段名 即可（如 p eventLoop->maxfd ）
```

#### next、step、until、finish、return、jump

|  命令  | 缩写 |                             作用                             |
| :----: | :--: | :----------------------------------------------------------: |
|  next  |  n   |            step over，单步步过，不进入函数体内部             |
|  step  |  s   |             step into，单步步入，进入函数体内部              |
| until  |  u   |        until 程序行号，指定程序运行到某一行代码处停下        |
| finish |      |              立即执行当前函数剩余代码并正常返回              |
| return |      | 立即结束执行当前函数并返回（如果当前函数还有剩余的代码未执行，也不会执行） |
|  jump  |  j   | jump location，location 可以是程序行号或者函数地址，jump 会让程序执行流跳转到指定位置执行 |

- until 注意

  与 `break` 命令不同，`until` 命令中的行号是 GDB 中显示的行号，而不是源代码文件中的行号。这里的行号以 gdb 调试器中的行号为准（可以通过list查看），不是源码文件中的行号，由于存在条件编译，部分代码可能不会被编译进可执行文件中，所以实际的调试符号文件中的行号与源码文件中的行号可能会不完全一致。

- jump 注意：

```c++
int somefunc()
{
 //代码A
 //代码B
 //代码C
 //代码D
 //代码E
 //代码F
}
//假设断点初始位置在行号 3 处（代码 A），这个时候使用 jump 6，那么程序会跳过代码 B
//和 C 的执行，执行完代码 D（ 跳转点），程序并不会停在代码 6 处，而是继续执行后续代码，因此如
//果想查看执行跳转处的代码后的结果，需要在行号 6、7 或 8 处设置断点。
```

```c++
//jump命令有一个妙用就是可以执行一些想要执行的代码，而这些代码在正常的逻辑下可能并不会执行
int main()
{
 int a = 0;
 if (a != 0)
 {
    printf("if condition\n");
 }
 else
 {
    printf("else condition\n");
 }
return 0;
}
//在行号 4 、14 处设置一个断点，当触发行号 4 处的断点后，正常情况下程序执行流会走 else 分
//支，可以使用 jump 7 强行让程序执行 if 分支，接着 gdb 会因触发行号 14 处的断点而停下来，此
//时接着执行 jump 11，程序会将 else 分支中的代码重新执行一遍
```

#### disassemble

在一些高级调试时，我们可能要查看某段代码的汇编指令去排查问题，或者是在调试一些没有调试信息的发布版程序时，也只能通过反汇编代码去定位问题。

disassemble 会输出当前所在函数的汇编指令，假设我们现在在 redis 的 initServer() 中执行该命令会输出 initServer() 函数的汇编指令.

```bash
(gdb) b initServer
Breakpoint 2 at 0x555555599666: file server.c, line 3137.
(gdb) r
The program being debugged has been started already.
Start it from the beginning? (y or n) y
Starting program: /home/changhe/gdb/redis-6.2.4/src/redis-server 
[Thread debugging using libthread_db enabled]
Using host libthread_db library "/lib/x86_64-linux-gnu/libthread_db.so.1".
235674:C 12 Oct 2023 16:00:36.542 # oO0OoO0OoO0Oo Redis is starting oO0OoO0OoO0Oo
235674:C 12 Oct 2023 16:00:36.542 # Redis version=6.2.4, bits=64, commit=00000000, modified=0, pid=235674, just started
235674:C 12 Oct 2023 16:00:36.542 # Warning: no config file specified, using the default config. In order to specify a config file use /home/changhe/gdb/redis-6.2.4/src/redis-server /path/to/redis.conf

Breakpoint 2, initServer () at server.c:3137
3137	void initServer(void) {
(gdb) disassemble 
Dump of assembler code for function initServer:
=> 0x0000555555599666 <+0>:	endbr64 
   0x000055555559966a <+4>:	push   %rbp
   0x000055555559966b <+5>:	mov    %rsp,%rbp
   0x000055555559966e <+8>:	push   %rbx
   0x000055555559966f <+9>:	sub    $0x18,%rsp
   ……
```

gdb 默认反汇编为 AT&T 格式的指令，可以通过 show disassembly-ﬂavor 查看。如果习惯 intel 汇编
格式的，用命令 set disassembly-flavor intel 来设置。

```bash
(gdb) show disassembly-ﬂavor
The disassembly flavor is "att".
(gdb) set disassembly-flavor intel
(gdb) disassemble 
Dump of assembler code for function initServer:
=> 0x0000555555599666 <+0>:	endbr64 
   0x000055555559966a <+4>:	push   rbp
   0x000055555559966b <+5>:	mov    rbp,rsp
   0x000055555559966e <+8>:	push   rbx
   0x000055555559966f <+9>:	sub    rsp,0x18
   ……
```

这个命令在我们只有程序崩溃后产生 core 文件，且无对应的调试符号时非常有用，我们可以通过分析
汇编代码定位一些错误。