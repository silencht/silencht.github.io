---
title: 性能分析工具之perf
toc: true
typora-root-url: ../../..
---

### 1. 简介

**perf** 是 Linux 内核提供的性能分析工具，可以分析程序运行的性能瓶颈和优化点。通常随着 Linux 内核的安装而自动安装。它通常包含在 `linux-tools` 或 `perf-tools` 包中，具体取决于你所使用的 Linux 发行版。

使用`perf --version`便可以查看perf的软件版本。

```shell
$  perf --version
perf version 5.15.111
```

如果提示找不到该命令，可通过以下命令进行安装（Ubuntu发行版）

```shell
sudo apt-get install linux-tools-common
```



### 2. 使用

#### 2.1 分析正在运行的程序

如果要分析的程序可执行文件名称为 grid_map，使用如下命令

```shell
#查看目录下的文件，可见存在可执行文件grid_map，运行之
ls 
grid_map grid_map.cpp CMakeLists.txt
./grid_map
#假设该程序正在运行中，通过ps命令查看它的进程号，可得PID进程号为880154
ps -ef | grep grid_map
changhe   880154  880083 99 18:01 pts/5    00:00:24 ./grid_map
#使用perf命令分析该程序
sudo perf record -F 99 -p 880154 -g  -- sleep 30
[ perf record: Woken up 1 times to write data ]
[ perf record: Captured and wrote 0.522 MB perf.data (4534 samples) ]
ls
grid_map grid_map.cpp CMakeLists.txt perf.data
```

命令说明

- `sudo`: 以超级用户权限运行命令，因为 `perf` 需要访问内核信息
- `perf record`: 用于收集性能数据
- `-F 99`: 设置采样的频率为 99 Hz。这意味着 `perf` 将每秒对正在运行的程序进行99次采样
- `-p 880154`: 指定要监视的进程的进程 ID (PID)。在这个例子中，`880154` 是希望分析的grid_map进程PID。
- `-g`: 启用堆栈跟踪，记录每个采样时的调用堆栈信息，以便更详细地了解程序的执行路径
- `-- sleep 30`: 持续采集30秒

采集完成后，输出一个`perf.data`文件

#### 2.2 分析可执行文件

使用如下命令

```shell
sudo perf record -F 99 -g ./grid_map --sleep 30
[ perf record: Woken up 2 times to write data ]
[ perf record: Captured and wrote 0.606 MB perf.data (5112 samples) ]
```

命令说明

- 其实同2.1，就是将-p PID替换为了可执行文件路径

### 3. 使用火焰图可视化结果

FlameGraph是由[BrendanGregg](http://www.brendangregg.com/index.html)开发的一款开源可视化性能分析工具，形象的称为火焰图。其基于 perf 结果产生的SVG 图片，用来展示 CPU 的调用栈。

要使用火焰图可视化perf分析结果，首先下载FlameGraph

```shell
#注意：FlameGraph仓库下载的目录影响下面代码./FlameGraph/stackcollapse-perf.pl的执行路径
git clone https://github.com/brendangregg/FlameGraph
```

分析结果可视化

```shell
#现在要对分析结果文件perf.data进行火焰图可视化
ls
grid_map grid_map.cpp CMakeLists.txt perf.data
sudo perf script -i perf.data > perf.unfold
sudo ./FlameGraph/stackcollapse-perf.pl perf.unfold > perf.folded
sudo ./FlameGraph/flamegraph.pl perf.folded > perf.svg
#现在目录下便有一个名为perf.svg的文件了
```

命令说明

- `perf script`: 从 `perf.data` 文件中生成可读的文本格式的性能数据。该命令会将数据写入标准输出
- `-i perf.data`: 指定 `perf` 工具读取 `perf.data` 文件
- `> perf.unfold`: 将标准输出重定向到名为 `perf.unfold` 的文件中。这个文件包含了可读的性能数据

- `./FlameGraph/stackcollapse-perf.pl`: 该脚本将 `perf` 的输出进行处理
- `> perf.folded`: 将处理后的输出重定向到名为 `perf.folded` 的文件中。这个文件包含了折叠后的性能数据
- `./FlameGraph/flamegraph.pl`: 该脚本将`perf.folded`转换为 Flame Graph 的 SVG 图形。



使用浏览器将perf.svg文件打开便可以查看结果图，示例图如下：

![](/image/性能分析工具perf/perf.png)

- y 轴表示调用栈，每一层都是一个函数。调用栈越深，火焰就越高，顶部就是正在执行的函数，下方都是它的父函数。

- x 轴表示抽样数，如果一个函数在 x 轴占据的宽度越宽，就表示它被抽到的次数多，即执行的时间长。注意，x 轴不代表时间，而是所有的调用栈合并后，按字母顺序排列的。

也可以实时查看当前系统程序性能分析数据

```shell
perf record -F 999 -a -g -- sleep 60
perf script | ./stackcollapse-perf.pl | ./flamegraph.pl > out.svg
```

**`perf script`：**

- 这个命令从实时数据中读取。它会连接到正在运行的 `perf record` 进程（如果有的话）或者当前的系统事件，然后输出相应的性能数据。
- 这个命令通常用于实时查看当前系统的性能数据，而不是读取之前保存的数据文件。



由火焰图可见，grid_map::MedianFilter::update和grid_map::NormalVectorsFilter::update两个函数是程序性能的关键所在。继续向上回溯便可以逐渐发现程序的执行的“热点函数”。

有了这些分析，便可以针对性的对程序提出一些优化。



### 4. 参考资料
> - [如何读懂火焰图？](https://www.ruanyifeng.com/blog/2017/09/flame-graph.html)
> - [Linux C/C++性能优化分析工具Perf使用教程](https://www.bilibili.com/video/BV1hg4y1o7Sb/?share_source=copy_web&vd_source=32cbeca2ce39d95a4b3686678d95dc87)
