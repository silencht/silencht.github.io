---
title: Eigen代码第90行保证半正定的简单证明
mathjax: true
toc: true
---
《视觉SLAM十四讲》第二版，3.2节 实践：Eigen代码第50页第90行代码如下


```c++
matrix_NN = matrix_NN * matrix_NN.transpose(); //保证半正定
```
这里涉及一个简单的概念证明，如下：

【定义】矩阵 $A$ 为 $n$ 阶实数方阵，当且仅当存在非零的 $n$ 维向量 $x$ ，使得 $x^{T}Ax\geq0$ 恒成立，则矩阵 $A$ 是一个**半正定矩阵**。

 【$A^{T}A$ 的半正定性证明】

设矩阵 $A$ 为 $n$ 阶可逆实矩阵， $x$ 为任意非零 $n$ 维向量，

则 $x^{T}(A^{T}A)x=(x^{T}A^{T})(Ax)=(Ax)^{T}(Ax)=||(Ax)||^{2}\geq0$ 

即 $A^{T}A$ 半正定

