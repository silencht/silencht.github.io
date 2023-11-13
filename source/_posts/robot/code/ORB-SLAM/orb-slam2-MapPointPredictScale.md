---
typora-root-url: ../../../..
title: MapPoint::PredictScale()小记
mathjax: true
toc: true
---


### **0. 前提假设**

每次相机拍摄后，将原始图像进行金字塔缩放操作，以获得尺度不变性。

图像金字塔层数nlevels设置为level 0 至 level 7，原始图像为level 0，最深层金字塔图像帧为level 7，共8层；

每层金字塔缩放系数因子scale为1.2；

常识：相机拍摄物体时，近大远小；

相机在参考关键帧 $RefKF$ 位置**首次 "观测/提取/生成"** 了一个地图点 $M$ 。

### 1. 地图点 观测距离范围

由参考帧在世界坐标系下的光心坐标和地图点 $M$ 在世界系下的坐标可以算出两者之间的距离 $dist$ 。

现在假设该距离值为 $dist=1$ 米。


```c++
void MapPoint::UpdateNormalAndDepth()
{
    // 略去部分无需关注的代码
    // 获得观测到该地图点的所有关键帧、坐标等信息
    pRefKF=mpRefKF;             // 观测到该点的参考关键帧（第一次创建地图点时的关键帧）
    Pos = mWorldPos.clone();    // 地图点在世界坐标系中的位置
    // 参考关键帧相机指向地图点的向量（在世界坐标系下的表示）
    cv::Mat PC = Pos - pRefKF->GetCameraCenter();        
    // 该地图点到参考关键帧相机光心的距离 
    const float dist = cv::norm(PC);                                        
    const int level = pRefKF->mvKeysUn[observations[pRefKF]].octave;// 观测到该地图点的当前帧的特征点在金字塔的第几层
    const float levelScaleFactor =  pRefKF->mvScaleFactors[level];  // 当前金字塔层对应的尺度因子，scale^n，scale=1.2，n为层数
    const int nLevels = pRefKF->mnScaleLevels;                      // 金字塔总层数，默认为8
    {
        //现假设dist=1
        mfMaxDistance = dist\*levelScaleFactor;                          // 观测到该点的距离上限 1\*(1.2)^7=3.583
        mfMinDistance = mfMaxDistance/pRefKF->mvScaleFactors[nLevels-1];// 观测到该点的距离下限 1/(1.2)^7=0.279
        mNormalVector = normal/n;                                       // 获得地图点平均的观测方向
    }
}

```
由以上信息可以推断：

**要想使以后的相机（图像帧）可以再次观测到该地图点 $M$ ，需要相机与地图点的距离在 [mfMinDistance，mfMaxDistance] 范围内。**

分情况讨论：

**1. mfMinDistance**

假设，参考关键帧 $RefKF$ （相机首次）"提取/生成"地图点 $M$ 时的图像金字塔层级是level 0，也即地图点 $M$ 所对应的特征点在参考帧中原始图像（level 0）中提取而来，且相机光心距离地图点为1米。

那么，相机（图像帧）可以拍摄到该地图点的最小距离为 $dist_{min}=dist*scale^{level+1-nlevels}=dist*scale^{0+1-8}=1*1.2^{-7}\approx 0.279$ ，

就是说，相机在距离该地图点 $M$ 大约0.279米时拍摄的图像帧，经过7层缩放的level 7图像帧提取出的该地图点对应的特征点，与相机距离该地图点1米时在原始图像（level 0）提取的该地图点对应的特征点**相似**。

根据相机拍摄物体时近大远小原则，相机在距离 $M$ 最近约0.279米时进行的图像金字塔的最深层缩放模拟形成的拍摄效果，类似于相机（参考帧）在距离约1米时对 $M$ 拍摄的原始图像。

如果相机与 $M$ 的距离小于0.279米，那么即使使用最深层的金字塔图像帧，也难以模拟出参考帧当时的拍摄效果。

**2. mfMaxDistance**

假设，参考关键帧 $RefKF$ （相机首次）"提取/生成"地图点 $M$ 时的图像金字塔层级是level 7，也即地图点 $M$ 所对应的特征点在参考帧的最深层金字塔图像帧（level 7）中提取而来，且相机光心距离地图点为1米。

那么，相机（图像帧）可以拍摄到该地图点的最大距离为 

$dist_{max}=dist*scale^{level}=dist*scale^{7}=1*1.2^{7}\approx 3.583$ ，

就是说，相机在距离该地图点 $M$ 大约3.583米时拍摄的图像帧，在原始图像（level 0）提取出的该地图点对应的特征点，与相机距离该地图点1米时在最深层金字塔图像帧（level 7）提取的该地图点对应的特征点**相似**。

根据相机拍摄物体时近大远小原则，相机在距离 $M$ 最远约3.583米时的原始图像形成的拍摄效果，类似于相机（参考帧）在距离约1米时对 $M$ 拍摄的最深层金字塔图像帧（level 7）。

如果相机与 $M$ 的距离大于3.583米，那么即使使用原始图像，也难以模拟出参考帧当时的拍摄效果。

### 2. 预测 地图点 金字塔尺度/层数


```c++
/\*\*
 \* @brief 预测地图点对应特征点所在的图像金字塔尺度层数
 \* @param[in] currentDist 相机光心距离地图点距离
 \* @param[in] pKF 关键帧
 \* @return int 预测的金字塔尺度
 \*/
int MapPoint::PredictScale(const float &currentDist, KeyFrame\* pKF)
{
    float ratio;
    {
        unique\_lock<mutex> lock(mMutexPos);
        // mfMaxDistance = ref\_dist\*levelScaleFactor 为参考帧考虑上尺度后的距离
        // ratio = mfMaxDistance/currentDist = ref\_dist/cur\_dist
        ratio = mfMaxDistance/currentDist;
    }
    // 取对数
    int nScale = ceil(log(ratio)/pKF->mfLogScaleFactor);
    if(nScale<0)
        nScale = 0;
    else if(nScale>=pKF->mnScaleLevels)
        nScale = pKF->mnScaleLevels-1;

    return nScale;
}

```
明白了地图点的观测距离范围，PredictScale()就容易理解：

现在假设：在非参考帧外的其他图像帧（如 $pKF$ ）位置又观测/关联到了地图点 $M$ ，且已知两者之间的距离为 $currentDist$ （假设其位于观测距离范围内）。

那么，我们可以通过该距离 $currentDist$ 反向推断，该地图点 $M$ 最有可能从 $pKF$ 的哪一层金字塔图像帧上提取而来。其实，这个推断过程与 **地图点 观测距离范围** 的推断过程正好相反。

例如，如果当前帧 $pKF$ 的相机光心位置与地图点 $M$ 的距离 $currentDist\approx3.583$ 米。那么由第1节（地图点 观测距离范围）反向推理可知，此时观测/关联到地图点 $M$的特征点最有可能是从当前帧 $pKF$的原始图像（level 0）提取而来的。其他金字塔缩放层提取的特征点所对应的地图点，基本上与 $pKF$ 之间的距离会远大于3.583米。

这个推断过程用数学表述就是利用mfMaxDistance的公式 反向求解 $level$ ，即

$dist_{max}=dist*scale^{level}\\ \frac{dist_{max}}{dist}=scale^{level}\\ level=\frac{log(\frac{d_{max}}{dist})}{log(scale)} $ 

代码中ratio便是 $\frac{dist_{max}}{dist}$ ，pKF->mfLogScaleFactor即为 $log(scale)$ 。

故当前帧与地图点的观测近，那么使用的图像金字塔层级便高；

 当前帧与地图点的观测远，那么使用的图像金字塔层级便低。

![](/image/orb-slam2-MapPointPredictScale/1.jpg)

https://blog.csdn.net/ncepu_Chen/article/details/116784652