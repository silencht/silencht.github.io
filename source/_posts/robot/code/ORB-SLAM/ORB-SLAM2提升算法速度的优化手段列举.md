---
typora-root-url: ../../../..
title: ORB-SLAM2提升算法速度的优化手段列举
mathjax: true
toc: true
---

 **Author:** [矩阵的秩]

 **Link:** [https://zhuanlan.zhihu.com/p/618562057]

### 1. 跟踪初始化求本质矩阵和单应矩阵时，开双线程进行并行加速


```c++
// 构造线程来计算H矩阵及其得分
// thread方法比较特殊，在传递引用的时候，外层需要用ref来进行引用传递，否则就是浅拷贝
thread threadH(&Initializer::FindHomography,	//该线程的主函数
				   this,					
				   ref(vbMatchesInliersH), 			
				   ref(SH), 				
				   ref(H));
// 计算fundamental matrix并打分
thread threadF(&Initializer::FindFundamental,this,ref(vbMatchesInliersF), ref(SF), ref(F));

```
### 2. 计算关键点灰度质心方向时，利用圆的对称性每次循环求解对称的两组值。最后使用FastAtan2函数加速计算角度


```c++
static float IC_Angle(const Mat& image, Point2f pt,  const vector<int> & u_max)
{
//……部分代码略
    for (int v = 1; v <= HALF_PATCH_SIZE; ++v)
    {
        int v_sum = 0;
        int d = u_max[v];
	// 在坐标范围内挨个像素遍历，实际是一次遍历2个
        // 假设每次处理的两个点坐标，中心线下方为(x,y),中心线上方为(x,-y) 
        // 对于某次待处理的两个点：m_10 = Σ x*I(x,y) =  x*I(x,y) + x*I(x,-y) = x*(I(x,y) + I(x,-y))
        // 对于某次待处理的两个点：m_01 = Σ y*I(x,y) =  y*I(x,y) - y*I(x,-y) = y*(I(x,y) - I(x,-y))
        for (int u = -d; u <= d; ++u)
        {
            int val_plus = center[u + v*step], val_minus = center[u - v*step];
	    //在v（y轴）上，2行所有像素灰度值之差
            v_sum += (val_plus - val_minus);
	    //u轴（也就是x轴）方向上用u坐标加权和（u坐标也有正负符号），相当于同时计算两行
            m_10 += u * (val_plus + val_minus);
        }
        m_01 += v * v_sum;
    }
    //为了加快速度还使用了fastAtan2()函数，输出为[0,360)角度，精度为0.3°
    return fastAtan2((float)m_01, (float)m_10);
}
```
### 3. 通过近邻窗口半径（提取特征时预先分配至网格）、投影后坐标窗口附近、Bow词袋等几种方法加速特征匹配


```
void Frame::AssignFeaturesToGrid()
int SearchByBoW()
//……
//等等

```
### 4. 每次先将某些变量的倒数值计算出来存放至inverse变量，减少计算时除法的计算量


```c++
//例如：1
mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
//2
Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight,……)
{
……
// 猜测是因为这种除法计算需要的时间略长，所以这里直接存储了这个中间计算结果
    invfx = 1.0f/fx;
    invfy = 1.0f/fy;
……
}
//3
int ORBmatcher::SearchByProjection(KeyFrame* pKF, cv::Mat Scw, const vector<MapPoint*> &vpPoints, vector<MapPoint*> &vpMatched, int th)
{
……
     const float invz = 1/p3Dc.at<float>(2);
     const float x = p3Dc.at<float>(0)*invz;
     const float y = p3Dc.at<float>(1)*invz;
……
}
//等等
```
### 5. 循环或函数开始前先把vector等变量reserve扩容至足够大，防止循环时因容量不够发生临时扩容


```c++
//例如：1
int ORBmatcher::SearchByBoW(KeyFrame* pKF,Frame &F, vector<MapPoint*> &vpMapPointMatches)
{
……
    // 特征点角度旋转差统计用的直方图
    vector<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);
……
}
//2
void Frame::AssignFeaturesToGrid()
{
……
    // 给存储特征点的网格数组 Frame::mGrid 预分配空间
    int nReserve = 0.5f*N/(FRAME_GRID_COLS*FRAME_GRID_ROWS);
    // 开始对mGrid这个二维数组中的每一个vector元素遍历并预分配空间
    for(unsigned int i=0; i<FRAME_GRID_COLS;i++)
        for (unsigned int j=0; j<FRAME_GRID_ROWS;j++)
            mGrid[i][j].reserve(nReserve);
……
}
//等等
```
### 6. 计算双目特征匹配时，只在某近邻像素行范围内查找，不全局暴力匹配


```c++
void Frame::ComputeStereoMatches()
{
    /*两帧图像稀疏立体匹配（即：ORB特征点匹配，非逐像素的密集匹配，但依然满足行对齐）
     * 输入：两帧立体矫正后的图像img_left 和 img_right 对应的orb特征点集
     * 过程：
          1. 行特征点统计. 统计img_right每一行上的ORB特征点集，便于使用立体匹配思路(行搜索/极线搜索）进行同名点搜索, 避免逐像素的判断.
          2. 粗匹配. 根据步骤1的结果，对img_left第i行的orb特征点pi，在img_right的第i行上的orb特征点集中搜索相似orb特征点, 得到qi
          3. 精确匹配. 以点qi为中心，半径为r的范围内，进行块匹配（归一化SAD），进一步优化匹配结果
          4. 亚像素精度优化. 步骤3得到的视差为uchar/int类型精度，并不一定是真实视差，通过亚像素差值（抛物线插值)获取float精度的真实视差
          5. 最优视差值/深度选择. 通过胜者为王算法（WTA）获取最佳匹配点。
          6. 删除离群点(outliers). 块匹配相似度阈值判断，归一化sad最小，并不代表就一定是正确匹配，比如光照变化、弱纹理等会造成误匹配
     * 输出：稀疏特征点视差图/深度图（亚像素精度）mvDepth 匹配结果 mvuRight
     */
……
    // 右图特征点数量，N表示数量 r表示右图，且不能被修改
    const int Nr = mvKeysRight.size();
    // Step 1. 行特征点统计. 考虑到尺度金字塔特征，一个特征点可能存在于多行，而非唯一的一行
    for(int iR = 0; iR < Nr; iR++) {
        // 获取特征点ir的y坐标，即行号
        const cv::KeyPoint &kp = mvKeysRight[iR];
        const float &kpY = kp.pt.y;
        // 计算特征点ir在行方向上，可能的偏移范围r，即可能的行号为[kpY + r, kpY -r]
        // 2 表示在全尺寸(scale = 1)的情况下，假设有2个像素的偏移，随着尺度变化，r也跟着变化
        const float r = 2.0f * mvScaleFactors[mvKeysRight[iR].octave];
        const int maxr = ceil(kpY + r);
        const int minr = floor(kpY - r);
        // 将特征点ir保证在可能的行号中
        for(int yi=minr;yi<=maxr;yi++)
            vRowIndices[yi].push_back(iR);
    }
……
}
```
### 7. 访问多线程加锁数据时，快速解锁，避免长时间独占共享数据


```c++
//多线程编程注意，例如
void LocalMapping::ProcessNewKeyFrame()
{
    {
        unique_lock<mutex> lock(mMutexNewKFs);
        mpCurrentKeyFrame = mlNewKeyFrames.front();
        mlNewKeyFrames.pop_front();
    }
    //上锁处理数据后，迅速解锁
……//do anotherthing
}
```
### 8. 为不同作用的数据变量选用合适的容器

例如：

1. 存储特征点的**vector容器**（频繁随机访问，变化较小：提取完成后很少再进行插入和删除）

2. 存储共视关键帧mConnectedKeyFrameWeights的**map容器。** mConnectedKeyFrameWeights是一个用于保存当前关键帧与其连接的其他关键帧之间权重信息的数据结构，其中键是连接的关键帧 ID，值是权重。此变量在 ORB-SLAM2 中被频繁地使用，因此它需要支持高效的查找和插入操作。

个人猜测，由于该变量的大小通常比较小，因此使用 `std::map` 。

`std::map` 是一种有序关联容器，它使用红黑树实现。由于它是一个有序容器，因此它提供了按键排序的功能，这使得它在查找和插入操作方面具有较高的效率。另外，由于红黑树的高效性能和可预测的内存占用，`std::map` 通常比 `std::unordered_map` 在处理小规模数据时表现更好。

3. 四叉树均匀化分配特征点以及记录连接关键帧、局部地图点（如*lLocalMapPoints*）时使用**list链表容器**（不经常随机访问，而是遍历迭代访问，经常需要插入，list插入效率高）

4. Map中的地图点和关键帧用**set容器**，因为经常需要做插入删除动作（印象中orbslam2这里地图点的删除没实现，可能会造成内存泄漏）set容器有序，无重复元素，方便转vector（返回关键帧集合转化的vector给可视化和BA优化程序用）

### 9. 待补充

