---
typora-root-url: ../../../..
title: 使用ORB-SLAM构建octomap要注意的一些问题
mathjax: true
toc: true
---

# 使用ORB-SLAM构建octomap要注意的一些问题

以ORB-SLAM系统做为基本框架，RGB-D相机作为基本输入。然后在此基础上，新建一个点云建图线程，将普通帧或关键帧对应的深度图像转换为三维点云，再发布至octomap\_server功能包（或者直接仿照此功能包在线程中直接重写八叉树地图构建代码）是目前诸多开源代码的主要流程。


> 1. 结构点云与非结构点云、深度值合法性、坐标参考系转换  
> 2. ORB-SLAM与ROS系统的坐标系模式转换  
> 3. octomap\_server建图时部分方格无故消失问题

在复现此类仓库代码时，有一些问题需要注意，在此总结如下。

### 1. 结构点云与非结构点云、深度值合法性、坐标参考系转换

如上述代码，根据关键帧pkf的深度图想构建了一个位于相机坐标系下的三维点云团camera\_pc\_和一个位于世界坐标系下的三维点云团world\_pc\_。需要注意的问题，已在注释中说明。


```c++
void PointCloudMapping::generatePointCloud(KeyFrame* pkf ,pcl::PointCloud<pcl::PointXYZRGB>::Ptr camera_pc_ ,
pcl::PointCloud<pcl::PointXYZRGB>::Ptr world_pc_)
{
//如果后续需要点云团在深度图像中的像素坐标信息（比如要和深度学习获取的目标检测框、语义分割信息等进行处理）
//那么可以生成此类“结构化点云（organized point cloud）”来保留点云像素坐标信息。
//如果不需要像素坐标信息，那么可以直接略过以下三行代码，直接对深度图像进行处理计算，然后往点云团对象中
//push_back点云点元素，如高翔老师仓库的实现：
//https://github.com/gaoxiang12/ORBSLAM2_with_pointcloud_map/blob/cf94d66a9e4253abd66398c7ef461a7a903efffe/ORB_SLAM2_modified/src/pointcloudmapping.cc#L79

//is_dense设置为false的原因是深度相机有很多位置的信息/点云不可信（不合法），做此标记后，后续的点云处理算法会
//知道该点云团内部还有非法点，从而进行单独处理，避免算法因异常点引发崩溃 
    camera_pc_->resize(pkf->mImDep.rows * pkf->mImDep.cols);
    camera_pc_->width = pkf->mImDep.cols;
    camera_pc_->height = pkf->mImDep.rows;
    camera_pc_->is_dense = false;//false if have points are invalid (e.g., have NaN or Inf values).
//有部分开源代码为节省算力，会对深度图进行跨点采样。具体做法是设置一个步长int stride = 3;然后嵌套for循环处的
//行列m+=1和n+=1均改为m+=stride和n+=stride。
    for ( int m=0; m<pkf->mImDep.rows; m+=1)
    {
        for ( int n=0; n<pkf->mImDep.cols; n+=1)
        {
            float d = pkf->mImDep.ptr<float>(m)[n];
//在深度图像中得到深度值d后，需要对其进行合法性判断（深度相机的有效深度范围）。
//多数仓库代码实现均为简单的深度相机值阈值判断：
//如 if (d < 0.01 || d>10)，此处的0.01和10对应下述代码的camera_valid_depth_Min和camera_valid_depth_Max。
//根据我的测试，如果d为nan值无法通过该范围滤除，因此最好判断条件里再加上nan值判断语句isnan(d)
            if(d < camera_valid_depth_Min || d > camera_valid_depth_Max || isnan(d)) continue;
            size_t index = m * pkf->mImDep.cols + n;
            camera_pc_->points[index].z = d;
            camera_pc_->points[index].x = ( n - pkf->cx) * d / pkf->fx;
            camera_pc_->points[index].y = ( m - pkf->cy) * d / pkf->fy;
            camera_pc_->points[index].r = pkf->mImRGB.ptr<uchar>(m)[n*3+2];
            camera_pc_->points[index].g = pkf->mImRGB.ptr<uchar>(m)[n*3+1];
            camera_pc_->points[index].b = pkf->mImRGB.ptr<uchar>(m)[n*3+0];
        }
    }
//上述点云团坐标均位于相机坐标系下，如果要得到世界系下的点云团，则使用关键帧的位姿变换矩阵Twc，将其转换至世界系。即，
//P_{world) = Twc * P_{camera}
//world_pc_便为世界系下的点云团
    Eigen::Isometry3d Twc = ORB_SLAM2::Converter::toSE3Quat( pkf->GetPoseInverse() );
    pcl::transformPointCloud(*camera_pc_, *world_pc_, Twc.matrix());

//3D Semantic Object Detect
    if(pkf->mvObjects2D.size() > 0 && is_octo_semantic_map_construction)
        mpDetector3D->Detect(pkf->mvObjects2D,pkf->mImDep,world_pc_);
//如果要进行全局三维点云地图重建
    if(is_global_pc_reconstruction)
        slam_to_ros_mode_transform(*world_pc_, *world_pc_);
}
```
### 2. ORB-SLAM与ROS系统的坐标系模式转换

此处可参考此文章：[ORB\_SLAM到ROS坐标转换](https://zhuanlan.zhihu.com/p/342481675)。用图来说明便是，

![](/image/使用ORB-SLAM构建octomap要注意的一些问题/v2-1c1cbd0c007eaaa48c0af556b27675f4_r.jpg)

SLAM坐标系与ROS坐标系之间的转换 在三维点云重建功能中，一般实现方法是：将连续到来的每张关键帧计算得到的世界系部分点云团world\_pc\_都累加到一个global\_map\_pc对象中，然后通过ROS将全图点云团global\_map\_pc发布至Rviz进行可视化。

问题是，即使从关键帧的深度图中导出了该帧的世界系点云团world\_pc\_，此世界系位姿也是相对于SLAM系统的初始帧（默认初始帧位姿为世界系单位阵）而言的。它的坐标系表示方式是上图中上方的SLAM（Z超前，X朝右，Y朝下）模式的。如果直接将该点云发布至Rviz进行可视化，那么此点云的显示相对于Rviz的grid网格是有一个错位颠倒的变换的，看起来是“垂直于”网格地面。所以我们发布前需要统一位姿表示模式。

这就是做三维点云地图重建时，代码里要做一个T\_slam\_to\_ros的位姿转换的原因：从SLAM位姿模式的Z、-X、-Y转变为ROS模式下的X、Y、Z，以便在Rviz中正确显示。


```c++
//如果要进行全局三维点云地图重建
if(is_global_pc_reconstruction)
        slam_to_ros_mode_transform(*world_pc_, *world_pc_);
//将点云位姿从SLAM表示模式转换至ROS表示模式
void PointCloudMapping::slam_to_ros_mode_transform(pcl::PointCloud<pcl::PointXYZRGB>& source, pcl::PointCloud<pcl::PointXYZRGB>& out)
{
	Eigen::Matrix4f T_slam_to_ros;
	T_slam_to_ros<< 0,0,1,0,
	               -1,0,0,0,
		           0,-1,0,0,
                    0,0,0,0;
	Eigen::Affine3f transform(T_slam_to_ros);
	pcl::transformPointCloud(source, out, transform);
}

```
同理，向ROS发布当前相机位姿，也是如此处理：


```c++
Camera_Pose = SLAM.TrackRGBD(imRGB,imD,tframe);//从Track获得当前帧位姿，即Tcw
Pub_CamPose(Camera_Pose); //将该位姿发布，在Rviz中进行显示

void Pub_CamPose(cv::Mat &pose)
{
    cv::Mat Rwc(3,3,CV_32F);
	cv::Mat twc(3,1,CV_32F);
	Eigen::Matrix<double,3,3> rotationMat;
	sg_slam_tf_broadcaster = new tf::TransformBroadcaster;
	if(pose.dims < 2 || pose.rows < 3){
                Rwc = Rwc;twc = twc;
	}else{
		Rwc = pose.rowRange(0,3).colRange(0,3).t();//pose is Tcw, so Rwc need .t()
		twc = -Rwc*pose.rowRange(0,3).col(3);
		rotationMat << Rwc.at<float>(0,0), Rwc.at<float>(0,1), Rwc.at<float>(0,2),
					   Rwc.at<float>(1,0), Rwc.at<float>(1,1), Rwc.at<float>(1,2),
					   Rwc.at<float>(2,0), Rwc.at<float>(2,1), Rwc.at<float>(2,2);
		Eigen::Quaterniond Q(rotationMat);
		// sg-slam's trans. x is twc.at<float>(0), y is twc.at<float>(1), z is twc.at<float>(2)
		// ros's x 为 slam's Z; ros's y 为 slam's -x; ros's z 为 slam's -y
		sg_slam_tf.setOrigin(tf::Vector3(twc.at<float>(2), -twc.at<float>(0), -twc.at<float>(1)));
		sg_slam_tf.setRotation(tf::Quaternion(Q.z(), -Q.x(), -Q.y(), Q.w()));
		//sg_slam_tf_broadcaster.sendTransform(tf::StampedTransform(sg_slam_tf, ros::Time::now(), "/map", "/camera"));
		//delete sg_slam_tf_broadcaster;
		Cam_Pose.header.stamp = ros::Time::now();
		Cam_Pose.header.frame_id = "/map";
		tf::pointTFToMsg(sg_slam_tf.getOrigin(), Cam_Pose.pose.position);
		tf::quaternionTFToMsg(sg_slam_tf.getRotation(), Cam_Pose.pose.orientation);
		CamPose_Pub.publish(Cam_Pose);
	}
}

```
### 3. octomap\_server建图时部分方格无故消失问题

通过点云建图线程向ROS发布关键帧的点云话题，然后使用octomap\_server功能包订阅点云话题构建八叉树地图（或者可以根据octomap库api自己直接编写相关功能）进行增量地图构建时，地图在扩建的过程中，之前的部分体素格会出现”逐渐消失“情况。如：

**DS-SLAM的这部分代码**

[DS-SLAM发布点云话题相关代码](https://github.com/ivipsourcecode/DS-SLAM/blob/c7577c8300f73c2e15cb540a963492659d972c73/src/pointcloudmapping.cc#L205-L219) 
```c++
for ( size_t i=lastKeyframeSize; i<N ; i++ )
{
    PointCloud::Ptr p = generatePointCloud( keyframes[i],semanticImgs_color[i], semanticImgs[i],colorImgs[i], depthImgs[i] );
    *KfMap += *p;
    *globalMap += *p;	    
}	
    PointCloud::Ptr tmp1(new PointCloud());
    voxel.setInputCloud( KfMap );
    voxel.filter( *tmp1 );
    KfMap->swap( *tmp1 );	
    pcl_cloud_kf = *KfMap;	

    Cloud_transform(pcl_cloud_kf,pcl_filter);
    pcl::toROSMsg(pcl_filter, pcl_point);
    pcl_point.header.frame_id = "/pointCloud";
    pclPoint_pub.publish(pcl_point);

```
将点云团由相机系转换为世界系（SLAM模式）后，再由SLAM模式转换为ROS模式（ Cloud\_transform(,) ）。其点云参考系为“frame\_id = "/pointCloud"”，随后便直接发布至ROS系统。在启动octomap建图时，要先使用roslaunch启动两个节点：

一是[octomap\_server](https://link.zhihu.com/?target=https%3A//github.com/ivipsourcecode/DS-SLAM/blob/master/Examples/ROS/ORB_SLAM2_PointMap_SegNetM/launch/Octomap.launch)，其参考系为/map


```xml
<param name="frame_id" type="string" value="/map" />
```
二是[transform.launch](https://link.zhihu.com/?target=https%3A//github.com/ivipsourcecode/DS-SLAM/blob/master/Examples/ROS/ORB_SLAM2_PointMap_SegNetM/launch/transform.launch)，作用是向ROS输出一个全局静态TF变换，即参考系/pointCloud与参考系/map（实际上，在这里map系就是默认的Rviz世界系）的固定变换为"0.195 -0.095 0.9 0 0 0"（x y z r p y表示）。


```xml
<node pkg="tf" type="static_transform\_publisher" name="map" args="0.195 -0.095 0.9 0 0 0 /map /pointCloud 70" >
```
以上便是DS-SLAM发布点云至octomap\_server增量建图的基本思路。

接下来，看octomap\_server源码中接收点云建图的相关代码。[octomap\_server节点启动，对象构建时，参数设置完毕后便会启用回调函数进行无限循环（#179）](https://link.zhihu.com/?target=https%3A//github.com/OctoMap/octomap_mapping/blob/6e96c831795c664ba554bdff53844657a7916f83/octomap_server/src/OctomapServer.cpp%23L179)，回调函数上方是两个话题订阅器，顾名思义，一个订阅点云话题，一个订阅点云位姿tf话题。


```c++
//https://github.com/OctoMap/octomap_mapping/blob/kinetic-devel/octomap_server/src/OctomapServer.cpp
//#L179
  m_pointCloudSub = new message_filters::Subscriber<sensor_msgs::PointCloud2> (m_nh, "cloud_in", 5);
  m_tfPointCloudSub = new tf::MessageFilter<sensor_msgs::PointCloud2> (*m_pointCloudSub, m_tfListener, m_worldFrameId, 5);
  m_tfPointCloudSub->registerCallback(boost::bind(&OctomapServer::insertCloudCallback, this, boost::placeholders::_1));

```
[回调函数](https://link.zhihu.com/?target=https%3A//github.com/OctoMap/octomap_mapping/blob/6e96c831795c664ba554bdff53844657a7916f83/octomap_server/src/OctomapServer.cpp%23L263)的主要功能在代码中注释如下：


```c++
void OctomapServer::insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud){
  ros::WallTime startTime = ros::WallTime::now();

  // ground filtering in base frame
  PCLPointCloud pc; // input cloud for filtering and ground-detection
  pcl::fromROSMsg(*cloud, pc);

//从ROS系统的话题中订阅获取传感器参考系（此文中即相机系）到世界系的位姿变换放入sensorToWorldTf
  tf::StampedTransform sensorToWorldTf;
  try {
    m_tfListener.lookupTransform(m_worldFrameId, cloud->header.frame_id, cloud->header.stamp, sensorToWorldTf);
  } catch(tf::TransformException& ex){
    ROS_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", quitting callback");
    return;
  }
  Eigen::Matrix4f sensorToWorld;
  pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);//格式转换

  // 一些滤波处理（部分代码省略）
  pcl::PassThrough<PCLPoint> pass_x;
  ……
  if (m_filterGroundPlane){
   //如果设置了滤除地面，部分处理操作（省略）
  ……
  } 
  else {
    // directly transform to map frame:将点云从传感器坐标系变换至世界系
    pcl::transformPointCloud(pc, pc, sensorToWorld);
    // 滤波处理（省略）
  ……
    // pc_nonground is empty without ground segmentation
    pc_ground.header = pc.header;
    pc_nonground.header = pc.header;
  }
//将点云插入octomap，进行增量更新
  insertScan(sensorToWorldTf.getOrigin(), pc_ground, pc_nonground);
  ……
}
```
点云插入增量更新代码如下，


```c++
void OctomapServer::insertScan(const tf::Point& sensorOriginTf, const PCLPointCloud& ground, const PCLPointCloud& nonground){
  point3d sensorOrigin = pointTfToOctomap(sensorOriginTf);

……
  // instead of direct scan insertion, compute update to filter ground:
  KeySet free_cells, occupied_cells;
  // insert ground points only as free:
 ……
  // 对于所有非地面点云: free on ray, occupied on endpoint:
  for (PCLPointCloud::const_iterator it = nonground.begin(); it != nonground.end(); ++it){
    point3d point(it->x, it->y, it->z);
    if ((m_minRange > 0) && (point - sensorOrigin).norm() < m_minRange) continue;
    // maxrange check
    if ((m_maxRange < 0.0) || ((point - sensorOrigin).norm() <= m_maxRange) ) {
      // free cells
      if (m_octree->computeRayKeys(sensorOrigin, point, m_keyRay)){
        free_cells.insert(m_keyRay.begin(), m_keyRay.end());
      }
      // occupied endpoint
      OcTreeKey key;
      if (m_octree->coordToKeyChecked(point, key)){
        occupied_cells.insert(key);
      ……
    } else {// ray longer than maxrange:;
      ……
      }
    }
  }

  // mark free cells only if not seen occupied in this cloud
……
  // now mark all occupied cells:
……
……
}
```
从OctomapServer::insertScan()函数中可以看出，每次插入点云时，以传感器在世界系下的原点为起点，以点云（世界系下）坐标为终点，更新octomap。即，从原点到终点光线ray上的所有点设置为空闲free，点云所在点（终点）设置为占据occupied状态（free on ray, occupied on endpoint），如下图所示。

![](/image/使用ORB-SLAM构建octomap要注意的一些问题/v2-5512e3adef92396b21c39c8d25d1bdd1_r.jpg) 

https://www.arminhornung.de/Research/pub/hornung13roscon.pdf 

综上，那么DS-SLAM以及一些类似的问题[[1]](#ref_1) [[2]](#ref_2)出现体素格莫名其妙消失（清除cleared)的原因可以分析如下：

在SLAM系统生成点云时，在SLAM中便将3D点云转换为了世界系（此时均为SLAM位姿模式），然后又将SLAM模式转换为ROS模式（见本文第二节），最后将点云消息进行发布。

octomap\_server功能包的建图逻辑是订阅 【相机系下的3D点云信息】、【与该点云信息有关的传感器系（本文即相机系）到世界系的位姿变换 tf 】两种数据（octomap\_server默认所有消息的位姿类型均为ROS模式）。

DS-SLAM并没有在SLAM系统中发布相机系到世界系的位姿变换 tf ，而且它将相机系下的3D点云（SLAM模式）手动转换为了世界系下的3D点云（ROS模式）并发布出去。

为了使得octomap\_server可以处理DS-SLAM发布的点云数据，DS-SLAM直接roslaunch了一个tf静态变换文件，持续向ROS发布相机系/pointCloud到世界系/map的位姿变换tf，只不过这个tf除了一些xyz的单纯位置变换外，并没有其他的旋转操作。


```xml
<node pkg="tf" type="static_transform_publisher" name="map" args="0.195 -0.095 0.9 0 0 0 /map /pointCloud 70" >
```
两种数据齐备，那么octomap\_server此时确实已经可以运行。不过这里有一个小小的问题，那便是：

由于每次octomap\_server接收到的相机系到世界系的位姿变换tf是DS-SLAM手动发布的静态信息，因此octomap\_server每次处理3D点云并对octomap进行增量更新时，它所参考的传感器原点（上文图中的sensor origin）一直是"0.195 -0.095 0.9 0 0 0"这个相对于世界系的静态坐标，从来不会发生变化。

再由八叉树更新原理可知，每次的空闲和占据状态是根据传感器真正原点到点云坐标点的“光线”（下图中以true sensor origin起始的黑色箭头）来进行处理的。本来每次的原点sensor origin都应该是传感器实时相对于世界系的真实原点，而点云则是基于传感器系下的坐标，然后回调函数insertCloudCallback()会自己对这些数据进行坐标系变换处理。

![](/image/使用ORB-SLAM构建octomap要注意的一些问题/v2-87db55fd53e188aab6c9287fbfc1df9d_r.jpg) 

正确的传感器原点、错误的传感器原点设置 然而，如上图所示，现在DS-SLAM的部分代码一边对点云的坐标系和模式进行了手动处理（点云变为了ROS模式的世界系坐标），一边发布了一个静态tf变换对octomap\_server进行“蒙蔽”。这就导致一旦相机拍摄到已经占据的体素格的点云，新产生的光线线路（图中绿色箭头）上所有的体素格（无论占据与否）都将更新为空闲free。那么便会有一部分实际为占据状态的体素（如end point)，被更新为空闲状态。这就导致了部分体素“消失”的现象。

**解决的方法很简单，按照octomap\_server的正常使用方式使用即可：**

**1. generatePointCloud( )直接输出相机系下的点云团，**[只将SLAM位姿模式改为ROS模式即可](https://link.zhihu.com/?target=https%3A//github.com/silencht/SG-SLAM/blob/c5d613653d2ad9cbea0109cc7dc1d0d9b7d8eca2/src/sg-slam/src/PointcloudMapping.cc%23L283)**，**[并将点云团参考系设为/camera](https://link.zhihu.com/?target=https%3A//github.com/silencht/SG-SLAM/blob/c5d613653d2ad9cbea0109cc7dc1d0d9b7d8eca2/src/sg-slam/src/PointcloudMapping.cc%23L285)**.**

**2.** [求解相机系到世界系的位姿变换tf（ROS模式的）](https://link.zhihu.com/?target=https%3A//github.com/silencht/SG-SLAM/blob/c5d613653d2ad9cbea0109cc7dc1d0d9b7d8eca2/src/sg-slam/src/PointcloudMapping.cc%23L263-L274)**，**

**3.** [发布点云团和tf变换，标记tf参考系变换为/camera\_sensor至/map](https://link.zhihu.com/?target=https%3A//github.com/silencht/SG-SLAM/blob/c5d613653d2ad9cbea0109cc7dc1d0d9b7d8eca2/src/sg-slam/src/PointcloudMapping.cc%23L291-L293)

那么，octomap\_server现在接收到的信息便是：

1. 每个关键帧的位于相机系下的ROS模式点云团数据，参考系为/camera\_sensor；

2. 每个关键帧的自相机系/camera\_sensor至世界系/map的ROS模式位姿变换tf；

然后octomap\_server便可正确处理这些数据了。

还可参考代码或文章

[高翔老师octomap tutor](https://github.com/gaoxiang12/octomap\_tutor/blob/f4a6a8eaf835fb6189345c0ee598509ed6faa280/src/joinMap.cpp#L135) [](https://zhuanlan.zhihu.com/p/176513040) 本文部分代码摘自仓库

[SG-SLAM](https://github.com/silencht/SG-SLAM) 参考
--

1. [^](#ref_1_0)<https://answers.ros.org/question/224488/octomap-decreasing-probabilities-when-obstacle-is-not-there-anymore/>
2. [^](#ref_2_0)<https://answers.ros.org/question/51837/octomap_server-globally-referenced-pointcloud-and-transform/>
