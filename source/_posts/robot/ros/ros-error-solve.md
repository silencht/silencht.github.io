---
title: ros error solve
toc: true 
---


## 1. Failed to load library libgrid_map_rviz_plugin.so
grid_map 的 rviz 插件出现错误，导致无法显示GridMap类型的地图.
错误信息如下：

> GridMap
> The class required for this display, 'grid_map_rviz_plugin/GridMap', could not be loaded.
> Error:
> Failed to load library /opt/ros/noetic/lib//libgrid_map_rviz_plugin.so. Make sure that you are calling the PLUGINLIB_EXPORT_CLASS macro in the library code, and that names are consistent between this macro and your XML. Error string: Could not load library (Poco exception = libgrid_map_core.so: cannot open shared object file: No such file or directory)

### 解决方法

```bash
cd /opt/ros/noetic/lib
ls | grep grid
changhe@changhe:/opt/ros/noetic/lib$ ls | grep grid
grid_map_demos
grid_map_loader
grid_map_visualization
libgrid_map_cv.so
libgrid_map_octomap.so
libgrid_map_ros.so
libgrid_map_rviz_plugin.so
libvoxel_grid.so
#发现是有libgrid_map_rviz_plugin.so库的，继续查看该库的依赖信息
changhe@changhe:/opt/ros/noetic/lib$ ldd libgrid_map_rviz_plugin.so 
	linux-vdso.so.1 (0x00007ffe6f79e000)
	librviz.so => /opt/ros/noetic/lib/librviz.so (0x00007ff49eeaf000)
	libOgreMain.so.1.9.0 => /lib/x86_64-linux-gnu/libOgreMain.so.1.9.0 (0x00007ff49e8da000)
	libgrid_map_ros.so => /opt/ros/noetic/lib/libgrid_map_ros.so (0x00007ff49e887000)
	libgrid_map_core.so => not found
	libclass_loader.so => /opt/ros/noetic/lib/libclass_loader.so (0x00007ff49e85a000)
	libmessage_filters.so => /opt/ros/noetic/lib/libmessage_filters.so (0x00007ff49e850000)
……略
#果然，其中有一个依赖libgrid_map_core.so => not found找不到。原因是几天前手动将该库删除过。现在将grid_map源码编译后的库复制过来即可
changhe@changhe:/opt/ros/noetic/lib$ cd ~/grid_map_core/lib/
changhe@changhe:~/grid_map_core/lib$ ls
libgrid_map_core.so
changhe@changhe:~/grid_map_core/lib$ sudo cp libgrid_map_core.so /opt/ros/noetic/lib/
#再次打开rviz，add GridMap类型，发现错误果然消失了
```
