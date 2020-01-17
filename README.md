# map_tools
博客讲解地址：http://xchu.net/2019/10/30/32pcd-divider/#more

点云地图网格划分及可视化工具．

- pcd_grid_divider

- gridmap_viewer

## **pcd_grid_divider**

pcd_grid_divider是一个ros节点，需要放在catkin workspace下编译．

```bash
mkdir -p catkin_ws/src
cd catkin_ws/src
catkin_make
cd src
git clone https://github.com/JokerJohn/map_tools.git
cd ..
catkin_make
```

在`grid_divider.launch`中修改点云地图PCD和化分完的grid_map文件路径，网格size大小．

```bash
source devel/setup.bash
roslaunch map_tools grid_divider.launch 
```

map目录和pcd_grid_divider目录平级，原始的点云PCD文件需要放在map/pcd目录下，生成的网格pcd自动存放在map/grid_map目录下，并且每次执行此节点前都会清空map/grid_map目录．

## gridmap_viewer

gridmap_viewer是一个cmake工程，用于可视化划分好的网格地图．使用方法．代码中使用的是相对路径，按照上述目录要求，map/grid_map路径下有文件的话，无需修改代码．

```bash
cd gridmap_viewer/build
cmake..
make
./gridmap_viewer
```

map目录和pcd_grid_divider目录平级，原始的点云PCD文件需要放在map/pcd目录下，生成的网格pcd自动存放在map/grid_map目录下，并且每次执行此节点前都会清空map/grid_map目录．效果如图

![image-20200117185605659](README/image-20200117185605659.png)

## 致谢

Autoware 