# ros-catkin-ws

一个面向 ROS1 `catkin` 工作流的综合机器人实验工作空间，整合了：

- `image_pkg`：基于 Kinect2 / RGB-D 数据的视觉与点云实验
- `waterplus_map_tools`：地图航点编辑与导航辅助工具
- `wpb_home`：启智 ROS 机器人相关功能包集合
- `wpr_simulation`：WPR / WPB 系列机器人仿真环境与示例

该仓库适合用于 ROS 课程实验、移动机器人仿真、RGB-D 感知验证，以及基于现有仿真平台扩展自定义感知节点。

---

## 1. 工作空间结构

```text
catkin_ws/
├── .catkin_workspace
├── .gitignore
├── README.md
├── Pictures/
│   └── L4/
│       ├── fig01_gazebo_scene.png
│       ├── fig02_rviz_pointcloud.png
│       ├── fig03_pcl_viewer.png
│       ├── 实验4_RGBD点云与目标测距实验报告.pdf
│       └── 检查点云话题和频率.png
└── src/
    ├── CMakeLists.txt
    ├── image_pkg/
    ├── waterplus_map_tools/
    ├── wpb_home/
    └── wpr_simulation/
```

### 1.1 image_pkg

自定义视觉实验包，主要围绕 Kinect2 图像、深度图和点云处理展开。

关键依赖见 `src/image_pkg/package.xml:1`：

- `cv_bridge`
- `geometry_msgs`
- `ros_numpy`
- `rospy`
- `sensor_msgs`
- `std_msgs`

主要脚本：

- `src/image_pkg/scripts/follow_node.py:1`
  - 基于 HSV 颜色阈值识别目标色块
  - 发布 `cmd_vel` 控制底盘进行视觉跟随
- `src/image_pkg/scripts/get_pointcloud.py:1`
  - 将 RGB 图像与深度图转换为 `PointCloud2`
  - 发布到 `/pointcloud_output`
- `src/image_pkg/scripts/get_pointcloud3.py:1`
  - 在点云发布基础上支持一次性保存 PCD 文件
- `src/image_pkg/scripts/red_bottle_distance.py:1`
  - 检测红色瓶子并输出目标距离、最近障碍物距离
- `src/image_pkg/scripts/green_bottle_move_distance.py:1`
  - 在巡航过程中检测绿色瓶子，同时结合最近障碍物进行简单避障

主要 launch：

- `src/image_pkg/launch/get_pointcloud.launch:1`
  - 启动实时点云发布节点
- `src/image_pkg/launch/get_pointcloud3.launch:1`
  - 启动点云发布并保存 PCD
- `src/image_pkg/launch/red_bottle_distance.launch:1`
  - 启动红瓶检测与测距节点
- `src/image_pkg/launch/green_bottle_move_distance.launch:1`
  - 启动绿色瓶检测、测距、运动控制节点
- `src/image_pkg/launch/green_bottle_demo.launch:1`
  - 联合 `wpr_simulation` 的瓶子场景进行完整演示

### 1.2 waterplus_map_tools

地图航点工具，原始说明见 `src/waterplus_map_tools/README.md:1`。

主要能力：

- 在 RViz 中交互式设置航点
- 保存航点
- 执行航点遍历测试

典型命令（摘自原 README）：

```bash
roslaunch waterplus_map_tools add_waypoint.launch
rosrun waterplus_map_tools wp_saver
rosrun waterplus_map_tools wp_nav_test
```

### 1.3 wpb_home

启智 ROS 机器人功能包集合，原始说明见 `src/wpb_home/README.md:1`。

包含：

- 机器人 URDF 模型
- 里程计、IMU、视觉、雷达等能力配套功能
- 语音、导航、跟随、抓取等演示基础

该目录下包含多个子包，例如：

- `wpb_home_bringup`
- `wpb_home_behaviors`
- `wpb_home_python`
- `wpb_home_python3`
- `wpb_home_tutorials`
- `wpbh_local_planner`

### 1.4 wpr_simulation

WPR / WPB 系列机器人仿真环境，原始说明见 `src/wpr_simulation/README.md:1`。

主要用途：

- Gazebo 仿真场景启动
- SLAM / Navigation 演示
- 物体抓取、桌面任务等实验环境构建
- 为 `image_pkg` 提供带目标物的视觉实验场景

典型命令（摘自原 README）：

```bash
roslaunch wpr_simulation wpb_simple.launch
roslaunch wpr_simulation wpb_gmapping.launch
roslaunch wpr_simulation wpb_navigation.launch
roslaunch wpr_simulation wpb_table.launch
rosrun wpb_home_tutorials wpb_home_grab_client
```

---

## 2. 运行环境建议

推荐环境：

- Ubuntu 20.04
- ROS Noetic
- Python 3
- Catkin 工作空间

> 说明：
> `wpb_home`、`waterplus_map_tools` 的原始 README 中也给出了 Kinetic / Melodic 等环境说明；
> 而 `wpr_simulation` 当前 README 明确标注的是 ROS Noetic / Ubuntu 20.04。
> 如果你希望最小化兼容性问题，优先采用 **Ubuntu 20.04 + ROS Noetic**。

---

## 3. 构建步骤

### 3.1 克隆仓库

```bash
git clone git@github.com:AEI-boop/ros-catkin-ws.git
cd ros-catkin-ws
```

### 3.2 编译

```bash
catkin_make
```

### 3.3 加载环境

```bash
source devel/setup.bash
```

如果你希望每个终端自动加载，也可以加入 `~/.bashrc`：

```bash
echo "source /你的路径/ros-catkin-ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 4. 依赖安装说明

不同子项目有各自的安装脚本，建议优先参考各自自带 README。

### 4.1 waterplus_map_tools

参考 `src/waterplus_map_tools/README.md:1`：

```bash
~/catkin_ws/src/waterplus_map_tools/scripts/install_for_noetic.sh
```

### 4.2 wpb_home

参考 `src/wpb_home/README.md:3`：

```bash
cd ~/catkin_ws/src/wpb_home/wpb_home_bringup/scripts
./install_for_noetic.sh
```

### 4.3 wpr_simulation

参考 `src/wpr_simulation/README.md:29`：

```bash
cd ~/catkin_ws/src/wpr_simulation/scripts
./install_for_noetic.sh
```

如果你没有完全按照 `~/catkin_ws` 命名工作空间，需要把命令里的路径替换为你自己的实际路径。

---

## 5. 常见运行方式

### 5.1 点云发布

实时生成 RGB-D 点云：

```bash
roslaunch image_pkg get_pointcloud.launch
```

带 PCD 保存：

```bash
roslaunch image_pkg get_pointcloud3.launch
```

默认会将点云保存到：

```text
~/output_cloud.pcd
```

对应实现见 `src/image_pkg/scripts/get_pointcloud3.py:20`。

### 5.2 红色瓶子测距

```bash
roslaunch image_pkg red_bottle_distance.launch
```

功能：

- 识别红色目标
- 输出红瓶距离
- 输出最近障碍物距离

### 5.3 绿色瓶子巡航测距

```bash
roslaunch image_pkg green_bottle_move_distance.launch
```

功能：

- 检测绿色瓶子
- 输出距离变化
- 发布 `/cmd_vel`
- 当最近障碍物过近时执行原地转向避障

### 5.4 绿色瓶子完整演示

```bash
roslaunch image_pkg green_bottle_demo.launch
```

这个 launch 会：

1. 启动 `wpr_simulation` 的瓶子场景
2. 启动 `image_pkg` 的绿色瓶检测与运动控制节点

见 `src/image_pkg/launch/green_bottle_demo.launch:1`。

### 5.5 地图航点工具

```bash
roslaunch waterplus_map_tools add_waypoint.launch
```

保存航点：

```bash
rosrun waterplus_map_tools wp_saver
```

航点导航测试：

```bash
rosrun waterplus_map_tools wp_nav_test
```

### 5.6 仿真与导航

简单仿真场景：

```bash
roslaunch wpr_simulation wpb_simple.launch
```

SLAM 建图：

```bash
roslaunch wpr_simulation wpb_gmapping.launch
```

导航：

```bash
roslaunch wpr_simulation wpb_navigation.launch
```

抓取演示：

```bash
roslaunch wpr_simulation wpb_table.launch
rosrun wpb_home_tutorials wpb_home_grab_client
```

---

## 6. image_pkg 实验说明

`image_pkg` 主要是围绕 RGB 图像、深度图、点云与目标距离估计做实验扩展。

### 6.1 话题约定

从脚本可见，默认订阅的 Kinect2 话题包括：

- `/kinect2/hd/image_color_rect`
- `/kinect2/sd/image_depth_rect`
- `/kinect2/sd/camera_info`

如果你的相机驱动使用了不同话题名，需要在 launch 或源码中调整参数。

### 6.2 点云坐标处理

在 `get_pointcloud.py` 与 `get_pointcloud3.py` 中，会将深度数据投影为 3D 点，并在颜色图和深度图之间做分辨率映射。发布的 `frame_id` 默认为：

```text
kinect2_camera_frame
```

### 6.3 颜色阈值检测

- `follow_node.py` 当前阈值配置更接近蓝色目标跟随
- `red_bottle_distance.py` 使用两段 HSV 红色阈值
- `green_bottle_move_distance.py` 使用绿色阈值，并叠加简单运动逻辑

因此在实际运行时，建议确认：

- 光照条件是否稳定
- Gazebo / 相机输出是否为彩色图像
- 颜色阈值是否与你当前场景一致

---

## 7. 实验资料目录说明

`Pictures/L4/` 中保留了与 RGB-D / 点云实验相关的资料：

- `fig01_gazebo_scene.png`：Gazebo 场景截图
- `fig02_rviz_pointcloud.png`：RViz 点云可视化截图
- `fig03_pcl_viewer.png`：PCL Viewer 可视化截图
- `实验4_RGBD点云与目标测距实验报告.pdf`：实验报告
- `检查点云话题和频率.png`：调试记录截图

这些文件用于帮助理解实验流程与结果，不参与构建。

---

## 8. 大文件说明

以下两个大文件**没有纳入 Git 仓库**：

- `Pictures/L4/实验4_图像与视频附件.zip`
- `Pictures/L4/手动，自动检测距离视频.mp4`

原因：

- ZIP 文件接近 GitHub 推荐上限
- MP4 文件超过 GitHub 100MB 硬限制，无法直接 push

如果需要分发这些文件，建议使用以下方式之一：

1. GitHub Releases
2. Git LFS
3. 网盘 / 对象存储
4. 在 README 中提供单独下载说明

---

## 9. 嵌套仓库说明

本仓库已经将原先位于 `src/` 下的嵌套 git 仓库整理为普通源码目录统一管理，避免 clone 后出现只有 gitlink、拿不到完整源码的问题。

涉及目录：

- `src/waterplus_map_tools`
- `src/wpb_home`
- `src/wpr_simulation`

这样处理后，克隆本仓库即可直接获得完整工作空间源码。

---

## 10. 常见问题

### 10.1 `catkin_make` 失败

建议检查：

- 是否已正确安装 ROS Noetic
- 是否已 `source /opt/ros/noetic/setup.bash`
- 各子包依赖是否已执行对应 `install_for_noetic.sh`

### 10.2 没有点云或深度无效

建议检查：

- Kinect2 驱动是否正常工作
- 深度图话题与相机内参话题是否存在
- 订阅话题名是否与当前系统一致
- RViz 的 Fixed Frame 是否设置为 `kinect2_camera_frame`

### 10.3 颜色检测效果差

建议检查：

- 图像是否为真正彩色输出
- HSV 阈值是否与实际目标颜色一致
- Gazebo 材质颜色是否偏移
- 是否存在强反光或阴影

---

## 11. 参考来源

本 README 汇总并复用了以下子项目中的现有说明：

- `src/waterplus_map_tools/README.md:1`
- `src/wpb_home/README.md:1`
- `src/wpr_simulation/README.md:1`
- `src/image_pkg/package.xml:1`
- `src/image_pkg/launch/*.launch`
- `src/image_pkg/scripts/*.py`

如果你要继续扩展功能，建议优先阅读各子包 README 与 launch 文件，再决定是新增节点还是复用现有能力。
