# 无人机避障仿真 - APF算法实现

## 项目概述

本项目使用RoboSense Airy激光雷达数据，实现了基于人工势场法(APF)的无人机避障仿真系统。系统模拟了无人机在复杂环境中的自主导航和避障能力，通过可视化界面实时展示无人机运动轨迹和环境感知。

## 代码逻辑

1. **数据获取**：
   - 订阅`/rslidar_points`话题获取RoboSense Airy雷达点云数据
   - 将ROS点云消息转换为PCL点云格式

2. **人工势场计算**：
   - **吸引力场**：从当前位置指向目标点的力
   - **斥力场**：从障碍物指向无人机的力
   - **合力计算**：吸引力与斥力的矢量和

3. **运动控制**：
   - 根据合力方向更新无人机位置
   - 限制最大飞行高度
   - 平滑运动轨迹

4. **可视化**：
   - 点云按强度值着色渲染
   - 显示无人机当前位置(绿色球体)
   - 显示目标点位置(蓝色球体)
   - 绘制运动轨迹和力向量

## 运行效果

### 仿真截图
![APF避障仿真截图](apf_simulation.png)

### 演示视频
[观看仿真视频](apf_simulation.mp4)

### 编译运行

```bash

git clone https://github.com/Nick-the-remaker/RoboSense_APF.git

# 编译
catkin_make

# 运行节点
source devel/setup.bash
rosrun APF_node APF_node
```

### 启动雷达

确保RoboSense Airy雷达已连接并运行：
```bash
roslaunch rslidar_sdk start.launch
```

## 参数调整

可在代码中调整以下参数：
- `k_attractive_`: 吸引力增益系数
- `k_repulsive_`: 斥力增益系数
- `safe_distance_`: 安全避障距离
- `step_size_`: 运动步长
- `max_height_`: 最大飞行高度
