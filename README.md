# miniSLAM

`miniSLAM` 是 Krisea 工作空间中的视觉/视觉惯性定位模块。当前主要运行方式是处理 ROS 2 bag 中的 RGB-D 与 IMU 数据，并通过 ROS 发布位姿和轨迹。

当前阶段采用“视觉优先”的建设策略：先把视觉观测、深度关联和视觉位姿估计做稳定，再引入 IMU 预积分与滑动窗口联合优化。

## 当前数据流

```text
ROS bag
  │ compressed RGB / Depth / Imu
  ▼
ROS adapter
  │ Frame / ImuData
  ▼
VioPlugin
  │ 流程调度与结果回调
  ▼
Estimator
  ├── FeatureTracker
  ├── RGB-D 3D-2D 对应
  └── PnP 位姿估计
  │
  ▼
PoseData
  │
  ▼
ROS pose / path
```

模块边界：

- `VioPlugin`：接收内部数据、调度估计器、输出 `PoseData`。
- `FeatureTracker`：维护长期特征轨迹、特征 ID、跟踪寿命和几何过滤。
- `FeatureManager`：后续用于管理同一特征在多帧中的观测。
- `Estimator`：负责视觉位姿、滑动窗口状态和后续视觉惯性优化。
- `RosAdapter`：只负责 ROS 消息转换、订阅和发布，不包含算法。

## 当前视觉链路的问题

旧实现采用“每两帧重新检测角点 + KLT + 单像素深度 + PnP”。主要风险包括：

1. 每帧重新检测全部角点，没有长期特征 ID 和跟踪寿命。
2. 特征可能集中在局部高纹理区域，空间分布不稳定。
3. 仅依赖 Forward-Backward 光流过滤，没有完整的几何外点剔除。
4. 直接读取单个深度像素，容易受到空洞、飞点和深度边缘影响。
5. PnP 虽然计算内点数量和内点率，但没有用它们拒绝坏位姿。
6. 当前是相邻帧位姿累计，没有多帧约束和滑动窗口优化。

## VINS 思路下的视觉优先路线

本项目使用 RGB-D，相比单目 VINS 可以直接获得部分路标深度，但仍采用 VINS 的长期特征轨迹、空间均匀化、多帧观测和滑动窗口思想。

参考：

- [VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono)
- [VINS-Mono FeatureTracker](https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/master/feature_tracker/src/feature_tracker.cpp)
- [VINS-Mono paper](https://arxiv.org/abs/1708.03852)

### 阶段 0：确认 RGB-D 数据契约

算法改进前必须确认：

- RGB 和 Depth 分辨率一致。
- Depth 已对齐到 RGB 光学坐标系。
- RGB 特征坐标可以直接访问同位置深度。
- 深度格式、单位和 `depth_scale` 正确。
- 相机内参对应当前 RGB 分辨率。
- RGB 与 Depth 时间戳差在同步阈值内。

如果深度没有对齐到 RGB，任何后续 PnP 或窗口优化都会建立在错误三维点上。

### 阶段 1：持续 FeatureTracker

FeatureTracker 每帧按以下顺序工作：

```text
跟踪上一帧已有点
  → 边界、LK 状态和误差过滤
  → Forward-Backward 检查
  → 基础矩阵 RANSAC
  → 按 track_count 优先保留长期点
  → mask 控制空间最小距离
  → 补充新特征并分配新 ID
```

每个特征至少维护：

```text
feature_id
track_count
previous_pixel
current_pixel
```

后续再增加归一化坐标、像素速度、深度及深度有效标志。

### 阶段 2：鲁棒深度关联

不再只读取单个深度像素。计划采用：

- `3 x 3` 或 `5 x 5` 邻域。
- 忽略零值和超出有效范围的深度。
- 使用中值降低飞点影响。
- 深度离散程度过大时拒绝该特征。
- 位于前后景深度跳变处的特征不参与 PnP。

### 阶段 3：视觉位姿质量门控

PnP 结果必须同时满足：

- 足够的 3D-2D 对应数量。
- 足够的 RANSAC 内点数。
- 内点率达到阈值。
- 重投影误差可接受。
- 单帧平移和旋转增量没有异常跳变。
- 位姿不包含 NaN 或 Inf。

拒绝坏位姿时保持上一有效位姿，不允许异常增量继续累计。

### 阶段 4：FeatureManager

按特征 ID 保存多帧观测：

```text
FeatureTrack
  ├── frame 10: normalized point + depth
  ├── frame 11: normalized point + depth
  └── frame 12: normalized point + depth
```

FeatureManager 是从相邻帧 PnP 过渡到多帧估计的边界。

### 阶段 5：视觉滑动窗口

建立固定长度窗口，联合优化窗口内相机位姿和路标：

```text
F0 - F1 - F2 - ... - F9
```

RGB-D 深度首先作为路标深度先验，优化目标以多帧重投影误差为主。纯视觉窗口稳定后，再加入速度、重力、IMU 零偏和预积分因子。

## 初始参数建议

```yaml
feature_tracker:
  max_features: 200
  min_distance: 30.0
  quality_level: 0.01
  block_size: 7
  lk_window_size: 21
  pyramid_levels: 3
  fb_threshold: 0.8
  ransac_threshold: 1.0
  ransac_confidence: 0.99

depth:
  min_meters: 0.2
  max_meters: 8.0
  window_size: 3
  max_deviation: 0.1

pnp:
  min_correspondences: 30
  min_inliers: 20
  min_inlier_ratio: 0.5
  max_reprojection_error: 2.0
  max_translation_per_frame: 0.5
  max_rotation_degrees_per_frame: 20.0
```

阈值需要用实际 ROS bag 统计结果调整，不能直接视为最终标定值。

## 阶段验收指标

视觉前端第一阶段建议观察：

```text
有效特征数             100～200
中位跟踪寿命           大于 5 帧
几何 RANSAC 内点率     大于 60%
有效深度特征           大于 30
PnP 内点数             大于 20
PnP 中位重投影误差     小于 2 px
静止时位姿             无明显漂移
连续运动时             无单帧巨大跳变
```

日志至少记录：

```text
tracked / rejected / newly detected
平均或中位跟踪寿命
有效深度数量
PnP matches / inliers / inlier ratio
平移增量 / 旋转增量 / 重投影误差
```

## ROS bag 主回归流程

当前 ROS bag 中图像为压缩格式，需要先解压成节点订阅的 raw 话题。确认以下输入持续发布：

```bash
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
ros2 topic hz /camera/gyro_accel/sample
```

检查输出：

```bash
ros2 topic hz /mini_slam/pose
ros2 topic hz /mini_slam/path
```

RViz 配置：

```text
Fixed Frame: map
Pose: /mini_slam/pose
Path: /mini_slam/path
```

建议播放 bag 时启用时钟：

```bash
ros2 bag play <bag_path> --clock --loop
rviz2 --ros-args -p use_sim_time:=true
```

## 编译

在工作空间根目录执行：

```bash
cd /home/liuxz/workspace/krisea_ws
cmake -S . -B build
cmake --build build -j$(nproc)
```

只编译视觉算法库：

```bash
cmake --build build --target mini_slam_vio -j$(nproc)
```

函数功能描述格式
/**
 * @brief   把特征点放入feature的list容器中，计算每一个点跟踪次数和它在次新帧和次次新帧间的视差，返回是否是关键帧
 * @param[in]   frame_count 窗口内帧的个数
 * @param[in]   image 某帧所有特征点的[camera_id,[x,y,z,u,v,vx,vy]]s构成的map,索引为feature_id
 * @param[in]   td IMU和cam同步时间差
 * @return  bool true：次新帧是关键帧;false：非关键帧
*/