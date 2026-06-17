# Base 目录 VO 程序详解

本文档面向“想把 `Base/My_VO.cpp` 真正读懂”的读者。目标不是讲一本视觉 SLAM 教材，而是帮助你顺着当前工程里的 C++ 实现，理解程序从启动、初始化、逐帧跟踪、位姿恢复、短时地图构建到输出轨迹的完整过程。

源码对应关系：

- 核心实现：`Base/My_VO.cpp`
- 接口与数据结构声明：`Base/My_VO.h`
- 构建入口：`Base/CMakeLists.txt`

---

## 1. 程序目标与整体定位

这份程序实现的不是完整意义上的 SLAM，而是一个以单目视觉里程计（Monocular Visual Odometry, VO）为主干的程序。它的核心目标是：

1. 从连续灰度图像中跟踪特征点。
2. 利用两帧之间的极几何关系恢复相机相对运动。
3. 将逐帧相对运动累积成整段轨迹。
4. 在条件合适时，利用短时三角化得到的 3D 点和 PnP 进行辅助修正。
5. 输出可用于 `evo` 评估的 TUM 格式轨迹，以及一张轨迹对比图。

更准确地说，这个程序的定位是：

- 主干仍然是 2D-2D 几何，即通过 `findEssentialMat` + `recoverPose` 从两帧匹配点恢复相对位姿。
- 三角化和 PnP 不是主干，而是“短时辅助模块”。
- 绝对尺度不是从算法本身恢复，而是直接使用真值轨迹中相邻帧的距离。

因此，如果只看结果，这段程序像是一个“带真值尺度辅助的单目 VO”。如果从架构上看，它又比最简单的两帧 VO 多了一层“短寿命地图 + PnP 约束”的增强机制。

程序依赖关系也很清楚：

- 图像输入来自 `Kitti/left/%06d.png`
- 相机内参来自 `Kitti/camera-info.txt`
- 真值轨迹来自 `Kitti/gt-tum07.txt`
- 核心算法库使用 OpenCV
- 输出轨迹格式遵循 TUM，可直接给 `evo_ape`、`evo_rpe`、`evo_traj` 使用

---

## 2. 目录与文件角色

先理解文件角色，再读代码会轻松很多。

### 2.1 `Base/My_VO.cpp`

这是主程序实现文件，包含：

- 路径管理和命令行参数解析
- 相机内参与真值轨迹读取
- FAST 特征检测
- LK 光流跟踪
- 本质矩阵估计与 `recoverPose`
- 三角化与短时地图点筛选
- PnP 位姿求解和一致性检查
- 轨迹输出和可视化
- `main` 主循环

可以把它理解成“所有逻辑都在一个文件里串起来”的实现。

### 2.2 `Base/My_VO.h`

头文件中主要放：

- 数据结构定义
- 对外声明的函数接口
- OpenCV 头文件依赖

它像一张“程序功能目录”，非常适合在正式读 `cpp` 之前快速扫一遍。

### 2.3 `Base/CMakeLists.txt`

这是构建入口，作用比较简单：

- 指定 C++17
- 查找 OpenCV 4
- 编译 `My_VO.cpp`
- 生成可执行文件 `VO`

它不涉及算法，但能帮助你确认程序只有一个主可执行入口，没有复杂的多模块链接关系。

### 2.4 `Kitti/`

这是运行时依赖的数据目录，当前程序默认需要：

- `Kitti/camera-info.txt`
- `Kitti/gt-tum07.txt`
- `Kitti/left/%06d.png`

这里虽然目录名叫 `Kitti`，但真值轨迹和输出接口都采用了 TUM 风格的数据表达。

### 2.5 `output/`

这是程序默认输出目录。

完整运行时默认写入：

- `output/full_run/position/position.txt`
- `output/full_run/position/geometry_position.txt`
- `output/full_run/traj/map2.png`

如果通过 `--output-root` 指定了输出根目录，则上述文件会写到新的根目录下。

---

## 3. 先建立读代码的最小知识框架

在读这份 VO 代码之前，先建立 4 个最小概念。

### 3.1 相机内参（Camera Intrinsics）

针孔相机内参包括：

- `fx`、`fy`：水平方向和垂直方向焦距，单位是像素
- `cx`、`cy`：主点坐标，单位是像素

代码里用 `CameraIntrinsics` 保存这些值，再由 `buildCameraMatrix()` 转成 OpenCV 常用的相机矩阵：

```text
K = [ fx  0  cx
      0  fy  cy
      0   0   1 ]
```

这个矩阵会被用于：

- `findEssentialMat`
- `recoverPose`
- 像素坐标到归一化坐标的转换
- 三角化
- PnP 重投影

### 3.2 特征点与光流跟踪

程序没有使用 ORB/SIFT 这类描述子匹配，而是采用：

1. 在某一帧上用 FAST 检测角点。
2. 用 LK 金字塔光流把这些点从上一帧跟踪到当前帧。
3. 再做一次反向跟踪，检查前后向一致性（forward-backward check）。

因此，程序的匹配关系不是“描述子匹配”，而是“时序光流跟踪”。

### 3.3 相对位姿

对连续两帧来说，程序首先恢复的是“上一帧相机到当前帧相机”的相对运动。它包含：

- 旋转 `R`
- 平移方向 `t`

注意，这里的 `t` 只有方向，没有真实长度。单目两帧几何本身不能恢复绝对尺度，所以代码必须额外调用 `getAbsoluteScale()` 用真值求相邻帧位移长度。

### 3.4 世界位姿

程序不是只保留两帧相对运动，而是将它们累积成“相机在世界坐标系中的姿态”。这里用 `PoseRecord` 表示一帧绝对位姿：

- `rotation`：`R_cw`，相机坐标系到世界坐标系的旋转
- `translation`：`t_cw`，相机中心在世界坐标系中的位置

这一点非常重要。很多 OpenCV 接口使用的是世界到相机表示：

```text
X_cam = R_wc * X_world + t_wc
```

但 `PoseRecord` 内部存的是相反方向：

- `R_cw`
- `t_cw`

所以代码里才会频繁出现：

- `poseToWorldToCamera()`
- `poseFromPnP()`
- `relativeMotionFromPoses()`

这几类“坐标变换函数”。

### 3.5 为什么 `PoseRecord` 特别容易读乱

因为程序里同时存在三种层面：

1. `recoverPose()` 输出的相对运动
2. `PoseRecord` 表示的绝对世界位姿
3. PnP 所需的世界到相机外参

如果不先记住 `PoseRecord` 存的是 `R_cw` 和 `t_cw`，后面读 PnP 和投影矩阵那部分会非常痛苦。

---

## 4. 输入、输出与运行方式

### 4.1 输入文件

程序运行时依赖三类输入：

#### 相机内参

文件：`Kitti/camera-info.txt`

代码通过 `loadCameraIntrinsics()` 读取下面 4 个字段：

- `Camera.fx:`
- `Camera.fy:`
- `Camera.cx:`
- `Camera.cy:`

#### 图像序列

路径模板：`Kitti/left/%06d.png`

代码通过 `formatImagePath()` 将帧号格式化成六位数字，例如：

- `frame_id = 7` 对应 `000007.png`

图像以灰度模式读取，函数是 `readImageOrEmpty()`。

#### 真值轨迹

文件：`Kitti/gt-tum07.txt`

这是 TUM 风格轨迹，每一行形如：

```text
timestamp tx ty tz qx qy qz qw
```

程序分别读取：

- `loadGroundTruthPositions()`：提取平移位置
- `loadGroundTruthTimestamps()`：提取时间戳

位置主要用于：

- 恢复单目绝对尺度
- 绘制真值轨迹

时间戳主要用于：

- 按 TUM 格式输出估计轨迹

### 4.2 输出文件

程序最终会写出三类结果。

#### 最终输出轨迹

文件：`position.txt`

对应变量：

- `trajectory`
- `geometry_pose`

这是“最终对外发布”的轨迹，也是默认给 `evo` 评估的轨迹。

#### 几何主干轨迹

文件：`geometry_position.txt`

对应变量：

- `geometry_trajectory`
- `two_view_pose`

这条轨迹是“纯 `recoverPose` 2D-2D 主干”的诊断轨迹，用来和最终输出主干对照。

#### 轨迹对比图

文件：`map2.png`

图中：

- 黑色点表示真值轨迹
- 绿色点表示 VO 估计轨迹

绘制发生在 `renderTraceCanvas()` 中，轨迹画在 X-Z 平面上。

### 4.3 命令行参数

`printUsage()` 给出了三种主要用法：

```bash
./build/VO [frame_count]
./build/VO --start-frame N --end-frame M [--output-root PATH]
./build/VO --start-frame N --frame-count K [--output-root PATH]
```

参数语义如下：

- 位置参数 `frame_count`：从起始帧开始处理多少帧
- `--start-frame`：指定起始帧
- `--end-frame`：指定结束帧，闭区间
- `--frame-count`：从起始帧起处理 K 帧
- `--output-root`：指定输出根目录

参数解析由 `parseRunOptions()` 完成，范围校验由 `parseNonNegativeInt()` 和 `main` 中的逻辑负责。

### 4.4 环境变量

程序里有四个运行时环境变量值得注意。

#### `VO_ENABLE_GUI`

- `0`：强制关闭 OpenCV 窗口
- 其他值或未设置：程序自动判断当前环境是否能连上 X11/Wayland 显示服务

相关函数：

- `isVisualizationEnabled()`
- `isAccessibleX11Display()`
- `isAccessibleWaylandDisplay()`

#### `VO_VERBOSE_LOG`

- `1`：打印详细日志
- 默认：只打印关键进度

相关函数：

- `isVerboseLoggingEnabled()`

#### `VO_ENABLE_PNP`

- `0`：关闭 PnP 辅助
- 默认：开启 PnP 辅助逻辑

相关函数：

- `isPnPEnabled()`

#### `VO_IMAGE_SCALE`

- 允许取值 `(0, 1]`
- 默认值 `1.0`

作用：

- 读图后按比例缩放图像
- 同时缩放相机内参，保持像素坐标模型一致

相关函数：

- `getImageScale()`
- `scaleCameraIntrinsics()`
- `readAndResizeImage()`

---

## 5. 数据结构与全局阈值

### 5.1 数据结构

### `CameraIntrinsics`

作用：保存针孔相机内参。

字段：

- `fx`
- `fy`
- `cx`
- `cy`

主流程中的位置：

- 初始化时读取
- 构造 `camera_matrix`
- 提供 `findEssentialMat`、`recoverPose`、三角化和 PnP 所需的相机模型

### `ProjectPaths`

作用：集中保存输入输出路径。

字段：

- `project_root`
- `camera_info_path`
- `left_image_pattern`
- `gt_pose_path`
- `output_pose_path`
- `output_geometry_pose_path`
- `output_map_path`

主流程中的位置：

- 由 `makeProjectPaths()` 构造
- 后续所有输入输出都从这里取路径

### `PoseRecord`

作用：保存一帧相机在世界坐标系中的绝对姿态。

字段：

- `rotation`
- `translation`

语义：

- `rotation` 是 `R_cw`
- `translation` 是 `t_cw`

主流程中的位置：

- `geometry_pose`
- `two_view_pose`
- `keyframe_pose`
- `trajectory`
- `geometry_trajectory`

### `RunOptions`

作用：保存命令行传入的运行区间和输出目录配置。

字段：

- `start_frame`
- `end_frame`
- `frame_count`
- `output_root_override`

主流程中的位置：

- `main` 开头解析参数时使用

### 5.2 全局阈值

这些常量都定义在 `My_VO.cpp` 顶部匿名命名空间中。它们控制了整套程序的行为风格。

### A. 特征检测与跟踪

- `kTargetFeatureCount = 2000`
  - 每次重检测最多保留 2000 个 FAST 特征点。
- `kMinTrackedFeatures = 1000`
  - 当前跟踪点少于该值时，会在当前帧重新检测特征。
- `kMaxForwardBackwardError = 1.0f`
  - LK 光流前后向一致性误差阈值，越小越严格。

### B. 位姿估计

- `kMinInitialPoseInliers = 5`
  - 初始化阶段 `recoverPose` 的最低内点数要求。
- `kMinPoseInliers = 20`
  - 正常主循环中 `recoverPose` 的最低内点数要求。
- `kMinTranslationScale = 0.1`
  - 相邻真值位移过小则跳过平移更新，避免噪声主导。

### C. 图像与进度显示

- `kDefaultImageScale = 1.0`
  - 默认不缩放图像。
- `kTraceCanvasSize = 1000`
  - 轨迹画布大小。
- `kProgressReportInterval = 50`
  - 进度打印间隔。
- `kTracePadding = 40.0`
  - 轨迹图边距。

### D. 三角化与地图点质量控制

- `kMinTriangulatedLandmarks = 4`
  - 允许三角化的最低匹配数量。
- `kTriangulationReprojectionError = 5.0`
  - 三角化点在两帧中的重投影误差阈值。
- `kMinTriangulationParallaxPixels = 5.0`
  - 允许三角化的最低中位视差。
- `kMinTriangulationBaseline = 0.1`
  - 允许三角化的最低真实基线。
- `kMinLandmarkDepth = 0.1`
  - 地图点最小深度。
- `kMaxLandmarkDepth = 80.0`
  - 地图点最大深度。

### E. PnP 求解与一致性检查

- `kMinPnPInliers = 12`
  - `solvePnPRansac` 的最低内点要求。
- `kPnPReprojectionError = 4.0f`
  - PnP RANSAC 重投影误差阈值。
- `kMinKeyframePnPInliers = 30`
  - 认为一批 keyframe 地图点足够支持稳定 PnP 的最低数量。
- `kPreferredKeyframePnPInliers = 80`
  - 允许 PnP 真正接管最终主干时更偏好的内点数。
- `kMaxKeyframeTrackAge = 20`
  - 短时地图或其观测被连续跟踪的最长寿命。
- `kMaxPnPRotationDeltaDegrees = 8.0`
  - PnP 位姿与 2D-2D 主干位姿之间允许的最大旋转差。
- `kMaxPnPTranslationDeltaScale = 3.0`
  - PnP 位姿与 2D-2D 主干位姿之间允许的最大平移差尺度因子。
- `kTrustedPnPRotationDeltaDegrees = 1.5`
  - PnP 真正接管主干时要求更严格的旋转一致性。
- `kTrustedPnPTranslationDeltaScale = 0.75`
  - PnP 真正接管主干时要求更严格的平移一致性。
- `kMaxOutputPnPScale = 1.0`
  - 仅在小尺度运动时允许 PnP 介入最终主干。
- `kMinOutputPnPKeyframeAge = 1`
  - 地图至少存在一定年龄后，才允许 PnP 接管主干。
- `kMinOutputPnPRotationDegrees = 0.0`
  - 当前配置下只要满足其他条件，不额外要求最小旋转角。
- `kMaxOutputPnPTrackedFeatures = 2200`
  - 当 2D-2D 跟踪点过多时，不让 PnP 抢主导。

这些阈值共同体现了一个设计取向：程序信任 2D-2D 主干，把 PnP 视为保守的候选接管者，而不是默认位姿来源。

---

## 6. 程序启动阶段

这一阶段对应 `main()` 开头，作用是把所有运行前提准备好。

### 6.1 解析命令行参数

入口函数先调用：

- `parseRunOptions()`

它会解析起始帧、结束帧、帧数和输出目录，并处理：

- `--help`
- 非法整数
- 缺失参数值
- 多个位置参数冲突

如果解析失败，程序会打印错误并退出。

### 6.2 定位工程根目录

接着调用：

- `makeProjectPaths()`

而它内部会先调用：

- `findProjectRoot()`

这个函数会从当前工作目录开始向上找，直到同时发现：

- `Kitti`
- `Base`

这样做的目的是允许你从项目根目录运行，也允许从 `build/` 目录运行，而不用手工切换路径。

### 6.3 读取相机内参

函数：

- `loadCameraIntrinsics()`

如果 `fx` 或 `fy` 读取失败，程序直接退出。因为没有正确内参，本质矩阵估计和投影几何都失去基础。

### 6.4 读取图像缩放配置并同步调整内参

程序会：

1. 调用 `getImageScale()` 读取 `VO_IMAGE_SCALE`
2. 调用 `scaleCameraIntrinsics()` 缩放内参
3. 调用 `buildCameraMatrix()` 构造 `camera_matrix`

这一步很关键。因为如果图像被缩小，但内参不缩小，那么像素坐标和相机模型就不一致，后面的几何估计会明显出错。

### 6.5 读取真值轨迹和时间戳

函数：

- `loadGroundTruthPositions()`
- `loadGroundTruthTimestamps()`

程序随后会取这两者长度的较小值作为可用帧数，避免后续访问越界。

### 6.6 确定处理帧区间

`main` 中后续几段逻辑会根据：

- `start_frame`
- `end_frame`
- `frame_count`

推导出：

- `segment_start_frame`
- `segment_end_frame`
- `segment_frame_count`

这里的约束是：

- 选中的区间至少要包含两帧
- 结束帧不能超出真值长度
- `frame_count` 和 `end_frame` 同时给出时，会取更短的范围

为什么要求至少两帧？因为初始化必须用前两帧先建立相对位姿。

---

## 7. 初始化阶段（前两帧）

初始化阶段用的是：

- `segment_start_frame`
- `segment_start_frame + 1`

这两帧的作用不是普通逐帧更新，而是为整个主循环建立“第一份可用状态”。

### 7.1 读取前两帧图像

函数：

- `readAndResizeImage()`

这一步会读图，并在需要时执行缩放。

如果任意一帧读取失败，程序直接退出，因为后续状态无法初始化。

### 7.2 在第一帧检测特征

函数：

- `featureDetection()`

实现上使用：

- FAST 角点检测
- `nth_element` 保留响应值最高的一部分点
- `cornerSubPix` 做亚像素优化

初始化后：

- `points1` 保存第一帧特征点

### 7.3 用光流跟踪到第二帧

函数：

- `featureTracking()`

输出后：

- `points1` 仍表示第一帧中的点
- `points2` 表示这些点在第二帧中的对应位置

同时会做四类过滤：

- 正向跟踪失败
- 反向跟踪失败
- 跟踪点越界
- 前后向误差过大

如果最后点数太少，程序直接退出。

### 7.4 估计本质矩阵并恢复相对位姿

初始化阶段主干是：

1. `findEssentialMat(points1, points2, focal_length, principal_point, RANSAC, ...)`
2. `recoverPose(...)`

你可以把它理解成：

- `findEssentialMat`：从匹配点中估计两帧满足的极几何关系
- `recoverPose`：在相机内参已知的前提下，从本质矩阵中恢复旋转和单位平移方向

代码中的 `mask` 表示最终被认为是几何内点的匹配。

随后程序会调用：

- `compactPointsByMask()`

把外点从 `points1` 和 `points2` 中同步删掉。

### 7.5 用真值恢复初始尺度

函数：

- `getAbsoluteScale(gt_positions, segment_start_frame + 1)`

这会计算起始两帧真值位置之间的欧氏距离，作为初始化这一步真实平移长度。

这一步是整个程序的关键事实之一：

- `recoverPose` 给出的是方向
- 真值给出的是长度

因此最终初始化平移不是“纯视觉恢复的尺度”，而是“视觉方向 + 真值长度”。

### 7.6 建立最终主干与诊断支路

初始化后程序同时建立：

- `geometry_pose = poseFromInitialRelativeMotion(...)`
- `two_view_pose = geometry_pose`

这两者在初始化时数值相同，但后续语义不同。

它们的区别先提前记住一句话：

- `geometry_pose`：最终对外发布的主干位姿
- `two_view_pose`：纯 2D-2D / `recoverPose` 诊断支路

后面主循环会继续维持这两条线并行存在：

- `geometry_pose` 可以在严格门控通过时被 PnP 整个位姿接管
- `two_view_pose` 始终只保留纯 2D-2D 积分结果

### 7.7 初始化短时地图

如果开启 PnP，程序会尝试基于初始化两帧做第一批三角化：

1. 判断是否满足三角化条件：`shouldTriangulateTracks()`
2. 真正执行三角化和筛点：`triangulateWorldLandmarks()`

三角化成功后会得到：

- `tracked_landmarks`

也就是第一批短寿命 3D 地图点。

如果失败：

- 程序不会退出
- 只是暂时没有 PnP 可用地图
- 后续主循环会在基线和视差合适时重建

### 7.8 初始化 keyframe 状态与轨迹容器

初始化结束后，程序会建立：

- `prev_image = img_2`
- `prev_features = init_curr_features`
- `keyframe_image = img_2`
- `keyframe_features = init_curr_landmark_features`
- `keyframe_landmarks = tracked_landmarks`
- `keyframe_pose = geometry_pose`

以及轨迹容器：

- `trajectory`
- `geometry_trajectory`
- `trajectory_frame_ids`

这意味着：从主循环开始时，上一帧状态已经是“第二帧”。

---

## 8. 主循环逐帧流程

这是全文最重要的一节。`main` 中的主循环从第三帧开始：

```text
for frame_id = segment_start_frame + 2 ... segment_end_frame
```

每一帧基本都按下面的固定顺序处理。

### 8.1 预读图像

程序使用 `std::async` 启动下一帧读图任务，这样当前帧在做光流和位姿估计时，后台线程可以提前把下一张图读到内存里。

对应变量：

- `next_frame_future`
- `curr_image`

这一步是纯工程优化，与几何本身无关，但能降低 IO 阻塞。

### 8.2 2D-2D 光流跟踪

程序优先尝试从：

- `prev_image`
- `prev_features`

跟踪到：

- `curr_image`
- `curr_features`

函数：

- `featureTracking()`

这里得到的是“上一帧到当前帧”的 2D-2D 匹配，是整个主干位姿恢复的基础。

### 8.3 特征不足时重检测

如果当前可用匹配点太少，程序会：

1. 在 `prev_image` 上重新做一次 FAST 检测
2. 再重新跟踪到 `curr_image`

如果重新检测后仍不足以支持稳定几何恢复：

- 本帧位姿不更新
- 但会把上一状态写入轨迹，保持帧号对齐
- 然后在当前帧重新检测特征，为下一帧恢复做准备

这是一个典型的“保守跳帧”策略：宁可少更新，也不强行用低质量匹配估计错误位姿。

### 8.4 本质矩阵与 `recoverPose`

一旦 2D-2D 匹配数量足够，程序就进入主干位姿恢复：

1. `findEssentialMat(prev_features, curr_features, ...)`
2. `recoverPose(...)`

输出得到：

- `rotation`
- `translation`
- `mask`
- `inlier_count`

含义是：

- `rotation`：上一帧到当前帧的相对旋转
- `translation`：上一帧到当前帧的单位平移方向
- `mask`：几何内点

如果 `inlier_count < kMinPoseInliers`：

- 当前帧位姿被认为不可靠
- 程序保留上一帧 pose
- 重新在当前帧检测特征

### 8.5 内点筛选

一旦位姿恢复成功，程序调用：

- `compactPointsByMask(prev_features, curr_features, mask)`

将所有与 `recoverPose` 解不一致的匹配删掉。

这样做的目的有两个：

1. 让后续三角化和 PnP 使用更干净的观测
2. 让下一轮继续跟踪的特征质量更稳定

### 8.6 真值尺度恢复

函数：

- `getAbsoluteScale(gt_positions, frame_id)`

得到当前帧相对于上一帧的真实位移长度 `scale`。

如果 `scale <= kMinTranslationScale`：

- 认为这一帧真实移动太小
- 跳过位姿更新

为什么要这样做？因为在几乎静止或极小位移情况下，`recoverPose` 的平移方向会非常容易被噪声放大，强行积分反而会让轨迹更差。

### 8.7 更新最终主干与 2D-2D 诊断支路

一旦当前帧 2D-2D 位姿和尺度都可用，程序会先构造两份基线结果：

- `base_two_view_pose = poseFromRelativeMotion(two_view_pose, rotation, translation, scale)`
- `base_geometry_pose = poseFromRelativeMotion(geometry_pose, rotation, translation, scale)`

这里非常关键：

- `base_two_view_pose` 是纯 2D-2D 支路当前帧的结果
- `base_geometry_pose` 是最终主干在“不采用 PnP 时”的默认结果
- 两者都使用 `poseFromRelativeMotion()`，位姿语义自洽
- 区别不再是“积分公式不同”，而是“是否允许 PnP 接管”

接着程序先设定：

- `estimated_geometry_pose = base_geometry_pose`

意思是：如果后面 PnP 没有通过，就完全使用 `base_geometry_pose` 作为最终输出主干。

### 8.8 尝试 PnP 修正

如果满足以下前提，程序会尝试 PnP：

- `VO_ENABLE_PNP` 没被关闭
- 已经存在 `keyframe_image`
- 已经存在 `keyframe_features`
- 已经存在 `keyframe_landmarks`
- 当前 keyframe 跟踪年龄和地图年龄没超过 `kMaxKeyframeTrackAge`

处理流程如下。

#### 第一步：将 keyframe 地图点跟踪到当前帧

函数：

- `featureTrackingWithLandmarks()`

输入：

- keyframe 图像中的 2D 点
- 对应 3D 地图点

输出：

- 当前帧中的 2D 观测
- 过滤后的 3D 地图点

这里和普通 `featureTracking()` 的差别是：它不仅跟踪 2D 点，还要同步维护 3D-2D 对应关系。

#### 第二步：用 3D-2D 对应求解 PnP

函数：

- `solvePoseWithPnP()`

内部步骤是：

1. 用 `solvePnPRansac` 去除外点
2. 只保留内点
3. 再用 `solvePnP(..., SOLVEPNP_ITERATIVE)` 进行 refine

PnP 初值来自：

- `poseToPnPGuess(base_geometry_pose, ...)`

这说明 PnP 不是“盲求”，而是以当前 2D-2D 主干位姿为初值，在局部优化。

#### 第三步：检查 PnP 与主干是否一致

函数：

- `isPnPPoseConsistent()`

它会比较：

- PnP 位姿和平面 2D-2D 主干位姿之间的平移差
- 旋转角差

只有通过一致性检查，PnP 才会进入下一步。

#### 第四步：决定 PnP 是否真的接管最终主干

函数：

- `shouldAdoptPnPPose()`

这里门控更严格。即使 PnP 位姿是可解且一致的，也不一定真的会接管最终主干。代码当前策略非常保守，只有在下面条件都较好时才会介入：

- 当前帧尺度不大
- PnP 内点足够多
- PnP 和主干结果差异很小
- 地图存在时间足够
- 当前 2D-2D 跟踪点没有多到“完全不需要 PnP”

如果门控通过：

- `estimated_geometry_pose = pnp_pose`

这里有一个非常重要的结论：

- PnP 现在不是“只改旋转”
- 而是在通过门控时，直接用自己的绝对位姿接管当前主干
- 也就是说，旋转和平移会一起生效

这说明程序把 PnP 当成“可选主干接管者”，但前提是它必须和 2D-2D 主干高度一致。

### 8.9 尝试重建 keyframe 地图

如果当前短时地图不存在、点数不够或者寿命过长，程序会尝试基于当前 2D-2D 主干匹配重新三角化一批 3D 点。

主要判断函数：

- `shouldTriangulateTracks()`

真正构建函数：

- `triangulateWorldLandmarks()`

成立条件大致包括：

- 匹配数足够
- 基线足够
- 中位视差足够
- 三角化点深度合理
- 两帧重投影误差足够小

如果重建成功：

- `keyframe_image = curr_image`
- `keyframe_features = triangulation_curr_features`
- `keyframe_landmarks = rebuilt_landmarks`
- `keyframe_pose = estimated_geometry_pose`
- `keyframe_frame_id = frame_id`
- `keyframe_map_frame_id = frame_id`

这意味着“当前帧成为新的 keyframe，当前帧也是这批短时地图的出生帧”。

### 8.10 记录轨迹、可视化并进入下一帧

在本帧处理末尾，程序会：

1. 更新当前状态：
   - `two_view_pose = base_two_view_pose`
   - `geometry_pose = estimated_geometry_pose`
2. 将结果写入轨迹容器：
   - `trajectory.push_back(geometry_pose)`
   - `geometry_trajectory.push_back(two_view_pose)`
   - `trajectory_frame_ids.push_back(frame_id)`
3. 若当前跟踪点过少，则在当前帧重新检测特征
4. 若开启 GUI，则显示：
   - `curr_image`
   - `renderTraceCanvas()` 生成的轨迹图
5. 将：
   - `prev_image = curr_image`
   - `prev_features = curr_features`

至此，一帧处理结束，进入下一轮循环。

---

## 9. 一帧处理时序图（文字版）

下面用文字顺序图把主循环再压缩一遍，适合在读源码时快速对照。

### 标准成功路径

1. 取当前帧图像 `curr_image`
2. 用 `prev_image` 和 `prev_features` 跟踪出 `curr_features`
3. 若特征不足，先重检测再重跟踪
4. 用 `prev_features` 和 `curr_features` 估计本质矩阵
5. 用 `recoverPose` 恢复相对位姿
6. 用 `mask` 保留几何内点
7. 从真值读取当前帧绝对尺度 `scale`
8. 计算 `base_geometry_pose`
9. 计算 `base_two_view_pose`
10. 如果短时地图可用，则尝试 PnP
11. 如果地图不足，则尝试重新三角化新地图
12. 更新 `geometry_pose` 和 `two_view_pose`
13. 记录 `trajectory`、`geometry_trajectory`
14. 必要时重检测 FAST 特征
15. 可视化并进入下一帧

### 失败分支

如果任何一个关键步骤失败，程序通常采取保守策略：

- 跟踪点不够：重检测，必要时本帧不更新 pose
- `recoverPose` 内点太少：本帧不更新 pose
- 真值尺度太小：本帧不更新平移
- PnP 失败：继续使用 2D-2D 主干
- 三角化失败：不退出，只是暂时没有新地图

程序整体不是“每帧都必须成功”，而是“尽可能保持轨迹连续，并在失败时平稳退回主干策略”。

---

## 10. 变量追踪表

读 `main` 的时候，最容易被绕晕的不是函数名，而是状态变量在不同阶段代表谁。下面按变量语义整理。

| 变量名 | 含义 | 典型时刻代表什么 |
|---|---|---|
| `prev_image` | 主干上一帧图像 | 当前循环要从它跟踪到 `curr_image` |
| `curr_image` | 主干当前帧图像 | 当前正在处理的图像 |
| `prev_features` | `prev_image` 上的主干特征 | 将用于 `findEssentialMat` 的前一帧点 |
| `curr_features` | `curr_image` 上的主干特征 | 与 `prev_features` 一一对应 |
| `geometry_pose` | 上一时刻最终对外主干位姿 | 下一帧计算 `base_geometry_pose` 的起点 |
| `two_view_pose` | 上一时刻纯 2D-2D 诊断位姿 | 下一帧计算 `base_two_view_pose` 的起点 |
| `base_geometry_pose` | 当前帧按 2D-2D 主干积分得到的默认主干位姿 | PnP 一致性检查的参考值 |
| `estimated_geometry_pose` | 当前帧最终采用的主干位姿 | 可能是 `base_geometry_pose`，也可能是 `pnp_pose` |
| `keyframe_image` | 当前短时地图参考图像 | PnP 时从它把观测跟踪到当前帧 |
| `keyframe_features` | `keyframe_image` 上的 2D 特征 | 与 `keyframe_landmarks` 一一对应 |
| `keyframe_landmarks` | 当前短时地图的 3D 点 | PnP 的 3D 输入 |
| `keyframe_pose` | keyframe 对应几何位姿 | 地图语义上的参考姿态 |
| `keyframe_frame_id` | 当前 keyframe 观测的最近帧号 | 用来计算跟踪年龄 |
| `keyframe_map_frame_id` | 这批地图的出生帧号 | 用来限制地图寿命 |
| `trajectory` | 最终输出轨迹序列 | 最终写入 `position.txt` |
| `geometry_trajectory` | 纯 2D-2D 诊断轨迹序列 | 最终写入 `geometry_position.txt` |
| `trajectory_frame_ids` | 每个轨迹条目对应的数据集帧号 | 用来对齐真值时间戳 |

重点记忆：

- `prev_*` / `curr_*` 是 2D-2D 主干状态
- `keyframe_*` 是短时地图与 PnP 状态
- `geometry_pose` / `trajectory` 是最终对外主干
- `two_view_pose` / `geometry_trajectory` 是纯 2D-2D 诊断轨迹

---

## 11. PnP 与短时地图机制

这是当前程序相对“基础版单目 VO”最有辨识度的部分。

### 11.1 为什么在 2D-2D 主干之外还需要 PnP

纯 2D-2D VO 的优点是：

- 不需要维护长期地图
- 两帧之间就能恢复相对运动
- 实现简单

但它也有局限：

- 只依赖两帧局部几何，容易受短时误匹配影响
- 恢复的是相对运动，不直接利用已知 3D 结构

如果能从前面的帧里三角化出一批可靠 3D 点，再在当前帧观察到它们，那么就可以建立 3D-2D 对应，并通过 PnP 对当前姿态形成额外约束。

因此，这段代码的思路不是“用 PnP 替代 2D-2D”，而是：

- 以 2D-2D 为主干
- 在几何条件合适时生成一批短时地图点
- 在后续若干帧用这些点做 PnP 辅助

### 11.2 短时地图是如何生成的

地图生成基于两帧主干内点：

1. 当前两帧已经通过 `recoverPose` 得到相对姿态
2. 基线和视差满足阈值
3. 调用 `triangulateWorldLandmarks()`

这个函数内部做了几层筛选：

- 三角化结果必须是有限值
- 点必须在参考相机前方
- 深度必须在合理范围内
- 在前一帧和当前帧上的重投影误差都要足够小

最终保留下来的 3D 点会被转换到世界坐标系中保存。

### 11.3 为什么说它是“短时地图”

因为这批地图点不会长期维护，也不会形成全局地图结构。代码只让它们在有限帧数内使用：

- `kMaxKeyframeTrackAge = 20`

超过这个寿命，程序就倾向于重建新地图。

这样设计的好处是：

- 实现简单
- 不需要维护复杂的数据关联和长期优化
- 可以避免长期错误地图点持续污染 PnP

代价是：

- 地图无法跨长时间稳定复用
- 没有闭环
- 不具备完整 SLAM 的地图一致性能力

### 11.4 keyframe 相关变量的角色

当前短时地图由下面几个变量共同表示：

- `keyframe_image`
  - 当前地图参考图像
- `keyframe_features`
  - 该图像上的 2D 特征
- `keyframe_landmarks`
  - 与这些特征一一对应的 3D 点
- `keyframe_frame_id`
  - 当前观测参考帧号
- `keyframe_map_frame_id`
  - 当前地图出生帧号

可以这样理解：

- `keyframe_frame_id` 管“观测被跟踪到哪一帧了”
- `keyframe_map_frame_id` 管“这批 3D 点已经活了多久”

### 11.5 为什么当前策略仍然对 PnP 很保守

代码中最值得注意的一点是：

- `two_view_pose` 始终按照 2D-2D 主干推进
- `geometry_pose` 默认也按 2D-2D 主干推进
- 只有在门控通过时，PnP 才会直接替换当前 `estimated_geometry_pose`

这说明作者对 PnP 的使用态度非常保守：

- 不让 PnP 在弱证据下随意改主干
- 但在强证据下，允许它整个位姿接管当前帧

这也是为什么文档里必须分清：

- 最终对外主干轨迹
- 纯 2D-2D 诊断轨迹

否则很容易以为 `geometry_position.txt` 才是主干，或者误以为 PnP 仍然只改旋转。

---

## 12. 坐标系与位姿表示专题

这一节专门解释几个最容易读混的函数。

### 12.1 `PoseRecord` 到底保存了什么

`PoseRecord` 保存的是相机在世界坐标系中的姿态：

- `R_cw`
- `t_cw`

含义是：

- 把相机坐标系中的向量转到世界坐标系，要乘 `R_cw`
- 相机中心在世界坐标系中的位置是 `t_cw`

### 12.2 OpenCV PnP 和投影常用的却是另一套形式

OpenCV 中 PnP 和投影矩阵通常用：

```text
X_cam = R_wc * X_world + t_wc
```

也就是：

- `R_wc`：世界到相机
- `t_wc`：世界原点在相机坐标系下的位置

它和 `PoseRecord` 的关系是：

```text
R_wc = R_cw^T
t_wc = -R_wc * t_cw
```

代码里就是由 `poseToWorldToCamera()` 完成这个转换。

### 12.3 `poseFromRelativeMotion()`

作用：

- 根据上一帧绝对位姿和当前相对运动，计算当前绝对位姿

输入：

- `prev_pose`
- `relative_rotation`
- `relative_translation`
- `scale`

输出：

- 当前帧 `PoseRecord`

在主流程中的位置：

- 初始化后构造 `geometry_pose`
- 主循环中构造 `base_geometry_pose`

理解重点：

- `recoverPose` 给出的 `translation` 只有方向
- 这里必须结合 `scale` 才能变成真实位移

### 12.4 `accumulateOutputPose()`

作用：

- 保留旧版本的历史兼容输出积分方式

输入输出与 `poseFromRelativeMotion()` 类似，但积分公式不同。

在主流程中的位置：

- 当前默认主流程已不再使用

理解重点：

- 它代表一条历史兼容支路
- 当前最终对外主干已经统一切到 `poseFromRelativeMotion()/geometry_pose`

### 12.5 `poseToWorldToCamera()`

作用：

- 将 `PoseRecord` 的 `R_cw`、`t_cw` 转成 PnP / 投影所需的 `R_wc`、`t_wc`

输入：

- `PoseRecord`

输出：

- `rotation_wc`
- `translation_wc`

在主流程中的位置：

- `buildProjectionMatrix()`
- `projectWorldPoint()`
- `poseToPnPGuess()`

### 12.6 `poseFromPnP()`

作用：

- 将 `solvePnP` 输出的世界到相机位姿转换回 `PoseRecord`

输入：

- `rvec`
- `tvec`

输出：

- `PoseRecord`

在主流程中的位置：

- `solvePoseWithPnP()` refine 完成后

理解重点：

- `solvePnP` 给的是外参
- 程序内部最终仍统一存回 `PoseRecord`

### 12.7 `relativeMotionFromPoses()`

作用：

- 根据两个绝对位姿反推出相对运动表达

输入：

- `prev_pose`
- `curr_pose`

输出：

- `relative_rotation`
- `relative_translation`
- `relative_scale`

在主流程中的位置：

- 当前默认主流程已不再依赖它来决定是否采用 PnP

理解重点：

- 这个函数更多用于“比较和检查”
- 当前版本并没有用它来驱动主干切换

---

## 13. 关键算法函数索引

这一节不按源码顺序逐行列出，而是按用途分组，方便你回查。

### 13.1 路径与运行配置

#### `findProjectRoot()`

- 作用：向上查找项目根目录
- 输入：无
- 输出：项目根路径
- 主流程位置：`main` 早期，由 `makeProjectPaths()` 间接调用

#### `makeProjectPaths()`

- 作用：统一生成输入输出文件路径
- 输入：可选输出根目录
- 输出：`ProjectPaths`
- 主流程位置：`main` 早期

#### `ensureParentDirectory()`

- 作用：输出前自动创建父目录
- 输入：目标文件路径
- 输出：是否成功
- 主流程位置：程序结束写文件前

#### `parseRunOptions()`

- 作用：解析命令行参数
- 输入：`argc/argv`
- 输出：`RunOptions`
- 主流程位置：`main` 开头

#### `getImageScale()`

- 作用：读取图像缩放倍率
- 输入：环境变量
- 输出：缩放比例
- 主流程位置：构建相机矩阵前

### 13.2 输入解析

#### `loadCameraIntrinsics()`

- 作用：读取 `fx/fy/cx/cy`
- 输入：相机配置文件路径
- 输出：`CameraIntrinsics`
- 主流程位置：初始化早期

#### `loadGroundTruthPositions()`

- 作用：读取真值平移位置
- 输入：轨迹文件路径
- 输出：`vector<Vec3d>`
- 主流程位置：初始化早期

#### `loadGroundTruthTimestamps()`

- 作用：读取真值时间戳
- 输入：轨迹文件路径
- 输出：`vector<double>`
- 主流程位置：初始化早期

#### `readImageOrEmpty()`

- 作用：读取灰度图
- 输入：路径模板和帧号
- 输出：图像或空 `Mat`
- 主流程位置：初始化和逐帧处理

#### `readAndResizeImage()`

- 作用：读取图像并按环境变量缩放
- 输入：路径模板、帧号、缩放比例
- 输出：图像
- 主流程位置：初始化和逐帧处理

### 13.3 特征检测与跟踪

#### `featureDetection()`

- 作用：FAST 检测并做亚像素优化
- 输入：图像、最大角点数
- 输出：特征点数组
- 主流程位置：初始化、重检测、失败恢复

#### `featureTracking()`

- 作用：普通 2D-2D LK 光流跟踪
- 输入：前后两帧图像和上一帧特征
- 输出：过滤后的前后两帧对应点
- 主流程位置：初始化和主循环主干

#### `featureTrackingWithLandmarks()`

- 作用：在 LK 跟踪时同步维护 3D-2D 对应
- 输入：keyframe 2D 点、当前图像、3D 地图点
- 输出：过滤后的当前帧观测和地图点
- 主流程位置：PnP 分支

#### `compactPointsByMask()`

- 作用：按 `recoverPose` 的 `mask` 同步压缩匹配点
- 输入：前后两帧特征和 mask
- 输出：只保留几何内点
- 主流程位置：初始化和主循环成功路径

### 13.4 极几何与位姿恢复

#### `buildCameraMatrix()`

- 作用：根据内参构造 `K`
- 输入：`CameraIntrinsics`
- 输出：3x3 相机矩阵
- 主流程位置：初始化阶段

#### `getAbsoluteScale()`

- 作用：根据真值相邻帧距离恢复单目尺度
- 输入：真值位置和当前帧号
- 输出：尺度长度
- 主流程位置：初始化和主循环

#### `poseFromRelativeMotion()`

- 作用：将相对运动积分为世界位姿
- 输入：上一帧绝对位姿、相对运动和尺度
- 输出：当前帧 `PoseRecord`
- 主流程位置：几何主干积分

#### `accumulateOutputPose()`

- 作用：历史兼容输出积分函数
- 输入：上一输出 pose、相对运动和尺度
- 输出：当前输出 pose
- 主流程位置：当前默认主流程已不再使用，仅保留作历史结果对照

### 13.5 三角化与地图点筛选

#### `pixel2cam()`

- 作用：像素坐标转归一化相机坐标
- 输入：像素点和相机矩阵
- 输出：归一化坐标
- 主流程位置：`triangulation()`

#### `triangulation()`

- 作用：根据两帧对应点和相对位姿恢复局部 3D 点
- 输入：两帧点、相机矩阵、相对位姿
- 输出：局部 3D 点
- 主流程位置：被 `triangulateWorldLandmarks()` 调用

#### `medianParallax()`

- 作用：计算匹配点中位视差
- 输入：前后两帧点
- 输出：视差中位数
- 主流程位置：判断是否适合三角化

#### `shouldTriangulateTracks()`

- 作用：根据匹配数、基线和视差决定是否应三角化
- 输入：两帧匹配和真实基线
- 输出：布尔值
- 主流程位置：初始化地图和重建地图前

#### `triangulateWorldLandmarks()`

- 作用：三角化并筛选出可用于 PnP 的世界坐标地图点
- 输入：两帧匹配、相机矩阵、参考 pose、相对运动等
- 输出：世界坐标地图点及对应过滤后的 2D 观测
- 主流程位置：初始化地图和主循环重建地图

### 13.6 PnP 与一致性检查

#### `poseToPnPGuess()`

- 作用：将 `PoseRecord` 转成 `solvePnP` 可用初值
- 输入：内部 pose
- 输出：`rvec/tvec`
- 主流程位置：PnP 求解前

#### `solvePoseWithPnP()`

- 作用：用 3D-2D 对应估计当前相机 pose
- 输入：地图点、图像点、相机矩阵、初始猜测
- 输出：PnP 位姿和内点下标
- 主流程位置：PnP 主求解函数

#### `isPnPPoseConsistent()`

- 作用：检查 PnP 与 2D-2D 主干是否一致
- 输入：PnP pose、参考 pose、当前尺度
- 输出：布尔值
- 主流程位置：PnP 门控第一层

#### `shouldAdoptPnPPose()`

- 作用：决定 PnP 是否真的接管当前最终主干位姿
- 输入：PnP pose、参考 pose、尺度、内点数、地图年龄等
- 输出：布尔值
- 主流程位置：PnP 门控第二层

#### `relativeMotionFromPoses()`

- 作用：把绝对 pose 还原成相对运动形式
- 输入：前后绝对 pose
- 输出：相对旋转、方向和平移尺度
- 主流程位置：当前默认主流程已不再依赖它决定是否采用 PnP

### 13.7 轨迹输出与可视化

#### `appendPose()`

- 作用：按 TUM 格式写出一帧位姿
- 输入：输出流、时间戳、`PoseRecord`
- 输出：文本行
- 主流程位置：程序结束写轨迹文件

#### `rotationToTumQuaternion()`

- 作用：旋转矩阵转 TUM 所需四元数
- 输入：`R_cw`
- 输出：`qx qy qz qw`
- 主流程位置：`appendPose()`

#### `renderTraceCanvas()`

- 作用：绘制真值和估计轨迹对比图
- 输入：真值位置、估计轨迹、帧号
- 输出：OpenCV 画布
- 主流程位置：实时显示和结束保存

#### `isVisualizationEnabled()`

- 作用：判断是否允许打开 GUI
- 输入：环境与 socket 可达性
- 输出：布尔值
- 主流程位置：初始化可视化配置

---

## 14. 程序中的设计取舍和局限

读懂一个程序，不只是知道它“怎么做”，还要知道它“为什么做成这样”以及“它故意没做什么”。

### 14.1 依赖真值恢复单目尺度

这是最重要的取舍。

程序并不是纯单目尺度恢复，而是：

- 视觉几何恢复方向
- 真值轨迹提供长度

优点：

- 实现简单
- 轨迹尺度正确

局限：

- 不能脱离真值独立运行成真实单目 VO 系统
- 更适合教学、实验比较或受控评估环境

### 14.2 2D-2D 是主干，PnP 只是辅助

程序明确保持：

- `two_view_pose` 始终保留纯 2D-2D 主干
- `geometry_pose` 默认由 2D-2D 主干推进
- PnP 只在严格门控下接管 `geometry_pose`

优点：

- 不容易因为错误 3D-2D 对应而整段轨迹跳飞

局限：

- 不是每次有 PnP 就立刻采用
- 最终主干和纯 2D-2D 诊断轨迹会并行存在，理解成本更高

### 14.3 地图是短时的，不是长期全局地图

当前地图只有短寿命、局部复用能力。

优点：

- 逻辑简单
- 容易控制错误传播

局限：

- 不支持闭环
- 不支持全局一致性优化
- 不具备完整 SLAM 地图管理能力

### 14.4 `findEssentialMat` 使用的是焦距 + 主点重载

代码当前调用形式是：

- 使用 `focal_length`
- 使用 `principal_point`

而不是直接使用完整 `camera_matrix` 重载。

好处：

- 写法简洁

前提：

- 默认 `fx` 和 `fy` 足够接近

局限：

- 如果相机横纵焦距差异很大，这种简化可能不如直接传完整 `K` 稳妥

### 14.5 可视化设计偏工程稳健

程序不是简单地检查 `DISPLAY` 是否存在，而是尝试连 Unix socket，判断 X11/Wayland 是否真的可用。

优点：

- 在 SSH、容器、无头环境中更不容易卡在 `imshow`

这反映出代码并不只是“算法 demo”，而是考虑过多运行环境的工程实现。

---

## 15. 建议阅读顺序

如果你现在准备正式读 `My_VO.cpp`，建议按下面顺序走，而不是从第一行机械读到最后一行。

### 第一遍：建立全局框架

按这个顺序扫：

1. `Base/My_VO.h`
2. `main()` 的整体流程
3. 本文档第 10 节变量追踪表

目标：

- 搞清程序有哪些输入输出
- 搞清状态变量分成哪几类

### 第二遍：读主循环

重点看：

- 初始化阶段
- 主循环逐帧路径
- 失败分支如何回退

目标：

- 搞清楚每一帧到底做了什么
- 搞清楚为什么有时位姿会“沿用上一帧”

### 第三遍：专门读坐标系和位姿函数

重点函数：

- `poseFromRelativeMotion()`
- `accumulateOutputPose()`
- `poseToWorldToCamera()`
- `poseFromPnP()`
- `relativeMotionFromPoses()`

目标：

- 搞清楚 `R_cw`、`R_wc`、`t_cw`、`t_wc`
- 搞清楚为什么现在保留的是“最终主干 + 诊断支路”

### 第四遍：读三角化和 PnP

重点函数：

- `shouldTriangulateTracks()`
- `triangulateWorldLandmarks()`
- `featureTrackingWithLandmarks()`
- `solvePoseWithPnP()`
- `isPnPPoseConsistent()`
- `shouldAdoptPnPPose()`

目标：

- 搞清楚短时地图是怎么生成、怎么老化、怎么失效的
- 搞清楚 PnP 为什么大多数时候只是候选修正

### 第五遍：最后再读工具函数

例如：

- 路径生成
- 命令行解析
- GUI 可用性检测
- 轨迹绘制

这些函数不难，但在没理解主流程前过早陷进去，收益不高。

---

## 16. 常见困惑

### 16.1 为什么单目 VO 还要读取真值

因为这份程序不是纯视觉独立恢复绝对尺度，而是借助真值相邻帧距离给 `recoverPose` 输出的平移方向补上长度。

### 16.2 为什么会有 `geometry_pose` 和 `two_view_pose` 两套 pose

因为程序把“最终对外主干状态”和“纯 2D-2D 诊断状态”分开维护。`geometry_pose` 是最终发布轨迹，`two_view_pose` 用于保留不受 PnP 接管影响的纯两视图结果。

### 16.3 为什么 `triangulation()` 不是长期建图

因为这里三角化出来的点主要服务于短时 PnP，不进入长期全局地图管理，也没有闭环或全局优化。

### 16.4 为什么 PnP 不直接替代主干

因为错误的 3D-2D 对应会让 PnP 发生跳变。当前代码选择保守策略，只在与 2D-2D 主干高度一致时，才让 PnP 整个位姿接管当前主干。

### 16.5 为什么有些帧会“没有更新位姿”

常见原因有：

- 跟踪点数量不足
- `recoverPose` 内点不足
- 真值尺度太小

这种情况下程序更愿意保留上一帧状态，而不是强行输出一帧低可信结果。

---

## 17. 读完后你应该能回答的问题

如果你已经理解这份程序，应该能比较流畅地回答下面几个问题：

1. 这份程序的主干位姿来源是什么？
2. 为什么单目平移还要读真值？
3. `geometry_pose` 和 `two_view_pose` 的区别是什么？
4. `keyframe_landmarks` 是怎么来的，又会因为什么失效？
5. PnP 为什么大多数时候只是候选接管而不是直接主导位姿？
6. 主循环里 `prev_features`、`curr_features`、`keyframe_features` 各自代表哪一层状态？

如果这些问题你都能说清楚，那么再去看 `My_VO.cpp` 时，代码就不再是“很多 OpenCV API 堆在一起”，而会变成一条完整可追踪的程序执行链。
