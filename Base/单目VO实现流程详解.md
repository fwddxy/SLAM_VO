# 单目视觉里程计（Monocular VO）实现流程详解

本文档基于 [Base/My_VO.cpp](Base/My_VO.cpp) 的实际代码，详细解释整个单目 VO 系统的完整实现流程。

---

## 整体架构：2D-2D 主干 + PnP 辅助修正

这个 VO 系统不是"纯单目"的——它用真值轨迹的相邻帧距离来恢复绝对尺度。核心思路是：

- **主干**：两帧之间的极几何（Essential Matrix → recoverPose）给出相对运动方向
- **尺度**：真值轨迹提供长度
- **辅助**：短寿命三角化地图点 + PnP，在严格条件下接管当前帧位姿

程序维护**两条并行轨迹**：

| 轨迹变量 | 输出文件 | 语义 |
|---|---|---|
| `geometry_pose` → `trajectory` | `position.txt` | **最终对外主干**（默认 2D-2D，可在门控通过时被 PnP 接管） |
| `two_view_pose` → `geometry_trajectory` | `geometry_position.txt` | **纯 2D-2D 诊断支路**（永远不受 PnP 影响） |

---

## 一、数据结构与坐标约定

### 1.1 核心数据结构

```
CameraIntrinsics    fx, fy, cx, cy（针孔内参）
PoseRecord          rotation(R_cw) + translation(t_cw)，相机在世界系下的姿态
ProjectPaths        所有输入输出路径的集中管理
RunOptions          命令行传入的帧区间和输出目录
```

### 1.2 坐标约定（最容易出错的地方）

**`PoseRecord` 存的是相机到世界**：

- `rotation` = `R_cw`：把相机坐标系下的向量转到世界坐标系
- `translation` = `t_cw`：相机光心在世界坐标系中的位置

**OpenCV 的 PnP/投影矩阵用的恰好相反**（世界到相机）：

```
X_cam = R_wc · X_world + t_wc
其中：R_wc = R_cw^T
      t_wc = -R_wc · t_cw
```

这个转换由 `poseToWorldToCamera()` 完成。**不理解这个区别，后面所有 PnP 和投影代码都会读乱**。

相机坐标系约定：x 向右，y 向下，z 向前。轨迹绘制在 X-Z 平面（俯视图）。

---

## 二、全局阈值常量

这些常量定义在 `My_VO.cpp` 顶部匿名命名空间中，控制了整套程序的行为风格。

### A. 特征检测与跟踪

| 常量 | 值 | 含义 |
|---|---|---|
| `kTargetFeatureCount` | 2000 | 每次重检测最多保留的 FAST 特征数 |
| `kMinTrackedFeatures` | 1000 | 跟踪点低于此值触发重检测 |
| `kMaxForwardBackwardError` | 1.0 px | LK 光流前后向一致性误差阈值 |

### B. 位姿估计

| 常量 | 值 | 含义 |
|---|---|---|
| `kMinInitialPoseInliers` | 5 | 初始化阶段 recoverPose 最低内点数 |
| `kMinPoseInliers` | 20 | 正常主循环 recoverPose 最低内点数 |
| `kMinTranslationScale` | 0.1m | 真值位移过小则跳过位姿更新 |

### C. 三角化与地图点质量控制

| 常量 | 值 | 含义 |
|---|---|---|
| `kMinTriangulatedLandmarks` | 4 | 允许三角化的最低匹配数量 |
| `kTriangulationReprojectionError` | 3.0 px | 三角化点两帧重投影误差阈值 |
| `kMinTriangulationParallaxPixels` | 8.0 px | 允许三角化的最低中位视差 |
| `kMinTriangulationBaseline` | 0.15m | 允许三角化的最低真实基线 |
| `kMinLandmarkDepth` | 0.1m | 地图点最小深度 |
| `kMaxLandmarkDepth` | 80.0m | 地图点最大深度 |

### D. PnP 求解与门控

| 常量 | 值 | 含义 |
|---|---|---|
| `kMinPnPInliers` | 12 | solvePnPRansac 最低内点数 |
| `kPnPReprojectionError` | 4.0 px | PnP RANSAC 重投影误差阈值 |
| `kMinKeyframePnPInliers` | 30 | 支持稳定 PnP 的最低 3D-2D 对应数 |
| `kPreferredKeyframePnPInliers` | 100 | 允许 PnP 接管主干时偏好的内点数 |
| `kMaxKeyframeTrackAge` | 20 帧 | 短时地图观测连续跟踪的最长寿命 |
| `kMaxPnPMapAge` | 8 帧 | 短时地图的绝对寿命上限 |
| `kMaxPnPRotationDeltaDegrees` | 8° | PnP 与 2D-2D 主干允许的最大旋转差（一致性检查） |
| `kMaxPnPTranslationDeltaScale` | 3.0× | PnP 与 2D-2D 主干允许的最大平移差因子 |
| `kTrustedPnPRotationDeltaDegrees` | 1.5° | PnP 接管主干时的严格旋转差限制 |
| `kTrustedPnPTranslationDeltaScale` | 0.75× | PnP 接管主干时的严格平移差限制 |
| `kMaxOutputPnPScale` | 1.0m | 仅在小尺度运动时允许 PnP 接管主干 |
| `kMinOutputPnPKeyframeAge` | 1 帧 | 地图至少存在的帧数才允许接管 |
| `kMaxOutputPnPTrackedFeatures` | 1600 | 2D-2D 跟踪点超过此数不让 PnP 接管 |
| `kMinPnPMapRetentionRatio` | 45% | 地图保留率低于此值不再尝试 PnP |
| `kMinAdoptPnPMapRetentionRatio` | 60% | 接管主干时的更高保留率要求 |
| `kMinAdoptPnPInlierRatio` | 70% | 接管主干时的 PnP 内点比例要求 |
| `kRefreshLandmarkRetentionRatio` | 50% | 保留率低于此值触发地图重建 |

---

## 三、初始化阶段（前两帧）

初始化建立整个主循环需要的"第一份可用状态"。使用的帧是 `segment_start_frame`（第 0 帧）和 `segment_start_frame + 1`（第 1 帧）。

### 步骤 1：读取前两帧图像

```cpp
Mat img_1 = readAndResizeImage(..., segment_start_frame, image_scale);
Mat img_2 = readAndResizeImage(..., segment_start_frame + 1, image_scale);
```

- 灰度图读取（`IMREAD_GRAYSCALE`）
- 如果设置了 `VO_IMAGE_SCALE`，同步缩放图像和内参（`scaleCameraIntrinsics`），保证像素坐标模型一致

### 步骤 2：第一帧 FAST 特征检测

```cpp
featureDetection(img_1, points1, kTargetFeatureCount);  // kTargetFeatureCount = 2000
```

内部流程：

1. FAST 角点检测（阈值 20，开启非极大值抑制）
2. `nth_element` 保留响应值最高的 2000 个点（不完整排序，比 `sort` 快）
3. `cornerSubPix` 亚像素精化（搜索窗口 10×10，最多 20 次迭代，精度 0.03 px）

### 步骤 3：LK 光流跟踪到第二帧

```cpp
featureTracking(img_1, img_2, points1, points2, status);
```

**这是整个系统特征匹配的核心机制——不是描述子匹配，而是时序光流跟踪**：

1. **正向跟踪**：`calcOpticalFlowPyrLK(img_1 → img_2)`，金字塔 3 层，窗口 21×21
2. **反向跟踪**：`calcOpticalFlowPyrLK(img_2 → img_1)`，把当前帧的点跟回上一帧
3. **四重过滤**：
   - 正向跟踪失败（`status[i] == 0`）
   - 反向跟踪失败（`backward_status[i] == 0`）
   - 跟踪点越界（超出图像范围）
   - 前后向误差过大（`||prev - backward|| > 1.0 px`）

过滤后 `points1` 和 `points2` 仍然一一对应，`points2[i]` 是 `points1[i]` 在第二帧中的位置。

### 步骤 4：本质矩阵估计 + 恢复相对位姿

```cpp
essential_matrix = findEssentialMat(points1, points2, focal_length, principal_point,
                                    RANSAC, 0.999, 1.0, mask);
int inliers = recoverPose(essential_matrix, points1, points2, rotation, translation,
                          focal_length, principal_point, mask);
```

- `findEssentialMat`：RANSAC 置信度 0.999，像素误差阈值 1.0 px。使用的是焦距+主点的简化重载，前提假设 `fx ≈ fy`
- `recoverPose`：从本质矩阵分解出 R 和 t，并通过 cheirality check 选出唯一正确解
- `rotation`：3×3，上一帧相机到当前帧相机的相对旋转
- `translation`：3×1，**单位方向**（不是真实位移！长度永远是 1）
- `mask`：标记哪些匹配点是几何内点

初始化内点要求较低：`kMinInitialPoseInliers = 5`。

### 步骤 5：内点筛选

```cpp
compactPointsByMask(points1, points2, mask);
```

将不符合极几何关系的匹配点从 `points1` 和 `points2` 中同步删除。目的：
1. 保证后续三角化和 PnP 使用更干净的观测
2. 让下一帧继续跟踪的特征更可靠

### 步骤 6：真值尺度恢复

```cpp
double scale = getAbsoluteScale(gt_positions, segment_start_frame + 1);
```

**这是"为什么单目 VO 还要读真值"的答案**：

```cpp
// getAbsoluteScale 内部实现
Vec3d delta = gt_positions[frame_id] - gt_positions[frame_id - 1];
return sqrt(delta.dot(delta));
```

`recoverPose` 给出的 `translation` 只有方向，真实走了多少米完全不知道。这里直接用真值轨迹中相邻帧的欧氏距离补上长度。

### 步骤 7：建立初始位姿

```cpp
PoseRecord geometry_pose = poseFromInitialRelativeMotion(rotation, translation, scale);
PoseRecord two_view_pose = geometry_pose;  // 初始化时两者相同
```

`poseFromRelativeMotion` 内部的积分公式：

```
R_cw_curr = R_cw_prev · R_rel^T
t_cw_curr = t_cw_prev + R_cw_curr · (-scale · t_rel)
```

解释：
- `R_rel^T`：因为累积的是相机到世界（`R_cw`），而 recoverPose 给出的是相机间的相对旋转
- `-t_rel`：要把相机坐标系下的位移方向转成世界坐标系下的位移方向
- `scale`：把单位方向变成真实米级位移

### 步骤 8：尝试初始化短时地图（可选）

```cpp
if (shouldTriangulateTracks(...) && triangulateWorldLandmarks(...))
```

三个条件同时满足才三角化：

1. 匹配点数量 ≥ `kMinTriangulatedLandmarks = 4`
2. 真实基线 ≥ `kMinTriangulationBaseline = 0.15m`
3. 中位视差 ≥ `kMinTriangulationParallaxPixels = 8.0 px`

三角化后的多层筛选流水线：

```
三角化 → 过滤 NaN/Inf → 深度 > 0（相机前方） → 深度 ∈ [0.1, 80.0] m
       → 上一帧重投影误差 < 3.0 px → 当前帧重投影误差 < 3.0 px
       → 转换到世界坐标系保存
```

如果成功，生成第一批 `tracked_landmarks`（世界坐标系下的 3D 点），成为后续 PnP 的地图种子。失败也不终止程序，只暂时没有 PnP 可用地图。

---

## 四、主循环（第三帧起）

```cpp
for (int frame_id = segment_start_frame + 2; frame_id <= segment_end_frame; ++frame_id)
```

每一帧走相同的处理流程。以下按实际代码执行顺序展开。

---

### 4.1 异步预读图像

```cpp
curr_image = next_frame_future.get();                          // 取上一轮预读的结果
next_frame_future = async(readAndResizeImage, ..., frame_id+1); // 发起下一帧预读
```

纯工程优化：让磁盘 IO 和当前帧的算法计算重叠，减少主循环阻塞。第一帧进入循环时，`next_frame_future` 已在初始化末尾启动。

---

### 4.2 2D-2D 光流跟踪

```cpp
featureTracking(prev_image, curr_image, prev_features, curr_features, tracking_status);
```

从 `prev_image` 上的 `prev_features` 跟踪到 `curr_image`，得到一一对应的 `curr_features`。过滤机制和初始化阶段完全一样。

---

### 4.3 特征不足 → 重检测 → 再失败则跳帧

```cpp
if (prev_features.size() < 8 || curr_features.size() < 8) {
    featureDetection(prev_image, prev_features, kTargetFeatureCount); // 重新检测
    featureTracking(...);  // 重新跟踪
    if (仍然不够) {
        // 本帧不更新位姿，保留上一帧状态
        trajectory.push_back(geometry_pose);
        geometry_trajectory.push_back(two_view_pose);
        trajectory_frame_ids.push_back(frame_id);
        // 在当前帧重新检测，为下一帧恢复做准备
        featureDetection(curr_image, prev_features, kTargetFeatureCount);
        continue;
    }
}
```

策略是**宁可少更新，也不强行用低质量匹配估计错误位姿**。

---

### 4.4 本质矩阵 + recoverPose

```cpp
essential_matrix = findEssentialMat(prev_features, curr_features, focal_length,
                                    principal_point, RANSAC, 0.999, 1.0, mask);
int inlier_count = recoverPose(essential_matrix, prev_features, curr_features,
                               rotation, translation, focal_length, principal_point, mask);
```

和初始化阶段相同，但内点要求更严格：`kMinPoseInliers = 20`（初始化只有 5）。

如果内点不足 → 同样跳帧，保留上一帧位姿输出，并在当前帧重新检测特征。

---

### 4.5 内点筛选 + 真值尺度恢复

```cpp
compactPointsByMask(prev_features, curr_features, mask);  // 保留几何内点
double scale = getAbsoluteScale(gt_positions, frame_id);   // 取相邻帧真值距离
```

如果 `scale <= kMinTranslationScale (0.1m)` → 跳过本帧位姿更新。因为极低速/静止时平移方向受噪声主导，强行积分反而更差。

---

### 4.6 构造基线位姿

```cpp
PoseRecord base_two_view_pose  = poseFromRelativeMotion(two_view_pose,  rotation, translation, scale);
PoseRecord base_geometry_pose  = poseFromRelativeMotion(geometry_pose, rotation, translation, scale);
PoseRecord estimated_geometry_pose = base_geometry_pose;  // 默认值，后续 PnP 可能覆盖
```

关键语义：`base_geometry_pose` 是"如果不采用 PnP 时的最终主干结果"。后面 PnP 如果通过全部门控，会把它替换为 `pnp_pose`。

---

### 4.7 尝试 PnP 修正（核心增强逻辑）

这是这个 VO 相对基础版的最大区别。整个 PnP 路径有**五层门控**。

#### 第一层：前置条件

```cpp
if (enable_pnp &&
    !keyframe_image.empty() &&
    !keyframe_features.empty() &&
    !keyframe_landmarks.empty() &&
    keyframe_track_age <= kMaxKeyframeTrackAge &&           // 观测寿命 ≤ 20 帧
    keyframe_map_age <= kMaxPnPMapAge &&                     // 地图寿命 ≤ 8 帧
    landmark_retention >= kMinPnPMapRetentionRatio &&        // 地图保留率 ≥ 45%
    featureTrackingWithLandmarks(...) >= kMinKeyframePnPInliers) // 至少 30 个成功的 3D-2D 对
```

`featureTrackingWithLandmarks` 和普通的 `featureTracking` 关键不同：它在 LK 跟踪的同时，把跟踪失败的 2D 点和对应的 3D 点一起丢弃，保证传给 PnP 的 3D-2D 对应始终索引一致。

#### 第二层：PnP 求解

```cpp
solvePoseWithPnP(landmarks, image_points, camera_matrix, base_geometry_pose, pnp_pose, inliers)
```

内部两步求解：

1. `solvePnPRansac`（EPNP + RANSAC，重投影误差 4.0 px，置信度 0.99）→ 初步剔除外点
2. `solvePnP(ITERATIVE)` — 只用 RANSAC 内点做迭代精化

**初值来自 `base_geometry_pose`**（即 2D-2D 主干位姿），通过 `poseToPnPGuess()` 转成 `rvec/tvec`。这意味着 PnP 不是盲搜，而是在 2D-2D 解附近做局部优化。

要求 RANSAC 内点 ≥ `kMinPnPInliers = 12`。

#### 第三层：一致性检查

```cpp
isPnPPoseConsistent(pnp_pose, base_geometry_pose, scale)
```

判断条件：
- 旋转差 ≤ `kMaxPnPRotationDeltaDegrees = 8°`
- 平移差 ≤ `max(0.5m, scale × 3.0)`

这保证 PnP 解不会和 2D-2D 主干差太远。不通过则直接丢弃本次 PnP 结果。

#### 第四层：接管决策（最严格的门控）

```cpp
shouldAdoptPnPPose(...)
```

即使 PnP 解是可解的且一致的，也不一定真的接管最终主干。需要**全部满足**以下条件：

| 条件 | 阈值 | 设计理由 |
|---|---|---|
| 当前帧尺度 | ≤ `kMaxOutputPnPScale = 1.0m` | 大尺度时 2D-2D 本身足够可靠 |
| PnP 内点数 | ≥ `kPreferredKeyframePnPInliers = 100` | 需要强证据 |
| 地图保留率 | ≥ `kMinAdoptPnPMapRetentionRatio = 60%` | 避免稀疏匹配 |
| PnP 内点比例 | ≥ `kMinAdoptPnPInlierRatio = 70%` | 保证观测质量 |
| 平移差 | ≤ `max(0.2m, scale × 0.75)` | 比一致性检查更严格的平移限制 |
| 旋转差 | ≤ `kTrustedPnPRotationDeltaDegrees = 1.5°` | 比一致性检查更严格的旋转限制 |
| 地图年龄 | ≥ `kMinOutputPnPKeyframeAge = 1 帧` | 不要刚出生的地图 |
| 2D-2D 跟踪数 | ≤ `kMaxOutputPnPTrackedFeatures = 1600` | 2D-2D 点太多时不需要 PnP |
| 当前旋转量 | ≥ `kMinOutputPnPRotationDegrees = 0.5°` | 旋转主导时 PnP 更有价值 |

**如果全部通过**：`estimated_geometry_pose = pnp_pose`（PnP 整个位姿接管当前主干）。

设计意图：PnP 是**高度保守的候选接管者**，不是默认位姿来源。绝大多数帧走 2D-2D 主干，只有极少数旋转主导、小尺度、地图质量极好的帧才启用 PnP。

#### 第五层：推进 keyframe 观测

无论 PnP 是否被采纳，只要 map 点跟踪成功，就把 2D 观测推进到当前帧：

```cpp
keyframe_image = curr_image;
keyframe_features = pnp_curr_features;
keyframe_landmarks = pnp_landmarks;
keyframe_frame_id = frame_id;
```

这样避免了长距离光流导致的丢点问题——每一帧都用"上一帧观测 → 当前帧"的单步跟踪，而不是"出生帧观测 → 当前帧"的多步跟踪。

即使 PnP 解被拒绝，只要地图点还能跟踪，就继续推进观测、延长这批短时地图的可用时间。

---

### 4.8 重建短时地图

当地图为空、点数不足（< 30）、寿命过长（≥ 8 帧）或保留率过低（< 50%）时触发重建：

```cpp
if (should_refresh_keyframe &&
    shouldTriangulateTracks(...) &&
    triangulateWorldLandmarks(...) &&
    rebuilt_landmarks.size() >= kMinKeyframePnPInliers) {

    keyframe_image = curr_image;
    keyframe_features = triangulation_curr_features;
    keyframe_landmarks = rebuilt_landmarks;
    keyframe_pose = estimated_geometry_pose;
    keyframe_frame_id = frame_id;
    keyframe_map_frame_id = frame_id;    // 新地图的出生帧
    keyframe_initial_landmark_count = rebuilt_landmarks.size();
}
```

重建使用当前帧和上一帧的 2D-2D 内点，三角化条件和筛选流程与初始化阶段相同。

---

### 4.9 记录轨迹、特征重检测、状态推进

```cpp
two_view_pose = base_two_view_pose;                   // 纯 2D-2D 永远不变
geometry_pose = estimated_geometry_pose;               // 可能已被 PnP 接管
trajectory.push_back(geometry_pose);
geometry_trajectory.push_back(two_view_pose);
trajectory_frame_ids.push_back(frame_id);

if (curr_features.size() < kMinTrackedFeatures)        // < 1000
    featureDetection(curr_image, curr_features, ...);   // 重检测，为下一帧准备

prev_image = curr_image;
prev_features = curr_features;
```

---

## 五、一帧处理的时序总结

### 标准成功路径

```
1. 获取当前帧图像（异步预读）
2. 用 prev_image 和 prev_features LK 光流跟踪出 curr_features
3. （若特征不足）重检测 → 重跟踪 → 再不足则跳帧
4. 用 prev_features 和 curr_features 估计本质矩阵
5. recoverPose 恢复相对位姿（R + 单位 t 方向）
6. mask 保留几何内点
7. 从真值读取当前帧绝对尺度 scale
8. 用 poseFromRelativeMotion 计算 base_geometry_pose 和 base_two_view_pose
9. （如果有可用短时地图）尝试 PnP → 通过门控则接管 estimated_geometry_pose
10.（如果地图需要刷新）尝试三角化重建新地图
11. 更新 geometry_pose 和 two_view_pose
12. 记录 trajectory 和 geometry_trajectory
13.（若跟踪点不足）重检测 FAST 特征
14. 推进 prev_image/prev_features，进入下一帧
```

### 各种失败分支的保守策略

| 失败场景 | 处理策略 |
|---|---|
| 跟踪点不够 | 重检测，必要时本帧不更新 pose |
| recoverPose 内点不足 | 本帧不更新 pose，输出上一帧位姿 |
| 真值尺度太小 | 本帧不更新平移 |
| PnP 前置条件不满足 | 跳过 PnP，继续使用 2D-2D 主干 |
| PnP 求解失败 | 跳过 PnP，继续使用 2D-2D 主干 |
| PnP 一致性检查失败 | 丢弃 PnP 结果，但仍推进 keyframe 观测 |
| PnP 接管决策不通过 | 保留 2D-2D 主干结果 |
| 三角化条件不满足 | 不退出，只是暂时没有新地图 |

程序整体不是"每帧都必须成功"，而是"尽可能保持轨迹连续，在失败时平稳退回主干策略"。

---

## 六、输出阶段

主循环结束后：

```cpp
for (size_t i = 0; i < trajectory.size(); ++i) {
    appendPose(output, gt_timestamps[frame_ids[i]], trajectory[i]);           // → position.txt
    appendPose(geometry_output, gt_timestamps[frame_ids[i]], geometry_trajectory[i]); // → geometry_position.txt
}
```

两幅轨迹图：

- `map2.png`：前 150 帧对齐（`kTracePrefixAlignmentFrames`）
- `map2_global_aligned.png`：全序列对齐

对齐使用 Umeyama 刚体变换（SVD 分解），不带尺度项，目的是让可视化效果更接近 evo 的 `-a` 对齐口径，不改变 `position.txt` 中保存的原始轨迹数据。

输出路径约定：

```
output/full_run/
├── position/
│   ├── position.txt           ← 最终对外主干（给 evo 评估的默认轨迹）
│   └── geometry_position.txt  ← 纯 2D-2D 诊断轨迹
└── traj/
    ├── map2.png                ← 前 150 帧对齐的轨迹对比图
    └── map2_global_aligned.png ← 全序列对齐的轨迹对比图
```

---

## 七、状态变量速查表

这张表对阅读 `main()` 最关键：

| 变量 | 含义 |
|---|---|
| `prev_image` | 2D-2D 主干的上一帧图像 |
| `curr_image` | 2D-2D 主干的当前帧图像 |
| `prev_features` | `prev_image` 上的 2D-2D 主干特征点 |
| `curr_features` | `curr_image` 上与 `prev_features` 一一对应的特征点 |
| `geometry_pose` | 当前最终对外主干位姿（可能已被 PnP 接管） |
| `two_view_pose` | 当前纯 2D-2D 诊断位姿（永不受 PnP 影响） |
| `keyframe_image` | 短时地图的参考图像 |
| `keyframe_features` | `keyframe_image` 上与 `keyframe_landmarks` 一一对应的 2D 观测 |
| `keyframe_landmarks` | 当前短时地图的 3D 世界坐标点 |
| `keyframe_pose` | keyframe 对应的几何位姿 |
| `keyframe_frame_id` | 观测被跟踪到的最新帧号（计算跟踪年龄用） |
| `keyframe_map_frame_id` | 这批 3D 点的出生帧号（计算地图寿命用） |
| `keyframe_initial_landmark_count` | 地图出生时的点数（计算保留率用） |
| `trajectory` | 最终输出轨迹序列 → 写入 `position.txt` |
| `geometry_trajectory` | 纯 2D-2D 诊断轨迹序列 → 写入 `geometry_position.txt` |
| `trajectory_frame_ids` | 每个轨迹条目对应的数据集帧号 → 用于对齐真值时间戳 |

---

## 八、关键函数索引

### 路径与配置

| 函数 | 作用 |
|---|---|
| `findProjectRoot()` | 向上查找同时包含 `Kitti/` 和 `Base/` 的目录 |
| `makeProjectPaths()` | 生成所有输入输出路径 |
| `getImageScale()` | 读取 `VO_IMAGE_SCALE` 环境变量 |
| `isVisualizationEnabled()` | 通过实际连接 Unix socket 判断 GUI 是否可用 |
| `isVerboseLoggingEnabled()` | 读取 `VO_VERBOSE_LOG` 环境变量 |
| `isPnPEnabled()` | 读取 `VO_ENABLE_PNP` 环境变量 |

### 输入解析

| 函数 | 作用 |
|---|---|
| `loadCameraIntrinsics()` | 从 `camera-info.txt` 读取 fx/fy/cx/cy |
| `loadGroundTruthPositions()` | 从 `gt-tum07.txt` 读取每帧平移 |
| `loadGroundTruthTimestamps()` | 从 `gt-tum07.txt` 读取每帧时间戳 |
| `readImageOrEmpty()` | 读灰度图，失败返回空 Mat |
| `readAndResizeImage()` | 读图 + 按环境变量缩放 |

### 特征检测与跟踪

| 函数 | 作用 |
|---|---|
| `featureDetection()` | FAST 角点 + nth_element + cornerSubPix |
| `featureTracking()` | 前后向 LK 光流 + 四重过滤（纯 2D 跟踪） |
| `featureTrackingWithLandmarks()` | LK 光流 + 同步维护 3D-2D 对应关系 |
| `compactPointsByMask()` | 按 recoverPose 的 mask 同步删除外点 |

### 极几何与位姿恢复

| 函数 | 作用 |
|---|---|
| `buildCameraMatrix()` | 从 CameraIntrinsics 构造 3×3 K 矩阵 |
| `scaleCameraIntrinsics()` | 图像缩放后同步缩放内参 |
| `getAbsoluteScale()` | 用真值相邻帧距离恢复单目尺度 |
| `poseFromRelativeMotion()` | 相对运动积分到世界位姿（几何主干公式） |
| `poseFromInitialRelativeMotion()` | 从单位位姿开始的首次积分 |
| `poseToWorldToCamera()` | PoseRecord 转 OpenCV 世界到相机格式 |
| `poseFromPnP()` | solvePnP 输出的 rvec/tvec 转回 PoseRecord |
| `relativeMotionFromPoses()` | 两绝对位姿反推相对运动（仅用于诊断） |

### 三角化与地图点

| 函数 | 作用 |
|---|---|
| `pixel2cam()` | 像素坐标转归一化相机坐标 |
| `triangulation()` | 两帧对应点 + 相对位姿 → 局部 3D 点 |
| `medianParallax()` | 计算匹配点中位视差 |
| `shouldTriangulateTracks()` | 判断匹配数/基线/视差是否满足三角化条件 |
| `triangulateWorldLandmarks()` | 三角化 + 多层筛选 + 转到世界坐标系 |

### PnP 与门控

| 函数 | 作用 |
|---|---|
| `poseToPnPGuess()` | PoseRecord 转 solvePnP 的 rvec/tvec 初值 |
| `solvePoseWithPnP()` | PnP RANSAC + Iterative Refine |
| `compactTracksByIndices()` | 按 PnP 内点下标同步压缩 2D 轨迹和 3D 地图 |
| `isPnPPoseConsistent()` | PnP 与 2D-2D 主干位姿一致性检查（第三层门控） |
| `shouldAdoptPnPPose()` | PnP 接管决策（第四层门控，最严格） |
| `landmarkRetentionRatio()` | 当前地图点保留比例 |

### 轨迹输出与可视化

| 函数 | 作用 |
|---|---|
| `appendPose()` | 按 TUM 格式写一帧位姿 |
| `rotationToTumQuaternion()` | R_cw → qx qy qz qw |
| `projectWorldPoint()` | 世界 3D 点投影到像素平面 |
| `buildProjectionMatrix()` | 构造 P = K[R_wc \| t_wc] |
| `renderTraceCanvas()` | 绘制 X-Z 平面轨迹对比图 |
| `alignEstimatedPositionsToGroundTruth()` | Umeyama 刚体对齐（用于可视化） |

---

## 九、核心设计取舍与局限

### 9.1 依赖真值恢复单目尺度

程序不是纯视觉恢复绝对尺度，而是视觉几何给出方向、真值轨迹提供长度。

- **优点**：实现简单、轨迹尺度正确
- **局限**：不能脱离真值独立运行，更适合教学和受控评估环境

### 9.2 2D-2D 是主干，PnP 只是辅助

- `two_view_pose` 始终保留纯 2D-2D 主干结果
- `geometry_pose` 默认由 2D-2D 主干推进，PnP 只在严格门控下接管
- **优点**：不容易因为错误 3D-2D 对应导致整段轨迹跳飞
- **局限**：两套轨迹并行存在，理解成本更高

### 9.3 短时地图而非长期地图

- 地图寿命上限 8 帧（`kMaxPnPMapAge`），观测寿命上限 20 帧（`kMaxKeyframeTrackAge`）
- **优点**：逻辑简单，容易控制错误传播
- **局限**：不支持闭环、不支持全局 BA、不具备完整 SLAM 的地图一致性能力

### 9.4 `findEssentialMat` 使用焦距+主点重载

当前代码使用 `focal_length` + `principal_point` 而非完整 `camera_matrix` 重载。

- **前提**：`fx` 和 `fy` 足够接近（KITTI 07 中 fx=fy=707.09，满足此条件）
- **局限**：若相机横纵焦距差异很大，应改用完整 K 矩阵重载

### 9.5 GUI 检测偏工程稳健

程序不是简单检查 `DISPLAY` 环境变量，而是尝试实际 `connect()` Unix socket 来判断 X11/Wayland 是否可用。这避免了 SSH、容器、无头环境中 `imshow` 卡住的问题。

### 9.6 异步图像预读

用 `std::async` 在后台线程提前读下一帧，让磁盘 IO 与当前帧的算法计算重叠。纯工程优化。

---

## 十、建议的代码阅读顺序

1. **第一遍**：读 `Base/My_VO.h`，了解所有数据结构和函数声明
2. **第二遍**：读 `main()` 的整体结构和本文档第七节的变量追踪表
3. **第三遍**：精读初始化阶段（前两帧处理）
4. **第四遍**：精读主循环中的 2D-2D 主干路径（跳过 PnP 分支）
5. **第五遍**：专门读坐标系转换函数（`poseToWorldToCamera`、`poseFromPnP`、`poseFromRelativeMotion`）
6. **第六遍**：读三角化和 PnP 全路径（`triangulateWorldLandmarks`、`solvePoseWithPnP`、门控函数）
7. **最后**：读工具函数（路径解析、GUI 检测、轨迹绘制）

---
