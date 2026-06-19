# VO 常见问题整理

本文档整理自阅读 `Base/My_VO.cpp` 过程中的问答，只保留对当前代码仍然有效的问题，并按当前实现重新表述。

适用范围：

- 当前 `Base/My_VO.cpp`
- 当前 `Base/My_VO.h`
- 当前 `Base/VO程序详解.md`

不再收录已经过时的问题，例如旧版本里“PnP 只改旋转、不改平移”的分支说明。当前代码中，PnP 在通过门控后可以直接接管当前绝对位姿，见 [My_VO.cpp:1917](/home/xywu/桌面/SLAM/Base/My_VO.cpp:1917) 到 [My_VO.cpp:1919](/home/xywu/桌面/SLAM/Base/My_VO.cpp:1919)。

---

## 1. RANSAC 的作用是什么？内点筛选和 mask 又是什么？

在这份程序里，`RANSAC`、内点筛选和 `mask` 是一条连续的处理链。

### 1.1 RANSAC 的作用

当前程序先用 `findEssentialMat(..., RANSAC, ...)` 从两帧匹配点中估计本质矩阵，见 [My_VO.cpp:1800](/home/xywu/桌面/SLAM/Base/My_VO.cpp:1800) 到 [My_VO.cpp:1808](/home/xywu/桌面/SLAM/Base/My_VO.cpp:1808)。

RANSAC 的作用是：

1. 在所有匹配点里随机抽样。
2. 用抽样结果估计一个候选本质矩阵。
3. 检查全部匹配点里哪些点支持这个模型。
4. 重复多次，选出“支持点最多、误差最小”的那组模型。

它解决的问题是：光流跟踪得到的点不可能全对，里面会混入误跟踪、遮挡、动态物体上的点。如果直接拿全部点估计位姿，少量错误点就可能把结果带偏。

### 1.2 内点是什么

内点的意思是：

- 这对匹配点符合当前估计出的极几何关系
- 也就是它支持当前估计出来的本质矩阵 / 相对位姿

外点则是：

- 不符合当前几何模型的点
- 常见原因是误匹配、动态物体、光流漂移或遮挡

### 1.3 mask 是什么

`mask` 是 OpenCV 返回的逐点标记数组，见 [My_VO.h:68](/home/xywu/桌面/SLAM/Base/My_VO.h:68) 和 [My_VO.cpp:1374](/home/xywu/桌面/SLAM/Base/My_VO.cpp:1374)。

语义是：

- 非 0：这个匹配点是内点，保留
- 0：这个匹配点是外点，丢弃

### 1.4 为什么 `recoverPose` 也会得到 mask 和内点

程序在 `findEssentialMat` 后又调用了 `recoverPose(...)`，见 [My_VO.cpp:1809](/home/xywu/桌面/SLAM/Base/My_VO.cpp:1809) 到 [My_VO.cpp:1817](/home/xywu/桌面/SLAM/Base/My_VO.cpp:1817)。

`recoverPose` 不只是把本质矩阵分解成 `R` 和 `t`，它还会进一步判断：

- 哪些点真正支持当前恢复出来的这组位姿
- 哪些点虽然在 RANSAC 阶段像内点，但在当前位姿下其实不合理

所以这里的 `mask` 可以理解成：

- “支持当前 `recoverPose` 解的那批匹配点”

### 1.5 `compactPointsByMask()` 是干什么的

函数 `compactPointsByMask(prev_points, curr_points, mask)` 会按 `mask` 同步压缩两帧点集，见 [My_VO.cpp:1374](/home/xywu/桌面/SLAM/Base/My_VO.cpp:1374) 到 [My_VO.cpp:1395](/home/xywu/桌面/SLAM/Base/My_VO.cpp:1395)。

它做的不是“重新算内点”，而是：

1. 遍历所有匹配点。
2. 只保留 `mask` 里标为内点的那些点。
3. 用过滤后的点集替换原来的 `prev_points` 和 `curr_points`。

这样后面的流程，例如：

- 继续光流跟踪
- 三角化
- 构建短时地图
- PnP

都会建立在更干净的对应点上，减少错误累积。

---

## 2. keyframe 在整个流程中是干什么的？

在当前程序里，`keyframe` 不是闭环 SLAM 里的长期关键帧，而是“短时地图的参考帧”。

它的核心作用是：

- 保存一批短时间可复用的 3D 地图点
- 保存这些 3D 点在某张参考图像中的 2D 观测
- 供后续若干帧做 3D-2D PnP 使用

相关状态在 [My_VO.cpp:1700](/home/xywu/桌面/SLAM/Base/My_VO.cpp:1700) 到 [My_VO.cpp:1712](/home/xywu/桌面/SLAM/Base/My_VO.cpp:1712)：

- `keyframe_image`
- `keyframe_features`
- `keyframe_landmarks`
- `keyframe_pose`
- `keyframe_frame_id`
- `keyframe_map_frame_id`
- `keyframe_initial_landmark_count`

### 2.1 它们分别代表什么

- `keyframe_image`
  - 当前短时地图参考图像
- `keyframe_features`
  - 这些地图点在参考图像中的 2D 位置
- `keyframe_landmarks`
  - 对应的 3D 地图点
- `keyframe_frame_id`
  - 当前参考观测已经推进到哪一帧
- `keyframe_map_frame_id`
  - 当前这批 3D 地图是在哪一帧重建出来的
- `keyframe_initial_landmark_count`
  - 这批地图刚生成时的点数，用来计算保留率

### 2.2 keyframe 在流程中怎么用

每一帧程序先走 2D-2D 主干，得到 `base_geometry_pose`。  
如果当前还保留着一批可用短时地图，程序就尝试：

1. 把 `keyframe_features` 从 `keyframe_image` 跟踪到当前帧。
2. 形成 `3D 点 -> 当前帧 2D 点` 的对应关系。
3. 用这些 3D-2D 对应做 PnP。
4. 如果 PnP 解足够可靠，就让它接管当前最终主干位姿。

对应代码在 [My_VO.cpp:1877](/home/xywu/桌面/SLAM/Base/My_VO.cpp:1877) 到 [My_VO.cpp:1958](/home/xywu/桌面/SLAM/Base/My_VO.cpp:1958)。

### 2.3 keyframe 为什么要重建

因为这不是长期地图，而是短时地图。  
当前代码会在以下情况尝试重建 keyframe 地图，见 [My_VO.cpp:1970](/home/xywu/桌面/SLAM/Base/My_VO.cpp:1970) 到 [My_VO.cpp:1975](/home/xywu/桌面/SLAM/Base/My_VO.cpp:1975)：

- `keyframe_landmarks` 为空
- 地图点数下降到太少
- 地图寿命过长
- 当前保留率太低

也就是说，keyframe 的本质作用不是长期记忆，而是“给最近几帧的 PnP 提供稳定的 3D 参考”。

---

## 3. 三角化的原理是什么？

三角化的核心直觉是：

- 同一个空间点在两张图像中各有一个像素位置
- 已知两帧相机之间的相对位姿
- 就可以反推出这个点的 3D 位置

直观上，你可以把它想成：

- 第一台相机通过像素点射出一条视线
- 第二台相机通过对应像素点再射出一条视线
- 这两条视线在空间中的交会位置，就是该点的 3D 位置

当然，真实数据里有噪声，两条线往往不会精确相交，所以算法求的是“最合理的交会点”。

### 3.1 当前代码是怎么做的

当前程序里的三角化入口是：

- `triangulateWorldLandmarks(...)`，见 [My_VO.cpp:511](/home/xywu/桌面/SLAM/Base/My_VO.cpp:511)
- 它内部调用 `triangulation(...)`，见 [My_VO.cpp:1406](/home/xywu/桌面/SLAM/Base/My_VO.cpp:1406)

步骤是：

1. 把两帧像素点通过 `pixel2cam()` 转成归一化相机坐标。
2. 构造两帧投影矩阵。
3. 调用 `triangulatePoints(...)` 得到齐次 4D 点。
4. 对每个点除以 `w`，得到欧式 3D 坐标。
5. 这些点先位于参考相机坐标系下，再被转换到世界坐标系中。

转换到世界坐标系这一段在 [My_VO.cpp:574](/home/xywu/桌面/SLAM/Base/My_VO.cpp:574) 到 [My_VO.cpp:577](/home/xywu/桌面/SLAM/Base/My_VO.cpp:577)。

### 3.2 为什么三角化出来的点还要再筛一遍

因为不是每个三角化点都可信。  
当前代码会过滤：

- `NaN` / `Inf`
- 在参考相机后方的点
- 深度太近或太远的点
- 两帧重投影误差过大的点

见 [My_VO.cpp:552](/home/xywu/桌面/SLAM/Base/My_VO.cpp:552) 到 [My_VO.cpp:609](/home/xywu/桌面/SLAM/Base/My_VO.cpp:609)。

程序不是为了“把所有点都三角化出来”，而是为了保留一批适合后续 PnP 的可靠 3D 点。

---

## 4. 三角化为什么要有“最低匹配、重投影误差、中位视差、基线”这些参数？

这些阈值的共同目标是：

- 避免在几何条件很差时，生成不可靠的 3D 点

当前阈值定义在 [My_VO.cpp:42](/home/xywu/桌面/SLAM/Base/My_VO.cpp:42) 到 [My_VO.cpp:46](/home/xywu/桌面/SLAM/Base/My_VO.cpp:46)：

- `kMinTriangulatedLandmarks = 4`
- `kTriangulationReprojectionError = 3.0`
- `kMinTriangulationParallaxPixels = 8.0`
- `kMinTriangulationBaseline = 0.15`
- 以及深度阈值 `kMinLandmarkDepth`、`kMaxLandmarkDepth`

### 4.1 最低匹配数是什么

对应：

- `kMinTriangulatedLandmarks`
- `shouldTriangulateTracks()`，见 [My_VO.cpp:499](/home/xywu/桌面/SLAM/Base/My_VO.cpp:499) 到 [My_VO.cpp:508](/home/xywu/桌面/SLAM/Base/My_VO.cpp:508)

意思是：

- 如果当前两帧能用于三角化的匹配点太少，就不要建图

原因：

- 点太少时，三角化出来的 3D 点既稀疏又不稳
- 后面拿它们去做 PnP，约束力也不够

### 4.2 基线是什么

对应：

- `kMinTriangulationBaseline`

基线就是：

- 两帧相机中心之间的真实位移长度

在这份程序里，基线来自真值尺度。  
如果两帧相机移动太小，即使像素上有少量变化，三角化出来的深度也会非常不稳定。

你可以把它理解成：

- 相机位置变化太小，两条视线夹角太小
- 夹角太小，3D 交会位置对噪声极端敏感

### 4.3 中位视差是什么

对应：

- `medianParallax()`
- `kMinTriangulationParallaxPixels`

视差就是：

- 同一个点在前后两帧图像里移动了多少像素

当前代码对所有匹配点的像素位移求中位数，见 [My_VO.cpp:480](/home/xywu/桌面/SLAM/Base/My_VO.cpp:480) 到 [My_VO.cpp:497](/home/xywu/桌面/SLAM/Base/My_VO.cpp:497)。

为什么要用中位数：

- 比平均值更稳健
- 不容易被少数异常误匹配点带偏

为什么视差太小不行：

- 视差太小意味着两次观测几乎没分开
- 三角化出来的深度对像素噪声非常敏感

### 4.4 重投影误差是什么

对应：

- `kTriangulationReprojectionError`
- [My_VO.cpp:579](/home/xywu/桌面/SLAM/Base/My_VO.cpp:579) 到 [My_VO.cpp:596](/home/xywu/桌面/SLAM/Base/My_VO.cpp:596)

做法是：

1. 先三角化出一个 3D 点。
2. 再把这个 3D 点投影回前一帧和当前帧图像。
3. 看投影位置和原始观测点差多少像素。

这个差值就是重投影误差。

如果误差太大，通常说明：

- 原始匹配可能错了
- 或三角化深度不可靠

所以程序会直接丢弃这个点。

### 4.5 这些条件合起来是什么意思

程序的策略可以概括成：

- 匹配点数量要够
- 相机真实位移要够
- 图像视差要够
- 三角化后的点要在相机前方
- 深度范围要合理
- 投回图像后还要解释得通原始观测

全部满足，才允许进入 `keyframe_landmarks`，再用于后续 PnP。

---

## 5. “特征不足时重检测”应该怎么理解？

这一段对应主循环中的 [My_VO.cpp:1768](/home/xywu/桌面/SLAM/Base/My_VO.cpp:1768) 到 [My_VO.cpp:1795](/home/xywu/桌面/SLAM/Base/My_VO.cpp:1795)。

很多人第一次看这里会疑惑：

- “在 `prev_image` 上再检测一次，不还是这些特征吗？”

答案是：不是。

### 5.1 为什么不是“还是那些点”

因为当前的 `prev_features` 并不是“`prev_image` 上所有最好的角点”，而是：

- 从更早帧开始一路跟踪幸存下来的那批点

也就是说，它们是“历史遗留下来的活跃点”，不一定还是当前 `prev_image` 上最适合和 `curr_image` 建立对应关系的那批点。

这些旧点可能已经：

- 连续跟踪后越来越少
- 分布不均
- 局部漂移
- 集中在少数区域

### 5.2 重检测真正做了什么

程序的做法是：

1. 先尝试用当前 `prev_features` 直接跟到 `curr_image`。
2. 如果剩下的匹配点太少，就放弃这批旧点。
3. 在 `prev_image` 上重新用 FAST 找一批新的强角点。
4. 再把这批新角点跟踪到 `curr_image`。

所以“重检测”的本质是：

- 不是重试旧点
- 而是基于当前上一帧图像，刷新一套新的候选点集

### 5.3 为什么在 `prev_image` 上重检测，而不是直接在 `curr_image` 上检测

因为这一步是为了估计：

- `prev_image -> curr_image`

的两帧几何关系。  
你必须先有成对的对应点，才能算 `findEssentialMat`。

如果只在 `curr_image` 上检测，只得到当前帧的孤立点，无法直接形成两帧匹配。

因此流程必须是：

1. 在 `prev_image` 上检测
2. 跟踪到 `curr_image`
3. 得到 `prev_points / curr_points`

### 5.4 如果重检测后仍然不够怎么办

程序会：

1. 本帧不更新位姿
2. 把当前已有 pose 直接写入轨迹，保持帧号对齐
3. 改为在 `curr_image` 上重新检测特征
4. 让下一帧从这里重新恢复跟踪

见 [My_VO.cpp:1781](/home/xywu/桌面/SLAM/Base/My_VO.cpp:1781) 到 [My_VO.cpp:1788](/home/xywu/桌面/SLAM/Base/My_VO.cpp:1788)。

所以这一段不是“盲目多试一次”，而是：

- 先刷新上一帧点集再试
- 如果还是失败，就在当前帧布点，给下一帧恢复

---

## 6. `recoverPose` 是怎么把本质矩阵分解成平移和旋转的？

从当前程序视角，`recoverPose` 的输入输出可以先记成：

- 输入：本质矩阵 `E` 和两帧匹配点
- 输出：相对旋转 `R` 和平移方向 `t`

代码调用见：

- 初始化阶段 [My_VO.cpp:1636](/home/xywu/桌面/SLAM/Base/My_VO.cpp:1636) 到 [My_VO.cpp:1638](/home/xywu/桌面/SLAM/Base/My_VO.cpp:1638)
- 主循环 [My_VO.cpp:1809](/home/xywu/桌面/SLAM/Base/My_VO.cpp:1809) 到 [My_VO.cpp:1817](/home/xywu/桌面/SLAM/Base/My_VO.cpp:1817)

### 6.1 核心关系

本质矩阵满足：

```text
E = [t]_x R
```

这里：

- `R` 是相机相对旋转
- `t` 是相机相对平移方向
- `[t]_x` 是由 `t` 构成的叉乘矩阵

所以 `E` 不是任意矩阵，而是“旋转 + 平移方向”揉在一起的特殊矩阵。

### 6.2 为什么要做 SVD

OpenCV 内部会先对 `E` 做奇异值分解：

```text
E = U Σ V^T
```

对理想本质矩阵来说，它的奇异值结构是：

```text
Σ = diag(s, s, 0)
```

这说明本质矩阵本身是有特殊结构的，不是普通 3x3 满秩矩阵。

SVD 的作用是：

- 把一个混在一起的本质矩阵拆成“标准结构 + 坐标变换”

### 6.3 为什么能从中得到平移方向

因为 `[t]_x` 本身的零空间方向和 `t` 有关。  
本质矩阵的 SVD 里，那个对应零奇异值的方向就携带了平移方向信息。

因此可以从 SVD 中恢复：

- `t` 的方向

但只能恢复方向，不能恢复长度。

### 6.4 为什么只能恢复平移方向，不能恢复真实长度

因为两帧单目极几何只约束方向，不约束绝对尺度。  
所以 `recoverPose` 返回的 `translation` 只是方向向量。

当前程序后面还必须调用：

- `getAbsoluteScale(...)`

再把真值相邻帧距离补进去，见 [My_VO.cpp:1460](/home/xywu/桌面/SLAM/Base/My_VO.cpp:1460) 到 [My_VO.cpp:1462](/home/xywu/桌面/SLAM/Base/My_VO.cpp:1462)。

### 6.5 为什么会有多个候选解

从本质矩阵分解出 `R,t` 时，数学上会有若干候选组合。  
`recoverPose` 会进一步做“正深度检验”：

- 用候选位姿解释匹配点
- 看三角化出来的点是否大多数位于两个相机前方

哪组位姿让更多点在相机前方，就更可能是真解。

### 6.6 这部分对当前程序最重要的结论

对读这份代码来说，最需要记住的不是完整推导，而是：

1. `findEssentialMat` 给出 `E`
2. `recoverPose` 从 `E` 恢复 `R` 和 `t`
3. 这个 `t` 只有方向，没有真实长度
4. 程序再用真值 `scale` 把方向变成真实位移

---

## 7. PnP 里面是怎么求解位姿的？数学原理是什么？

PnP 的本质是：

- 已知一批 3D 点
- 已知它们在当前图像中的 2D 位置
- 已知相机内参
- 反求当前相机位姿 `R,t`

### 7.1 数学模型

投影模型可以写成：

```text
s [u v 1]^T = K [R | t] [X Y Z 1]^T
```

这里：

- `(X,Y,Z)` 是世界坐标中的 3D 点
- `(u,v)` 是像素坐标
- `K` 是相机内参
- `R,t` 是要求的位姿
- `s` 是深度尺度因子

PnP 的任务就是：

- 找一个 `R,t`
- 让所有 3D 点投影回图像后的结果尽量贴近真实观测点

### 7.2 它真正最小化的是什么

PnP 本质上是在最小化重投影误差：

```text
min Σ || u_i^obs - u_i^proj(R,t) ||^2
```

意思是：

- `u_i^obs`：第 `i` 个点真实观测到的像素位置
- `u_i^proj(R,t)`：把第 `i` 个 3D 点用候选位姿投影回图像得到的位置

误差越小，这个位姿就越合理。

### 7.3 当前代码里怎么做

PnP 主求解在 `solvePoseWithPnP(...)` 中，见 [My_VO.cpp:669](/home/xywu/桌面/SLAM/Base/My_VO.cpp:669)。

它分两步：

1. `solvePnPRansac(...)`
   - 先从 3D-2D 对应里剔除外点
   - 当前用的是 `SOLVEPNP_EPNP`
2. `solvePnP(..., SOLVEPNP_ITERATIVE)`
   - 再只用内点做一次 refine

所以当前实现不是“一步直接求完”，而是：

- 先快算一个鲁棒初值
- 再做非线性优化细化

### 7.4 为什么要给 PnP 一个初值

当前代码会先把 2D-2D 主干位姿转成 PnP 可用的外参初值，见 [My_VO.cpp:364](/home/xywu/桌面/SLAM/Base/My_VO.cpp:364) 到 [My_VO.cpp:369](/home/xywu/桌面/SLAM/Base/My_VO.cpp:369) 与 [My_VO.cpp:683](/home/xywu/桌面/SLAM/Base/My_VO.cpp:683)。

也就是说，PnP 并不是完全盲求，而是：

- 从当前 2D-2D 主干位姿附近开始优化

这样更容易：

- 收敛到接近当前主干解的合理局部最优
- 减少突然跳到错误解的概率

### 7.5 PnP 求出来的位姿是什么形式

`solvePnP` 输出的是世界到相机外参：

```text
X_cam = R_wc X_world + t_wc
```

但程序内部统一使用 `PoseRecord`：

- `R_cw`
- `t_cw`

所以代码里还要通过 `poseFromPnP()` 转回内部表示，见 [My_VO.cpp:328](/home/xywu/桌面/SLAM/Base/My_VO.cpp:328) 到 [My_VO.cpp:339](/home/xywu/桌面/SLAM/Base/My_VO.cpp:339)。

---

## 8. PnP 是怎么辅助修正当前输出姿态的？

这一点必须按当前代码解释。  
旧版本里曾经存在“PnP 只改输出旋转”的分支，但当前实现已经不是这样了。

### 8.1 当前主干有两条线

当前主循环里同时维护：

- `two_view_pose`
  - 纯 2D-2D / `recoverPose` 诊断支路
- `geometry_pose`
  - 最终对外发布的主干位姿

见 [My_VO.cpp:1649](/home/xywu/桌面/SLAM/Base/My_VO.cpp:1649) 到 [My_VO.cpp:1652](/home/xywu/桌面/SLAM/Base/My_VO.cpp:1652) 和 [My_VO.cpp:2020](/home/xywu/桌面/SLAM/Base/My_VO.cpp:2020) 到 [My_VO.cpp:2023](/home/xywu/桌面/SLAM/Base/My_VO.cpp:2023)。

### 8.2 先用 2D-2D 生成基础解

每一帧程序先正常计算：

- `base_two_view_pose`
- `base_geometry_pose`

其中：

- `base_two_view_pose` 是纯 2D-2D 支路
- `base_geometry_pose` 是当前最终主干默认候选

见 [My_VO.cpp:1863](/home/xywu/桌面/SLAM/Base/My_VO.cpp:1863) 到 [My_VO.cpp:1865](/home/xywu/桌面/SLAM/Base/My_VO.cpp:1865)。

### 8.3 keyframe 提供 3D-2D 对应

如果当前有可用短时地图，程序会：

1. 把 `keyframe_features` 从 `keyframe_image` 跟踪到当前帧
2. 形成 `keyframe_landmarks -> pnp_curr_features` 的 3D-2D 对应
3. 用这些对应求 `pnp_pose`

见 [My_VO.cpp:1877](/home/xywu/桌面/SLAM/Base/My_VO.cpp:1877) 到 [My_VO.cpp:1901](/home/xywu/桌面/SLAM/Base/My_VO.cpp:1901)。

### 8.4 PnP 不会无条件接管

PnP 解出来以后，程序先做两层检查：

#### 第一层：一致性检查

函数：

- `isPnPPoseConsistent(...)`

它比较：

- `pnp_pose` 和 `base_geometry_pose` 的平移差
- `pnp_pose` 和 `base_geometry_pose` 的旋转差

如果差太大，直接拒绝。

#### 第二层：是否允许接管主干

函数：

- `shouldAdoptPnPPose(...)`

除了看内点数外，它还看：

- 地图点当前保留率
- PnP 内点比例
- 地图年龄
- 当前 2D-2D 跟踪点数
- 当前帧是不是处于允许 PnP 接管的小尺度区间

这说明当前代码虽然允许 PnP 接管整个位姿，但门控仍然很严格。

### 8.5 真正的修正发生在哪里

真正的接管在 [My_VO.cpp:1917](/home/xywu/桌面/SLAM/Base/My_VO.cpp:1917) 到 [My_VO.cpp:1919](/home/xywu/桌面/SLAM/Base/My_VO.cpp:1919)：

```text
estimated_geometry_pose = pnp_pose;
status_text = "pnp";
```

这意味着：

- 当前帧最终主干位姿直接改成 PnP 求出来的绝对位姿
- 不再是“只改旋转”
- 当前实现里，PnP 接管的是整个位姿

### 8.6 最终轨迹怎么写

每一帧结束时：

- `two_view_pose = base_two_view_pose`
- `geometry_pose = estimated_geometry_pose`

然后：

- `trajectory.push_back(geometry_pose)`
- `geometry_trajectory.push_back(two_view_pose)`

见 [My_VO.cpp:2020](/home/xywu/桌面/SLAM/Base/My_VO.cpp:2020) 到 [My_VO.cpp:2026](/home/xywu/桌面/SLAM/Base/My_VO.cpp:2026)。

所以当前输出语义是：

- `trajectory` / `position.txt`
  - 当前最终主干，可在门控通过时被 PnP 接管
- `geometry_trajectory` / `geometry_position.txt`
  - 纯 2D-2D 诊断轨迹

### 8.7 一句话总结当前版 PnP 的角色

当前版 PnP 的作用不是“小修一下旋转”，而是：

- 在与 2D-2D 主干高度一致、地图质量足够好的时候
- 允许 3D-2D PnP 结果整个位姿接管最终主干

但它仍然是保守接管，不是默认主干。

---

## 9. 读这份代码时最容易混淆的当前版结论

最后把和这些问题最相关的几个“当前版结论”集中列一下。

### 9.1 `trajectory` 和 `geometry_trajectory` 不再表示旧文档中的那组含义

当前代码中：

- `trajectory` 保存最终主干 `geometry_pose`
- `geometry_trajectory` 保存纯 2D-2D 的 `two_view_pose`

见 [My_VO.cpp:1714](/home/xywu/桌面/SLAM/Base/My_VO.cpp:1714) 到 [My_VO.cpp:1724](/home/xywu/桌面/SLAM/Base/My_VO.cpp:1724)。

### 9.2 PnP 当前可以接管整个位姿

不是旧版本里“只改旋转”。  
当前通过门控后，`estimated_geometry_pose = pnp_pose`。

### 9.3 三角化不是长期建图

当前三角化的核心目标是：

- 生成短时 3D 地图点
- 给后续几帧的 PnP 提供 3D-2D 约束

不是构建长期全局地图。

### 9.4 `recoverPose` 给的是方向，不是完整平移

当前程序仍然需要：

- `getAbsoluteScale(...)`

来从真值恢复平移长度。

### 9.5 “重检测”不是重复旧点，而是刷新当前上一帧点集

它的目的不是机械重试，而是给当前 `prev -> curr` 这对图像重新选一批更适合建立对应关系的角点。
