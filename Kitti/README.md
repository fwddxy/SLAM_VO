# Kitti

本目录保存当前工程运行所需的 KITTI 07 数据和派生文件。

## 文件

- `left/`
  - KITTI 07 左目图像，共 `1101` 张，已去畸变
- `camera-info.txt`
  - 相机内参
  - 相机坐标系约定：`x` 向右，`y` 向下，`z` 向前
- `times07.txt`
  - 每张图像对应的时间戳，共 `1101` 行，单位秒
- `gt-tum07.txt`
  - 按 `times07.txt` 对齐后的真值轨迹，格式为 TUM

## `gt-tum07.txt` 格式

每行格式：

```text
timestamp tx ty tz qx qy qz qw
```

第一行对应 `000000.png`，含义依次是：

- 时间戳
- 相机位置 `x y z`，单位米
- 相机姿态四元数 `qx qy qz qw`
