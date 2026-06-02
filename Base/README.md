# Base

本目录只保留单目前端视觉里程计源码和构建文件：
- `My_VO.cpp`
- `My_VO.h`
- `CMakeLists.txt`
- `README.md`

所有运行输出统一放在项目根目录的 `output/` 下。

## 运行程序

运行前确认项目根目录下存在：
- `Kitti/camera-info.txt`
- `Kitti/gt-tum07.txt`
- `Kitti/left/%06d.png`

编译：

```bash
cmake -S Base -B build
cmake --build build -j
```

运行完整序列：

```bash
VO_ENABLE_GUI=0 ./build/VO
```

只运行前 300 帧：

```bash
VO_ENABLE_GUI=0 ./build/VO 300
```

程序输出：
- `output/position/position.txt`：TUM 格式估计轨迹，格式为 `timestamp tx ty tz qx qy qz qw`
- `output/traj/map2.png`：程序自绘的真值/VO 轨迹对比图

可选环境变量：
- `VO_ENABLE_GUI=0`：强制关闭 OpenCV 窗口，适合无桌面环境
- `VO_VERBOSE_LOG=1`：输出更详细的重检测和跳帧日志
- `VO_IMAGE_SCALE=0.5`：缩小输入图像以提升速度，默认 `1.0`

## 使用 evo

如果未安装 evo，可安装到用户级全局目录：

```bash
python3 -m pip install --user --break-system-packages evo
```

确认 `~/.local/bin` 在 PATH 中。若刚安装完后命令不可用，重新打开 VSCode 终端，或执行：

```bash
source ~/.bashrc
```

首次使用前建议把绘图后端设为 `Agg`，这样无 GUI 环境也能保存 PDF/PNG：

```bash
evo_config set plot_backend Agg
mkdir -p output/ape output/rpe output/traj
```

如果运行时提示 Matplotlib 无法写入缓存目录，可执行：

```bash
export MPLCONFIGDIR=/tmp/evo-mpl
```

APE 绝对轨迹误差：

```bash
evo_ape tum Kitti/gt-tum07.txt output/position/position.txt -va --plot_mode xz --save_plot output/ape/evo_ape_plot.pdf --no_warnings | tee output/ape/evo_ape_metrics.txt
```

RPE 相对位姿误差：

```bash
evo_rpe tum Kitti/gt-tum07.txt output/position/position.txt -va --plot_mode xz --save_plot output/rpe/evo_rpe_plot.pdf --no_warnings | tee output/rpe/evo_rpe_metrics.txt
```

轨迹叠加图和轨迹统计：

```bash
evo_traj tum output/position/position.txt --ref Kitti/gt-tum07.txt --sync -a --plot_mode xz --save_plot output/traj/evo_traj_xz.pdf --save_table output/traj/evo_traj_table.csv --no_warnings
```

## 输出目录

- `output/position/`：VO 输出的 TUM 轨迹
- `output/ape/`：APE 指标和图
- `output/rpe/`：RPE 指标和图
- `output/traj/`：evo 轨迹图、统计表和程序自绘轨迹图
