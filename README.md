# SLAM

本项目当前包含两层内容：

- `Base/`：C++ 视觉里程计主程序
- `experiments/motion_modes/` + `scripts/`：运动模式实验定义、批量运行和结果汇总

## 运行前准备

项目根目录下需要存在：

- `Kitti/camera-info.txt`
- `Kitti/gt-tum07.txt`
- `Kitti/left/%06d.png`

## 编译

```bash
cmake -S Base -B build
cmake --build build -j
```

编译后可执行文件为：

```bash
./build/VO
```

## 运行完整序列

```bash
VO_ENABLE_GUI=0 ./build/VO
```

只运行前 300 帧：

```bash
VO_ENABLE_GUI=0 ./build/VO 300
```

完整序列默认输出目录：

- `output/full_run/position/position.txt`
- `output/full_run/traj/map2.png`

## 运行指定片段

```bash
VO_ENABLE_GUI=0 ./build/VO --start-frame 695 --end-frame 714 --output-root output/motion_modes/01_static/static_01
```

或者：

```bash
VO_ENABLE_GUI=0 ./build/VO --start-frame 785 --frame-count 20 --output-root output/motion_modes/05_fast_translation/fast_01
```

当使用 `--output-root` 时，输出会写到对应根目录下的：

- `position/position.txt`
- `traj/map2.png`

## 运行运动模式实验

跑全部模式：

```bash
python3 scripts/run_motion_mode.py --all
```

只跑每种模式的代表片段：

```bash
python3 scripts/run_motion_mode.py --all --primary-only
```

只跑某一种模式：

```bash
python3 scripts/run_motion_mode.py --mode 01_static
```

只跑某一个片段：

```bash
python3 scripts/run_motion_mode.py --mode 05_fast_translation --segment-id fast_01
```

汇总结果：

```bash
python3 scripts/summarize_motion_modes.py
```

实验结果输出到：

- `output/motion_modes/<mode>/<segment_id>/`
- `output/motion_modes/summary/`

## 使用 evo

如果未安装 `evo`：

```bash
python3 -m pip install --user --break-system-packages evo
source ~/.bashrc
```

建议设置：

```bash
evo_config set plot_backend Agg
mkdir -p output/full_run/ape output/full_run/rpe output/full_run/traj
export MPLCONFIGDIR=/tmp/evo-mpl
```

APE：

```bash
evo_ape tum Kitti/gt-tum07.txt output/full_run/position/position.txt -va --plot_mode xz --save_plot output/full_run/ape/evo_ape_plot.pdf --no_warnings | tee output/full_run/ape/evo_ape_metrics.txt
```

RPE：

```bash
evo_rpe tum Kitti/gt-tum07.txt output/full_run/position/position.txt -va --plot_mode xz --save_plot output/full_run/rpe/evo_rpe_plot.pdf --no_warnings | tee output/full_run/rpe/evo_rpe_metrics.txt
```

轨迹图：

```bash
evo_traj tum output/full_run/position/position.txt --ref Kitti/gt-tum07.txt --sync -a --plot_mode xz --save_plot output/full_run/traj/evo_traj_xz.pdf --save_table output/full_run/traj/evo_traj_table.csv --no_warnings
```

## 输出目录

- `output/full_run/`：完整序列结果
- `output/motion_modes/`：运动模式实验结果

