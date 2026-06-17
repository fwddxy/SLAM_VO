# Experiments

本目录只放实验定义，不放算法源码、构建产物和运行脚本。

## 子目录

- `motion_modes/`
  - 运动模式片段定义；每个模式目录下用 `segments.csv` 描述候选片段

## 说明

- 当前实验定义集中在 `experiments/motion_modes/`
- 负责“跑哪一段”的描述
- 真正执行实验的是项目根目录下的 `scripts/run_motion_mode.py`
- 运行方法见项目根目录的 [README.md](/home/xywu/桌面/SLAM/README.md)
