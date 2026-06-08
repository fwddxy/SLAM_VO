# Scripts

本目录只放实验流程脚本，不放 VO 算法源码。

## 文件

- `run_motion_mode.py`：读取 `experiments/motion_modes/*/segments.csv`，调用 `build/VO` 跑片段，整理轨迹、日志和评估输入
- `summarize_motion_modes.py`：读取各片段输出目录中的 `segment.json`、`run.log`、`evo` 指标文件，生成汇总表
- `README.md`：本目录说明

## 输出关系

- `run_motion_mode.py` 的输出目录是 `output/motion_modes/<mode>/<segment_id>/`
- `summarize_motion_modes.py` 的输出目录是 `output/motion_modes/summary/`

## 说明

- 本目录脚本依赖已经编译好的 `build/VO`
- 运行说明见项目根目录的 `README.md`
