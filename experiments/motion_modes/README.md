# Motion Modes

本目录只放运动模式实验的片段定义，不放算法代码和运行脚本。

## 子目录

- `01_static/`：近静止或极低速抖动片段
- `02_rotation_dominant/`：转动占主导片段
- `03_low_translation/`：低位移片段
- `04_nominal_translation/`：正常平移片段
- `05_fast_translation/`：高速平移片段
- `06_transition/`：启动、减速、急变等过渡片段
- `README.md`：本目录说明

## 文件格式

每个模式子目录下都有一个 `segments.csv`，用于定义候选片段。字段如下：

- `segment_id`：片段唯一标识
- `warmup_start_frame`：预热起点
- `start_frame`：评估起始帧，含
- `end_frame`：评估结束帧，含
- `sequence`：数据序列编号
- `is_primary`：是否为该模式代表片段，`1` 表示是
- `notes`：选段依据或补充说明

## 说明

- 当前所有片段均来自 `KITTI 07`
- 本目录只负责描述“跑哪一段”
- 运行说明见项目根目录的 `README.md`
