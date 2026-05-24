# Base

当前 `Base` 目录只保留一版单目前端视觉里程计实现。

运行前提：
- 项目根目录下存在 `Kitti/` 数据与标定文件
- 系统已安装 `OpenCV 4`

程序行为：
- 自动定位项目根目录下的 `Kitti/camera-info.txt`
- 自动定位项目根目录下的 `Kitti/gt-tum07.txt`
- 自动定位项目根目录下的 `Kitti/left/%06d.png`
- 将 VO 轨迹写入 `Base/position.txt`
- 默认按原始分辨率运行，优先保证长序列稳定性
- 轨迹图会按当前轨迹范围自动缩放并居中显示，同时标注帧数与坐标范围
- 终端默认会周期性输出当前处理帧、跟踪点数和位姿状态

示例：
```bash
cmake -S Base -B build
cmake --build build -j
./build/VO
```

可选环境变量：
- `VO_ENABLE_GUI=0`：强制关闭 GUI；默认在检测到可用本地图形会话时自动显示窗口
- `VO_VERBOSE_LOG=1`：输出重检测和坏帧跳过日志
- `VO_IMAGE_SCALE=0.5`：按比例缩小输入图像以换取速度，默认 `1.0`

可选命令行参数：
- `./build/VO 300`：仅处理前 `300` 帧；不传参数时默认处理整个数据集
