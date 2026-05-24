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

示例：
```bash
cmake -S Base -B build
cmake --build build -j
./build/VO 300
```
