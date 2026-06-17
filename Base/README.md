# Base

本目录只放 VO 主程序源码、头文件和构建入口。

## 文件

- `My_VO.cpp`：主程序实现，包含数据读取、特征跟踪、位姿恢复、轨迹输出和命令行参数解析
- `My_VO.h`：数据结构、函数声明和头文件依赖
- `CMakeLists.txt`：`VO` 可执行文件的构建配置
- `README.md`：本目录说明
- `VO程序详解.md`：对当前实现的逐段说明

## 说明

- 本目录不保存实验定义和批量实验脚本
- 编译入口是本目录下的 `CMakeLists.txt`
- 编译产物默认输出到项目根目录的 `build/`
- 运行说明见项目根目录的 [README.md](/home/xywu/桌面/SLAM/README.md)
- 如果要读实现细节，优先看 [VO程序详解.md](/home/xywu/桌面/SLAM/Base/VO程序详解.md)
