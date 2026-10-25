# 依赖说明

`ros2_sources/px4_msgs` 来自打包时本机工作区，包含 VehicleStatus v1 和 direct_actuator。
源码来源 Git 状态在 `09_PROVENANCE/px4_msgs`；它是快照，不带子模块 Git 元数据。
不复制 install/build 的绝对软链接，也不复制二进制虚拟环境。

`environment.json` 是本地观测，不是远程锁版本；`requirements-px4-venv.txt` 是原仓库历史固定依赖，
其 NumPy 等版本可能与现有 ROS 二进制不兼容，不能不经核对就在机载主机覆盖安装。
优先复用已工作的 ROS 与系统 numpy。离线分析可在单独环境安装下面的未锁定依赖表，按运行平台解析。

`requirements-debugging.txt` 不含 rclpy、px4_msgs、sensor_msgs：这些由 ROS 及兼容消息工作区提供。
Gazebo/PX4 构建依赖另由完整固件的 setup 文档管理；本包不是跨架构离线安装镜像。
