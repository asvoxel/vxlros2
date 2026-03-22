# VxlROS2

VxlSense SDK 的 ROS2 封装，将 VxlSense 深度相机集成到 ROS2 生态系统中。

## 项目结构

```
VxlROS2/
├── vxlsense-sdk/        # VxlSense SDK 发行包 (解压到此)
├── vxl_camera/          # 主节点包 (C++)
├── vxl_camera_msgs/     # 自定义消息/服务定义
├── vxl_description/     # URDF 模型描述
└── docs/                # 设计文档
```

## 技术约束

- **C++ 标准**: C++17
- **ROS2 版本**: Humble+ (最低支持)
- **构建系统**: ament_cmake
- **节点架构**: 必须使用 rclcpp_components 组件化节点，支持 intra-process 零拷贝
- **SDK 依赖**: VxlSense SDK 发行包解压到 `vxlsense-sdk/`，不引用源码仓库
- **SDK 路径优先级**: CMake 参数 > 环境变量 VXL_SDK_DIR > 默认 `../vxlsense-sdk/`
- **API 统一**: 全部使用 VxlSense C++ API (vxl.hpp)，禁止直接调用 C API

## 编码规范

- 文件名: snake_case (如 `vxl_camera_node.cpp`)
- 类名: PascalCase (如 `VxlCameraNode`)
- ROS2 话题/服务名: snake_case，以 `~/` 前缀表示私有
- 参数名: snake_case，按功能分组 (如 `color.width`, `depth.enabled`)
- 所有公共 API 函数必须有错误处理，不允许裸异常传播

## 关键设计决策

1. **单节点多传感器**: 一个节点管理一个设备的所有传感器 (color/depth/ir)
2. **帧同步**: 使用 VxlSense Pipeline API 的 SyncMode 进行硬件级同步
3. **输出模式**: 默认 RGBD 或 RGB+Depth 两种模式，IR/单流等仅供调测
4. **参数系统**: 通过 ROS2 动态参数暴露 VxlSense 选项，支持运行时调整
5. **资源释放顺序**: Stream → Sensor → Device → Context，严格遵守

## 构建与测试

```bash
colcon build --packages-select vxl_camera_msgs vxl_camera
colcon test --packages-select vxl_camera
```

## 参考

- 竞品 ROS2 集成: `../VxlSense/comps/{orbbec,realsense,stereolabs}`
- VxlSense SDK API: `vxlsense-sdk/include/vxl*.hpp`
- SDK 发行包: https://github.com/asvoxel/VxlSense/releases
