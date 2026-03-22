# VxlROS2 架构设计

## 1. 总体架构

```
┌──────────────────────────────────────────────────────┐
│                    ROS2 Ecosystem                     │
│  ┌────────────┐  ┌──────────────┐  ┌───────────────┐│
│  │ vxl_camera  │  │vxl_camera_msgs│  │vxl_description││
│  │  (C++ node) │  │ (msg/srv)    │  │   (URDF)     ││
│  └──────┬──────┘  └──────────────┘  └───────────────┘│
│         │ C++ API (vxl.hpp)                           │
│  ┌──────┴──────────────────────────┐                  │
│  │    VxlSense SDK (libasvxl.a)    │                  │
│  │  Context → Device → Pipeline    │                  │
│  │          → FrameSet → Frame     │                  │
│  └─────────────────────────────────┘                  │
└──────────────────────────────────────────────────────┘
```

## 2. SDK 集成

SDK 发行包解压到 `vxlsense-sdk/`，通过 CMake imported target 链接：

```
VxlROS2/
├── vxlsense-sdk/           ← SDK 发行包
│   ├── include/             # vxl.hpp, vxl_types.hpp, ...
│   └── lib/{linux,darwin}/  # libasvxl.a
├── vxl_camera/
├── vxl_camera_msgs/
└── vxl_description/
```

CMake 路径优先级：`-DVXL_SDK_DIR` > `$VXL_SDK_DIR` 环境变量 > `../vxlsense-sdk/`

全部使用 C++ API（`vxl.hpp`），禁止直接调用 C API。

## 3. 包结构（实际代码）

### 3.1 vxl_camera_msgs

```
vxl_camera_msgs/
├── msg/
│   ├── DeviceInfo.msg      # name, serial_number, firmware_version, vid, pid
│   ├── Extrinsics.msg      # float64[9] rotation, float64[3] translation
│   ├── Metadata.msg        # timestamp_us, frame_number, exposure_us, gain
│   └── RGBD.msg            # header, rgb, depth, rgb_camera_info, depth_camera_info
├── srv/
│   ├── GetDeviceInfo.srv
│   ├── GetInt32.srv         # option_name → value
│   └── SetInt32.srv         # option_name, value → success
├── CMakeLists.txt
└── package.xml
```

### 3.2 vxl_camera

```
vxl_camera/
├── include/vxl_camera/
│   ├── vxl_camera_node.hpp       # 主节点类 + OutputMode 枚举
│   ├── stream_manager.hpp        # Pipeline 封装 + poll 线程
│   └── point_cloud_generator.hpp # Z16 → PointCloud2
├── src/
│   ├── vxl_camera_node.cpp       # 节点实现 (~400行)
│   ├── stream_manager.cpp        # Pipeline 生命周期管理
│   ├── point_cloud_generator.cpp # 深度图 → 点云转换
│   └── main.cpp                  # 独立可执行入口
├── config/
│   ├── default.yaml              # 默认参数
│   └── vxl435.yaml               # VXL435 专用参数
├── launch/
│   ├── vxl_camera.launch.py      # 单相机
│   └── multi_camera.launch.py    # 多相机
├── test/
│   ├── test_output_mode.cpp      # 参数/消息单元测试
│   ├── test_point_cloud.cpp      # 点云生成单元测试
│   └── test_integration.launch.py # 集成测试（需设备）
├── CMakeLists.txt
└── package.xml
```

### 3.3 vxl_description

```
vxl_description/
├── urdf/vxl_camera.urdf.xacro   # 参数化 URDF（支持 tf_prefix）
├── launch/description.launch.py
├── CMakeLists.txt
└── package.xml
```

## 4. 核心类设计

### 4.1 VxlCameraNode

```cpp
class VxlCameraNode : public rclcpp::Node {
  enum class OutputMode { RGBD, RGBDepth, IR, DepthOnly, ColorOnly, All };

  // 生命周期
  VxlCameraNode(options);      // try { init... } catch { shutdownDevice(); throw; }
  ~VxlCameraNode();            // shutdownDevice()

  // 初始化链
  declareParameters();         // 声明参数 + 注册动态回调
  initDevice();                // Context → Device → open → 读取内参/外参
  setupPublishers();           // 按 output_mode 创建对应发布者
  setupServices();             // get_device_info, get/set_option, hw_reset
  startStreaming();             // 创建 StreamManager → 启动

  // 帧回调（由 StreamManager poll 线程调用）
  framesetCallback(frameset);  // 按 output_mode 分发到对应 publish 方法

  // VxlSense 对象
  vxl::ContextPtr context_;
  vxl::DevicePtr device_;
  std::unique_ptr<StreamManager> stream_manager_;
  std::unique_ptr<PointCloudGenerator> pc_generator_;
};
```

### 4.2 StreamManager

封装 `vxl::Pipeline`，管理帧轮询线程。

```cpp
class StreamManager {
  StreamManager(device, params);   // 创建 Pipeline，设置 sync mode
  enableColor() / enableDepth() / enableIR();  // 配置流

  start(callback);   // 1.设置回调 → 2.pipeline.start() → 3.创建poll线程
  stop();            // 1.running=false → 2.join线程 → 3.pipeline.stop()

  vxl::Pipeline pipeline_;
  std::thread poll_thread_;        // 100ms 超时轮询
  std::atomic<bool> running_;
  FrameSetCallback callback_;      // start()之前设置，线程安全
};
```

### 4.3 PointCloudGenerator

从 Z16 深度图 + 内参生成 PointCloud2，可选 RGB 着色。

```cpp
class PointCloudGenerator {
  configure(depth_info, scale);    // 提取 fx/fy/cx/cy，校验焦距 > 1.0
  generate(depth, color, frame_id);  // Z16 → XYZ（米），可选 BGR/RGB/BGRA 着色

  float fx_, fy_, cx_, cy_;
  bool configured_;
};
```

## 5. 输出模式

| 模式 | 参数值 | 默认 | 启用流 | 发布话题 |
|------|--------|------|--------|----------|
| **RGBD** | `"rgbd"` | Yes | Color+Depth | `~/rgbd` |
| **RGB+Depth** | `"rgb+depth"` | — | Color+Depth | `~/color/*`, `~/depth/*` |
| IR | `"ir"` | — | IR | `~/ir/*` |
| Depth-only | `"depth_only"` | — | Depth | `~/depth/*` |
| Color-only | `"color_only"` | — | Color | `~/color/*` |
| All | `"all"` | — | 全部 | 全部 |

所有模式共享：`~/extrinsics/depth_to_color`（latched）

点云可在任何含 Depth 的模式下叠加启用：`~/depth/points`

## 6. 服务

| 服务 | 类型 | 说明 |
|------|------|------|
| `~/get_device_info` | `GetDeviceInfo` | 设备名称、序列号、固件版本 |
| `~/get_option` | `GetInt32` | 按 option ID 获取参数值 |
| `~/set_option` | `SetInt32` | 按 option ID 设置参数值 |
| `~/hw_reset` | `std_srvs/Trigger` | 硬件重启 |

Option 服务按 ID 范围自动路由到对应传感器：`<100` → Color，`>=100` → Depth。
使用 C++ API `sensor->getOption()` / `sensor->setOption()`，调用前检查 `isOptionSupported()`。

## 7. TF 坐标系

```
{tf_prefix}vxl_camera_link                # 相机物理中心
  +-- {tf_prefix}vxl_color_optical_frame  # Z前 X右 Y下
  +-- {tf_prefix}vxl_depth_optical_frame  # 含外参平移偏移
  +-- {tf_prefix}vxl_ir_optical_frame
```

光学坐标系旋转：`RPY(-pi/2, 0, -pi/2)` 将相机坐标系转换为光学坐标系。

## 8. 线程模型

```
[主线程]                     [poll 线程]
  │                              │
  ├─ 构造 Node                   │
  ├─ initDevice()               │
  ├─ setupPublishers()          │
  ├─ StreamManager::start()     │
  │    ├─ callback_ = ...       │
  │    ├─ pipeline_.start()     │
  │    └─ 创建 poll_thread_ ──→ │ while(running_) {
  │                              │   frameset = pipeline_.waitForFrameSet(100ms)
  ├─ spin (ROS2 事件循环)        │   callback_(frameset)  // → framesetCallback
  │    ├─ 服务回调               │ }
  │    └─ 参数回调               │
  │                              │
  ├─ ~Node()                    │
  │    └─ StreamManager::stop() │
  │         ├─ running_ = false  │
  │         ├─ join ←────────── │ 退出
  │         └─ pipeline_.stop() │
```

关键安全保证：
- `callback_` 在 `pipeline_.start()` 之前设置，线程启动前不可能被读取
- `pipeline_.start()` 失败时自动回滚，不创建线程
- 构造函数异常时 `shutdownDevice()` 保证所有资源释放

## 9. 开发路线

### Phase 1: 核心输出模式 (当前)
- [x] vxl_camera_msgs 包
- [x] RGBD 模式 + RGB+Depth 模式
- [x] 静态 TF 发布
- [x] Launch 文件 + 参数配置
- [x] SDK imported target 集成

### Phase 2: 完善功能
- [ ] 点云生成优化
- [ ] 动态参数（运行时调整曝光/增益）
- [ ] 调测模式完善
- [ ] 设备热插拔处理
- [ ] Lifecycle 节点

### Phase 3: 高级功能
- [ ] 多相机验证
- [ ] 硬件解码器（可选编译）
- [ ] 诊断信息发布
- [ ] CI/CD 与自动测试
