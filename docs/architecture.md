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
│  │     ASVXL SDK (libasvxl.a)      │                  │
│  │  Context → Device → Pipeline    │                  │
│  │          → FrameSet → Frame     │                  │
│  └─────────────────────────────────┘                  │
└──────────────────────────────────────────────────────┘
```

## 2. SDK 集成

SDK 通过 CMake imported target 链接。两种来源：

1. **同级 `vxlsdk` 项目编译产物**（开发推荐）：`../vxlsdk/sdk/current/` 由 `scripts/release-{linux,macos}.sh` 生成（符号链接到当前版本目录）
2. **发行包解压到 `vxlsdk/`**（无源码用户）

```
asVoxel/
├── vxlsdk/                  ← SDK 源码项目（同级，编译输出 sdk/current/）
└── vxlros2/
    ├── vxlsdk/              ← SDK 占位目录（兜底；解压发行包用）
    │   ├── include/         # vxl.hpp, vxl_types.hpp, ...
    │   └── lib/{linux,darwin}/  # libasvxl.a
    ├── vxl_camera/
    ├── vxl_camera_msgs/
    └── vxl_description/
```

CMake 路径优先级：`-DVXL_SDK_DIR` > `$VXL_SDK_DIR` > `../../vxlsdk/sdk/current/` > `../vxlsdk/`

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

  // ASVXL 对象
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

## 9. 多相机部署

两套 launch 满足不同需求：

| Launch | 进程模型 | 适用场景 | 帧零拷贝 |
|---|---|---|---|
| `multi_camera.launch.py` | 每相机一进程 | 默认；最大隔离，单相机崩溃不影响其他 | ❌ |
| `multi_camera_composable.launch.py` | 单进程 + ComposableNodeContainer | 同进程下游消费者（如点云融合）享受 intra-process 零拷贝 | ✅ |

**Context 策略**：每个节点持有独立 `vxl::Context`（`Context::create()`）。SDK 没有强制单例；底层 libusb 自身处理多 context 协调。多相机时建议：
- 每台相机使用独立 USB 控制器（避免带宽争用）
- USB 3.0+ 必需（USB 2.0 单台 1280×720@30 已饱和）
- 必要时降帧率/分辨率：`color.fps:=15` 或 `color.width:=640`

**Composable 容器选择**：用 `component_container_isolated` 而非 `component_container`。前者每组件独立线程，避免一个相机的 100ms 轮询阻塞其他相机；后者单线程会导致帧延迟。

**Lifecycle 多相机**：lifecycle 节点目前以独立进程运行（`multi_camera.launch.py`）。在 ComposableNodeContainer 中运行 lifecycle 节点需要外部 `nav2_lifecycle_manager` 协调，本仓库未提供 launch — 用户可参考 nav2 文档自行配置。

## 10. 硬件解码器（Phase 3.2 评估结论）

**结论：N/A，不需要 ROS2 层硬件解码器集成。**

调研发现：ASVXL SDK 在 `lib/{linux,darwin,windows}/libasvxl.a` 内部已通过 libuvc 完成 MJPEG/H264 → 原始像素的解码（参见 `vxl_frame.h:vxl_frame_convert`）。Frame 到达 ROS2 节点时已经是 BGR/RGB/Gray8/Z16 等可直接使用的格式。

如果未来需求变化（例如 SDK 暴露原始压缩流）：
- 在 `vxl_camera_node` 与 SDK 之间引入 `IDecoder` 抽象层（CPU 后备 + NVDEC/VAAPI 实现）
- CMake option `ENABLE_HW_DECODER` 条件编译，避免强制 CUDA/VAAPI 依赖
- 帧格式枚举增加 `MJPEG/H264` → 解码器选择路由

但目前**没有实际收益**，不实现。

## 11. 开发路线

### Phase 1: 核心输出模式
- [x] vxl_camera_msgs 包
- [x] RGBD 模式 + RGB+Depth 模式
- [x] 静态 TF 发布
- [x] Launch 文件 + 参数配置
- [x] SDK imported target 集成

### Phase 2: 完善功能
- [x] 点云生成优化（worker 线程 + drop-old + min/max_z + decimation + organized/dense）
- [x] 动态参数（运行时调整曝光/增益等；冷参数明确拒绝）
- [x] 调测模式完善（`~/<stream>/metadata` 话题；`log_level` launch 参数）
- [x] 设备热插拔处理（lifecycle 节点内置；`auto_recover_on_reconnect` 控制）
- [x] Lifecycle 节点（`VxlCameraLifecycleNode` + `vxl_camera_lifecycle_node` 可执行）

### Phase 3: 高级功能
- [x] 多相机部署（多进程 + ComposableNodeContainer 两套 launch；策略已文档化）
- [x] 硬件解码器（评估为 N/A — SDK 已内部解码，参见 §10）
- [x] 诊断信息发布（`/diagnostics` 中的 vxl_camera 状态：fps、drops、connection state）
- [x] CI/CD 与自动测试（GitHub Actions：humble/jazzy × Ubuntu 22.04/24.04）
