# VxlROS2 中文文档

[ASVXL](https://github.com/asvoxel/vxlsdk-rel) (VXL435 / VXL6X5) 深度相机的 ROS2 驱动。

> English: [../README.md](../README.md)

## 功能特性

- **RGBD 输出** — RGB + Depth 同步帧，单话题发布（默认模式）
- **RGB+Depth 输出** — Color 和 Depth 分离话题，各带 CameraInfo
- **点云生成** — 可选的 PointCloud2 输出
- **多相机支持** — TF 前缀隔离
- **ROS2 服务** — 设备信息查询、参数控制、硬件重启
- **组件化节点** — 支持 rclcpp_components 进程内零拷贝

## 环境要求

- ROS2 Humble 或更高版本
- ASVXL SDK ([下载](https://github.com/asvoxel/vxlsdk-rel/releases))

### 安装 ROS2 依赖

```bash
sudo apt install \
  ros-humble-image-transport \
  ros-humble-camera-info-manager \
  ros-humble-cv-bridge \
  ros-humble-tf2-ros \
  ros-humble-xacro \
  ros-humble-robot-state-publisher
```

### 安装 ASVXL SDK

> 仓库内的 `vxlsdk/` 是空的占位目录 —— 必须先按下面方式一或方式二把 SDK
> 准备好，`colcon build` 才能成功。

**方式一 — 从同级 `vxlsdk` 源码项目编译（开发推荐）：**

```bash
# 目录布局：asVoxel/{vxlros2, vxlsdk}/   （vxlsdk 与 vxlros2 同级）
cd ../vxlsdk
./scripts/release-linux.sh    # 或 release-macos.sh
# 输出：vxlsdk/sdk/current/  （符号链接，vxlros2 CMake 自动识别）
```

**方式二 — 解压 SDK 发行包：**

```bash
cd vxlros2/vxlsdk/
wget https://github.com/asvoxel/vxlsdk-rel/releases/download/vX.Y.Z/asvxl-sdk-X.Y.Z-linux-x86_64.tar.gz
tar xzf asvxl-sdk-X.Y.Z-linux-x86_64.tar.gz --strip-components=1

# 验证
ls include/vxl.hpp lib/linux/libasvxl.a
```

解压后目录结构：
```
vxlsdk/
├── include/          # C/C++ API 头文件
│   ├── vxl.h
│   ├── vxl.hpp
│   └── ...
├── lib/
│   └── linux/libasvxl.a
└── VERSION
```

## 编译

```bash
# 创建工作空间
mkdir -p ~/vxlros2_ws/src && cd ~/vxlros2_ws/src

# 符号链接（开发推荐）或复制
ln -s /path/to/vxlros2/vxl_camera_msgs .
ln -s /path/to/vxlros2/vxl_camera .
ln -s /path/to/vxlros2/vxl_description .

# 编译
cd ~/vxlros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select vxl_camera_msgs && source install/setup.bash
colcon build --packages-select vxl_camera vxl_description && source install/setup.bash
```

**自定义 SDK 路径**（如果 SDK 不在默认位置）：

```bash
# 方式一：CMake 参数
colcon build --cmake-args -DVXL_SDK_DIR=/opt/vxlsdk/sdk

# 方式二：环境变量
export VXL_SDK_DIR=/opt/vxlsdk/sdk
colcon build
```

路径优先级：CMake 参数 > 环境变量 > 同级 `../vxlsdk/sdk/current/` > 项目内 `vxlsdk/`

## 使用

### RGBD 模式（默认）

```bash
ros2 launch vxl_camera vxl_camera.launch.py
```

发布话题：
- `~/rgbd` — 同步的 RGBD 消息（`vxl_camera_msgs/RGBD`）
- `~/extrinsics/depth_to_color` — 深度到彩色的外参（latched）

### RGB+Depth 模式

```bash
ros2 launch vxl_camera vxl_camera.launch.py output_mode:=rgb+depth
```

发布话题：
- `~/color/image_raw` + `~/color/camera_info`
- `~/depth/image_raw` + `~/depth/camera_info`

### 调测模式

IR、单流等模式仅供开发调试，不在 launch 文件中暴露：

```bash
ros2 run vxl_camera vxl_camera_node --ros-args -p output_mode:=ir
ros2 run vxl_camera vxl_camera_node --ros-args -p output_mode:=all
```

所有模式：`rgbd` | `rgb+depth` | `ir` | `depth_only` | `color_only` | `all`

### 点云

```bash
ros2 launch vxl_camera vxl_camera.launch.py \
  --ros-args -p point_cloud.enabled:=true
```

### 多相机

```bash
ros2 launch vxl_camera multi_camera.launch.py \
  serial_1:=VXL435_001 serial_2:=VXL435_002
```

### 服务调用

```bash
# 设备信息
ros2 service call /vxl_camera/get_device_info vxl_camera_msgs/srv/GetDeviceInfo

# 获取选项（option_name 为选项 ID 字符串）
ros2 service call /vxl_camera/get_option vxl_camera_msgs/srv/GetInt32 "{option_name: '1'}"

# 设置选项
ros2 service call /vxl_camera/set_option vxl_camera_msgs/srv/SetInt32 "{option_name: '1', value: 5000}"

# 硬件重启
ros2 service call /vxl_camera/hw_reset std_srvs/srv/Trigger
```

## 参数列表

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `output_mode` | `"rgbd"` | 输出模式 |
| `device_serial` | `""` | 设备序列号（空 = 自动选择） |
| `color.width` | `1280` | 彩色流宽度 |
| `color.height` | `720` | 彩色流高度 |
| `color.fps` | `30` | 彩色流帧率 |
| `depth.width` | `640` | 深度流宽度 |
| `depth.height` | `480` | 深度流高度 |
| `depth.fps` | `30` | 深度流帧率 |
| `point_cloud.enabled` | `false` | 启用点云 |
| `point_cloud.color` | `true` | 点云附带 RGB 颜色 |
| `sync_mode` | `"strict"` | 帧同步模式：strict / approximate / none |
| `tf_prefix` | `""` | TF 坐标系前缀 |
| `publish_tf` | `true` | 发布静态 TF |

## TF 坐标系

```
{tf_prefix}vxl_camera_link          # 相机物理中心
  +-- {tf_prefix}vxl_color_optical_frame   # 彩色传感器光心
  +-- {tf_prefix}vxl_depth_optical_frame   # 深度传感器光心
  +-- {tf_prefix}vxl_ir_optical_frame      # 红外传感器光心
```

## 项目结构

```
vxlros2/
├── vxlsdk/                # SDK 占位目录（兜底；优先用同级 ../vxlsdk 项目编译产物）
├── vxl_camera/             # 主驱动节点
├── vxl_camera_msgs/        # 自定义消息与服务
├── vxl_description/        # URDF 相机模型
└── docs/
    ├── README_zh.md        # 本文档
    └── architecture.md     # 架构设计
```

## 测试

```bash
# 单元测试（无需设备）
colcon test --packages-select vxl_camera
colcon test-result --all --verbose

# 集成测试（需要连接设备）
ros2 launch vxl_camera test/test_integration.launch.py
```

## 常见问题

### SDK 找不到

```
CMake Error: ASVXL SDK not found
```

解决（任一）：
1. 编译同级 vxlsdk 项目：`cd ../vxlsdk && ./scripts/release-linux.sh`，输出会在 `vxlsdk/sdk/current/` 自动被识别
2. 解压发行包到 `vxlros2/vxlsdk/`
3. 通过 `-DVXL_SDK_DIR=/path` 显式指定路径

### 没有检测到设备

检查 USB 连接和权限：
```bash
lsusb | grep -i vxl
sudo chmod 666 /dev/bus/usb/xxx/yyy
```

### 帧率低

- 确认使用 USB 3.0 端口
- 降低分辨率：`-p color.width:=640 -p color.height:=480`

## 许可证

Apache-2.0
