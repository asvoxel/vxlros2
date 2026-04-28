# VxlROS2 使用说明

当前实现支持 **VXL435 / VXL615**（VXL605 待 PID 确认）。两个节点变体：
- `vxl_camera_node` — 普通节点，启动即工作
- `vxl_camera_lifecycle_node` — Lifecycle 节点，含 USB 热插拔自动恢复（**生产推荐**）

---

## 1. 最快上手

```bash
# 已编译好 vxl_camera_msgs + vxl_camera + vxl_description 后
source /opt/ros/humble/setup.bash
source install/setup.bash

# 按相机型号启动（带产品特定默认值）
ros2 launch vxl_camera vxl435.launch.py
# 或
ros2 launch vxl_camera vxl615.launch.py
```

启动后默认发布到 `/vxl_camera/rgbd`（同步 RGBD 合成消息）。

```bash
# 看看发了哪些话题
ros2 topic list | grep vxl

# 实际帧率
ros2 topic hz /vxl_camera/rgbd

# 简单看一帧
ros2 topic echo /vxl_camera/rgbd --once --field rgb.encoding
```

---

## 2. 输出模式

通过 `output_mode` 参数控制，启动时设：

```bash
ros2 launch vxl_camera vxl615.launch.py --ros-args -p output_mode:=rgb+depth
```

| 模式 | 默认 | 发布的话题 | 用途 |
|---|---|---|---|
| `rgbd` | ✅ | `~/rgbd`（同步合成消息）| RGBD 一起消费的下游（SLAM、点云融合）|
| `rgb+depth` | | `~/color/image_raw` + `~/color/camera_info`<br>`~/depth/image_raw` + `~/depth/camera_info` | 标准 ROS2 相机 topic，兼容 image_view 之类工具 |
| `ir` | | `~/ir/image_raw` + `~/ir/camera_info` | 调试 IR 投影器 |
| `depth_only` | | 只 `~/depth/...` | 不需要彩色时 |
| `color_only` | | 只 `~/color/...` | 当普通 USB 摄像头用 |
| `all` | | 全部 | 调测全开 |

每个流都额外发布 metadata：
- `~/color/metadata`、`~/depth/metadata`、`~/ir/metadata` —— `vxl_camera_msgs/Metadata`（timestamp_us / sequence / exposure_us / gain）

外参（latched，TRANSIENT_LOCAL QoS）：
- `~/extrinsics/depth_to_color`

---

## 3. 参数

### 冷参数（要 launch 时定，运行时改不了）

| 参数 | 默认 | 说明 |
|---|---|---|
| `device_serial` | `""` | 序列号；空=自动选第一台 |
| `output_mode` | `"rgbd"` | 见上表 |
| `color.{width,height,fps}` | 1280×720@30 | 彩色流配置 |
| `depth.{width,height,fps}` | 640×480@30 | 深度流配置 |
| `ir.{width,height,fps}` | 640×480@30 | IR 流配置 |
| `point_cloud.enabled` | `false` | 启用 PointCloud2 输出 |
| `sync_mode` | `"strict"` | `strict` / `approximate` / `none` |
| `tf_prefix` | `""` | 多相机隔离用，如 `cam1_` |
| `publish_tf` | `true` | 发静态 TF |

### 热参数（运行时 `ros2 param set` 即时生效）

#### 传感器选项（按设备能力自动暴露）
- `color.exposure` / `color.gain` / `color.auto_exposure`
- `color.white_balance` / `color.auto_white_balance`
- `color.brightness` / `contrast` / `saturation` / `sharpness` / `gamma` / `hue`
- `depth.exposure` / `depth.gain` / `depth.auto_exposure`
- `depth.min_distance` / `depth.max_distance` / `depth.ir_enable`

**互斥保护**：`auto_exposure=1` 时，set `exposure` / `gain` 会被拒绝，error 包含明确原因。要改 manual 值，先关 auto 或同批次一起设：

```bash
# ❌ 拒绝（auto 还开着）
ros2 param set /vxl_camera color.exposure 5000

# ✅ 同批次：关 auto + 设 manual
ros2 service call /vxl_camera/set_parameters rcl_interfaces/srv/SetParameters \
  "{parameters: [{name: color.auto_exposure, value: {type: 2, integer_value: 0}},
                 {name: color.exposure, value: {type: 2, integer_value: 5000}}]}"
```

#### 深度对齐
- `align_depth.enabled` (bool) — 启用后多发布 `~/aligned_depth_to_color/{image_raw,camera_info}`，深度被重投影到 color 视角，camera_info 用 color 内参
- `align_depth.scale` (double) — raw → mm 缩放系数：**VXL435 = 1.0**（vxl435.launch.py 默认），**VXL615 = 8.0**（vxl615.launch.py 默认），VXL605 = 16.0

```bash
ros2 param set /vxl_camera align_depth.enabled true
```

#### Host 端深度滤镜（所有产品可用，吃 CPU）
应用顺序：decimation → threshold → spatial → temporal → hole_filling

| 参数 | 默认 | 说明 |
|---|---|---|
| `filters.decimation.{enabled,scale}` | off, 2 | 降采样 2/4/8 |
| `filters.threshold.{enabled,min_mm,max_mm}` | off, 100, 5000 | 范围裁剪 |
| `filters.spatial.{enabled,magnitude,alpha,delta}` | off | 边缘保持平滑 |
| `filters.temporal.{enabled,alpha,delta}` | off | 时域平滑（多帧）|
| `filters.hole_filling.{enabled,mode}` | off, 0 | 孔洞填充；mode=0/1/2 = nearest/farthest/fill_left |

```bash
# 室内导航推荐：开 hole_filling
ros2 param set /vxl_camera filters.hole_filling.enabled true
```

#### Device 端滤镜（VXL6X5 only，零 CPU；VXL435 静默忽略）

| 参数 | 默认（vxl615）| 说明 |
|---|---|---|
| `filters.device.denoise.{enabled,level}` | true, 2 | 设备降噪，level 1-3 |
| `filters.device.median.{enabled,kernel_size}` | true, 3 | 中值滤波，kernel 3 或 5 |
| `filters.device.outlier_removal.enabled` | true | 异常值移除 |

VXL615 推荐：device 滤镜全开 + host 关闭（避免双重处理）；VXL435 只能用 host 端。

#### 点云（启用时生效）
- `point_cloud.color` (bool) — RGB 着色
- `point_cloud.{min_z,max_z}` (m) — 距离裁剪（0 = 不限）
- `point_cloud.decimation` (int) — 像素跳采
- `point_cloud.organized` (bool) — true = W×H 保 NaN；false = 只发有效点

#### 其他
- `auto_recover_on_reconnect` (bool, lifecycle node) — USB 拔插后自动重 activate

---

## 4. 服务

| 服务 | 类型 | 用途 |
|---|---|---|
| `~/get_device_info` | `vxl_camera_msgs/GetDeviceInfo` | 设备名 / S/N / 固件版本 |
| `~/get_option` | `vxl_camera_msgs/GetInt32` | 按 vxl_option_t ID 读值 |
| `~/set_option` | `vxl_camera_msgs/SetInt32` | 按 ID 写值（绕过 ROS 参数依赖检查）|
| `~/hw_reset` | `std_srvs/Trigger` | 硬件重启 |

```bash
ros2 service call /vxl_camera/get_device_info vxl_camera_msgs/srv/GetDeviceInfo
ros2 service call /vxl_camera/hw_reset std_srvs/srv/Trigger
```

---

## 5. Lifecycle 操作

`vxl_camera_lifecycle_node` 启动默认自动 `configure → activate`。手动控制：

```bash
ros2 launch vxl_camera vxl_camera_lifecycle.launch.py auto_activate:=false
# 然后：
ros2 lifecycle list /vxl_camera
ros2 lifecycle set /vxl_camera configure
ros2 lifecycle set /vxl_camera activate
ros2 lifecycle set /vxl_camera deactivate    # 暂停采集，节点保留
ros2 lifecycle set /vxl_camera activate      # 恢复
ros2 lifecycle set /vxl_camera cleanup       # 关闭设备，回 UNCONFIGURED
```

USB 拔出时节点自动 → INACTIVE，重新插入 → 自动回 ACTIVE（受 `auto_recover_on_reconnect` 控制）。

发布的状态话题：
- `~/connection_state` (`std_msgs/String`) — `ACTIVE/CONNECTED` / `INACTIVE/DISCONNECTED` 等

---

## 6. 多相机

```bash
# 多进程（隔离最强）
ros2 launch vxl_camera multi_camera.launch.py \
  serial_1:=VXL435_001 serial_2:=VXL615_002

# 同进程 ComposableNodeContainer（intra-process 零拷贝）
ros2 launch vxl_camera multi_camera_composable.launch.py \
  serial_1:=VXL435_001 serial_2:=VXL615_002
```

每相机独立 namespace（`/camera_1/...`、`/camera_2/...`）+ TF 前缀（`cam1_*` / `cam2_*`）。

混搭不同型号时，复制 `config/vxl435.yaml` / `config/vxl615.yaml` 到工作区改一改，launch 里 `parameters: [...]` 替换。

---

## 7. 诊断

`/diagnostics` (1 Hz) 包含：
- 每流实测 fps + 期望 fps（low fps 触发 WARN）
- 设备名 / 序列号 / 固件 / 连接状态
- output_mode

```bash
ros2 topic echo /diagnostics --once
# 或图形化
rqt_robot_monitor
```

---

## 8. 常见问题

### `No VxlSense device found`
设备没识别。检查：
```bash
lsusb | grep -iE 'vxl|asvxl|0567'
ls -l /dev/bus/usb/  # 权限
```
如果有设备但没权限，装 udev rule：`scripts/99-vxl-cameras.rules`（在 vxlsdk 仓库里）。

### `Filter chain: off` 但我开了 hole_filling
检查参数实际值：`ros2 param get /vxl_camera filters.hole_filling.enabled`。如果是 false，可能 launch 文件里 yaml 优先级覆盖了你的 set。yaml 加载在 launch 时一次性发生。

### auto_exposure 拒绝设 manual
看第 3 节的「互斥保护」。同批次设置或先关 auto。

### VXL615 上 device 滤镜没效果
确认 launch 用的是 `vxl615.launch.py`（VXL435 launch 不会启用 device 滤镜）。再确认参数：
```bash
ros2 param get /vxl_camera filters.device.denoise.enabled
```

### 深度对齐看起来不对
检查 `align_depth.scale` 是否匹配设备：VXL435=1.0、VXL615=8.0、VXL605=16.0。用产品 launch 自动是对的；用通用 launch 要手动设。

### 怎么校验环境
```bash
./scripts/smoke_test_real.sh           # 默认 rgbd 15s
./scripts/smoke_test_real.sh --align   # 加深度对齐
./scripts/smoke_test_real.sh --duration 60
```

---

## 9. 编译/运行环境

| 平台 | 路径 |
|---|---|
| **Linux 真机** | Ubuntu 22.04 + ROS Humble apt install（推荐生产）|
| **macOS 开发** | Lima VM (Ubuntu 22.04 ARM64) — 见 `docs/DEV_VM.md` |
| **macOS 原生** | 不支持 ROS2 Tier 1，**不要走这条线** |

构建：
```bash
# 假设 vxlsdk 在 ../vxlsdk 已编译
colcon build --packages-select vxl_camera_msgs vxl_camera vxl_description
```

---

## 一句话总结

**默认开 `vxl<型号>.launch.py`，缺什么 `ros2 param set` 改，要稳定就用 `vxl_camera_lifecycle_node` 系列 launch。** 现在所有改动都进 102 个单测覆盖，但**还没有真机小时数**——上线前请跑 `smoke_test_real.sh`。
