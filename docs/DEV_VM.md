# 本地开发 VM (Lima + Ubuntu 22.04 + ROS 2 Humble)

`vxlros2` 在 Apple Silicon Mac 上的开发约定：**Mac 编辑代码，Linux VM 跑构建/测试**。

## 为什么不直接在 macOS 跑 ROS2

- macOS 是 ROS2 Tier 3 平台，`cv_bridge` / `image_transport` 等驱动相关包时不时有 macOS 特有 bug
- 生产机器人跑 Ubuntu，开发环境跟生产对齐，调通就是真调通
- Linux 用 `apt install ros-humble-*-dbgsym` 能拿到完整 debug 符号，gdb step into rclcpp 一气呵成

## 一次性安装（约 15 分钟）

### 前置
- macOS Apple Silicon
- Homebrew

### 步骤

```bash
# 1. 装 Lima（如未装）
brew install lima

# 2. 创建并启动 ros-humble 实例（用本仓库 scripts/ros-humble.yaml）
cd /path/to/vxlros2
limactl create --name=ros-humble scripts/ros-humble.yaml
limactl start ros-humble    # 首次启动 3-5 min（下载 Ubuntu 镜像 + cloud-init）

# 3. 进 VM 装 ROS Humble + 开发工具
limactl shell ros-humble
bash ~/Workspace/asVoxel/vxlros2/scripts/install_ros2_humble.sh
# 完成后退出
exit
```

完成后：
- VM 名: `ros-humble`
- VM 内你的 Mac home 自动挂在同名路径下
- `~/Workspace` 是可写挂载（Lima virtiofs，比 sshfs 快很多）
- ROS Humble 装在 `/opt/ros/humble/`，shell 启动自动 `source` 好

## 日常开发流程

### 编辑：Mac 上用你常用的 IDE
代码在 `/Users/<you>/Workspace/asVoxel/vxlros2/`，VM 立即可见。

### 构建/测试：进 VM

```bash
limactl shell ros-humble        # SSH 进 VM
cd ~/Workspace/asVoxel/vxlros2  # 同一份代码

# 第一次需要先编 vxlsdk（在 VM 内）
cd ~/Workspace/asVoxel/vxlsdk
./scripts/release-linux.sh
cd ~/Workspace/asVoxel/vxlros2

# 构建 vxl_camera
colcon build --packages-select vxl_camera_msgs vxl_camera

# 跑测试（mock 单测 + launch_test，无需真机）
colcon test --packages-select vxl_camera
colcon test-result --all --verbose
```

### 调试

```bash
# 装调试符号包（按需）
sudo apt install ros-humble-rclcpp-dbgsym ros-humble-image-transport-dbgsym

# 用 gdb 调试节点
gdb --args install/vxl_camera/lib/vxl_camera/vxl_camera_node
```

## 一些可选优化

### 用 alias 简化

```bash
# 在你的 ~/.zshrc 或 ~/.bashrc (Mac)
alias vmsh='limactl shell ros-humble'
alias vmstop='limactl stop ros-humble'
alias vmstart='limactl start ros-humble'
```

然后 `vmsh` 直接进 VM。

### 跑 GUI 工具（rqt / rviz2）

VM 是 server 镜像（无 GUI），跑 `rqt_graph` / `rviz2` 需要 X11 forwarding：

```bash
# Mac 装 XQuartz
brew install --cask xquartz   # 需要重启 Mac 一次让 launchd 注册

# Lima 配 X11 forwarding（在 VM 内）
ssh -X lima@127.0.0.1 -p $(limactl show-ssh ros-humble --format=json | jq .Port)
# 或者直接用 limactl 的 SSH 配置 + -X
limactl shell ros-humble -- env DISPLAY=host.lima.internal:0 rviz2
```

### 资源调整

`scripts/ros-humble.yaml` 默认：4 CPU / 6GB RAM / 40GB disk。如果你笔记本资源紧张：
- `cpus: 2` / `memory: 4GiB` 也能跑（编译慢些）
- 或者 `vmType: vz` → `vmType: qemu` 可以跑 x86 应急（但慢得多）

修改后：
```bash
limactl stop ros-humble
limactl edit ros-humble
limactl start ros-humble
```

## 常见问题

| 症状 | 原因 / 修复 |
|---|---|
| `limactl start` 卡在 "Updating images" | 网络慢，等下载完；或用代理 `HTTPS_PROXY=... limactl start` |
| VM 内看不到 `~/Workspace` 写权限 | 检查 `scripts/ros-humble.yaml` 的 `mounts` writable: true |
| 编译时 `ASVXL SDK not found` | 先在 VM 里跑 `cd ~/Workspace/asVoxel/vxlsdk && ./scripts/release-linux.sh` |
| `colcon build` 卡死 | 资源不够；`limactl edit ros-humble` 加 CPU/RAM |
| Mac 上改了文件 VM 没看到 | virtiofs 偶尔有缓存问题；`limactl restart ros-humble` 或 `sync` |
| GUI 工具启动报 `cannot open display` | XQuartz 没启动或没装；按上面 GUI 章节配 |

## 卸载

```bash
limactl stop ros-humble
limactl delete ros-humble    # 删 VM，保留 Mac 上的代码
```
