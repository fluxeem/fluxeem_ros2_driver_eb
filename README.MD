<h1 align="center">Fluxeem ROS2 Driver</h1>

<p align="center">
  <b>High-performance ROS2 driver for Fluxeem event cameras with zero-copy shared-memory transport</b>
</p>

<p align="center">
  <a href="fluxeem_ros2_publisher/LICENSE"><img src="https://img.shields.io/badge/License-Apache%202.0-blue.svg" alt="License: Apache 2.0"></a>
  <img src="https://img.shields.io/badge/ROS2-Foxy%2B-brightgreen.svg" alt="ROS2: Foxy+">
  <img src="https://img.shields.io/badge/platform-Ubuntu%2020.04%2F22.04-orange.svg" alt="Platform: Ubuntu 20.04/22.04">
  <img src="https://img.shields.io/badge/C%2B%2B-20-blue.svg" alt="C++20">
</p>

<p align="center">
  <a href="#快速开始">快速开始</a> •
  <a href="#安装">安装</a> •
  <a href="#架构">架构</a> •
  <a href="#配置">配置</a> •
  <a href="#模块开发指南">模块开发</a> •
  <a href="#故障排查">故障排查</a>
</p>

---

## 概述

**FluxeemROS2Driver** 是由 [瞬极科技 - Fluxeem](https://fluxeem.com/) 开发的 ROS2 事件相机驱动栈。它将 Fluxeem 事件相机（或预录制的 `.raw` 文件）桥接至 ROS2 生态，专注于**高带宽、低延迟的同机通信**。

由于原始事件流与累帧图像的带宽占用极高，驱动采用 **POSIX 共享内存** 作为主要传输方式：各模块将数据直接写入 `/dev/shm`，仅发布包含内存标识的轻量消息。同时提供标准的 `sensor_msgs::msg::Image` 话题，以便快速接入现有 ROS2 图像管道进行原型验证。

### 核心特性

- **零拷贝共享内存传输**：支持原始事件流、Time Surface 图像与 Histogram 图像
- **双路输入源**：支持 Fluxeem 实时相机或 `.raw` 文件回放
- **时间窗口化发布**：每个模块可独立配置 `timespan`
- **硬件 TriggerIn 支持**：基于外部触发信号的时间同步发布
- **动态相机控制**：通过 ROS2 Service 实时调节 Bias、ERC、Trail Filter 与录制
- **模块化架构**：无需重构节点即可扩展第三方处理模块
- **自动重连**：相机断线后自动检测并重连

### 仓库结构

| 包名 | 说明 |
|------|------|
| `event_msgs` | 自定义 ROS2 消息定义（共享内存元数据、`ReplayWindow` 等） |
| `camera_interfaces` | 相机配置相关的 ROS2 Service 定义 |
| `fluxeem_ros2_publisher` | 核心发布节点（相机 / `.raw` → 共享内存 / ROS 话题） |
| `fluxeem_ros2_viewer` | 订阅共享内存消息并可视化的示例程序 |
| `scripts` | 辅助脚本（如 rosbag 录制） |

---

## 快速开始

```bash
# 1. 克隆到 ROS2 工作空间
cd ${WORKSPACE}/src
git clone https://github.com/fluxeem/fluxeem_ros2_driver.git

# 2. 确认已手动安装 OpenEB / Metavision SDK，并让 CMake 能找到 MetavisionSDK
cd ${WORKSPACE}
# 若 OpenEB 未安装到系统默认路径，请按实际路径设置:
# export CMAKE_PREFIX_PATH=/path/to/openeb/install:${CMAKE_PREFIX_PATH}
colcon build --packages-up-to fluxeem_ros2_publisher fluxeem_ros2_viewer

# 3. Source 并启动
source install/setup.bash
ros2 launch fluxeem_ros2_publisher launch.py
```


---

## 安装

### 依赖

| 依赖 | 版本 | 说明 |
|------|------|------|
| ROS2 | Foxy 及以上 | `rclcpp`、`sensor_msgs` |
| OpenEB / Metavision SDK | 手动安装 | 需提供 `MetavisionSDKConfig.cmake`，可通过 `CMAKE_PREFIX_PATH` 或 `MetavisionSDK_DIR` 暴露给 CMake |
| OpenCV | — | `fluxeem_ros2_viewer` 需要 |
| 平台 | Ubuntu 20.04 / 22.04 LTS | x86_64 / ARM64 |
| 编译器 | GCC 9+ / Clang 12+ | 需要 C++20 |

### 编译

```bash
cd ${WORKSPACE}

# 全工作空间编译
colcon build

# 仅编译发布节点及依赖
colcon build --packages-up-to fluxeem_ros2_publisher fluxeem_ros2_viewer
```

可选：开启架构原生优化：

```bash
colcon build --cmake-args -DBUILD_WITH_MARCH_NATIVE=ON
```

---

## 架构

### 模块继承关系

发布节点为单节点可执行文件（`fluxeem_ros2_publisher_node`），根据 ROS 参数条件实例化**处理模块**。模块为静态链接类，无需 `pluginlib`。

```
FluxeemRos2Publisher (rclcpp::Node)
│
├── WindowedEventModuleBase
│   ├── EventTopicModule<MsgType>
│   │   ├── ImageMemoryModuleBase<MsgType>
│   │   │   ├── TimeSurfaceMemoryModule  →  event_msgs::msg::TimeSurfaceMemory
│   │   │   └── HistogramMemoryModule    →  event_msgs::msg::HistogramMemory
│   │   └── EventMemoryModule            →  event_msgs::msg::EventMemory
│   └── EventToImageModule               →  sensor_msgs::msg::Image
│
├── pixel_operations (批量像素写入辅助)
├── CameraSettings (Bias / ERC / Trail Filter 校验)
└── ROS2 Service 与定时器
```

### 传输模式

| 模块 | 传输方式 | 消息类型 | 适用场景 |
|------|----------|----------|----------|
| `EventMemoryModule` | 共享内存 | `event_msgs::msg::EventMemory` | 高带宽原始事件流 |
| `TimeSurfaceMemoryModule` | 共享内存 | `event_msgs::msg::TimeSurfaceMemory` | Time Surface 累帧图像 |
| `HistogramMemoryModule` | 共享内存 | `event_msgs::msg::HistogramMemory` | 事件计数 Histogram 图像 |
| `EventToImageModule` | ROS 话题 | `sensor_msgs::msg::Image` | 快速接入现有图像管道 |

### 事件流处理流程

```
┌─────────────────┐     ┌──────────────────┐     ┌─────────────────────┐
│  Camera / .raw  │────▶│  Event Windowing │────▶│  Module Processing  │
│    Stream       │     │  (timespan/trig) │     │  (shared mem / ROS) │
└─────────────────┘     └──────────────────┘     └─────────────────────┘
```

1. **输入**：通过 OpenEB `Metavision::Camera` 回调读取实时相机数据，或通过 `CameraStreamSlicer` 在独立线程中回放 `.raw` 文件。
2. **窗口切分**：`WindowedEventModuleBase` 按相机侧微秒级时间戳将事件切分为固定 `timespan` 窗口，或在硬件 TriggerIn 之间持续累积。
3. **发布**：模块完成缓冲区写入后发布。共享内存模块写入 `/dev/shm/<identifier>_<topic>_idx<N>`；标准模块直接发布完整消息。

---

## 配置

所有运行时行为通过参数文件 `fluxeem_ros2_publisher/params/param.yaml` 控制。

### 通用参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `identifier` | `string` | `"fluxeem"` | 话题前缀，最终话题名为 `${identifier}/${topic_name}` |
| `raw_file` | `string` | `""` | `.raw` 回放文件路径，为空则使用实时相机 |
| `publish_strategy.mode` | `string` | `"timespan"` | `"timespan"` 按时间窗口发布，`"trigger"` 按 TriggerIn 触发发布 |
| `publish_strategy.trigger_polarity` | `int` | `1` | Trigger 模式下接受的 TriggerIn 极性 |

### Raw 回放参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `raw_playback.interval_us` | `int` | `0` | 输入切片长度（µs），`0` 表示自动使用已启用模块中最小的 `timespan` |
| `raw_playback.rate` | `double` | `1.0` | 回放速度倍率，`1.0` 为实时同步 |
| `raw_playback.loop` | `bool` | `true` | 文件播放到结尾后是否循环 |

### ROI 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `roi.width_fraction` | `double` | `1.0` | 以画面中心为准的宽度裁剪比例，范围 `(0, 1]` |
| `roi.height_fraction` | `double` | `1.0` | 以画面中心为准的高度裁剪比例，范围 `(0, 1]` |

### 相机硬件参数

| 参数 | 类型 | 默认值 | 有效范围 | 说明 |
|------|------|--------|----------|------|
| `camera.trail_filter_enabled` | `bool` | `true` | — | Trail Filter 开关 |
| `camera.trail_filter_type` | `string` | `"STC_CUT_TRAIL"` | `STC_CUT_TRAIL`、`STC_KEEP_TRAIL`、`TRAIL` | Trail Filter 类型 |
| `camera.trail_filter_threshold` | `int` | `10` | `1`–`100` [ms] | Trail Filter 阈值 |
| `camera.erc_enabled` | `bool` | `false` | — | Event Rate Control 开关 |
| `camera.erc_rate` | `int` | `45` | `0`–`320` | 最大事件率 [Mevents/s] |
| `camera.bias_diff` | `int` | `0` | `-25`–`23`  | Diff Bias |
| `camera.bias_diff_off` | `int` | `0` | `-30`–`200`  | Diff-OFF Bias |
| `camera.bias_diff_on` | `int` | `0` | `-80`–`145`  | Diff-ON Bias |
| `camera.bias_fo` | `int` | `0` | `-20`–`0`  | FO Bias |
| `camera.bias_hpf` | `int` | `0` | `0`–`120`  | HPF Bias |
| `camera.bias_refr` | `int` | `0` | `-20`–`235`  | Refractory Bias |

> 超出范围的值会在运行时被静默截断到有效区间。

### 模块参数

#### event_memory_module

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `on_off` | `bool` | `true` | 是否启用该模块 |
| `topic_name` | `string` | `"event_memory"` | 相对话题名 |
| `timespan` | `int` | `2000` | 发布间隔（µs） |
| `spatial_sample_rate` | `double` | `1.0` | 空间下采样因子，需大于等于 1 |
| `temporal_sample_rate_min` | `int` | `1` | 时间下采样最小因子 |
| `target_num_events_per_sec` | `int` | `0` | 自适应时间下采样目标事件率（events/s），`0` 表示关闭 |

#### time_surface_memory_module

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `on_off` | `bool` | `true` | 是否启用该模块 |
| `topic_name` | `string` | `"time_surface_memory"` | 相对话题名 |
| `timespan` | `int` | `33333` | 发布间隔（µs） |
| `channel` | `int` | `1` | `1` 为灰度图，`3` 为 BGR |
| `keep_fraction` | `double` | `1.0` | 时间窗口内保留的最新事件比例，范围 `[0, 1]` |

#### histogram_memory_module

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `on_off` | `bool` | `false` | 是否启用该模块 |
| `topic_name` | `string` | `"histogram_memory"` | 相对话题名 |
| `timespan` | `int` | `33333` | 发布间隔（µs） |
| `channel` | `int` | `1` | `1` 为灰度图，`3` 为 BGR |
| `keep_fraction` | `double` | `1.0` | 时间窗口内保留的最新事件比例，范围 `[0, 1]` |

#### event_to_image_module

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `on_off` | `bool` | `false` | 是否启用该模块 |
| `topic_name` | `string` | `"event_frame"` | 相对话题名 |
| `timespan` | `int` | `33333` | 发布间隔（µs） |
| `image_type` | `string` | `"time_surface"` | 累帧类型：`"time_surface"` 或 `"histogram"` |
| `channel` | `int` | `1` | `1` 为灰度图，`3` 为 BGR |
| `keep_fraction` | `double` | `1.0` | 时间窗口内保留的最新事件比例，范围 `[0, 1]` |

### 内置常量

定义位于 `publisher_constant.h`：

| 常量 | 值 | 说明 |
|------|-----|------|
| `QOS_QUEUE` | `8` | 发布端 QoS 历史深度 |
| `CLEAN_TIME_SPAN_SEC` | `2.0` | 共享内存文件发布后的存活时间（s） |
| `REST_TIME_SPAN` | `1000` | 相机重连成功后恢复写入前的等待时间（ms） |
| `BG_COLOR` | `128` | 单通道 Time Surface 背景填充色 |
| `POSITIVE_COLOR` | `255` | 单通道 Time Surface 正事件填充色 |
| `NEGATIVE_COLOR` | `0` | 单通道 Time Surface 负事件填充色 |

---

## ROS2 服务

节点暴露以下服务用于运行时相机控制：

| 服务 | 类型 | 说明 |
|------|------|------|
| `/${identifier}/camera_info` | `camera_interfaces/srv/CameraInfo` | 查询相机分辨率及当前配置（Trail Filter、ERC、Bias） |
| `/${identifier}/set_biases` | `camera_interfaces/srv/SetBiases` | 动态设置 Bias 电压 |
| `/${identifier}/set_erc` | `camera_interfaces/srv/SetErc` | 开启/关闭 Event Rate Control |
| `/${identifier}/set_trail_filter` | `camera_interfaces/srv/SetTrailFilter` | 配置 Trail Filter |
| `/${identifier}/record` | `camera_interfaces/srv/Record` | `enabled=true` 且指定 `file_path` 时开始录制 `.raw`；`enabled=false` 时停止 |

> 在 `.raw` 文件回放模式下，硬件相关服务将返回失败。

---

## 时间戳语义

- **事件窗口边界**（`timespan`、`ReplayWindow.raw_window_start`、`ReplayWindow.raw_window_end`）使用**相机侧微秒时间**。
- **ROS 消息头**（`header.stamp`）使用**主机系统时间**：
  - *实时模式*：SDK 回调进入时刻的系统时间。
  - *回放模式*：与播放计划对齐的模拟到达时间。

若订阅端需要与其他 ROS 话题按系统时间同步，应使用 `header.stamp`；若需要关联 `.raw` 文件或相机事件时间线，应使用 `ReplayWindow` 中的字段。

在 **Trigger 模式**下，`time_surface_memory`、`histogram_memory` 和 `event_to_image` 在相邻匹配 TriggerIn 之间持续累积事件，触发时发布完整区间。`event_memory` 仍按自身 `timespan` 发布，同时在匹配的 TriggerIn 到达时额外发布并清空当前部分窗口。`ReplayWindow.trigger_marked`、`trigger_id`、`trigger_polarity` 和 `trigger_timestamp` 用于标注 Trigger 对齐的窗口。

---

## Raw 文件回放

当 `raw_file` 非空时，节点绕过物理相机，通过 OpenEB `Metavision::Camera::from_file` 与 `CameraStreamSlicer` 读取事件。

- 回放按 `raw_playback.interval_us` 切片（若为 `0` 则使用已启用模块中最小 `timespan`）。
- 独立线程通过 `steady_clock` 与 `raw_playback.rate` 控制发布节奏。
- `camera_info` 服务仍可从 raw 元信息返回分辨率。
- Bias、ERC、Trail Filter、录制服务在此模式下不可用。

---

## 信号处理与生命周期

节点安装了自定义的 SIGINT / SIGTERM 信号桥接。收到终止信号时：

1. 停止相机回调与 Raw 回放线程。
2. 对所有共享内存文件执行 `munmap` 与 `shm_unlink`。
3. 调用 `rclcpp::shutdown()`。

这确保了跨次运行不会在 `/dev/shm` 中遗留孤儿文件。

---

## 模块开发指南

架构设计允许在不重构节点的前提下扩展自定义事件处理模块。

### 第一步：选择基类

| 输出形态 | 推荐基类 | 已提供的功能 |
|----------|----------|--------------|
| 共享内存 + 自定义消息 | `EventTopicModule<MsgType>` | ROS Publisher、POSIX 共享内存生命周期、`ReplayWindow` 填充 |
| 共享内存 + uint8 图像 | `ImageMemoryModuleBase<MsgType>` | 上述全部 + ROI、channel、`keep_fraction`、buffer reset/remap、`start_timestamp_` 管理 |
| 标准 ROS 消息（无需共享内存） | `WindowedEventModuleBase` | 时间窗口切分、receive stamp、Trigger 逻辑、`publishUntil` |

### 第二步：实现模块（共享内存示例）

创建 `include/fluxeem_ros2_publisher/custom_module.hpp` 与 `src/custom_module.cpp`：

```cpp
#include <fluxeem_ros2_publisher/event_topic_module.hpp>
#include <event_msgs/msg/custom_memory.hpp>

class CustomMemoryModule : public EventTopicModule<event_msgs::msg::CustomMemory>
{
public:
  CustomMemoryModule(rclcpp::Node *node, const std::string &identifier,
                     const std::string &topic_name, size_t history_depth,
                     double lifetime);

  void setupModule(RegionOfInterest roi, int timespan /*, ... */);

  void reset() override;
  void processEvent(const Event *event_ptr) override;

private:
  void setupMessage() override;
  void publishCurrentWindow() override;

  // 可选：批量优化时覆盖
  void processEventRange(const Event *begin, const Event *end) override;

  // 可选：窗口 cutoff 推进时响应
  void onTimestampCutoffChanged() override;
};
```

**实现 checklist：**

- [ ] 构造函数委托给 `EventTopicModule(node, identifier, topic_name, history_depth, lifetime)`。
- [ ] `setupModule(...)` 调用 `configureTimeWindow(timespan, "custom memory module")`，然后 `createSharedMemory(data_size, initial_value)`。
- [ ] `reset()` 清空缓冲区 / 计数器，并调用 `resetTimeWindow()`。
- [ ] `processEvent(const Event*)` 处理单个事件（默认由 `processEventRange` 逐个调用）。
- [ ] （可选）`processEventRange(begin, end)` 覆盖默认的逐个事件循环，实现批量写入优化。
- [ ] （可选）`onTimestampCutoffChanged()` 在窗口边界推进时响应（如重新计算 `start_timestamp_`）。
- [ ] `setupMessage()` 填充 `message_.identifier`、`message_.header`、模块元信息，并调用 `fillReplayWindow(...)`。
- [ ] `publishCurrentWindow()` 完成缓冲区并触发 `publish()`。

### 第三步：图像内存模块快捷方式

若模块输出固定大小的 `uint8_t` 图像，继承 `ImageMemoryModuleBase<MsgType>`，仅需实现：

```cpp
class CustomImageModule : public ImageMemoryModuleBase<event_msgs::msg::CustomImageMemory>
{
public:
  CustomImageModule(rclcpp::Node *node, const std::string &identifier,
                    const std::string &topic_name, size_t history_depth,
                    double lifetime);

private:
  int initialValue() const override;                 // 如 BG_COLOR 或 0
  void processEvent(const Event *event_ptr) override; // 单像素写入
  void processEventRange(const Event *begin,
                         const Event *end) override;  // 批量优化
};
```

适当时可复用现有像素辅助函数：

```cpp
// 在 processEventRange(...) 中
pixel_operations::stackTimeSurfaceRange(
    data_, begin, end, start_timestamp_, region_of_interest_, width_, channel_);

pixel_operations::stackHistogramRange(
    data_, begin, end, start_timestamp_, region_of_interest_, width_, channel_);
```

### 第四步：接入发布节点

修改 **`fluxeem_ros2_publisher.h`**：

1. 添加 `#include` 与前向声明。
2. 添加成员：`std::unique_ptr<CustomModule> custom_module_;`。
3. 添加参数设置函数声明：`void setCustomModuleParams();`。
4. 在模板辅助函数中注册该模块：
   - `forEachWindowedModule` — 所有窗口化模块
   - `forEachSharedMemoryModule` — 共享内存模块（用于清理与回放元数据）
   - `forEachTriggerWindowModule` — 跟随 Trigger 发布的模块（注意：`EventMemoryModule` 被排除在外，由独立逻辑处理）

修改 **`fluxeem_ros2_publisher_setup.cpp`**：

1. 实现 `setCustomModuleParams()`：
   - 声明 `custom_module.on_off`、`topic_name`、`timespan` 等参数。
   - 若启用：`std::make_unique<CustomModule>(...)` → `setupModule(...)`。
   - 调用 `updateMinModuleTimespan(timespan)`。
2. 在 `setParams()` 中调用 `setCustomModuleParams()`。

修改 **`fluxeem_ros2_publisher_core.cpp`**：

- 在 `accumulateEvents(...)` 中分发到 `custom_module_->processEvents(...)`（timespan 模式）或 `custom_module_->processEventsWithoutWindow(...)`（trigger 累积模式）。

修改 **`CMakeLists.txt`**：

- 将 `src/custom_module.cpp` 追加到 `SRC_FILES`。

修改 **`params/param.yaml`**：

```yaml
custom_module:
  on_off: false
  topic_name: "custom_topic"
  timespan: 33333
  # ... 自定义参数
```

### 第五步：编译与验证

```bash
colcon build --packages-select event_msgs camera_interfaces fluxeem_ros2_publisher
```

验证项：

- [ ] `timespan` 模式：模块按配置的 `timespan` 间隔发布。
- [ ] `trigger` 模式：模块跟随 Trigger 边沿发布；`ReplayWindow.trigger_*` 字段正确。
- [ ] `raw_file` 模式：`raw_playback.interval_us=0` 时能正确回退到最小已启用模块的 `timespan`。
- [ ] 订阅端可通过 `message.identifier` 调用 `shm_open`，并根据消息元信息计算正确的 `mmap` 大小。

---

## 源码结构

```
fluxeem_ros2_publisher/
├── include/fluxeem_ros2_publisher/
│   ├── fluxeem_ros2_publisher.h          # 主节点类 + 模块遍历模板辅助函数
│   ├── event_topic_module.hpp            # WindowedEventModuleBase、EventTopicModule
│   ├── modules.hpp                       # ImageMemoryModuleBase、具体模块
│   ├── pixel_operations.hpp              # 批量像素写入辅助
│   ├── camera_settings.hpp               # Bias / ERC / Trail Filter 校验
│   └── publisher_constant.h              # QoS、清理 TTL、颜色常量
├── src/
│   ├── fluxeem_ros2_publisher.cpp        # main()、信号桥接、cleanShutdown()
│   ├── fluxeem_ros2_publisher_core.cpp   # 相机/Raw I/O、事件分发、Trigger 处理
│   ├── fluxeem_ros2_publisher_setup.cpp  # 参数声明与模块实例化
│   ├── fluxeem_ros2_publisher_services.cpp  # ROS Service 回调
│   ├── fluxeem_ros2_publisher_camera.cpp    # OpenEB 相机控制封装
│   ├── event_memory.cpp                  # 原始事件共享内存模块
│   ├── time_surface_memory.cpp           # Time Surface 共享内存模块
│   ├── histogram_memory.cpp              # Histogram 共享内存模块
│   ├── event_to_image.cpp                # 图像话题模块
│   └── pixel_operations.cpp              # stackTimeSurfaceRange、stackHistogramRange
├── launch/
│   └── launch.py                         # 加载 param.yaml 的启动文件
├── params/
│   └── param.yaml                        # 默认运行参数
└── CMakeLists.txt
```

---

## 故障排查

| 现象 | 可能原因 | 解决方法 |
|------|----------|----------|
| 编译失败：找不到 `MetavisionSDK` | OpenEB 安装路径未暴露给 CMake | source OpenEB 安装环境，或设置 `CMAKE_PREFIX_PATH` / `MetavisionSDK_DIR` |
| 启动失败：找不到 `libmetavision_*.so` | OpenEB 运行库路径未被动态链接器加载 | 执行 `sudo ldconfig`，或将 OpenEB 的 `lib` 目录加入 `LD_LIBRARY_PATH` |
| 未检测到相机 | 相机未连接或权限不足 | 检查 USB 连接；确认用户在相关用户组（如 `plugdev`） |
| `/dev/shm` 下共享内存文件累积 | 节点异常退出，未执行优雅关闭 | 手动清理：`rm /dev/shm/fluxeem_*` |
| 回放模式 CPU 占用高 | `raw_playback.rate` 过高或切片间隔过小 | 增大 `interval_us` 或降低 `rate` |
| `event_memory` 丢事件 | `target_num_events_per_sec` 开启且限制过严 | 调整 `target_num_events_per_sec` 或 `temporal_sample_rate_min` |
| Trigger 模式下图像不发布 | `trigger_polarity` 不匹配或没有 TriggerIn 信号 | 检查极性设置与相机 Trigger 接线 |

---

## 贡献指南

欢迎贡献！请遵循现有代码风格（由 `ament_lint_auto` 强制执行）：

1. Fork 本仓库。
2. 创建功能分支（`git checkout -b feature/my-feature`）。
3. 提交变更并确保提交信息清晰。
4. 确保 `colcon test` 对修改的包通过。
5. 提交 Pull Request，描述动机与设计。

---

## 许可证

本项目基于 **Apache License 2.0** 开源。完整的许可证文本见各包目录下的 `LICENSE` 文件。

```
Copyright (c) Fluxeem Technology Co., Ltd.
```

---

<p align="center">
  Built with love by <a href="https://fluxeem.com/">Fluxeem (瞬极科技)</a>
</p>
