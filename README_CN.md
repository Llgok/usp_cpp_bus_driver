<h1 align="center">semtech_cpp_bus_driver</h1>

## **[英文](./README.md) | 中文**

[![Release](https://img.shields.io/github/v/release/Llgok/semtech_cpp_bus_driver?style=flat-square)](https://github.com/Llgok/semtech_cpp_bus_driver/releases)
[![License](https://img.shields.io/github/license/Llgok/semtech_cpp_bus_driver?style=flat-square)](./LICENSE)
[![ESP-IDF](https://img.shields.io/badge/ESP--IDF-v5.5.3%2B-ff6f00?style=flat-square)](https://github.com/espressif/esp-idf)

`semtech_cpp_bus_driver` 是 Semtech 官方无线驱动与
[`cpp_bus_driver`](https://github.com/Llgok/cpp_bus_driver) 之间的 ESP-IDF
桥接组件。它使用 C++ 对象管理传输资源，并实现官方驱动所需的底层硬件接口。

## 目录

- [特性](#特性)
- [支持框架](#支持框架)
- [快速开始](#快速开始)
- [注意事项](#注意事项)

## 特性

- 使用 `cpp_bus_driver` 提供 SPI、GPIO、延时和日志能力。
- 提供复位、中断、计时、BUSY 等待及休眠唤醒等底层适配。
- 通过 `context()` 和 `Invoke()` 复用 Semtech 官方 API，无需重复封装协议接口。
- 支持由应用通过回调实现开发板相关操作。
- 对 BUSY 等待设置超时，避免永久阻塞。

组件的实际支持范围以公开头文件、构建配置和随附的官方驱动源码为准。

## 支持框架

| 框架 | 状态 | 说明 |
| --- | --- | --- |
| ESP-IDF | 支持 | 通过 `cpp_bus_driver` 提供底层硬件访问 |

## 快速开始

### 集成方式

#### 作为 ESP-IDF component 使用

推荐把 `cpp_bus_driver` 与本仓库放入工程的 `components` 目录：

```text
your_project/
├── components/
│   ├── cpp_bus_driver/
│   └── semtech_cpp_bus_driver/
├── main/
└── CMakeLists.txt
```

然后在代码中包含统一入口：

```cpp
#include "semtech_cpp_bus_driver_library.h"
```

如只使用部分功能，可直接包含对应目录下的聚合头文件。可用入口以组件中的
公开头文件为准。

#### 作为 Git submodule 使用

```bash
git submodule add https://github.com/Llgok/cpp_bus_driver.git
git submodule add https://github.com/Llgok/semtech_cpp_bus_driver.git
git submodule update --init --recursive
```

子模块目录可以按工程结构自定义，只需确保 ESP-IDF 能够找到对应组件。

### 创建 SPI 总线

先使用 `cpp_bus_driver` 创建 SPI 总线，再把总线传入无线驱动对象：

```cpp
#include <memory>

#include "cpp_bus_driver_library.h"
#include "semtech_cpp_bus_driver_library.h"

auto spi_bus = std::make_shared<cpp_bus_driver::HardwareSpi>(
    mosi_pin, sclk_pin, miso_pin, SPI2_HOST, 0);
```

SPI 主机、引脚和通信频率应根据实际硬件设置。

### 创建驱动对象

驱动对象建立在 SPI 总线之上。根据目标硬件选择统一入口中公开的驱动类型，
并传入所需的片选、状态引脚及板级回调。构造参数以对应公开头文件为准。

### 生命周期与官方 API

典型流程如下：

```cpp
if (!radio.Init(spi_frequency_hz)) {
  return;
}

// Configure and use the radio here.

radio.Deinit(false);
```

初始化后，可以使用组件提供的便捷接口，也可以通过 `Invoke()` 或 `context()`
调用 Semtech 官方 API。

硬件参数、初始化顺序和协议配置应按照所用开发板及 Semtech 官方文档设置。

## 注意事项

`Invoke()` 会自动传入已检查的驱动上下文，并保留官方函数的参数和返回类型。
它只能在对象成功初始化后使用。

- `Deinit(false)` 仅释放当前对象使用的 SPI 设备。
- `Deinit(true)` 同时请求删除底层 SPI 总线；仅应在当前对象独占该总线时使用。
- 对象及其上下文默认不提供多任务并发访问保护。
- 复位、唤醒和中断等回调由应用根据实际硬件实现。

详细 API 请查看公开头文件及 Semtech 官方驱动文档。
