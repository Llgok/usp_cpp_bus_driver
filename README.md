<h1 align="center">usp_cpp_bus_driver</h1>

## **English | [Chinese](./README_CN.md)**

[![Release](https://img.shields.io/github/v/release/Llgok/usp_cpp_bus_driver?style=flat-square)](https://github.com/Llgok/usp_cpp_bus_driver/releases)
[![License](https://img.shields.io/github/license/Llgok/usp_cpp_bus_driver?style=flat-square)](./LICENSE)
[![ESP-IDF](https://img.shields.io/badge/ESP--IDF-v5.5.3%2B-ff6f00?style=flat-square)](https://github.com/espressif/esp-idf)

`usp_cpp_bus_driver` is an ESP-IDF bridge between the official USP radio
driver and
[`cpp_bus_driver`](https://github.com/Llgok/cpp_bus_driver).
It manages transport resources with C++ objects and implements the low-level
hardware interfaces required by the official drivers.

## Table of Contents

- [Features](#features)
- [Supported Frameworks](#supported-frameworks)
- [Quick Start](#quick-start)
- [Notes](#notes)

## Features

- Uses `cpp_bus_driver` for SPI, GPIO, delay, and logging services.
- Adapts reset, interrupts, timers, BUSY waits, and sleep wake-up operations.
- Reuses the official USP APIs through `context()` and `Invoke()`
  instead
  of duplicating protocol interfaces.
- Allows applications to supply board-specific operations through callbacks.
- Applies a timeout to BUSY waits to prevent permanent blocking.

The public headers, build configuration, and bundled official driver sources
are the source of truth for the currently supported functionality.

## Supported Frameworks

| Framework | Status | Description |
| --- | --- | --- |
| ESP-IDF | Supported | Uses `cpp_bus_driver` for low-level hardware access |

## Quick Start

### Integration

#### Use as ESP-IDF Components

Place `cpp_bus_driver` and this repository in the project's `components`
directory:

```text
your_project/
├── components/
│   ├── cpp_bus_driver/
│   └── usp_cpp_bus_driver/
├── main/
└── CMakeLists.txt
```

When cloning this repository, please also fetch its submodules:

```bash
git clone --recursive https://github.com/Llgok/usp_cpp_bus_driver.git
```

If you did not use `--recursive` when cloning, initialize the submodules manually:

```bash
git submodule update --init --recursive
```

Then include the unified entry header:

```cpp
#include "usp_cpp_bus_driver_library.h"
```

When only part of the component is needed, include the relevant aggregate
header directly. Refer to the component's public headers for available entry
points.

#### Use as Git Submodules

```bash
git submodule add https://github.com/Llgok/cpp_bus_driver.git
git submodule add https://github.com/Llgok/usp_cpp_bus_driver.git
git submodule update --init --recursive
```

Submodule directories may be customized to match the project layout, as long
as ESP-IDF can discover the components.

### Create the SPI Bus

Create the SPI bus with `cpp_bus_driver`, then pass it to the radio driver
object:

```cpp
#include <memory>

#include "cpp_bus_driver_library.h"
#include "usp_cpp_bus_driver_library.h"

auto spi_bus = std::make_shared<cpp_bus_driver::HardwareSpi>(
    mosi_pin, sclk_pin, miso_pin, SPI2_HOST, 0);
```

Select the SPI host, pins, and communication frequency for the target hardware.

### Create the Driver Object

Driver objects are built on top of the SPI bus. Select a driver type exposed
by the unified entry point for the target hardware, then provide its required
chip-select, status pins, and board callbacks. Refer to the corresponding
public header for constructor parameters.

### Lifecycle and Official APIs

A typical lifecycle is:

```cpp
if (!radio.Init(spi_frequency_hz)) {
  return;
}

// Configure and use the radio here.

radio.Deinit(false);
```

After initialization, use the component's convenience interface or call the
official USP APIs through `Invoke()` or `context()`.

Set hardware parameters, initialization order, and protocol configuration
according to the target board and the official USP documentation.

## Notes

`Invoke()` supplies a checked driver context while preserving the official
function's argument and return types. It may only be used after successful
initialization.

- `Deinit(false)` releases only the SPI device used by the current object.
- `Deinit(true)` also requests deletion of the underlying SPI bus; use it only
  when the current object exclusively owns that bus.
- Objects and their contexts do not provide multi-task access protection by
  default.
- The application implements reset, wake-up, interrupt, and other callbacks
  according to its hardware.

Refer to the public headers and the official USP documentation for
detailed APIs.
