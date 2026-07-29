/*
 * @Description: 声明 USP SX126x 传输上下文
 * @Author: LILYGO_L
 * @Date: 2026-07-10 00:00:00
 * @LastEditTime: 2026-07-16 10:47:02
 * @License: GPL 3.0
 */
#pragma once

#include <cstdint>
#include <functional>
#include <memory>

#include "bus/bus_guide.h"

namespace usp_cpp_bus_driver {
// USP SX126x HAL 实现所需的传输状态
struct Sx126xContext {
  // 与应用程序共享的 SPI 传输对象
  std::shared_ptr<cpp_bus_driver::BusSpiGuide> bus;
  // 控制低电平有效的硬件复位信号
  std::function<bool(bool)> reset_callback;
  // 连接无线芯片 BUSY 输出的 GPIO
  int32_t busy_pin = -1;
  // 已配置的 SPI 时钟频率，单位为 Hz
  int32_t frequency_hz = -1;
  // 由硬件 SPI 控制的低电平有效 NSS GPIO
  int32_t cs_pin = -1;
  // SPI 初始化成功后为 true
  bool initialized = false;
  // SX126x 接受休眠命令后为 true
  bool sleeping = false;

  /**
   * @brief 等待 SX126x BUSY 输出变为低电平
   * @param timeout_us 最大等待时间，单位为 us
   * @return 超时前无线芯片准备完成时返回 true
   */
  bool WaitWhileBusy(uint32_t timeout_us = 1000000) const;

  /**
   * @brief 使用 NSS 低电平唤醒 SX126x 并恢复硬件 SPI Device
   * @return 唤醒时序执行成功时返回 true
   */
  bool Wakeup();
};
}  // namespace usp_cpp_bus_driver
