/*
 * @Description: 声明 LR11xx 与 LR20xx 共用的传输上下文
 * @Author: LILYGO_L
 * @Date: 2026-07-10 00:00:00
 * @LastEditTime: 2026-07-15 01:12:34
 * @License: GPL 3.0
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>

#include "bus/bus_guide.h"

namespace usp_cpp_bus_driver {
// LR11xx 和 LR20xx HAL 入口共用的传输状态
struct LrContext {
  // 驱动低电平有效的硬件复位信号
  using ResetCallback = std::function<bool(bool)>;

  // 执行开发板相关的 NSS 唤醒脉冲
  using WakeupCallback = std::function<bool()>;

  // 与应用程序共享的 SPI 传输对象
  std::shared_ptr<cpp_bus_driver::BusSpiGuide> bus;
  // 由应用程序提供的开发板复位操作
  ResetCallback reset_callback;
  // 由应用程序提供的开发板唤醒操作
  WakeupCallback wakeup_callback;
  // 连接无线芯片 BUSY 输出的 GPIO
  int32_t busy_pin = -1;
  // 已配置的 SPI 时钟频率，单位为 Hz
  int32_t frequency_hz = -1;
  // SPI 传输对象用作 NSS 的 GPIO
  int32_t cs_pin = -1;
  // SPI 初始化成功后为 true
  bool initialized = false;
  // 无线芯片接受休眠命令后为 true
  bool sleeping = false;

  /**
   * @brief 配置 BUSY 引脚并初始化 SPI 设备
   * @param new_frequency_hz SPI 时钟频率，单位为 Hz
   * @return GPIO 和 SPI 初始化成功时返回 true
   */
  bool Init(int32_t new_frequency_hz);

  /**
   * @brief 释放当前无线芯片使用的 SPI 设备
   * @param delete_bus 为 true 时同时释放底层共享 SPI 总线
   * @return 请求的资源全部释放成功时返回 true
   */
  bool Deinit(bool delete_bus);

  /**
   * @brief 等待无线芯片 BUSY 输出变为低电平
   * @param timeout_us 最大等待时间，单位为 us
   * @return 超时前无线芯片准备完成时返回 true
   */
  bool WaitWhileBusy(uint32_t timeout_us = 1000000) const;

  /**
   * @brief 检查无线芯片是否已经准备完成
   * @return 无线芯片已唤醒且 BUSY 为低电平时返回 true
   */
  bool CheckReady();

  /**
   * @brief 执行低电平有效的硬件复位时序
   * @param reset_time_ms 复位信号保持低电平的时间，单位为 ms
   * @param startup_time_ms 释放复位信号后的启动延时，单位为 ms
   * @return 复位信号切换成功且 BUSY 恢复低电平时返回 true
   */
  bool Reset(uint32_t reset_time_ms, uint32_t startup_time_ms);

  /**
   * @brief 休眠命令执行后调用开发板唤醒操作
   * @return 唤醒完成且 BUSY 变为低电平时返回 true
   */
  bool Wakeup();

  /**
   * @brief 在一次 NSS 事务中写入命令和负载
   * @param command 无线芯片命令字节
   * @param command_length 命令字节数
   * @param data 可选的命令负载
   * @param data_length 负载字节数
   * @return 整个 SPI 事务成功时返回 true
   */
  bool Write(const uint8_t* command, size_t command_length, const uint8_t* data,
      size_t data_length);

  /**
   * @brief 写入命令并等待 BUSY，然后读取原始响应字节
   * @param command 无线芯片命令字节
   * @param command_length 命令字节数
   * @param response 包含空数据在内的全部响应目标缓冲区
   * @param response_length 需要从无线芯片读取的响应字节数
   * @return 两次 NSS 事务均成功时返回 true
   */
  bool ReadRaw(const uint8_t* command, size_t command_length, uint8_t* response,
      size_t response_length);

  /**
   * @brief 执行两阶段读取并移除响应前面的空字节
   * @param command 无线芯片命令字节
   * @param command_length 命令字节数
   * @param dummy_length 需要丢弃的响应前导字节数
   * @param data 解析后响应数据的目标缓冲区
   * @param data_length 响应数据字节数
   * @return 命令和响应事务均成功时返回 true
   */
  bool Read(const uint8_t* command, size_t command_length, size_t dummy_length,
      uint8_t* data, size_t data_length);

  /**
   * @brief 在 MOSI 发送零字节的同时读取响应
   * @param data 响应数据目标缓冲区
   * @param data_length 需要读取的字节数
   * @return SPI 直接读取成功时返回 true
   */
  bool DirectRead(uint8_t* data, size_t data_length);

  /**
   * @brief 在同一次 NSS 事务中发送命令并读取数据
   * @param command 无线芯片命令字节
   * @param command_length 命令字节数
   * @param data 响应数据目标缓冲区
   * @param data_length 命令之后的响应字节数
   * @return SPI 全双工事务成功时返回 true
   */
  bool DirectRead(const uint8_t* command, size_t command_length, uint8_t* data,
      size_t data_length);
};
}  // namespace usp_cpp_bus_driver
