/*
 * @Description: 实现 USP SX126x 硬件抽象层与传输操作
 * @Author: LILYGO_L
 * @Date: 2026-07-10 00:00:00
 * @LastEditTime: 2026-07-15 01:12:34
 * @License: GPL 3.0
 */
#include "sx126x_hal.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "sx126x/sx126x_context.h"

namespace usp_cpp_bus_driver {
namespace {
constexpr size_t kMaxTransactionSize = 272;
constexpr uint8_t kGetStatusOpcode = 0xC0;
constexpr uint8_t kSetSleepOpcode = 0x84;
}  // namespace

bool Sx126xContext::WaitWhileBusy(uint32_t timeout_us) const {
  if ((bus == nullptr) || (busy_pin < 0)) {
    return false;
  }
  for (uint32_t elapsed_us = 0; elapsed_us < timeout_us; ++elapsed_us) {
    if (bus->GpioRead(busy_pin) == 0) {
      return true;
    }
    bus->DelayUs(1);
  }
  bus->LogMessage(cpp_bus_driver::Tool::LogLevel::kError, __FILE__, __LINE__,
      "SX126x busy timeout\n");
  return false;
}

bool Sx126xContext::Wakeup() {
  if (!sleeping) {
    return WaitWhileBusy();
  }

  const std::array<uint8_t, 2> write_buffer = {kGetStatusOpcode, 0};
  std::array<uint8_t, 2> read_buffer = {};
  if (!bus->WriteRead(
          write_buffer.data(), read_buffer.data(), write_buffer.size())) {
    return false;
  }
  sleeping = false;
  return WaitWhileBusy();
}
}  // namespace usp_cpp_bus_driver

/**
 * @brief 在一次 SPI 事务中写入 SX126x 命令和可选负载
 * @param context SX126x 桥接驱动上下文
 * @param command 命令字节缓冲区
 * @param command_length 命令字节数
 * @param data 可选的命令负载
 * @param data_length 负载字节数
 * @return HAL 事务执行结果
 */
extern "C" sx126x_hal_status_t sx126x_hal_write(const void* context,
    const uint8_t* command, uint16_t command_length, const uint8_t* data,
    uint16_t data_length) {
  auto* local_context = const_cast<usp_cpp_bus_driver::Sx126xContext*>(
      static_cast<const usp_cpp_bus_driver::Sx126xContext*>(context));
  if ((local_context == nullptr) || !local_context->initialized ||
      (command == nullptr) || (command_length == 0) ||
      ((data == nullptr) && (data_length != 0)) ||
      (command_length + data_length >
          usp_cpp_bus_driver::kMaxTransactionSize)) {
    return SX126X_HAL_STATUS_ERROR;
  }
  if (!local_context->Wakeup()) {
    return SX126X_HAL_STATUS_ERROR;
  }

  std::array<uint8_t, usp_cpp_bus_driver::kMaxTransactionSize> buffer = {};
  std::memcpy(buffer.data(), command, command_length);
  if (data_length != 0) {
    std::memcpy(buffer.data() + command_length, data, data_length);
  }
  if (!local_context->bus->Write(buffer.data(), command_length + data_length)) {
    return SX126X_HAL_STATUS_ERROR;
  }

  if (command[0] == usp_cpp_bus_driver::kSetSleepOpcode) {
    local_context->sleeping = true;
    return SX126X_HAL_STATUS_OK;
  }
  return local_context->WaitWhileBusy() ? SX126X_HAL_STATUS_OK
                                        : SX126X_HAL_STATUS_ERROR;
}

/**
 * @brief 发送零填充字节并读取 SX126x 命令响应
 * @param context SX126x 桥接驱动上下文
 * @param command 命令字节缓冲区
 * @param command_length 命令字节数
 * @param data 响应数据目标缓冲区
 * @param data_length 响应数据字节数
 * @return HAL 事务执行结果
 */
extern "C" sx126x_hal_status_t sx126x_hal_read(const void* context,
    const uint8_t* command, uint16_t command_length, uint8_t* data,
    uint16_t data_length) {
  auto* local_context = const_cast<usp_cpp_bus_driver::Sx126xContext*>(
      static_cast<const usp_cpp_bus_driver::Sx126xContext*>(context));
  if ((local_context == nullptr) || !local_context->initialized ||
      (command == nullptr) || (command_length == 0) || (data == nullptr) ||
      (command_length + data_length >
          usp_cpp_bus_driver::kMaxTransactionSize)) {
    return SX126X_HAL_STATUS_ERROR;
  }
  if (!local_context->Wakeup()) {
    return SX126X_HAL_STATUS_ERROR;
  }

  std::array<uint8_t, usp_cpp_bus_driver::kMaxTransactionSize>
      write_buffer = {};
  std::array<uint8_t, usp_cpp_bus_driver::kMaxTransactionSize> read_buffer =
      {};
  std::memcpy(write_buffer.data(), command, command_length);
  const size_t transaction_size = command_length + data_length;
  if (!local_context->bus->WriteRead(
          write_buffer.data(), read_buffer.data(), transaction_size)) {
    return SX126X_HAL_STATUS_ERROR;
  }
  std::memcpy(data, read_buffer.data() + command_length, data_length);
  return local_context->WaitWhileBusy() ? SX126X_HAL_STATUS_OK
                                        : SX126X_HAL_STATUS_ERROR;
}

/**
 * @brief 执行低电平有效的 SX126x 硬件复位时序
 * @param context SX126x 桥接驱动上下文
 * @return HAL 复位执行结果
 */
extern "C" sx126x_hal_status_t sx126x_hal_reset(const void* context) {
  auto* local_context = const_cast<usp_cpp_bus_driver::Sx126xContext*>(
      static_cast<const usp_cpp_bus_driver::Sx126xContext*>(context));
  if ((local_context == nullptr) || !local_context->initialized ||
      !local_context->reset_callback) {
    return SX126X_HAL_STATUS_ERROR;
  }

  bool result = local_context->reset_callback(false);
  local_context->bus->DelayMs(5);
  result &= local_context->reset_callback(true);
  local_context->bus->DelayMs(5);
  local_context->sleeping = false;
  result &= local_context->WaitWhileBusy();
  return result ? SX126X_HAL_STATUS_OK : SX126X_HAL_STATUS_ERROR;
}

/**
 * @brief 使用 GetStatus 命令唤醒 SX126x，并等待 BUSY 拉低
 * @param context SX126x 桥接驱动上下文
 * @return HAL 唤醒执行结果
 */
extern "C" sx126x_hal_status_t sx126x_hal_wakeup(const void* context) {
  auto* local_context = const_cast<usp_cpp_bus_driver::Sx126xContext*>(
      static_cast<const usp_cpp_bus_driver::Sx126xContext*>(context));
  if ((local_context == nullptr) || !local_context->initialized) {
    return SX126X_HAL_STATUS_ERROR;
  }
  return local_context->Wakeup() ? SX126X_HAL_STATUS_OK
                                 : SX126X_HAL_STATUS_ERROR;
}
