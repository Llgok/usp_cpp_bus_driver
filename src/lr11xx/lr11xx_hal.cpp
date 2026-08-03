/*
 * @Description: 实现 USP LR11xx 硬件抽象层
 * @Author: LILYGO_L
 * @Date: 2026-07-10 00:00:00
 * @LastEditTime: 2026-08-03 16:15:04
 * @License: GPL 3.0
 */
#include "smtc_rac_lib/radio_drivers/lr11xx_driver/src/lr11xx_hal.h"

#include <algorithm>
#include <array>
#include <memory>
#include <new>

#include "common/lr_context.h"

namespace {
constexpr uint8_t kSystemCommandGroup = 0x01;
constexpr uint8_t kSetSleepOpcode = 0x1B;
constexpr uint32_t kResetTimeMs = 5;
constexpr uint32_t kStartupTimeMs = 205;

/**
 * @brief 将 USP 不透明上下文转换为桥接驱动传输上下文
 * @param context USP 驱动传入的不透明上下文
 * @return 可写的桥接驱动传输上下文
 */
usp_cpp_bus_driver::LrContext* GetContext(const void* context) {
  return const_cast<usp_cpp_bus_driver::LrContext*>(
      static_cast<const usp_cpp_bus_driver::LrContext*>(context));
}

/**
 * @brief 写入 LR11xx 命令，启用 SPI CRC 时自动追加校验字节
 * @param radio 已初始化的传输上下文
 * @param command 在负载之前发送的命令字节
 * @param command_length 命令字节数
 * @param data 可选的命令负载
 * @param data_length 负载字节数
 * @return 内存分配、CRC 生成和 SPI 传输均成功时返回 true
 */
bool WriteCommand(usp_cpp_bus_driver::LrContext* radio, const uint8_t* command,
    uint16_t command_length, const uint8_t* data, uint16_t data_length) {
  if ((radio == nullptr) || (command == nullptr) || (command_length == 0) ||
      ((data == nullptr) && (data_length != 0))) {
    return false;
  }
#if defined(USE_LR11XX_CRC_OVER_SPI)
  auto payload = std::unique_ptr<uint8_t[]>(
      new (std::nothrow) uint8_t[static_cast<size_t>(data_length) + 1]);
  if (payload == nullptr) {
    return false;
  }
  if (data_length != 0) {
    std::copy_n(data, data_length, payload.get());
  }
  uint8_t crc = lr11xx_hal_compute_crc(0xFF, command, command_length);
  payload[data_length] = lr11xx_hal_compute_crc(crc, data, data_length);
  return radio->Write(command, command_length, payload.get(), data_length + 1);
#else
  return radio->Write(command, command_length, data, data_length);
#endif
}

/**
 * @brief 执行 LR11xx 命令和响应两阶段事务
 * @param radio 已初始化的传输上下文
 * @param command 第一次 NSS 事务发送的命令字节
 * @param command_length 命令字节数
 * @param data 空字节之后的响应数据目标缓冲区
 * @param data_length 请求读取的响应字节数
 * @return 传输和可选 CRC 校验均成功时返回 true
 */
bool ReadCommand(usp_cpp_bus_driver::LrContext* radio, const uint8_t* command,
    uint16_t command_length, uint8_t* data, uint16_t data_length) {
  if ((radio == nullptr) || (command == nullptr) || (command_length == 0) ||
      ((data == nullptr) && (data_length != 0))) {
    return false;
  }
#if defined(USE_LR11XX_CRC_OVER_SPI)
  auto command_with_crc = std::unique_ptr<uint8_t[]>(
      new (std::nothrow) uint8_t[static_cast<size_t>(command_length) + 1]);
  const size_t response_length = static_cast<size_t>(data_length) + 2;
  auto response =
      std::unique_ptr<uint8_t[]>(new (std::nothrow) uint8_t[response_length]());
  if ((command_with_crc == nullptr) || (response == nullptr)) {
    return false;
  }
  std::copy_n(command, command_length, command_with_crc.get());
  command_with_crc[command_length] =
      lr11xx_hal_compute_crc(0xFF, command, command_length);
  if (!radio->ReadRaw(command_with_crc.get(), command_length + 1,
          response.get(), response_length)) {
    return false;
  }
  const uint8_t expected_crc = lr11xx_hal_compute_crc(
      0xFF, response.get(), static_cast<uint16_t>(response_length - 1));
  if (response[response_length - 1] != expected_crc) {
    return false;
  }
  if (data_length != 0) {
    std::copy_n(response.get() + 1, data_length, data);
  }
  return true;
#else
  return radio->Read(command, command_length, 1, data, data_length);
#endif
}
}  // namespace

/**
 * @brief 通过共享传输上下文写入 LR11xx 命令
 * @param context LR11xx 桥接驱动上下文
 * @param command 命令字节缓冲区
 * @param command_length 命令字节数
 * @param data 可选的命令负载
 * @param data_length 负载字节数
 * @return HAL 事务执行结果
 */
extern "C" lr11xx_hal_status_t lr11xx_hal_write(const void* context,
    const uint8_t* command, uint16_t command_length, const uint8_t* data,
    uint16_t data_length) {
  auto* radio = GetContext(context);
  if ((radio == nullptr) ||
      !WriteCommand(radio, command, command_length, data, data_length)) {
    return LR11XX_HAL_STATUS_ERROR;
  }
  if ((command_length >= 2) && (command[0] == kSystemCommandGroup) &&
      (command[1] == kSetSleepOpcode)) {
    radio->sleeping = true;
    radio->bus->DelayUs(500);
  }
  return LR11XX_HAL_STATUS_OK;
}

/**
 * @brief 执行 LR11xx 命令和响应两阶段事务
 * @param context LR11xx 桥接驱动上下文
 * @param command 命令字节缓冲区
 * @param command_length 命令字节数
 * @param data 响应数据目标缓冲区
 * @param data_length 响应数据字节数
 * @return HAL 事务执行结果
 */
extern "C" lr11xx_hal_status_t lr11xx_hal_read(const void* context,
    const uint8_t* command, uint16_t command_length, uint8_t* data,
    uint16_t data_length) {
  auto* radio = GetContext(context);
  if ((radio == nullptr) ||
      !ReadCommand(radio, command, command_length, data, data_length)) {
    return LR11XX_HAL_STATUS_ERROR;
  }
  return LR11XX_HAL_STATUS_OK;
}

/**
 * @brief 发送零填充字节并直接读取 LR11xx 响应
 * @param context LR11xx 桥接驱动上下文
 * @param data 响应数据目标缓冲区
 * @param data_length 响应数据字节数
 * @return HAL 事务执行结果
 */
extern "C" lr11xx_hal_status_t lr11xx_hal_direct_read(
    const void* context, uint8_t* data, uint16_t data_length) {
  auto* radio = GetContext(context);
#if defined(USE_LR11XX_CRC_OVER_SPI)
  if ((data == nullptr) && (data_length != 0)) {
    return LR11XX_HAL_STATUS_ERROR;
  }
  const size_t response_length = static_cast<size_t>(data_length) + 1;
  auto response =
      std::unique_ptr<uint8_t[]>(new (std::nothrow) uint8_t[response_length]());
  if ((radio == nullptr) || (response == nullptr) ||
      !radio->DirectRead(response.get(), response_length)) {
    return LR11XX_HAL_STATUS_ERROR;
  }
  const uint8_t expected_crc =
      lr11xx_hal_compute_crc(0xFF, response.get(), data_length);
  if (response[data_length] != expected_crc) {
    return LR11XX_HAL_STATUS_ERROR;
  }
  if (data_length != 0) {
    std::copy_n(response.get(), data_length, data);
  }
#else
  if ((radio == nullptr) || !radio->DirectRead(data, data_length)) {
    return LR11XX_HAL_STATUS_ERROR;
  }
#endif
  return LR11XX_HAL_STATUS_OK;
}

/**
 * @brief 执行 LR11xx 低电平复位和固件启动延时
 * @param context LR11xx 桥接驱动上下文
 * @return HAL 复位执行结果
 */
extern "C" lr11xx_hal_status_t lr11xx_hal_reset(const void* context) {
  auto* radio = GetContext(context);
  if ((radio == nullptr) || !radio->Reset(kResetTimeMs, kStartupTimeMs)) {
    return LR11XX_HAL_STATUS_ERROR;
  }
  return LR11XX_HAL_STATUS_OK;
}

/**
 * @brief 按需唤醒 LR11xx，并等待 BUSY 拉低
 * @param context LR11xx 桥接驱动上下文
 * @return HAL 唤醒执行结果
 */
extern "C" lr11xx_hal_status_t lr11xx_hal_wakeup(const void* context) {
  auto* radio = GetContext(context);
  if ((radio == nullptr) || !radio->CheckReady()) {
    return LR11XX_HAL_STATUS_ERROR;
  }
  return LR11XX_HAL_STATUS_OK;
}

/**
 * @brief 发送四个零字节以中止 LR11xx 阻塞命令
 * @param context LR11xx 桥接驱动上下文
 * @return HAL 中止命令执行结果
 */
extern "C" lr11xx_hal_status_t lr11xx_hal_abort_blocking_cmd(
    const void* context) {
  auto* radio = GetContext(context);
  constexpr std::array<uint8_t, 4> kAbortCommand = {};
  if ((radio == nullptr) || !radio->initialized || (radio->bus == nullptr) ||
      !radio->bus->Write(kAbortCommand.data(), kAbortCommand.size()) ||
      !radio->WaitWhileBusy()) {
    return LR11XX_HAL_STATUS_ERROR;
  }
  return LR11XX_HAL_STATUS_OK;
}
