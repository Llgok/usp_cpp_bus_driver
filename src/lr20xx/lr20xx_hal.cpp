/*
 * @Description: 实现 USP LR20xx 硬件抽象层
 * @Author: LILYGO_L
 * @Date: 2026-07-10 00:00:00
 * @LastEditTime: 2026-07-15 01:12:34
 * @License: GPL 3.0
 */
#include "smtc_rac_lib/radio_drivers/lr20xx_driver/inc/lr20xx_hal.h"

#include "common/lr_context.h"

namespace {
constexpr uint8_t kSystemCommandGroup = 0x01;
constexpr uint8_t kSetSleepOpcode = 0x27;
constexpr uint32_t kResetTimeMs = 1;
constexpr uint32_t kStartupTimeMs = 1;

/**
 * @brief 将 USP 不透明上下文转换为桥接驱动传输上下文
 * @param context USP 驱动传入的不透明上下文
 * @return 可写的桥接驱动传输上下文
 */
usp_cpp_bus_driver::LrContext* GetContext(const void* context) {
  return const_cast<usp_cpp_bus_driver::LrContext*>(
      static_cast<const usp_cpp_bus_driver::LrContext*>(context));
}
}  // namespace

/**
 * @brief 执行 LR20xx 低电平有效的硬件复位时序
 * @param context LR20xx 桥接驱动上下文
 * @return HAL 复位执行结果
 */
extern "C" lr20xx_hal_status_t lr20xx_hal_reset(const void* context) {
  auto* radio = GetContext(context);
  if ((radio == nullptr) || !radio->Reset(kResetTimeMs, kStartupTimeMs)) {
    return LR20XX_HAL_STATUS_ERROR;
  }
  return LR20XX_HAL_STATUS_OK;
}

/**
 * @brief 执行 LR20xx NSS 唤醒并恢复硬件 SPI Device
 * @param context LR20xx 桥接驱动上下文
 * @return HAL 唤醒执行结果
 */
extern "C" lr20xx_hal_status_t lr20xx_hal_wakeup(const void* context) {
  auto* radio = GetContext(context);
  if ((radio == nullptr) || !radio->Wakeup()) {
    return LR20XX_HAL_STATUS_ERROR;
  }
  return LR20XX_HAL_STATUS_OK;
}

/**
 * @brief 在一次 SPI 事务中写入 LR20xx 命令和可选负载
 * @param context LR20xx 桥接驱动上下文
 * @param command 命令字节缓冲区
 * @param command_length 命令字节数
 * @param data 可选的命令负载
 * @param data_length 负载字节数
 * @return HAL 事务执行结果
 */
extern "C" lr20xx_hal_status_t lr20xx_hal_write(const void* context,
    const uint8_t* command, uint16_t command_length, const uint8_t* data,
    uint16_t data_length) {
  auto* radio = GetContext(context);
  if ((radio == nullptr) ||
      !radio->Write(command, command_length, data, data_length)) {
    return LR20XX_HAL_STATUS_ERROR;
  }
  if ((command_length >= 2) && (command[0] == kSystemCommandGroup) &&
      (command[1] == kSetSleepOpcode)) {
    radio->sleeping = true;
    radio->bus->DelayUs(500);
    if (!radio->bus->Deinit(false)) {
      return LR20XX_HAL_STATUS_ERROR;
    }
  }
  return LR20XX_HAL_STATUS_OK;
}

/**
 * @brief 执行 LR20xx 命令和带两个状态字节的响应事务
 * @param context LR20xx 桥接驱动上下文
 * @param command 命令字节缓冲区
 * @param command_length 命令字节数
 * @param data 响应数据目标缓冲区
 * @param data_length 响应数据字节数
 * @return HAL 事务执行结果
 */
extern "C" lr20xx_hal_status_t lr20xx_hal_read(const void* context,
    const uint8_t* command, uint16_t command_length, uint8_t* data,
    uint16_t data_length) {
  auto* radio = GetContext(context);
  if ((radio == nullptr) ||
      !radio->Read(command, command_length, 2, data, data_length)) {
    return LR20XX_HAL_STATUS_ERROR;
  }
  return LR20XX_HAL_STATUS_OK;
}

/**
 * @brief 发送零填充字节并直接读取 LR20xx 响应
 * @param context LR20xx 桥接驱动上下文
 * @param data 响应数据目标缓冲区
 * @param data_length 响应数据字节数
 * @return HAL 事务执行结果
 */
extern "C" lr20xx_hal_status_t lr20xx_hal_direct_read(
    const void* context, uint8_t* data, uint16_t data_length) {
  auto* radio = GetContext(context);
  if ((radio == nullptr) || !radio->DirectRead(data, data_length)) {
    return LR20XX_HAL_STATUS_ERROR;
  }
  return LR20XX_HAL_STATUS_OK;
}

/**
 * @brief 在同一次 NSS 事务中发送命令并读取 LR20xx FIFO 数据
 * @param context LR20xx 桥接驱动上下文
 * @param command FIFO 读取命令字节
 * @param command_length 命令字节数
 * @param data FIFO 数据目标缓冲区
 * @param data_length FIFO 数据字节数
 * @return HAL 事务执行结果
 */
extern "C" lr20xx_hal_status_t lr20xx_hal_direct_read_fifo(const void* context,
    const uint8_t* command, uint16_t command_length, uint8_t* data,
    uint16_t data_length) {
  auto* radio = GetContext(context);
  if ((radio == nullptr) ||
      !radio->DirectRead(command, command_length, data, data_length)) {
    return LR20XX_HAL_STATUS_ERROR;
  }
  return LR20XX_HAL_STATUS_OK;
}
