/*
 * @Description: None
 * @Author: LILYGO_L
 * @Date: 2026-07-11
 * @LastEditTime: 2026-07-12 12:32:29
 * @License: GPL 3.0
 */
#include "sx127x_hal.h"

#include "sx127x_context.h"

namespace {
/**
 * @brief 从 Semtech SX127x 状态取得可写的 C++ 传输上下文
 * @param radio Semtech SX127x 驱动状态
 * @return hal_context 有效时返回对应上下文，否则返回 nullptr
 */
semtech_cpp_bus_driver::Sx127xContext* GetContext(const sx127x_t* radio) {
  if ((radio == nullptr) || (radio->hal_context == nullptr)) {
    return nullptr;
  }
  return const_cast<semtech_cpp_bus_driver::Sx127xContext*>(
      static_cast<const semtech_cpp_bus_driver::Sx127xContext*>(
          radio->hal_context));
}
}  // namespace

/**
 * @brief 返回当前开发板安装的具体 SX127x 型号
 * @param radio Semtech SX127x 驱动状态
 * @return 上下文中配置的 SX1272/3/6/7/8/9 型号
 */
extern "C" sx127x_radio_id_t sx127x_hal_get_radio_id(const sx127x_t* radio) {
  const auto* context = GetContext(radio);
  return context == nullptr ? SX127X_RADIO_ID_SX1276 : context->radio_id;
}

/**
 * @brief 安装 SX127x DIO0、DIO1 和可选 DIO2 中断
 * @param radio Semtech SX127x 驱动状态
 */
extern "C" void sx127x_hal_dio_irq_attach(const sx127x_t* radio) {
  auto* context = GetContext(radio);
  if ((context != nullptr) && !context->AttachDioInterrupts(radio) &&
      (context->bus != nullptr)) {
    context->bus->LogMessage(cpp_bus_driver::Tool::LogLevel::kError, __FILE__,
        __LINE__, "SX127x DIO interrupt attach failed\n");
  }
}

/**
 * @brief 在同一次 NSS 事务中写入 SX127x 寄存器或 FIFO
 * @param radio Semtech SX127x 驱动状态
 * @param address 寄存器地址，地址 0 表示 FIFO
 * @param data 需要写入的数据
 * @param data_len 写入数据的字节数
 * @return SPI 事务成功时返回 SX127X_HAL_STATUS_OK
 */
extern "C" sx127x_hal_status_t sx127x_hal_write(const sx127x_t* radio,
    const uint16_t address, const uint8_t* data, const uint16_t data_len) {
  auto* context = GetContext(radio);
  return ((context != nullptr) && context->Write(address, data, data_len))
             ? SX127X_HAL_STATUS_OK
             : SX127X_HAL_STATUS_ERROR;
}

/**
 * @brief 在同一次 NSS 事务中读取 SX127x 寄存器或 FIFO
 * @param radio Semtech SX127x 驱动状态
 * @param address 寄存器地址，地址 0 表示 FIFO
 * @param data 读取数据的目标缓冲区
 * @param data_len 读取数据的字节数
 * @return SPI 事务成功时返回 SX127X_HAL_STATUS_OK
 */
extern "C" sx127x_hal_status_t sx127x_hal_read(const sx127x_t* radio,
    const uint16_t address, uint8_t* data, const uint16_t data_len) {
  auto* context = GetContext(radio);
  return ((context != nullptr) && context->Read(address, data, data_len))
             ? SX127X_HAL_STATUS_OK
             : SX127X_HAL_STATUS_ERROR;
}

/**
 * @brief 执行 SX127x 硬件复位时序并保存实际执行结果
 * @param radio Semtech SX127x 驱动状态
 */
extern "C" void sx127x_hal_reset(const sx127x_t* radio) {
  auto* context = GetContext(radio);
  if ((context != nullptr) && !context->Reset() && (context->bus != nullptr)) {
    context->bus->LogMessage(cpp_bus_driver::Tool::LogLevel::kError, __FILE__,
        __LINE__, "SX127x hardware reset failed\n");
  }
}

/**
 * @brief 返回 DIO1 边沿触发瞬间锁存的电平
 * @param radio Semtech SX127x 驱动状态
 * @return DIO1 为高电平时返回 1，否则返回 0
 */
extern "C" uint32_t sx127x_hal_get_dio_1_pin_state(const sx127x_t* radio) {
  const auto* context = GetContext(radio);
  return context == nullptr ? 0 : context->GetDio1State();
}

/**
 * @brief 启动 SX127x 驱动使用的一次性接收超时计时器
 * @param radio Semtech SX127x 驱动状态
 * @param time_in_ms 超时时间，单位为 ms
 * @param callback 到期后由普通工作任务调用的 Semtech 回调
 * @return 计时器成功启动时返回 SX127X_HAL_STATUS_OK
 */
extern "C" sx127x_hal_status_t sx127x_hal_timer_start(const sx127x_t* radio,
    const uint32_t time_in_ms, void (*callback)(void* context)) {
  auto* context = GetContext(radio);
  return ((context != nullptr) && context->StartTimer(time_in_ms, callback))
             ? SX127X_HAL_STATUS_OK
             : SX127X_HAL_STATUS_ERROR;
}

/**
 * @brief 幂等停止 SX127x 驱动使用的接收超时计时器
 * @param radio Semtech SX127x 驱动状态
 * @return 计时器停止或原本未运行时返回 SX127X_HAL_STATUS_OK
 */
extern "C" sx127x_hal_status_t sx127x_hal_timer_stop(const sx127x_t* radio) {
  auto* context = GetContext(radio);
  return ((context != nullptr) && context->StopTimer())
             ? SX127X_HAL_STATUS_OK
             : SX127X_HAL_STATUS_ERROR;
}

/**
 * @brief 查询 SX127x 接收超时计时器是否已经启动
 * @param radio Semtech SX127x 驱动状态
 * @return 计时器已启动且尚未处理到期事件时返回 true
 */
extern "C" bool sx127x_hal_timer_is_started(const sx127x_t* radio) {
  const auto* context = GetContext(radio);
  return (context != nullptr) && context->IsTimerStarted();
}
