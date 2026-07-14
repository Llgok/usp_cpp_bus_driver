/*
 * @Description: None
 * @Author: LILYGO_L
 * @Date: 2026-07-11
 * @LastEditTime: 2026-07-12 12:32:29
 * @License: GPL 3.0
 */
#include "sx127x/sx127x_driver.h"

#include <limits>
#include <utility>

#include "sx127x_context.h"

namespace semtech_cpp_bus_driver {
namespace {
/**
 * @brief 检查运行时指定的 SX127x 型号是否属于官方驱动支持范围
 * @param radio_id 需要检查的具体无线芯片型号
 * @return 型号属于 SX1272/3/6/7/8/9 时返回 true
 */
bool IsSupportedRadioId(sx127x_radio_id_t radio_id) {
  switch (radio_id) {
    case SX127X_RADIO_ID_SX1272:
    case SX127X_RADIO_ID_SX1273:
    case SX127X_RADIO_ID_SX1276:
    case SX127X_RADIO_ID_SX1277:
    case SX127X_RADIO_ID_SX1278:
    case SX127X_RADIO_ID_SX1279:
      return true;
  }
  return false;
}
}  // namespace

/**
 * @brief 创建 SX127x 传输、中断和软件计时器桥接对象
 * @param bus SPI 总线对象
 * @param radio_id 实际安装的 SX1272/3/6/7/8/9 型号
 * @param dio0_pin DIO0 上升沿中断 GPIO
 * @param dio1_pin DIO1 双边沿中断 GPIO
 * @param dio2_pin GFSK/OOK 同步检测使用的 DIO2 上升沿 GPIO，不使用时传入 -1
 * @param cs_pin SPI 总线用作 NSS 的 GPIO
 * @param reset_callback 断言及释放开发板复位信号的回调
 */
Sx127x::Sx127x(std::shared_ptr<cpp_bus_driver::BusSpiGuide> bus,
    sx127x_radio_id_t radio_id, int32_t dio0_pin, int32_t dio1_pin,
    int32_t dio2_pin, int32_t cs_pin, ResetCallback reset_callback)
    : context_(std::make_unique<Sx127xContext>()) {
  context_->bus = std::move(bus);
  context_->radio_id = radio_id;
  context_->dio0_pin = dio0_pin;
  context_->dio1_pin = dio1_pin;
  context_->dio2_pin = dio2_pin;
  context_->cs_pin = cs_pin;
  context_->reset_callback = std::move(reset_callback);
  context_->radio = &radio_;
  radio_.hal_context = context_.get();
}

/**
 * @brief 停止异步资源并释放当前对象持有的 SPI 设备
 */
Sx127x::~Sx127x() {
  if (context_->initialized) {
    Deinit(false);
  }
}

/**
 * @brief 初始化 SPI、异步事件资源并复位及初始化官方驱动
 * @param frequency_hz SPI 时钟频率，单位为 Hz
 * @return 所有资源与 Semtech 驱动初始化成功时返回 true
 */
bool Sx127x::Init(int32_t frequency_hz) {
  if (driver_initialized_) {
    return true;
  }
  if (context_->initialized) {
    return false;
  }
  if (!IsSupportedRadioId(context_->radio_id)) {
    return false;
  }

  radio_ = {};
  radio_.hal_context = context_.get();
  context_->radio = &radio_;
  if (!context_->Init(frequency_hz)) {
    radio_.hal_context = nullptr;
    return false;
  }
  if (!context_->Reset()) {
    context_->Deinit(false);
    if (!context_->initialized) {
      radio_.hal_context = nullptr;
    }
    return false;
  }

  const sx127x_status_t status = sx127x_init(&radio_);
  if ((status != SX127X_STATUS_OK) || !context_->dio_attach_succeeded) {
    if (context_->bus != nullptr) {
      context_->bus->LogMessage(cpp_bus_driver::Tool::LogLevel::kError,
          __FILE__, __LINE__, "sx127x_init failed (error code: %d)\n",
          static_cast<int>(status));
    }
    context_->Deinit(false);
    if (!context_->initialized) {
      radio_.hal_context = nullptr;
    }
    return false;
  }
  context_->EnableEventHandling();
  driver_initialized_ = true;
  return true;
}

/**
 * @brief 移除 DIO 中断、停止工作任务并释放 SPI 设备
 * @param delete_bus 为 true 时同时请求释放底层共享 SPI 总线
 * @return 所有已创建资源均成功释放时返回 true
 */
bool Sx127x::Deinit(bool delete_bus) {
  driver_initialized_ = false;
  lora_configured_ = false;
  const bool result = context_->Deinit(delete_bus);
  if (!context_->initialized) {
    radio_.hal_context = nullptr;
  }
  return result;
}

bool Sx127x::ConfigureLora(const LoraConfig& config) {
  lora_configured_ = false;
  if (!initialized() || config.frequency_hz == 0) {
    return false;
  }
  lora_configured_ =
      sx127x_set_pkt_type(context(), SX127X_PKT_TYPE_LORA) ==
          SX127X_STATUS_OK &&
      sx127x_set_rf_freq(context(), config.frequency_hz) == SX127X_STATUS_OK &&
      sx127x_set_lora_mod_params(context(), &config.modulation) ==
          SX127X_STATUS_OK &&
      sx127x_set_lora_pkt_params(context(), &config.packet) ==
          SX127X_STATUS_OK &&
      sx127x_set_lora_sync_word(context(), config.sync_word) ==
          SX127X_STATUS_OK &&
      sx127x_set_pa_cfg(context(), &config.pa) == SX127X_STATUS_OK &&
      sx127x_set_tx_params(context(), config.output_power_dbm,
          config.ramp_time) == SX127X_STATUS_OK;
  return lora_configured_;
}

bool Sx127x::WriteBuffer(uint8_t offset, const uint8_t* data, size_t size) {
  if (!initialized() || data == nullptr || size == 0 ||
      size > std::numeric_limits<uint8_t>::max() ||
      size > SX127X_RX_TX_BUFFER_SIZE_MAX - offset) {
    return false;
  }
  return sx127x_write_buffer(context(), offset, data,
             static_cast<uint8_t>(size)) == SX127X_STATUS_OK;
}

bool Sx127x::ReadBuffer(uint8_t offset, uint8_t* data, size_t size) {
  if (!initialized() || data == nullptr || size == 0 ||
      size > std::numeric_limits<uint8_t>::max() ||
      size > SX127X_RX_TX_BUFFER_SIZE_MAX - offset) {
    return false;
  }
  return sx127x_read_buffer(context(), offset, data,
             static_cast<uint8_t>(size)) == SX127X_STATUS_OK;
}

bool Sx127x::StartReceive(uint32_t timeout_ms) {
  return lora_configured_ &&
         sx127x_set_rx(context(), timeout_ms) == SX127X_STATUS_OK;
}

bool Sx127x::StartTransmit() {
  return lora_configured_ && sx127x_set_tx(context()) == SX127X_STATUS_OK;
}

bool Sx127x::initialized() const { return driver_initialized_; }

/**
 * @brief 返回可传给 Semtech SX127x 官方 API 的驱动状态
 * @return Init 成功后至 Deinit 前返回可写 SX127x 状态，否则返回 nullptr
 */
sx127x_t* Sx127x::context() { return driver_initialized_ ? &radio_ : nullptr; }

/**
 * @brief 返回只读 Semtech SX127x 官方驱动状态
 * @return Init 成功后至 Deinit 前返回只读 SX127x 状态，否则返回 nullptr
 */
const sx127x_t* Sx127x::context() const {
  return driver_initialized_ ? &radio_ : nullptr;
}
}  // namespace semtech_cpp_bus_driver
