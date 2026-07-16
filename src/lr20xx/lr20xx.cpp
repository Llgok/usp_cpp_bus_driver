/*
 * @Description: 实现 USP LR20xx 驱动的 C++ 桥接
 * @Author: LILYGO_L
 * @Date: 2026-07-10 00:00:00
 * @LastEditTime: 2026-07-16 10:53:04
 * @License: GPL 3.0
 */
#include "lr20xx/lr20xx_driver.h"

#include <utility>

#include "common/lr_context.h"
#include "smtc_rac_lib/radio_drivers/lr20xx_driver/inc/lr20xx_hal.h"

namespace usp_cpp_bus_driver {
namespace {
constexpr size_t kFifoCapacity = 256;
}  // namespace

Lr20xx::Lr20xx(std::shared_ptr<cpp_bus_driver::BusSpiGuide> bus,
    int32_t busy_pin, int32_t cs_pin, ResetCallback reset_callback,
    WakeupCallback wakeup_callback)
    : context_(std::make_unique<LrContext>()) {
  context_->bus = std::move(bus);
  context_->busy_pin = busy_pin;
  context_->cs_pin = cs_pin;
  context_->reset_callback = std::move(reset_callback);
  context_->wakeup_callback = std::move(wakeup_callback);
}

Lr20xx::~Lr20xx() {
  if (initialized()) {
    context_->Deinit(false);
  }
}

bool Lr20xx::Init(int32_t frequency_hz) {
  if (initialized()) {
    return true;
  }
  if (!context_->Init(frequency_hz)) {
    return false;
  }
  if (Reset()) {
    return true;
  }
  context_->Deinit(false);
  return false;
}

bool Lr20xx::Deinit(bool delete_bus) {
  if (!initialized()) {
    return true;
  }
  if (!context_->Deinit(delete_bus)) {
    return false;
  }
  lora_configured_ = false;
  return true;
}

bool Lr20xx::Reset() {
  if (!initialized() ||
      lr20xx_hal_reset(context_.get()) != LR20XX_HAL_STATUS_OK) {
    return false;
  }
  lora_configured_ = false;
  return true;
}

bool Lr20xx::Wakeup() {
  return initialized() &&
         lr20xx_hal_wakeup(context_.get()) == LR20XX_HAL_STATUS_OK;
}

bool Lr20xx::Configure(const LoraConfig& config) {
  lora_configured_ = false;
  if (!initialized() || config.frequency_hz == 0) {
    return false;
  }
  lora_configured_ =
      lr20xx_radio_common_set_pkt_type(
          context(), LR20XX_RADIO_COMMON_PKT_TYPE_LORA) == LR20XX_STATUS_OK &&
      lr20xx_radio_common_set_rf_freq(context(), config.frequency_hz) ==
          LR20XX_STATUS_OK &&
      lr20xx_radio_lora_set_modulation_params(context(), &config.modulation) ==
          LR20XX_STATUS_OK &&
      lr20xx_radio_lora_set_packet_params(context(), &config.packet) ==
          LR20XX_STATUS_OK &&
      lr20xx_radio_lora_set_syncword(context(), config.sync_word) ==
          LR20XX_STATUS_OK &&
      lr20xx_radio_common_set_pa_cfg(context(), &config.pa) ==
          LR20XX_STATUS_OK &&
      lr20xx_radio_common_select_pa(context(), config.pa.pa_sel) ==
          LR20XX_STATUS_OK &&
      lr20xx_radio_common_set_tx_params(context(), config.output_power_half_dbm,
          config.ramp_time) == LR20XX_STATUS_OK;
  return lora_configured_;
}

bool Lr20xx::WriteBuffer(const uint8_t* data, size_t size) {
  if (!initialized() || data == nullptr || size == 0 || size > kFifoCapacity) {
    return false;
  }
  return lr20xx_radio_fifo_write_tx(
             context(), data, static_cast<uint16_t>(size)) == LR20XX_STATUS_OK;
}

bool Lr20xx::ReadBuffer(uint8_t* data, size_t size) {
  if (!initialized() || data == nullptr || size == 0 || size > kFifoCapacity) {
    return false;
  }
  return lr20xx_radio_fifo_read_rx(
             context(), data, static_cast<uint16_t>(size)) == LR20XX_STATUS_OK;
}

bool Lr20xx::StartReceive(uint32_t timeout_ms) {
  return lora_configured_ &&
         lr20xx_radio_common_set_rx(context(), timeout_ms) == LR20XX_STATUS_OK;
}

bool Lr20xx::StartTransmit(uint32_t timeout_ms) {
  return lora_configured_ &&
         lr20xx_radio_common_set_tx(context(), timeout_ms) == LR20XX_STATUS_OK;
}

}  // namespace usp_cpp_bus_driver
