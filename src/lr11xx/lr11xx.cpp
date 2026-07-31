/*
 * @Description: 实现 USP LR11xx 驱动的 C++ 桥接
 * @Author: LILYGO_L
 * @Date: 2026-07-10 00:00:00
 * @LastEditTime: 2026-07-31 15:35:00
 * @License: GPL 3.0
 */
#include "lr11xx/lr11xx_driver.h"

#include <limits>
#include <utility>

#include "common/lr_context.h"
#include "smtc_rac_lib/radio_drivers/lr11xx_driver/src/lr11xx_hal.h"

namespace usp_cpp_bus_driver {
Lr11xx::Lr11xx(std::shared_ptr<cpp_bus_driver::BusSpiGuide> bus,
    int32_t busy_pin, int32_t cs_pin, ResetCallback reset_callback)
    : context_(std::make_unique<LrContext>()) {
  context_->bus = std::move(bus);
  context_->busy_pin = busy_pin;
  context_->cs_pin = cs_pin;
  context_->reset_callback = std::move(reset_callback);
}

Lr11xx::~Lr11xx() {
  if (initialized()) {
    context_->Deinit(false);
  }
}

bool Lr11xx::Init(int32_t frequency_hz) {
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

bool Lr11xx::Deinit(bool delete_bus) {
  if (!initialized()) {
    return true;
  }
  if (!context_->Deinit(delete_bus)) {
    return false;
  }
  lora_configured_ = false;
  return true;
}

bool Lr11xx::Reset() {
  if (!initialized() ||
      lr11xx_hal_reset(context_.get()) != LR11XX_HAL_STATUS_OK) {
    return false;
  }
  lora_configured_ = false;
  return true;
}

bool Lr11xx::Wakeup() {
  return initialized() &&
         lr11xx_hal_wakeup(context_.get()) == LR11XX_HAL_STATUS_OK;
}

bool Lr11xx::Configure(const LoraConfig& config) {
  lora_configured_ = false;
  if (!initialized() || config.frequency_hz == 0) {
    return false;
  }
  lora_configured_ =
      lr11xx_radio_set_pkt_type(context(), LR11XX_RADIO_PKT_TYPE_LORA) ==
          LR11XX_STATUS_OK &&
      lr11xx_radio_set_rf_freq(context(), config.frequency_hz) ==
          LR11XX_STATUS_OK &&
      lr11xx_radio_set_lora_mod_params(context(), &config.modulation) ==
          LR11XX_STATUS_OK &&
      lr11xx_radio_set_lora_pkt_params(context(), &config.packet) ==
          LR11XX_STATUS_OK &&
      lr11xx_radio_set_lora_sync_word(context(), config.sync_word) ==
          LR11XX_STATUS_OK &&
      lr11xx_radio_cfg_rx_boosted(context(), config.rx_boosted) ==
          LR11XX_STATUS_OK &&
      lr11xx_radio_set_pa_cfg(context(), &config.pa) == LR11XX_STATUS_OK &&
      lr11xx_radio_set_tx_params(context(), config.output_power_dbm,
          config.ramp_time) == LR11XX_STATUS_OK;
  return lora_configured_;
}

bool Lr11xx::WriteBuffer(const uint8_t* data, size_t size) {
  if (!initialized() || data == nullptr || size == 0 ||
      size > std::numeric_limits<uint8_t>::max()) {
    return false;
  }
  return lr11xx_regmem_write_buffer8(
             context(), data, static_cast<uint8_t>(size)) == LR11XX_STATUS_OK;
}

bool Lr11xx::ReadBuffer(uint8_t offset, uint8_t* data, size_t size) {
  if (!initialized() || data == nullptr || size == 0 ||
      size > std::numeric_limits<uint8_t>::max() || size > 256U - offset) {
    return false;
  }
  return lr11xx_regmem_read_buffer8(context(), data, offset,
             static_cast<uint8_t>(size)) == LR11XX_STATUS_OK;
}

bool Lr11xx::StartReceive(uint32_t timeout_ms) {
  return lora_configured_ &&
         lr11xx_radio_set_rx(context(), timeout_ms) == LR11XX_STATUS_OK;
}

bool Lr11xx::StartTransmit(uint32_t timeout_ms) {
  return lora_configured_ &&
         lr11xx_radio_set_tx(context(), timeout_ms) == LR11XX_STATUS_OK;
}

}  // namespace usp_cpp_bus_driver
