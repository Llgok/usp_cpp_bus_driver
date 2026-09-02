/*
 * @Description: 实现 USP LR20xx 驱动的 C++ 桥接
 * @Author: LILYGO_L
 * @Date: 2026-07-10 00:00:00
 * @LastEditTime: 2026-09-02 16:55:18
 * @License: GPL 3.0
 */
#include <utility>

#include "common/lr_context.h"
#include "lr20xx/lr20xx_driver.h"
#include "smtc_rac_lib/radio_drivers/lr20xx_driver/inc/lr20xx_hal.h"

namespace usp_cpp_bus_driver {
namespace {
constexpr size_t kFifoCapacity = 256;
constexpr uint32_t kWakeupPulseUs = 1000;
constexpr uint32_t kFrontEndCalibrationMaximumOffsetHz = 50000000U;
constexpr uint32_t kFrontEndCalibrationStepHz = 4000000U;
}  // namespace

Lr20xx::Lr20xx(std::shared_ptr<cpp_bus_driver::BusSpiGuide> bus,
    int32_t busy_pin, int32_t cs_pin, ResetCallback reset_callback)
    : context_(std::make_unique<LrContext>()) {
  context_->bus = std::move(bus);
  context_->busy_pin = busy_pin;
  context_->cs_pin = cs_pin;
  context_->reset_callback = std::move(reset_callback);
  context_->wakeup_pulse_us = kWakeupPulseUs;
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
  lora_configured_ = false;
  InvalidateFrontEndCalibration();
  if (!initialized()) {
    return true;
  }
  if (!context_->Deinit(delete_bus)) {
    return false;
  }
  return true;
}

bool Lr20xx::Reset() {
  lora_configured_ = false;
  InvalidateFrontEndCalibration();
  if (!initialized() ||
      lr20xx_hal_reset(context_.get()) != LR20XX_HAL_STATUS_OK) {
    return false;
  }
  return true;
}

bool Lr20xx::SetSleep(const lr20xx_system_sleep_cfg_t& config) {
  if (!initialized()) {
    return false;
  }
  if (context_->sleeping) {
    return context_->bus != nullptr && context_->bus->Deinit(false);
  }
  const bool result = lr20xx_system_set_sleep_mode(
                          context_.get(), &config, 0U) == LR20XX_STATUS_OK;
  if (result && !config.is_ram_retention_enabled) {
    InvalidateFrontEndCalibration();
  }
  return result;
}

bool Lr20xx::Wakeup() {
  return initialized() &&
         lr20xx_hal_wakeup(context_.get()) == LR20XX_HAL_STATUS_OK;
}

bool Lr20xx::Configure(const LoraConfig& config) {
  lora_configured_ = false;
  if (!initialized() || config.frequency_hz == 0 ||
      lr20xx_system_set_standby_mode(
          context(), LR20XX_SYSTEM_STANDBY_MODE_RC) != LR20XX_STATUS_OK ||
      !CalibrateFrontEndIfNeeded(config)) {
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
      lr20xx_radio_common_set_rx_path(context(), config.rx_path,
          config.rx_boost_mode) == LR20XX_STATUS_OK &&
      lr20xx_radio_common_set_pa_cfg(context(), &config.pa) ==
          LR20XX_STATUS_OK &&
      lr20xx_radio_common_select_pa(context(), config.pa.pa_sel) ==
          LR20XX_STATUS_OK &&
      lr20xx_radio_common_set_tx_params(context(), config.output_power_half_dbm,
          config.ramp_time) == LR20XX_STATUS_OK;
  return lora_configured_;
}

bool Lr20xx::CalibrateFrontEndIfNeeded(const LoraConfig& config) {
  const uint32_t frequency_difference_hz =
      config.frequency_hz >= front_end_calibration_frequency_hz_
          ? config.frequency_hz - front_end_calibration_frequency_hz_
          : front_end_calibration_frequency_hz_ - config.frequency_hz;
  const bool calibration_required =
      front_end_calibration_frequency_hz_ == 0 ||
      config.rx_path != front_end_calibration_rx_path_ ||
      frequency_difference_hz > kFrontEndCalibrationMaximumOffsetHz;
  if (!calibration_required) {
    return true;
  }

  const lr20xx_radio_common_front_end_calibration_value_t calibration = {
      .rx_path = config.rx_path,
      .frequency_in_hertz = config.frequency_hz,
  };
  if (lr20xx_radio_common_calibrate_front_end_helper(
          context(), &calibration, 1U) != LR20XX_STATUS_OK) {
    return false;
  }

  const uint64_t aligned_frequency_hz =
      ((static_cast<uint64_t>(config.frequency_hz) +
           kFrontEndCalibrationStepHz - 1U) /
          kFrontEndCalibrationStepHz) *
      kFrontEndCalibrationStepHz;
  front_end_calibration_frequency_hz_ =
      static_cast<uint32_t>(aligned_frequency_hz);
  front_end_calibration_rx_path_ = config.rx_path;
  return true;
}

void Lr20xx::InvalidateFrontEndCalibration() {
  front_end_calibration_frequency_hz_ = 0;
  front_end_calibration_rx_path_ = LR20XX_RADIO_COMMON_RX_PATH_LF;
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
