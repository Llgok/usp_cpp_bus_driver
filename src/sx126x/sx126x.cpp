/*
 * @Description: 实现 USP SX126x 驱动的 C++ 桥接
 * @Author: LILYGO_L
 * @Date: 2026-07-10 00:00:00
 * @LastEditTime: 2026-08-03 16:15:08
 * @License: GPL 3.0
 */
#include <utility>

#include "sx126x/sx126x_context.h"
#include "sx126x/sx126x_driver.h"

namespace usp_cpp_bus_driver {
namespace {
constexpr uint16_t kReceiveIrqMask = SX126X_IRQ_RX_DONE | SX126X_IRQ_TIMEOUT |
                                     SX126X_IRQ_CRC_ERROR |
                                     SX126X_IRQ_HEADER_ERROR;
constexpr uint16_t kTransmitIrqMask = SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT;

bool ShouldEnableLdro(const Sx126x::LoraConfig& config) {
  const uint32_t bandwidth_hz = sx126x_get_lora_bw_in_hz(config.bandwidth);
  const auto spreading_factor = static_cast<uint32_t>(config.spreading_factor);
  if (bandwidth_hz == 0 || spreading_factor < SX126X_LORA_SF5 ||
      spreading_factor > SX126X_LORA_SF12) {
    return false;
  }
  const uint64_t symbol_time_numerator = uint64_t{1} << spreading_factor;
  return symbol_time_numerator * 1000U >=
         static_cast<uint64_t>(bandwidth_hz) * 16U;
}
}  // namespace

Sx126x::Sx126x(std::shared_ptr<cpp_bus_driver::BusSpiGuide> bus,
    int32_t busy_pin, int32_t cs_pin, ResetCallback reset_callback)
    : Sx126x(std::move(bus), busy_pin, cs_pin, std::move(reset_callback),
          HardwareConfig{}) {}

Sx126x::Sx126x(std::shared_ptr<cpp_bus_driver::BusSpiGuide> bus,
    int32_t busy_pin, int32_t cs_pin, ResetCallback reset_callback,
    HardwareConfig hardware_config)
    : context_(std::make_unique<Sx126xContext>()) {
  context_->bus = std::move(bus);
  context_->busy_pin = busy_pin;
  context_->cs_pin = cs_pin;
  context_->reset_callback = std::move(reset_callback);
  hardware_config_ = hardware_config;
}

Sx126x::~Sx126x() {
  if (initialized()) {
    Deinit(false);
  }
}

bool Sx126x::Init(int32_t frequency_hz) {
  if (initialized()) {
    return true;
  }
  if ((context_->bus == nullptr) || !context_->reset_callback) {
    return false;
  }

  context_->frequency_hz = frequency_hz;
  if (!context_->bus->SetGpioMode(context_->busy_pin,
          cpp_bus_driver::Tool::GpioMode::kInput,
          cpp_bus_driver::Tool::GpioStatus::kDisable)) {
    return false;
  }
  if (!context_->bus->Init(frequency_hz, context_->cs_pin)) {
    return false;
  }

  context_->initialized = true;
  const auto fail_initialization = [this]() {
    context_->initialized = false;
    context_->bus->Deinit(false);
    return false;
  };
  if (!CheckStatus(sx126x_reset(context_.get()), "sx126x_reset")) {
    return fail_initialization();
  }
  if (!CheckStatus(sx126x_set_standby(context_.get(), SX126X_STANDBY_CFG_RC),
          "sx126x_set_standby")) {
    return fail_initialization();
  }
  if (!CheckStatus(
          sx126x_set_reg_mode(context_.get(), hardware_config_.regulator_mode),
          "sx126x_set_reg_mode")) {
    return fail_initialization();
  }
  if (!CheckStatus(sx126x_set_dio2_as_rf_sw_ctrl(context_.get(),
                       hardware_config_.dio2_controls_rf_switch),
          "sx126x_set_dio2_as_rf_sw_ctrl")) {
    return fail_initialization();
  }
  if (hardware_config_.enable_tcxo) {
    if (!CheckStatus(sx126x_set_dio3_as_tcxo_ctrl(context_.get(),
                         hardware_config_.tcxo_voltage,
                         hardware_config_.tcxo_startup_time_rtc_steps),
            "sx126x_set_dio3_as_tcxo_ctrl")) {
      return fail_initialization();
    }
  }
  if (hardware_config_.enable_tx_clamp_workaround) {
    if (!CheckStatus(
            sx126x_cfg_tx_clamp(context_.get()), "sx126x_cfg_tx_clamp")) {
      return fail_initialization();
    }
  }
  if (hardware_config_.initialize_retention_list) {
    if (!CheckStatus(sx126x_init_retention_list(context_.get()),
            "sx126x_init_retention_list")) {
      return fail_initialization();
    }
  }
  if (hardware_config_.standby_mode != SX126X_STANDBY_CFG_RC &&
      !CheckStatus(
          sx126x_set_standby(context_.get(), hardware_config_.standby_mode),
          "sx126x_set_standby")) {
    return fail_initialization();
  }
  return true;
}

bool Sx126x::Deinit(bool delete_bus) {
  if (!initialized()) {
    return true;
  }
  if (context_->bus == nullptr) {
    return false;
  }
  const bool result = context_->bus->Deinit(delete_bus);
  lora_configured_ = false;
  context_->initialized = false;
  context_->sleeping = false;
  return result;
}

bool Sx126x::SetSleep() {
  if (!initialized()) {
    return false;
  }
  if (context_->sleeping) {
    return context_->bus != nullptr && context_->bus->Deinit(false);
  }
  if (!CheckStatus(sx126x_set_standby(context_.get(), SX126X_STANDBY_CFG_RC),
          "sx126x_set_standby") ||
      !CheckStatus(
          sx126x_set_sleep(context_.get(), SX126X_SLEEP_CFG_WARM_START),
          "sx126x_set_sleep")) {
    return false;
  }
  return context_->bus != nullptr && context_->bus->Deinit(false);
}

bool Sx126x::Wakeup() {
  return initialized() &&
         CheckStatus(sx126x_wakeup(context_.get()), "sx126x_wakeup");
}

bool Sx126x::Configure() { return Configure(LoraConfig{}); }

bool Sx126x::Configure(const LoraConfig& config) {
  lora_configured_ = false;
  const uint32_t bandwidth_hz = sx126x_get_lora_bw_in_hz(config.bandwidth);
  const auto spreading_factor = static_cast<uint32_t>(config.spreading_factor);
  const auto coding_rate = static_cast<uint32_t>(config.coding_rate);
  if (!initialized() || config.frequency_hz == 0 ||
      config.max_payload_length == 0 || bandwidth_hz == 0 ||
      spreading_factor < SX126X_LORA_SF5 ||
      spreading_factor > SX126X_LORA_SF12 || coding_rate < SX126X_LORA_CR_4_5 ||
      coding_rate > SX126X_LORA_CR_4_8 ||
      config.image_calibration_min_mhz >= config.image_calibration_max_mhz) {
    return false;
  }

  const sx126x_mod_params_lora_t modulation_params = {
      .sf = config.spreading_factor,
      .bw = config.bandwidth,
      .cr = config.coding_rate,
      .ldro = static_cast<uint8_t>(ShouldEnableLdro(config)),
  };
  if (!CheckStatus(sx126x_set_standby(context_.get(), SX126X_STANDBY_CFG_RC),
          "sx126x_set_standby") ||
      !CheckStatus(sx126x_cal(context_.get(), SX126X_CAL_ALL), "sx126x_cal") ||
      !CheckStatus(sx126x_set_pkt_type(context_.get(), SX126X_PKT_TYPE_LORA),
          "sx126x_set_pkt_type") ||
      !CheckStatus(sx126x_set_buffer_base_address(context_.get(), 0, 0),
          "sx126x_set_buffer_base_address") ||
      !CheckStatus(
          sx126x_set_lora_mod_params(context_.get(), &modulation_params),
          "sx126x_set_lora_mod_params") ||
      !ApplyPacketParams(config, config.max_payload_length) ||
      !CheckStatus(sx126x_set_lora_sync_word(context_.get(), config.sync_word),
          "sx126x_set_lora_sync_word") ||
      !CheckStatus(sx126x_cal_img_in_mhz(context_.get(),
                       config.image_calibration_min_mhz,
                       config.image_calibration_max_mhz),
          "sx126x_cal_img_in_mhz") ||
      !CheckStatus(sx126x_set_rf_freq(context_.get(), config.frequency_hz),
          "sx126x_set_rf_freq") ||
      !CheckStatus(sx126x_set_pa_cfg(context_.get(), &config.pa_config),
          "sx126x_set_pa_cfg") ||
      !CheckStatus(sx126x_set_tx_params(context_.get(), config.output_power_dbm,
                       config.ramp_time),
          "sx126x_set_tx_params") ||
      !CheckStatus(
          sx126x_set_ocp_value(context_.get(), config.over_current_protection),
          "sx126x_set_ocp_value") ||
      !CheckStatus(sx126x_set_rx_tx_fallback_mode(
                       context_.get(), hardware_config_.fallback_mode),
          "sx126x_set_rx_tx_fallback_mode") ||
      !CheckStatus(sx126x_reset_stats(context_.get()), "sx126x_reset_stats") ||
      !ClearIrqStatus(SX126X_IRQ_ALL)) {
    return false;
  }

  lora_config_ = config;
  lora_configured_ = true;
  return true;
}

bool Sx126x::StartReceive() {
  if (!lora_configured_) {
    return false;
  }

  return CheckStatus(
             sx126x_set_standby(context_.get(), hardware_config_.standby_mode),
             "sx126x_set_standby") &&
         ApplyPacketParams(lora_config_, lora_config_.max_payload_length) &&
         CheckStatus(
             sx126x_cfg_rx_boosted(context_.get(), lora_config_.rx_boosted),
             "sx126x_cfg_rx_boosted") &&
         CheckStatus(sx126x_set_dio_irq_params(context_.get(), kReceiveIrqMask,
                         kReceiveIrqMask, 0, 0),
             "sx126x_set_dio_irq_params") &&
         ClearIrqStatus(SX126X_IRQ_ALL) &&
         CheckStatus(sx126x_set_rx_with_timeout_in_rtc_step(
                         context_.get(), SX126X_RX_CONTINUOUS),
             "sx126x_set_rx_with_timeout_in_rtc_step");
}

bool Sx126x::StartTransmit(
    const uint8_t* data, size_t size, uint32_t timeout_ms) {
  if (!lora_configured_ || (data == nullptr) || (size == 0) ||
      (size > UINT8_MAX)) {
    return false;
  }

  const auto payload_size = static_cast<uint8_t>(size);
  return CheckStatus(
             sx126x_set_standby(context_.get(), hardware_config_.standby_mode),
             "sx126x_set_standby") &&
         ApplyPacketParams(lora_config_, payload_size) &&
         CheckStatus(sx126x_write_buffer(context_.get(), 0, data, payload_size),
             "sx126x_write_buffer") &&
         CheckStatus(sx126x_set_dio_irq_params(context_.get(), kTransmitIrqMask,
                         kTransmitIrqMask, 0, 0),
             "sx126x_set_dio_irq_params") &&
         ClearIrqStatus(SX126X_IRQ_ALL) &&
         CheckStatus(
             sx126x_set_tx(context_.get(), timeout_ms), "sx126x_set_tx");
}

bool Sx126x::ReadPacket(uint8_t* data, size_t capacity, uint8_t& received_size,
    PacketMetrics* metrics) {
  received_size = 0;
  if (!lora_configured_ || (data == nullptr) || (capacity == 0)) {
    return false;
  }

  sx126x_rx_buffer_status_t buffer_status = {};
  if (!CheckStatus(sx126x_get_rx_buffer_status(context_.get(), &buffer_status),
          "sx126x_get_rx_buffer_status")) {
    return false;
  }
  if ((buffer_status.pld_len_in_bytes == 0) ||
      (buffer_status.pld_len_in_bytes > capacity)) {
    return false;
  }
  if (!CheckStatus(
          sx126x_read_buffer(context_.get(), buffer_status.buffer_start_pointer,
              data, buffer_status.pld_len_in_bytes),
          "sx126x_read_buffer")) {
    return false;
  }

  if (metrics != nullptr) {
    sx126x_pkt_status_lora_t packet_status = {};
    if (!CheckStatus(sx126x_get_lora_pkt_status(context_.get(), &packet_status),
            "sx126x_get_lora_pkt_status")) {
      return false;
    }
    metrics->rssi_dbm = packet_status.rssi_pkt_in_dbm;
    metrics->snr_db = packet_status.snr_pkt_in_db;
    metrics->signal_rssi_dbm = packet_status.signal_rssi_pkt_in_dbm;
  }

  received_size = buffer_status.pld_len_in_bytes;
  return true;
}

bool Sx126x::GetIrqStatus(sx126x_irq_mask_t& irq_mask) const {
  return initialized() &&
         CheckStatus(sx126x_get_irq_status(context_.get(), &irq_mask),
             "sx126x_get_irq_status");
}

bool Sx126x::ClearIrqStatus(sx126x_irq_mask_t irq_mask) const {
  return initialized() &&
         CheckStatus(sx126x_clear_irq_status(context_.get(), irq_mask),
             "sx126x_clear_irq_status");
}

bool Sx126x::GetChipStatus(sx126x_chip_status_t& status) const {
  return initialized() &&
         CheckStatus(
             sx126x_get_status(context_.get(), &status), "sx126x_get_status");
}

bool Sx126x::ApplyPacketParams(
    const LoraConfig& config, uint8_t payload_length) const {
  const sx126x_pkt_params_lora_t packet_params = {
      .preamble_len_in_symb = config.preamble_length,
      .header_type = SX126X_LORA_PKT_EXPLICIT,
      .pld_len_in_bytes = payload_length,
      .crc_is_on = config.crc_enabled,
      .invert_iq_is_on = config.invert_iq,
  };
  return CheckStatus(sx126x_set_lora_pkt_params(context_.get(), &packet_params),
      "sx126x_set_lora_pkt_params");
}

bool Sx126x::CheckStatus(sx126x_status_t status, const char* operation) const {
  if (status == SX126X_STATUS_OK) {
    return true;
  }
  if (context_->bus != nullptr) {
    context_->bus->LogMessage(cpp_bus_driver::Tool::LogLevel::kError, __FILE__,
        __LINE__, "%s failed (error code: %d)\n", operation,
        static_cast<int>(status));
  }
  return false;
}
}  // namespace usp_cpp_bus_driver
