/*
 * @Description: 声明 USP SX126x 驱动的 C++ 桥接接口
 * @Author: LILYGO_L
 * @Date: 2026-07-12 00:00:00
 * @LastEditTime: 2026-07-15 01:12:34
 * @License: GPL 3.0
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>

#include "bus/bus_guide.h"
#include "common/direct_driver.h"
#include "smtc_rac_lib/radio_drivers/sx126x_driver/src/lr_fhss_mac.h"
#include "smtc_rac_lib/radio_drivers/sx126x_driver/src/sx126x.h"
#include "smtc_rac_lib/radio_drivers/sx126x_driver/src/sx126x_bpsk.h"
#include "smtc_rac_lib/radio_drivers/sx126x_driver/src/sx126x_driver_version.h"
#include "smtc_rac_lib/radio_drivers/sx126x_driver/src/sx126x_lr_fhss.h"

namespace usp_cpp_bus_driver {

struct Sx126xContext;

/**
 * @brief 管理 SX126x SPI 资源并提供常用 LoRa 数据包操作
 *
 * GFSK、BPSK、LR-FHSS 和其他完整官方功能可以通过 Invoke() 调用。
 */
class Sx126x final : public DirectDriver<Sx126x> {
 public:
  // false 表示断言硬件复位，true 表示释放硬件复位。
  using ResetCallback = std::function<bool(bool)>;

  /**
   * @brief SX126x 开发板电源、时钟、RF Switch 和修正项配置
   */
  struct HardwareConfig {
    sx126x_standby_cfg_t standby_mode =
        SX126X_STANDBY_CFG_RC;  // 初始化完成后的待机时钟源。
    sx126x_fallback_modes_t fallback_mode =
        SX126X_FALLBACK_STDBY_RC;  // 收发完成后的回退模式。
    sx126x_reg_mod_t regulator_mode = SX126X_REG_MODE_DCDC;  // 芯片稳压器模式。
    bool dio2_controls_rf_switch = true;  // 是否由 DIO2 控制 RF Switch。
    bool enable_tcxo = true;              // 是否由 DIO3 控制 TCXO。
    sx126x_tcxo_ctrl_voltages_t tcxo_voltage =
        SX126X_TCXO_CTRL_1_6V;                   // DIO3 TCXO 输出电压。
    uint32_t tcxo_startup_time_rtc_steps = 320;  // TCXO 启动时间。
    bool enable_tx_clamp_workaround = true;      // 是否应用 TX Clamp 修正。
    bool initialize_retention_list = true;       // 是否初始化保持寄存器列表。
  };

  /**
   * @brief SX126x LoRa 调制、数据包、射频和功放配置
   */
  struct LoraConfig {
    uint32_t frequency_hz = 920000000;  // 射频载波频率，单位为 Hz。
    sx126x_lora_sf_t spreading_factor = SX126X_LORA_SF9;  // LoRa 扩频因子。
    sx126x_lora_bw_t bandwidth = SX126X_LORA_BW_125;      // LoRa 带宽。
    sx126x_lora_cr_t coding_rate = SX126X_LORA_CR_4_7;    // LoRa 编码率。
    uint16_t preamble_length = 8;                         // LoRa 前导码符号数。
    uint8_t sync_word = 0x12;                             // LoRa 同步字。
    uint8_t max_payload_length = 255;  // 接收允许的最大负载长度。
    int8_t output_power_dbm = 22;      // 发射功率，单位为 dBm。
    sx126x_pa_cfg_params_t pa_config = {
        .pa_duty_cycle = 0x04,
        .hp_max = 0x07,
        .device_sel = 0x00,
        .pa_lut = 0x01,
    };  // 功率放大器参数，默认值适用于 SX1262 高功率通路。
    sx126x_ramp_time_t ramp_time = SX126X_RAMP_40_US;  // 功率放大器上升时间。
    uint8_t over_current_protection =
        SX126X_OCP_PARAM_VALUE_140_MA;         // 过流保护寄存器值。
    uint16_t image_calibration_min_mhz = 902;  // 镜像校准下限。
    uint16_t image_calibration_max_mhz = 928;  // 镜像校准上限。
    bool crc_enabled = true;                   // 是否启用数据包 CRC。
    bool invert_iq = false;                    // 是否反转 LoRa IQ 极性。
    bool rx_boosted = true;                    // 是否使用增强接收增益。
  };

  /**
   * @brief 最近一次接收的 LoRa 数据包信号质量
   */
  struct PacketMetrics {
    int8_t rssi_dbm = 0;         // 数据包 RSSI，单位为 dBm。
    int8_t snr_db = 0;           // 数据包信噪比，单位为 dB。
    int8_t signal_rssi_dbm = 0;  // LoRa 信号 RSSI，单位为 dBm。
  };

  /**
   * @brief 使用默认开发板配置创建 SX126x 资源所有者
   * @param bus SPI 总线对象
   * @param busy_pin 芯片 BUSY 信号连接的 GPIO
   * @param cs_pin SPI 片选使用的 GPIO
   * @param reset_callback 控制开发板硬件复位信号的回调
   */
  explicit Sx126x(std::shared_ptr<cpp_bus_driver::BusSpiGuide> bus,
      int32_t busy_pin, int32_t cs_pin, ResetCallback reset_callback);

  /**
   * @brief 使用显式开发板配置创建 SX126x 资源所有者
   * @param bus SPI 总线对象
   * @param busy_pin 芯片 BUSY 信号连接的 GPIO
   * @param cs_pin SPI 片选使用的 GPIO
   * @param reset_callback 控制开发板硬件复位信号的回调
   * @param hardware_config 开发板电源、时钟和 RF Switch 配置
   */
  explicit Sx126x(std::shared_ptr<cpp_bus_driver::BusSpiGuide> bus,
      int32_t busy_pin, int32_t cs_pin, ResetCallback reset_callback,
      HardwareConfig hardware_config);

  /**
   * @brief 释放当前对象仍持有的 SPI 设备资源
   */
  ~Sx126x();

  // SX126x 对象独占传输上下文，禁止复制和移动。
  Sx126x(const Sx126x&) = delete;
  Sx126x& operator=(const Sx126x&) = delete;
  Sx126x(Sx126x&&) = delete;
  Sx126x& operator=(Sx126x&&) = delete;

  /**
   * @brief 初始化 SPI 并应用 SX126x 开发板配置
   * @param frequency_hz SPI 时钟频率，单位为 Hz
   * @return 传输、复位和开发板配置全部成功时返回 true
   */
  bool Init(int32_t frequency_hz = 10000000);

  /**
   * @brief 释放 SX126x 当前使用的 SPI 设备
   * @param delete_bus 是否同时请求释放底层共享 SPI 总线
   * @return 请求的资源全部释放成功时返回 true
   */
  bool Deinit(bool delete_bus = false);

  /**
   * @brief 查询 SX126x 传输上下文是否可以使用
   * @return Init() 成功后至 Deinit() 完成前返回 true
   */
  bool initialized() const;

  /**
   * @brief 应用默认 LoRa 配置
   * @return 所有官方配置命令成功时返回 true
   */
  bool ConfigureLora();

  /**
   * @brief 应用指定 LoRa、射频和功放配置
   * @param config 完整 LoRa 配置
   * @return 参数有效且所有官方配置命令成功时返回 true
   */
  bool ConfigureLora(const LoraConfig& config);

  /**
   * @brief 启动 SX126x LoRa 连续接收
   * @return LoRa 已配置且连续接收启动成功时返回 true
   */
  bool StartReceive();

  /**
   * @brief 写入 LoRa 数据包并启动发送
   * @param data 待发送负载
   * @param size 负载长度，有效范围为 1 至 255 字节
   * @param timeout_ms 发送超时时间，单位为 ms，零表示不使用超时
   * @return 数据写入和发送命令均成功时返回 true
   */
  bool StartTransmit(
      const uint8_t* data, size_t size, uint32_t timeout_ms = 1000);

  /**
   * @brief 从 SX126x 接收缓冲区读取最近的数据包
   * @param data 接收负载的目标缓冲区
   * @param capacity 目标缓冲区容量
   * @param received_size 返回实际读取的负载长度
   * @param metrics 可选的 LoRa 信号质量输出
   * @return 完整非空数据包读取成功时返回 true
   */
  bool ReadPacket(uint8_t* data, size_t capacity, uint8_t& received_size,
      PacketMetrics* metrics = nullptr);

  /**
   * @brief 读取 SX126x 当前中断标志
   * @param irq_mask 返回当前中断位掩码
   * @return 芯片已初始化且中断状态读取成功时返回 true
   */
  bool GetIrqStatus(sx126x_irq_mask_t& irq_mask) const;

  /**
   * @brief 清除指定 SX126x 中断标志
   * @param irq_mask 需要清除的中断位掩码
   * @return 芯片已初始化且中断状态清除成功时返回 true
   */
  bool ClearIrqStatus(sx126x_irq_mask_t irq_mask) const;

  /**
   * @brief 读取并解析 SX126x 芯片状态
   * @param status 返回芯片模式和命令状态
   * @return 芯片已初始化且状态读取成功时返回 true
   */
  bool GetChipStatus(sx126x_chip_status_t& status) const;

  /**
   * @brief 返回 USP SX126x 官方函数使用的不透明上下文
   * @return 已初始化时返回有效上下文，否则返回 nullptr
   */
  const void* context();

 private:
  /**
   * @brief 应用下一次收发使用的 LoRa 数据包参数
   * @param config 当前 LoRa 数据包配置
   * @param payload_length 数据包负载长度
   * @return 官方配置命令成功时返回 true
   */
  bool ApplyPacketParams(
      const LoraConfig& config, uint8_t payload_length) const;

  /**
   * @brief 将 USP 状态码转换为布尔结果并记录失败信息
   * @param status USP 官方函数返回状态
   * @param operation 日志中使用的操作名称
   * @return status 为 SX126X_STATUS_OK 时返回 true
   */
  bool CheckStatus(sx126x_status_t status, const char* operation) const;

  // 持有 SPI、BUSY、复位和休眠状态。
  std::unique_ptr<Sx126xContext> context_;
  HardwareConfig hardware_config_;  // 当前开发板硬件配置。
  LoraConfig lora_config_;          // 最近一次成功应用的 LoRa 配置。
  bool lora_configured_ = false;    // ConfigureLora() 全部成功后为 true。
};

}  // namespace usp_cpp_bus_driver
