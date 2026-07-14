/*
 * @Description: 声明 USP LR20xx 驱动的 C++ 桥接接口
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
#include "lr20xx_driver_version.h"
#include "lr20xx_radio_bluetooth_le.h"
#include "lr20xx_radio_bpsk.h"
#include "lr20xx_radio_common.h"
#include "lr20xx_radio_fifo.h"
#include "lr20xx_radio_flrc.h"
#include "lr20xx_radio_fsk.h"
#include "lr20xx_radio_lora.h"
#include "lr20xx_radio_lr_fhss.h"
#include "lr20xx_radio_ook.h"
#include "lr20xx_radio_oqpsk_15_4.h"
#include "lr20xx_radio_wi_sun.h"
#include "lr20xx_radio_wm_bus.h"
#include "lr20xx_radio_z_wave.h"
#include "lr20xx_regmem.h"
#include "lr20xx_rttof.h"
#include "lr20xx_system.h"
#include "lr20xx_workarounds.h"

namespace usp_cpp_bus_driver {

struct LrContext;

/**
 * @brief 管理 LR20xx SPI 传输资源并封装常用 LoRa 操作
 *
 * FLRC、FSK、OOK、BLE、LR-FHSS、IEEE 802.15.4、Wi-SUN、Wireless
 * M-Bus、Z-Wave 和 RTToF 等完整官方功能通过 Invoke() 调用。
 */
class Lr20xx final : public DirectDriver<Lr20xx> {
 public:
  // false 表示断言硬件复位，true 表示释放硬件复位。
  using ResetCallback = std::function<bool(bool)>;
  // 产生 NSS 低电平不少于 100 us 的芯片唤醒脉冲。
  using WakeupCallback = std::function<bool()>;

  /**
   * @brief LR20xx LoRa 调制、数据包、射频和功放配置
   */
  struct LoraConfig {
    uint32_t frequency_hz = 0;  // 射频载波频率，单位为 Hz。
    lr20xx_radio_lora_mod_params_t modulation = {
        .sf = LR20XX_RADIO_LORA_SF9,
        .bw = LR20XX_RADIO_LORA_BW_125,
        .cr = LR20XX_RADIO_LORA_CR_4_7,
        .ppm = LR20XX_RADIO_LORA_NO_PPM,
    };  // LoRa 调制参数。
    lr20xx_radio_lora_pkt_params_t packet = {
        .preamble_len_in_symb = 8,
        .pkt_mode = LR20XX_RADIO_LORA_PKT_EXPLICIT,
        .pld_len_in_bytes = 255,
        .crc = LR20XX_RADIO_LORA_CRC_ENABLED,
        .iq = LR20XX_RADIO_LORA_IQ_STANDARD,
    };  // LoRa 数据包参数。
    uint8_t sync_word = 0x12;              // LoRa 同步字。
    lr20xx_radio_common_pa_cfg_t pa = {};  // 功率放大器配置。
    int8_t output_power_half_dbm = 0;      // 发射功率，单位为 0.5 dBm。
    lr20xx_radio_common_ramp_time_t ramp_time =
        LR20XX_RADIO_COMMON_RAMP_48_US;  // 功率放大器上升时间。
  };

  /**
   * @brief 创建 LR20xx 传输资源所有者
   * @param bus SPI 总线对象
   * @param busy_pin 芯片 BUSY 信号连接的 GPIO
   * @param cs_pin SPI 片选使用的 GPIO
   * @param reset_callback 控制开发板硬件复位信号的回调
   * @param wakeup_callback 产生 NSS 低电平不少于 100 us 的可选回调
   */
  explicit Lr20xx(std::shared_ptr<cpp_bus_driver::BusSpiGuide> bus,
      int32_t busy_pin, int32_t cs_pin, ResetCallback reset_callback,
      WakeupCallback wakeup_callback = {});

  /**
   * @brief 释放当前对象仍持有的 SPI 设备资源
   */
  ~Lr20xx();

  // LR20xx 对象独占传输上下文，禁止复制和移动。
  Lr20xx(const Lr20xx&) = delete;
  Lr20xx& operator=(const Lr20xx&) = delete;
  Lr20xx(Lr20xx&&) = delete;
  Lr20xx& operator=(Lr20xx&&) = delete;

  /**
   * @brief 初始化 SPI 设备并执行 LR20xx 硬件复位
   * @param frequency_hz SPI 时钟频率，单位为 Hz
   * @return 传输初始化和芯片复位均成功时返回 true
   */
  bool Init(int32_t frequency_hz = 10000000);

  /**
   * @brief 释放 LR20xx 当前使用的 SPI 设备
   * @param delete_bus 是否同时请求释放底层共享 SPI 总线
   * @return 请求的资源全部释放成功时返回 true
   */
  bool Deinit(bool delete_bus = false);

  /**
   * @brief 执行 LR20xx 官方 HAL 硬件复位时序
   * @return 芯片已经初始化且复位成功时返回 true
   */
  bool Reset();

  /**
   * @brief 产生 NSS 唤醒脉冲并等待 BUSY 变为低电平
   * @return 芯片已经初始化且唤醒成功时返回 true
   */
  bool Wakeup();

  /**
   * @brief 按给定参数配置 LR20xx LoRa 数据包收发
   * @param config 完整 LoRa、射频和功放配置
   * @return 所有官方配置命令成功时返回 true
   */
  bool ConfigureLora(const LoraConfig& config);

  /**
   * @brief 将待发送数据写入 LR20xx TX FIFO
   * @param data 待写入数据
   * @param size 数据长度，范围为 1 至 256 且不能超过 FIFO 可用空间
   * @return 数据完整写入时返回 true
   */
  bool WriteBuffer(const uint8_t* data, size_t size);

  /**
   * @brief 从 LR20xx RX FIFO 读取数据
   * @param data 接收数据的目标缓冲区
   * @param size 读取长度，范围为 1 至 256 且不能超过 FIFO 数据量
   * @return 数据完整读出时返回 true
   */
  bool ReadBuffer(uint8_t* data, size_t size);

  /**
   * @brief 启动 LR20xx 数据包接收
   * @param timeout_ms 接收超时时间，单位为 ms，零表示不使用超时
   * @return 接收命令被芯片接受时返回 true
   */
  bool StartReceive(uint32_t timeout_ms);

  /**
   * @brief 启动 LR20xx 数据包发送
   * @param timeout_ms 发送超时时间，单位为 ms，零表示不使用超时
   * @return 发送命令被芯片接受时返回 true
   */
  bool StartTransmit(uint32_t timeout_ms);

  /**
   * @brief 查询 LR20xx 传输上下文是否可以使用
   * @return Init() 成功后至 Deinit() 完成前返回 true
   */
  bool initialized() const;

  /**
   * @brief 返回 USP LR20xx 官方函数使用的不透明上下文
   * @return 已初始化时返回有效上下文，否则返回 nullptr
   */
  const void* context();

 private:
  // 持有 SPI、BUSY、复位和休眠唤醒传输状态。
  std::unique_ptr<LrContext> context_;
  // ConfigureLora() 全部成功后为 true。
  bool lora_configured_ = false;
};

}  // namespace usp_cpp_bus_driver
