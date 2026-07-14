/*
 * @Description: None
 * @Author: LILYGO_L
 * @Date: 2026-07-12
 * @LastEditTime: 2026-07-12 16:30:56
 * @License: GPL 3.0
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>

#include "bus/bus_guide.h"
#include "common/direct_driver.h"
#include "lr11xx_bootloader.h"
#include "lr11xx_crypto_engine.h"
#include "lr11xx_driver_version.h"
#include "lr11xx_gnss.h"
#include "lr11xx_lr_fhss.h"
#include "lr11xx_radio.h"
#include "lr11xx_radio_timings.h"
#include "lr11xx_regmem.h"
#include "lr11xx_rttof.h"
#include "lr11xx_system.h"
#include "lr11xx_wifi.h"

namespace semtech_cpp_bus_driver {

struct RadioContext;

/**
 * @brief 管理 LR11xx SPI 传输资源并封装常用 LoRa 操作
 *
 * GNSS、Wi-Fi、Crypto、RTToF、Bootloader 和其他完整官方功能通过
 * Invoke() 调用，调用者不需要手动传入上下文。
 */
class Lr11xx final : public DirectDriver<Lr11xx> {
 public:
  // false 表示断言硬件复位，true 表示释放硬件复位。
  using ResetCallback = std::function<bool(bool)>;
  // 产生 NSS 低电平 100 us 的芯片唤醒脉冲。
  using WakeupCallback = std::function<bool()>;

  /**
   * @brief LR11xx LoRa 调制、数据包、射频和功放配置
   */
  struct LoraConfig {
    uint32_t frequency_hz = 0;  // 射频载波频率，单位为 Hz。
    lr11xx_radio_mod_params_lora_t modulation = {
        .sf = LR11XX_RADIO_LORA_SF9,
        .bw = LR11XX_RADIO_LORA_BW_125,
        .cr = LR11XX_RADIO_LORA_CR_4_7,
        .ldro = 0,
    };  // LoRa 调制参数。
    lr11xx_radio_pkt_params_lora_t packet = {
        .preamble_len_in_symb = 8,
        .header_type = LR11XX_RADIO_LORA_PKT_EXPLICIT,
        .pld_len_in_bytes = 255,
        .crc = LR11XX_RADIO_LORA_CRC_ON,
        .iq = LR11XX_RADIO_LORA_IQ_STANDARD,
    };  // LoRa 数据包参数。
    uint8_t sync_word = 0x12;       // LoRa 同步字。
    lr11xx_radio_pa_cfg_t pa = {};  // 功率放大器配置。
    int8_t output_power_dbm = 0;    // 发射功率，单位 dBm。
    lr11xx_radio_ramp_time_t ramp_time =
        LR11XX_RADIO_RAMP_48_US;  // 功率放大器上升时间。
  };

  /**
   * @brief 创建 LR11xx 传输资源所有者
   * @param bus SPI 总线对象
   * @param busy_pin 芯片 BUSY 信号连接的 GPIO
   * @param cs_pin SPI 片选使用的 GPIO
   * @param reset_callback 控制开发板硬件复位信号的回调
   * @param wakeup_callback 产生 NSS 低电平 100 us 的可选回调
   */
  explicit Lr11xx(std::shared_ptr<cpp_bus_driver::BusSpiGuide> bus,
      int32_t busy_pin, int32_t cs_pin, ResetCallback reset_callback,
      WakeupCallback wakeup_callback = {});

  /**
   * @brief 释放当前对象仍持有的 SPI 设备资源
   */
  ~Lr11xx();

  // LR11xx 对象独占传输上下文，禁止复制和移动。
  Lr11xx(const Lr11xx&) = delete;
  Lr11xx& operator=(const Lr11xx&) = delete;
  Lr11xx(Lr11xx&&) = delete;
  Lr11xx& operator=(Lr11xx&&) = delete;

  /**
   * @brief 初始化 SPI 设备并执行 LR11xx 硬件复位
   * @param frequency_hz SPI 时钟频率，单位为 Hz
   * @return 传输初始化和芯片复位均成功时返回 true
   */
  bool Init(int32_t frequency_hz = 10000000);

  /**
   * @brief 释放 LR11xx 当前使用的 SPI 设备
   * @param delete_bus 是否同时请求释放底层共享 SPI 总线
   * @return 请求的资源全部释放成功时返回 true
   */
  bool Deinit(bool delete_bus = false);

  /**
   * @brief 执行 LR11xx 官方 HAL 硬件复位时序
   * @return 芯片已经初始化且复位成功时返回 true
   */
  bool Reset();

  /**
   * @brief 产生 NSS 唤醒脉冲并等待 BUSY 变为低电平
   * @return 芯片已经初始化且唤醒成功时返回 true
   */
  bool Wakeup();

  /**
   * @brief 按给定参数配置 LR11xx LoRa 数据包收发
   * @param config 完整 LoRa、射频和功放配置
   * @return 所有官方配置命令成功时返回 true
   */
  bool ConfigureLora(const LoraConfig& config);

  /**
   * @brief 将待发送数据写入 LR11xx 无线缓冲区
   * @param data 待写入数据
   * @param size 数据长度，有效范围为 1 至 255 字节
   * @return 数据完整写入时返回 true
   */
  bool WriteBuffer(const uint8_t* data, size_t size);

  /**
   * @brief 从 LR11xx 接收缓冲区的指定偏移读取数据
   * @param offset 接收缓冲区起始偏移
   * @param data 接收数据的目标缓冲区
   * @param size 读取长度，有效范围为 1 至 255 字节
   * @return 请求范围有效且数据完整读出时返回 true
   */
  bool ReadBuffer(uint8_t offset, uint8_t* data, size_t size);

  /**
   * @brief 启动 LR11xx 数据包接收
   * @param timeout_ms 接收超时时间，单位为 ms，零表示不使用超时
   * @return 接收命令被芯片接受时返回 true
   */
  bool StartReceive(uint32_t timeout_ms);

  /**
   * @brief 启动 LR11xx 数据包发送
   * @param timeout_ms 发送超时时间，单位为 ms，零表示不使用超时
   * @return 发送命令被芯片接受时返回 true
   */
  bool StartTransmit(uint32_t timeout_ms);

  /**
   * @brief 查询 LR11xx 传输上下文是否可以使用
   * @return Init() 成功后至 Deinit() 完成前返回 true
   */
  bool initialized() const;

  /**
   * @brief 返回 Semtech LR11xx 官方函数使用的不透明上下文
   * @return 已初始化时返回有效上下文，否则返回 nullptr
   */
  const void* context();

 private:
  // 持有 SPI、BUSY、复位和休眠唤醒传输状态。
  std::unique_ptr<RadioContext> context_;
  // ConfigureLora() 全部成功后为 true。
  bool lora_configured_ = false;
};

}  // namespace semtech_cpp_bus_driver
