/*
 * @Description: None
 * @Author: LILYGO_L
 * @Date: 2026-07-12
 * @LastEditTime: 2026-07-12 12:32:29
 * @License: GPL 3.0
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>

#include "bus/bus_guide.h"
#include "common/direct_driver.h"
#include "sx127x.h"

namespace semtech_cpp_bus_driver {

struct Sx127xContext;

/**
 * @brief 管理 SX127x SPI、DIO 中断、软件计时器和官方驱动状态
 *
 * 对象将 GPIO ISR 与软件计时器事件转交给普通 FreeRTOS 工作任务处理，
 * 完整 SX127x 官方功能可以通过 Invoke() 调用。
 */
class Sx127x final : public DirectDriver<Sx127x> {
 public:
  // false 表示断言芯片对应的有效复位电平，true 表示释放为高阻输入。
  using ResetCallback = std::function<bool(bool)>;

  /**
   * @brief SX127x LoRa 调制、数据包、射频和功放配置
   */
  struct LoraConfig {
    uint32_t frequency_hz = 0;  // 射频载波频率，单位为 Hz。
    sx127x_lora_mod_params_t modulation = {
        .sf = SX127X_LORA_SF9,
        .bw = SX127X_LORA_BW_125,
        .cr = SX127X_LORA_CR_4_7,
        .ldro = 0,
    };  // LoRa 调制参数。
    sx127x_lora_pkt_params_t packet = {
        .preamble_len_in_symb = 8,
        .header_type = SX127X_LORA_PKT_EXPLICIT,
        .pld_len_in_bytes = 255,
        .crc_is_on = true,
        .invert_iq_is_on = false,
    };  // LoRa 数据包参数。
    uint8_t sync_word = 0x12;                          // LoRa 同步字。
    sx127x_pa_cfg_params_t pa = {};                    // 功率放大器配置。
    int8_t output_power_dbm = 0;                       // 发射功率，单位 dBm。
    sx127x_ramp_time_t ramp_time = SX127X_RAMP_40_US;  // 功率放大器上升时间。
  };

  /**
   * @brief 创建 SX127x 传输和异步事件资源所有者
   * @param bus SPI 总线对象
   * @param radio_id 实际安装的 SX1272/3/6/7/8/9 型号
   * @param dio0_pin 基本收发需要的 DIO0 GPIO
   * @param dio1_pin 基本收发需要的 DIO1 GPIO
   * @param dio2_pin GFSK/OOK 同步检测使用的 DIO2 GPIO，不使用时传 -1
   * @param cs_pin SPI 片选使用的 GPIO
   * @param reset_callback 控制芯片有效复位电平和高阻释放的回调
   */
  explicit Sx127x(std::shared_ptr<cpp_bus_driver::BusSpiGuide> bus,
      sx127x_radio_id_t radio_id, int32_t dio0_pin, int32_t dio1_pin,
      int32_t dio2_pin, int32_t cs_pin, ResetCallback reset_callback);

  /**
   * @brief 停止异步事件资源并释放当前 SPI 设备
   */
  ~Sx127x();

  // SX127x 对象独占驱动及异步资源，禁止复制和移动。
  Sx127x(const Sx127x&) = delete;
  Sx127x& operator=(const Sx127x&) = delete;
  Sx127x(Sx127x&&) = delete;
  Sx127x& operator=(Sx127x&&) = delete;

  /**
   * @brief 初始化 SPI、DIO、软件计时器和 SX127x 官方驱动状态
   * @param frequency_hz SPI 时钟频率，单位为 Hz
   * @return 全部资源和官方驱动初始化成功时返回 true
   */
  bool Init(int32_t frequency_hz = 10000000);

  /**
   * @brief 停止事件处理并释放 SX127x 当前使用的 SPI 设备
   * @param delete_bus 是否同时请求释放底层共享 SPI 总线
   * @return 请求的资源全部释放成功时返回 true
   */
  bool Deinit(bool delete_bus = false);

  /**
   * @brief 按给定参数配置 SX127x LoRa 数据包收发
   * @param config 完整 LoRa、射频和功放配置
   * @return 所有官方配置命令成功时返回 true
   */
  bool ConfigureLora(const LoraConfig& config);

  /**
   * @brief 向 SX127x 无线缓冲区指定偏移写入数据
   * @param offset 写入起始偏移
   * @param data 待写入数据
   * @param size 写入长度，有效范围为 1 至 255 字节
   * @return 请求范围有效且数据完整写入时返回 true
   */
  bool WriteBuffer(uint8_t offset, const uint8_t* data, size_t size);

  /**
   * @brief 从 SX127x 无线缓冲区指定偏移读取数据
   * @param offset 读取起始偏移
   * @param data 接收数据的目标缓冲区
   * @param size 读取长度，有效范围为 1 至 255 字节
   * @return 请求范围有效且数据完整读出时返回 true
   */
  bool ReadBuffer(uint8_t offset, uint8_t* data, size_t size);

  /**
   * @brief 启动 SX127x 数据包接收
   * @param timeout_ms 接收超时时间，单位为 ms，零表示连续接收
   * @return 接收命令被芯片接受时返回 true
   */
  bool StartReceive(uint32_t timeout_ms);

  /**
   * @brief 启动 SX127x 数据包发送
   * @return 发送命令被芯片接受时返回 true
   */
  bool StartTransmit();

  /**
   * @brief 查询 SX127x 官方驱动状态是否可以使用
   * @return Init() 成功后至 Deinit() 开始前返回 true
   */
  bool initialized() const;

  /**
   * @brief 返回可传给 SX127x 官方函数的可写状态
   * @return 已初始化时返回有效状态，否则返回 nullptr
   */
  sx127x_t* context();

  /**
   * @brief 返回可传给只读 SX127x 官方函数的状态
   * @return 已初始化时返回有效只读状态，否则返回 nullptr
   */
  const sx127x_t* context() const;

 private:
  // 持有 SPI、DIO、工作任务和软件计时器资源。
  std::unique_ptr<Sx127xContext> context_;
  // Semtech 官方 SX127x 驱动运行状态。
  sx127x_t radio_ = {};
  // 官方驱动和异步桥接全部初始化完成后为 true。
  bool driver_initialized_ = false;
  // ConfigureLora() 全部成功后为 true。
  bool lora_configured_ = false;
};

}  // namespace semtech_cpp_bus_driver
