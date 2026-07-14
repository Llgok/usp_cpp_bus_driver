/*
 * @Description: None
 * @Author: LILYGO_L
 * @Date: 2026-07-11
 * @LastEditTime: 2026-07-12 12:32:29
 * @License: GPL 3.0
 */
#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>

#include "bus/bus_guide.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sx127x.h"

namespace semtech_cpp_bus_driver {
// 管理 SX127x SPI、DIO 中断和软件接收超时资源
struct Sx127xContext {
  // false 表示断言复位，true 表示释放复位并将引脚恢复为高阻输入
  using ResetCallback = std::function<bool(bool)>;

  // Semtech 驱动内部使用的软件超时回调类型
  using TimerCallback = void (*)(void*);

  // 与应用程序共享的 SPI 传输对象
  std::shared_ptr<cpp_bus_driver::BusSpiGuide> bus;
  // 由开发板实现的复位控制回调
  ResetCallback reset_callback;
  // 由当前上下文服务的 Semtech SX127x 驱动状态
  sx127x_t* radio = nullptr;
  // 实际安装在开发板上的 SX127x 型号
  sx127x_radio_id_t radio_id = SX127X_RADIO_ID_SX1276;
  // DIO0 上升沿中断 GPIO
  int32_t dio0_pin = -1;
  // DIO1 双边沿中断 GPIO
  int32_t dio1_pin = -1;
  // GFSK/OOK 同步检测使用的 DIO2 上升沿 GPIO，不使用时允许为 -1
  int32_t dio2_pin = -1;
  // SPI 总线用作 NSS 的 GPIO
  int32_t cs_pin = -1;
  // 已配置的 SPI 时钟频率，单位为 Hz
  int32_t frequency_hz = -1;
  // SPI、工作任务和计时器均可用时为 true
  bool initialized = false;
  // 最近一次 DIO 中断安装是否全部成功
  bool dio_attach_succeeded = false;
  // 最近一次硬件复位回调是否成功
  bool reset_succeeded = false;

  /**
   * @brief 在销毁上下文前安全释放全部异步资源
   */
  ~Sx127xContext();

  /**
   * @brief 初始化 SPI 和 SX127x 异步事件运行资源
   * @param new_frequency_hz SPI 时钟频率，单位为 Hz
   * @return 所有传输和运行资源初始化成功时返回 true
   */
  bool Init(int32_t new_frequency_hz);

  /**
   * @brief 停止中断、计时器和工作任务并释放 SPI 设备
   * @param delete_bus 为 true 时同时请求释放底层共享 SPI 总线
   * @return 所有已创建资源均成功释放时返回 true
   */
  bool Deinit(bool delete_bus);

  /**
   * @brief 执行开发板提供的 SX127x 硬件复位时序
   * @return 复位断言和释放操作均成功时返回 true
   */
  bool Reset();

  /**
   * @brief 安装 DIO0、DIO1 和可选 DIO2 的 GPIO 中断
   * @param new_radio Semtech 驱动传入的 SX127x 状态
   * @return 所有必需中断安装成功时返回 true
   */
  bool AttachDioInterrupts(const sx127x_t* new_radio);

  /**
   * @brief 从当前 DIO1 事件锁存值读取引脚状态
   * @return DIO1 事件发生时锁存的高低电平
   */
  uint32_t GetDio1State() const;

  /**
   * @brief 在一次 NSS 事务中写入 SX127x 寄存器或 FIFO
   * @param address 寄存器地址，地址 0 表示 FIFO
   * @param data 需要写入的数据
   * @param data_length 写入数据的字节数
   * @return SPI 事务成功时返回 true
   */
  bool Write(uint16_t address, const uint8_t* data, uint16_t data_length);

  /**
   * @brief 在一次 NSS 事务中读取 SX127x 寄存器或 FIFO
   * @param address 寄存器地址，地址 0 表示 FIFO
   * @param data 读取数据的目标缓冲区
   * @param data_length 读取数据的字节数
   * @return SPI 事务成功时返回 true
   */
  bool Read(uint16_t address, uint8_t* data, uint16_t data_length);

  /**
   * @brief 启动 Semtech 驱动使用的一次性接收超时计时器
   * @param time_in_ms 超时时间，单位为 ms
   * @param callback 计时器到期后由工作任务调用的驱动回调
   * @return 计时器成功启动时返回 true
   */
  bool StartTimer(uint32_t time_in_ms, TimerCallback callback);

  /**
   * @brief 幂等停止当前接收超时计时器并废弃已排队的旧事件
   * @return 计时器处于停止状态时返回 true
   */
  bool StopTimer();

  /**
   * @brief 查询当前一次性接收超时计时器是否仍在运行
   * @return 计时器已启动且尚未到期时返回 true
   */
  bool IsTimerStarted() const;

  /**
   * @brief 允许成功初始化后的 DIO 和软件超时事件进入工作队列
   */
  void EnableEventHandling();

  /**
   * @brief 判断调用方当前是否正在 SX127x 事件工作任务中运行
   * @return 当前任务是事件工作任务时返回 true
   */
  bool IsEventTask() const;

 private:
  // 所有 DIO 和软件超时都由同一个工作任务顺序处理
  enum class EventType : uint8_t {
    kDio0,
    kDio1,
    kDio2,
    kTimer,
  };

  // 工作队列中的单个 SX127x 异步事件
  struct Event {
    EventType type = EventType::kDio0;
    // DIO1 事件使用触发瞬间电平，计时器事件使用代次编号
    uint32_t value = 0;
  };

  // 软件计时器区分空闲、运行、待处理和释放屏障四种状态
  enum class TimerState : uint8_t {
    kIdle,
    kArmed,
    kPending,
    kDraining,
  };

  // 队列满时保留至少一次同类硬件事件，供工作任务尽力补处理
  enum EventOverflowMask : uint32_t {
    kOverflowNone = 0,
    kOverflowDio0 = 1U << 0,
    kOverflowDio2 = 1U << 1,
  };

  /**
   * @brief 创建互斥锁、事件队列、停止信号量、计时器和工作任务
   * @return 所有运行资源创建成功时返回 true
   */
  bool InitRuntimeResources();

  /**
   * @brief 停止并释放当前上下文创建的全部运行资源
   * @return 所有可报告的停止和释放操作成功时返回 true
   */
  bool ReleaseRuntimeResources();

  /**
   * @brief 移除已经安装的 DIO GPIO 中断
   * @return 所有已安装中断均成功移除时返回 true
   */
  bool DetachDioInterrupts();

  /**
   * @brief 在其他处理器核心执行同步屏障，等待已经进入的 DIO ISR 返回
   * @return 所有跨核心同步调用成功时返回 true
   */
  bool SynchronizeDioIsrs();

  /**
   * @brief 安装一个具有下拉输入配置的 DIO GPIO 中断
   * @param index DIO 安装状态数组索引
   * @param pin DIO 对应的 GPIO 编号
   * @param interrupt_type GPIO 边沿触发类型
   * @param handler 仅投递事件的 GPIO ISR
   * @return GPIO 配置和中断处理器安装成功时返回 true
   */
  bool AttachDioInterrupt(size_t index, int32_t pin,
      gpio_int_type_t interrupt_type, gpio_isr_t handler);

  /**
   * @brief 从普通任务或 esp_timer 回调向工作任务投递事件
   * @param event 需要处理的异步事件
   * @return 事件成功进入队列时返回 true
   */
  bool QueueEvent(const Event& event);

  /**
   * @brief 从 GPIO ISR 向工作任务投递事件
   * @param event 需要处理的 DIO 事件
   */
  void IRAM_ATTR QueueEventFromIsr(const Event& event);

  /**
   * @brief 在普通任务上下文中调用对应的 Semtech 驱动回调
   * @param event 已从工作队列取出的异步事件
   */
  void HandleEvent(const Event& event);

  /**
   * @brief 处理队列满时锁存的 DIO 或软件超时事件
   */
  void HandleOverflowEvents();

  /**
   * @brief 推进软件计时器代次，并跳过表示“无待处理事件”的零值
   * @return 新的软件计时器代次
   */
  uint32_t AdvanceTimerGeneration();

  /**
   * @brief 用同一个 ESP 定时器执行屏障回调，等待此前已派发的回调全部结束
   * @return 屏障回调在限定时间内完成时返回 true
   */
  bool DrainTimerCallbacks();

  /**
   * @brief 停止异步事件工作任务并等待其退出
   * @return 工作任务已经退出时返回 true
   */
  bool StopWorkerTask();

  /**
   * @brief 获取 SPI 事务互斥锁
   * @return 成功持有互斥锁时返回 true
   */
  bool LockSpi();

  /**
   * @brief 释放 SPI 事务互斥锁
   */
  void UnlockSpi();

  /**
   * @brief 获取软件计时器复合状态互斥锁
   * @return 成功持有互斥锁时返回 true
   */
  bool LockTimer();

  /**
   * @brief 释放软件计时器复合状态互斥锁
   */
  void UnlockTimer();

  /**
   * @brief DIO0 GPIO ISR，仅负责投递上升沿事件
   * @param context 当前 SX127x 传输上下文
   */
  static void IRAM_ATTR Dio0Isr(void* context);

  /**
   * @brief DIO1 GPIO ISR，锁存边沿电平后投递事件
   * @param context 当前 SX127x 传输上下文
   */
  static void IRAM_ATTR Dio1Isr(void* context);

  /**
   * @brief DIO2 GPIO ISR，仅负责投递上升沿事件
   * @param context 当前 SX127x 传输上下文
   */
  static void IRAM_ATTR Dio2Isr(void* context);

  /**
   * @brief 过滤旧回调，并投递驱动超时事件或完成释放屏障
   * @param context 当前 SX127x 传输上下文
   */
  static void TimerExpired(void* context);

  /**
   * @brief 顺序处理 DIO 和软件超时事件的 FreeRTOS 工作任务
   * @param context 当前 SX127x 传输上下文
   */
  static void EventTask(void* context);

  // DIO 中断是否已经逐个安装
  std::array<bool, 3> dio_attached_ = {};
  // 已移除 DIO 处理器但尚未完成跨核心 ISR 退出屏障时为 true
  bool dio_sync_pending_ = false;
  // 工作任务停止期间阻止 ISR 和计时器继续投递事件
  std::atomic<bool> accept_events_{false};
  // 请求事件工作任务安全退出
  std::atomic<bool> stop_worker_{false};
  // DIO1 双边沿触发瞬间锁存的电平
  std::atomic<uint32_t> dio1_latched_level_{0};
  // 队列满时仍需由工作任务尽力补处理的无载荷 DIO 事件类型
  std::atomic<uint32_t> overflow_mask_{kOverflowNone};
  // 队列满时待补处理的 DIO1 电平加一，零表示没有待处理事件
  std::atomic<uint32_t> overflow_dio1_value_{0};
  // 队列满时待补处理的软件计时器代次，零表示没有待处理事件
  std::atomic<uint32_t> overflow_timer_generation_{0};
  // 单实例一次性软件计时器的当前状态
  std::atomic<TimerState> timer_state_{TimerState::kIdle};
  // 屏障回调最后一次访问上下文后以 release 顺序置为 true
  std::atomic<bool> timer_drain_completed_{false};
  // 每次启动或停止都会更新，用于淘汰已排队的过期超时事件
  uint32_t timer_generation_ = 0;
  // 当前计时器到期后需要调用的 Semtech 驱动回调
  TimerCallback timer_callback_ = nullptr;
  // 串行化 DIO 与软件超时事件的 FreeRTOS 队列
  QueueHandle_t event_queue_ = nullptr;
  // 执行 Semtech 中断处理函数的 FreeRTOS 工作任务
  TaskHandle_t event_task_ = nullptr;
  // 工作任务退出后用于通知释放方的二值信号量
  SemaphoreHandle_t event_task_stopped_ = nullptr;
  // 防止应用任务和中断工作任务同时访问同一个 SPI 设备
  SemaphoreHandle_t spi_mutex_ = nullptr;
  // 保护计时器状态、代次和回调指针的一致性
  SemaphoreHandle_t timer_mutex_ = nullptr;
  // 实现 Semtech 接收超时的一次性 ESP 定时器
  esp_timer_handle_t timer_ = nullptr;
};
}  // namespace semtech_cpp_bus_driver
