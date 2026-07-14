/*
 * @Description: None
 * @Author: LILYGO_L
 * @Date: 2026-07-11
 * @LastEditTime: 2026-07-12 12:32:29
 * @License: GPL 3.0
 */
#include "sx127x_context.h"

#include <algorithm>
#include <array>
#include <cstdlib>

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_ipc.h"
#include "sdkconfig.h"

namespace semtech_cpp_bus_driver {
namespace {
// Semtech SX127x FIFO 和单次寄存器访问的最大负载长度
constexpr size_t kMaxDataLength = SX127X_RX_TX_BUFFER_SIZE_MAX;
// SPI 地址阶段占用一个额外字节
constexpr size_t kMaxTransactionLength = kMaxDataLength + 1;
// 异步队列需要覆盖 DIO 双边沿和计时器同时到达的短时突发
constexpr UBaseType_t kEventQueueLength = 32;
// 官方中断处理函数包含多次 SPI 访问和局部缓冲区
constexpr uint32_t kEventTaskStackSize = 6 * 1024;
// 无线中断任务需要及时排空 GFSK FIFO
constexpr UBaseType_t kEventTaskPriority = 10;
// 工作任务使用有限等待，以便在没有事件时响应退出请求
constexpr TickType_t kEventTaskWaitTicks = pdMS_TO_TICKS(20);
// 释放上下文时等待当前驱动中断处理结束的最长时间
constexpr TickType_t kWorkerStopWaitTicks = pdMS_TO_TICKS(1000);
// 定时器屏障使用足够长的单次延时，避免低于 ESP 定时器的最小分辨率
constexpr uint64_t kTimerDrainDelayUs = 1000;
// 释放上下文时等待 ESP 定时器屏障回调的最长时间
constexpr TickType_t kTimerDrainWaitTicks = pdMS_TO_TICKS(1000);

// GPIO ISR 使用的原子类型必须由目标架构直接实现，不能回退到非 ISR 安全的软件锁
static_assert(std::atomic<bool>::is_always_lock_free,
    "SX127x ISR requires lock-free bool atomics");
static_assert(std::atomic<uint32_t>::is_always_lock_free,
    "SX127x ISR requires lock-free 32-bit atomics");

/**
 * @brief 判断 GPIO 编号是否能够作为 ESP-IDF 普通输入引脚使用
 * @param pin 需要检查的 GPIO 编号
 * @return GPIO 编号有效时返回 true
 */
bool IsValidInputPin(int32_t pin) {
  return (pin >= 0) && GPIO_IS_VALID_GPIO(static_cast<gpio_num_t>(pin));
}

/**
 * @brief 作为跨核心同步屏障，确认目标核心此前的中断处理已经结束
 * @param context 未使用的同步调用参数
 */
void DioIpcBarrier(void* context) { static_cast<void>(context); }
}  // namespace

/**
 * @brief 在销毁上下文前安全释放全部异步资源
 */
Sx127xContext::~Sx127xContext() {
  if (initialized || (event_queue_ != nullptr) || (timer_ != nullptr) ||
      (event_task_ != nullptr) || dio_sync_pending_ || dio_attached_[0] ||
      dio_attached_[1] || dio_attached_[2]) {
    const bool result = Deinit(false);
    if (!result &&
        (initialized || (event_queue_ != nullptr) || (timer_ != nullptr) ||
            (event_task_ != nullptr) || dio_sync_pending_ || dio_attached_[0] ||
            dio_attached_[1] || dio_attached_[2])) {
      std::abort();
    }
  }
}

/**
 * @brief 初始化 SPI 和 SX127x 异步事件运行资源
 * @param new_frequency_hz SPI 时钟频率，单位为 Hz
 * @return 所有传输和运行资源初始化成功时返回 true
 */
bool Sx127xContext::Init(int32_t new_frequency_hz) {
  if (initialized) {
    return true;
  }
  if ((event_queue_ != nullptr) || (event_task_ != nullptr) ||
      (event_task_stopped_ != nullptr) || (spi_mutex_ != nullptr) ||
      (timer_mutex_ != nullptr) || (timer_ != nullptr) || dio_sync_pending_ ||
      dio_attached_[0] || dio_attached_[1] || dio_attached_[2]) {
    return false;
  }
  if ((bus == nullptr) || !reset_callback || (new_frequency_hz <= 0) ||
      !IsValidInputPin(dio0_pin) || !IsValidInputPin(dio1_pin) ||
      (cs_pin < 0) || ((dio2_pin >= 0) && !IsValidInputPin(dio2_pin))) {
    return false;
  }
  if (!bus->Init(new_frequency_hz, cs_pin)) {
    return false;
  }

  frequency_hz = new_frequency_hz;
  if (!InitRuntimeResources()) {
    bus->Deinit(false);
    frequency_hz = -1;
    return false;
  }

  initialized = true;
  reset_succeeded = false;
  dio_attach_succeeded = false;
  dio_sync_pending_ = false;
  overflow_mask_.store(kOverflowNone);
  overflow_dio1_value_.store(0);
  overflow_timer_generation_.store(0);
  timer_drain_completed_.store(false);
  accept_events_.store(false);
  return true;
}

/**
 * @brief 停止中断、计时器和工作任务并释放 SPI 设备
 * @param delete_bus 为 true 时同时请求释放底层共享 SPI 总线
 * @return 所有已创建资源均成功释放时返回 true
 */
bool Sx127xContext::Deinit(bool delete_bus) {
  if (!initialized && (event_queue_ == nullptr) && (timer_ == nullptr) &&
      (event_task_ == nullptr) && !dio_sync_pending_ && !dio_attached_[0] &&
      !dio_attached_[1] && !dio_attached_[2]) {
    return true;
  }
  if (IsEventTask()) {
    if (bus != nullptr) {
      bus->LogMessage(cpp_bus_driver::Tool::LogLevel::kError, __FILE__,
          __LINE__, "Do not deinitialize SX127x from its IRQ callback\n");
    }
    return false;
  }

  accept_events_.store(false);
  if (!DetachDioInterrupts()) {
    return false;
  }
  if (!ReleaseRuntimeResources()) {
    return false;
  }

  if (initialized && ((bus == nullptr) || !bus->Deinit(delete_bus))) {
    return false;
  }

  initialized = false;
  reset_succeeded = false;
  dio_attach_succeeded = false;
  radio = nullptr;
  frequency_hz = -1;
  return true;
}

/**
 * @brief 执行开发板提供的 SX127x 硬件复位时序
 * @return 复位断言和释放操作均成功时返回 true
 */
bool Sx127xContext::Reset() {
  reset_succeeded = false;
  if (!initialized || (bus == nullptr) || !reset_callback) {
    return false;
  }

  // false 表示按实际芯片的有效电平断言复位
  bool result = reset_callback(false);
  bus->DelayMs(1);
  // true 表示释放复位，开发板回调应将复位引脚恢复为高阻输入
  result &= reset_callback(true);
  bus->DelayMs(6);

  reset_succeeded = result;
  return result;
}

/**
 * @brief 安装 DIO0、DIO1 和可选 DIO2 的 GPIO 中断
 * @param new_radio Semtech 驱动传入的 SX127x 状态
 * @return 所有必需中断安装成功时返回 true
 */
bool Sx127xContext::AttachDioInterrupts(const sx127x_t* new_radio) {
  dio_attach_succeeded = false;
  if (!initialized || (new_radio == nullptr) ||
      (new_radio->hal_context != this)) {
    return false;
  }

  if (!DetachDioInterrupts()) {
    return false;
  }
  radio = const_cast<sx127x_t*>(new_radio);

  const esp_err_t service_result = gpio_install_isr_service(0);
  if ((service_result != ESP_OK) && (service_result != ESP_ERR_INVALID_STATE)) {
    bus->LogMessage(cpp_bus_driver::Tool::LogLevel::kError, __FILE__, __LINE__,
        "SX127x gpio_install_isr_service failed: %#X\n",
        static_cast<unsigned>(service_result));
    return false;
  }

  bool result = AttachDioInterrupt(
      0, dio0_pin, GPIO_INTR_POSEDGE, &Sx127xContext::Dio0Isr);
  result &= AttachDioInterrupt(
      1, dio1_pin, GPIO_INTR_ANYEDGE, &Sx127xContext::Dio1Isr);
  if (dio2_pin >= 0) {
    result &= AttachDioInterrupt(
        2, dio2_pin, GPIO_INTR_POSEDGE, &Sx127xContext::Dio2Isr);
  }

  if (!result) {
    DetachDioInterrupts();
    radio = nullptr;
    return false;
  }

  dio1_latched_level_.store(
      static_cast<uint32_t>(gpio_get_level(static_cast<gpio_num_t>(dio1_pin))));
  dio_attach_succeeded = true;
  return true;
}

/**
 * @brief 安装一个具有下拉输入配置的 DIO GPIO 中断
 * @param index DIO 安装状态数组索引
 * @param pin DIO 对应的 GPIO 编号
 * @param interrupt_type GPIO 边沿触发类型
 * @param handler 仅投递事件的 GPIO ISR
 * @return GPIO 配置和中断处理器安装成功时返回 true
 */
bool Sx127xContext::AttachDioInterrupt(size_t index, int32_t pin,
    gpio_int_type_t interrupt_type, gpio_isr_t handler) {
  if ((index >= dio_attached_.size()) || !IsValidInputPin(pin) ||
      (handler == nullptr)) {
    return false;
  }

  gpio_config_t config = {};
  config.pin_bit_mask = uint64_t{1} << static_cast<uint32_t>(pin);
  config.mode = GPIO_MODE_INPUT;
  config.pull_up_en = GPIO_PULLUP_DISABLE;
  config.pull_down_en = GPIO_PULLDOWN_ENABLE;
  config.intr_type = interrupt_type;
  esp_err_t result = gpio_config(&config);
  if (result == ESP_OK) {
    result = gpio_isr_handler_add(static_cast<gpio_num_t>(pin), handler, this);
    if (result == ESP_OK) {
      dio_attached_[index] = true;
      result = gpio_intr_enable(static_cast<gpio_num_t>(pin));
    }
  }
  if (result != ESP_OK) {
    bus->LogMessage(cpp_bus_driver::Tool::LogLevel::kError, __FILE__, __LINE__,
        "SX127x DIO interrupt init failed: %#X\n",
        static_cast<unsigned>(result));
    gpio_intr_disable(static_cast<gpio_num_t>(pin));
    if (dio_attached_[index] &&
        (gpio_isr_handler_remove(static_cast<gpio_num_t>(pin)) == ESP_OK)) {
      dio_attached_[index] = false;
      dio_sync_pending_ = true;
    }
    if (!dio_attached_[index]) {
      gpio_reset_pin(static_cast<gpio_num_t>(pin));
    }
    return false;
  }

  return true;
}

/**
 * @brief 从当前 DIO1 事件锁存值读取引脚状态
 * @return DIO1 事件发生时锁存的高低电平
 */
uint32_t Sx127xContext::GetDio1State() const {
  return dio1_latched_level_.load();
}

/**
 * @brief 在一次 NSS 事务中写入 SX127x 寄存器或 FIFO
 * @param address 寄存器地址，地址 0 表示 FIFO
 * @param data 需要写入的数据
 * @param data_length 写入数据的字节数
 * @return SPI 事务成功时返回 true
 */
bool Sx127xContext::Write(
    uint16_t address, const uint8_t* data, uint16_t data_length) {
  if (!initialized || (bus == nullptr) || (address > 0x7F) ||
      (data_length > kMaxDataLength) ||
      ((data == nullptr) && (data_length != 0)) || !LockSpi()) {
    return false;
  }

  std::array<uint8_t, kMaxTransactionLength> transaction = {};
  transaction[0] = static_cast<uint8_t>(address) | 0x80;
  if (data_length != 0) {
    std::copy_n(data, data_length, transaction.data() + 1);
  }
  const bool result = bus->Write(transaction.data(), data_length + 1);
  UnlockSpi();
  return result;
}

/**
 * @brief 在一次 NSS 事务中读取 SX127x 寄存器或 FIFO
 * @param address 寄存器地址，地址 0 表示 FIFO
 * @param data 读取数据的目标缓冲区
 * @param data_length 读取数据的字节数
 * @return SPI 事务成功时返回 true
 */
bool Sx127xContext::Read(
    uint16_t address, uint8_t* data, uint16_t data_length) {
  if (!initialized || (bus == nullptr) || (address > 0x7F) ||
      (data_length > kMaxDataLength) ||
      ((data == nullptr) && (data_length != 0)) || !LockSpi()) {
    return false;
  }

  std::array<uint8_t, kMaxTransactionLength> write_buffer = {};
  std::array<uint8_t, kMaxTransactionLength> read_buffer = {};
  write_buffer[0] = static_cast<uint8_t>(address) & 0x7F;
  const size_t transaction_length = static_cast<size_t>(data_length) + 1;
  const bool result = bus->WriteRead(
      write_buffer.data(), read_buffer.data(), transaction_length);
  if (result && (data_length != 0)) {
    std::copy_n(read_buffer.data() + 1, data_length, data);
  }
  UnlockSpi();
  return result;
}

/**
 * @brief 启动 Semtech 驱动使用的一次性接收超时计时器
 * @param time_in_ms 超时时间，单位为 ms
 * @param callback 计时器到期后由工作任务调用的驱动回调
 * @return 计时器成功启动时返回 true
 */
bool Sx127xContext::StartTimer(uint32_t time_in_ms, TimerCallback callback) {
  if (!initialized || !accept_events_.load() || (timer_ == nullptr) ||
      (callback == nullptr) || (time_in_ms == 0) || !LockTimer()) {
    return false;
  }

  AdvanceTimerGeneration();
  timer_callback_ = nullptr;
  timer_state_.store(TimerState::kIdle);
  if (esp_timer_is_active(timer_)) {
    const esp_err_t stop_result = esp_timer_stop(timer_);
    if ((stop_result != ESP_OK) && (stop_result != ESP_ERR_INVALID_STATE)) {
      UnlockTimer();
      return false;
    }
  }

  AdvanceTimerGeneration();
  timer_callback_ = callback;
  timer_state_.store(TimerState::kArmed);
  const esp_err_t start_result =
      esp_timer_start_once(timer_, static_cast<uint64_t>(time_in_ms) * 1000ULL);
  if (start_result != ESP_OK) {
    AdvanceTimerGeneration();
    timer_callback_ = nullptr;
    timer_state_.store(TimerState::kIdle);
    UnlockTimer();
    bus->LogMessage(cpp_bus_driver::Tool::LogLevel::kError, __FILE__, __LINE__,
        "SX127x esp_timer_start_once failed: %#X\n",
        static_cast<unsigned>(start_result));
    return false;
  }
  UnlockTimer();
  return true;
}

/**
 * @brief 幂等停止当前接收超时计时器并废弃已排队的旧事件
 * @return 计时器处于停止状态时返回 true
 */
bool Sx127xContext::StopTimer() {
  if (timer_mutex_ == nullptr) {
    AdvanceTimerGeneration();
    timer_callback_ = nullptr;
    timer_state_.store(TimerState::kIdle);
    return true;
  }
  if (!LockTimer()) {
    return false;
  }

  AdvanceTimerGeneration();
  timer_callback_ = nullptr;
  timer_state_.store(TimerState::kIdle);
  esp_err_t result = ESP_OK;
  if ((timer_ != nullptr) && esp_timer_is_active(timer_)) {
    result = esp_timer_stop(timer_);
  }
  UnlockTimer();
  return (result == ESP_OK) || (result == ESP_ERR_INVALID_STATE);
}

/**
 * @brief 查询当前一次性接收超时计时器是否仍在运行
 * @return 计时器已启动且尚未处理到期事件时返回 true
 */
bool Sx127xContext::IsTimerStarted() const {
  const TimerState state = timer_state_.load();
  return (state == TimerState::kArmed) || (state == TimerState::kPending);
}

/**
 * @brief 允许成功初始化后的 DIO 和软件超时事件进入工作队列
 */
void Sx127xContext::EnableEventHandling() {
  overflow_mask_.store(kOverflowNone);
  overflow_dio1_value_.store(0);
  overflow_timer_generation_.store(0);
  accept_events_.store(true);
}

/**
 * @brief 判断调用方当前是否正在 SX127x 事件工作任务中运行
 * @return 当前任务是事件工作任务时返回 true
 */
bool Sx127xContext::IsEventTask() const {
  return (event_task_ != nullptr) &&
         (xTaskGetCurrentTaskHandle() == event_task_);
}

/**
 * @brief 创建互斥锁、事件队列、停止信号量、计时器和工作任务
 * @return 所有运行资源创建成功时返回 true
 */
bool Sx127xContext::InitRuntimeResources() {
  spi_mutex_ = xSemaphoreCreateRecursiveMutex();
  timer_mutex_ = xSemaphoreCreateMutex();
  event_queue_ = xQueueCreate(kEventQueueLength, sizeof(Event));
  event_task_stopped_ = xSemaphoreCreateBinary();
  if ((spi_mutex_ == nullptr) || (timer_mutex_ == nullptr) ||
      (event_queue_ == nullptr) || (event_task_stopped_ == nullptr)) {
    ReleaseRuntimeResources();
    return false;
  }

  esp_timer_create_args_t timer_args = {};
  timer_args.callback = &Sx127xContext::TimerExpired;
  timer_args.arg = this;
  timer_args.dispatch_method = ESP_TIMER_TASK;
  timer_args.name = "sx127x_rx_timeout";
  timer_args.skip_unhandled_events = true;
  if (esp_timer_create(&timer_args, &timer_) != ESP_OK) {
    ReleaseRuntimeResources();
    return false;
  }

  stop_worker_.store(false);
  if (xTaskCreate(&Sx127xContext::EventTask, "sx127x_event",
          kEventTaskStackSize, this, kEventTaskPriority,
          &event_task_) != pdPASS) {
    event_task_ = nullptr;
    ReleaseRuntimeResources();
    return false;
  }
  return true;
}

/**
 * @brief 停止并释放当前上下文创建的全部运行资源
 * @return 所有可报告的停止和释放操作成功时返回 true
 */
bool Sx127xContext::ReleaseRuntimeResources() {
  if (!StopTimer() || !StopWorkerTask() || !StopTimer() ||
      !DrainTimerCallbacks()) {
    return false;
  }

  if (timer_ != nullptr) {
    esp_err_t timer_result = esp_timer_delete(timer_);
    for (size_t retry = 0;
        (timer_result == ESP_ERR_INVALID_STATE) && (retry < 3); ++retry) {
      vTaskDelay(1);
      timer_result = esp_timer_delete(timer_);
    }
    if (timer_result != ESP_OK) {
      return false;
    }
    timer_ = nullptr;
  }
  if (event_queue_ != nullptr) {
    vQueueDelete(event_queue_);
    event_queue_ = nullptr;
  }
  if (event_task_stopped_ != nullptr) {
    vSemaphoreDelete(event_task_stopped_);
    event_task_stopped_ = nullptr;
  }
  if (spi_mutex_ != nullptr) {
    vSemaphoreDelete(spi_mutex_);
    spi_mutex_ = nullptr;
  }
  if (timer_mutex_ != nullptr) {
    vSemaphoreDelete(timer_mutex_);
    timer_mutex_ = nullptr;
  }
  return true;
}

/**
 * @brief 移除已经安装的 DIO GPIO 中断
 * @return 所有已安装中断均成功移除时返回 true
 */
bool Sx127xContext::DetachDioInterrupts() {
  bool result = true;
  const std::array<int32_t, 3> pins = {dio0_pin, dio1_pin, dio2_pin};
  for (size_t index = 0; index < dio_attached_.size(); ++index) {
    if (!dio_attached_[index]) {
      continue;
    }

    const auto pin = static_cast<gpio_num_t>(pins[index]);
    gpio_intr_disable(pin);
    const esp_err_t remove_result = gpio_isr_handler_remove(pin);
    if (remove_result != ESP_OK) {
      result = false;
      continue;
    }

    dio_attached_[index] = false;
    dio_sync_pending_ = true;
    if ((gpio_reset_pin(pin) != ESP_OK) && (bus != nullptr)) {
      bus->LogMessage(cpp_bus_driver::Tool::LogLevel::kWarning, __FILE__,
          __LINE__, "SX127x DIO gpio reset failed: %d\n", pins[index]);
    }
  }
  if (result && dio_sync_pending_ && !SynchronizeDioIsrs()) {
    result = false;
  }
  if (result) {
    dio_sync_pending_ = false;
    dio_attach_succeeded = false;
  }
  return result;
}

/**
 * @brief 在其他处理器核心执行同步屏障，等待已经进入的 DIO ISR 返回
 * @return 所有跨核心同步调用成功时返回 true
 */
bool Sx127xContext::SynchronizeDioIsrs() {
#if CONFIG_FREERTOS_NUMBER_OF_CORES > 1
  const BaseType_t current_core = xPortGetCoreID();
  for (BaseType_t core = 0; core < CONFIG_FREERTOS_NUMBER_OF_CORES; ++core) {
    if (core == current_core) {
      continue;
    }
    const esp_err_t result = esp_ipc_call_blocking(
        static_cast<uint32_t>(core), &DioIpcBarrier, nullptr);
    if (result != ESP_OK) {
      if (bus != nullptr) {
        bus->LogMessage(cpp_bus_driver::Tool::LogLevel::kError, __FILE__,
            __LINE__, "SX127x DIO ISR synchronization failed: %#X\n",
            static_cast<unsigned>(result));
      }
      return false;
    }
  }
#endif
  return true;
}

/**
 * @brief 从普通任务或 esp_timer 回调向工作任务投递事件
 * @param event 需要处理的异步事件
 * @return 事件成功进入队列时返回 true
 */
bool Sx127xContext::QueueEvent(const Event& event) {
  return accept_events_.load() && (event_queue_ != nullptr) &&
         (xQueueSend(event_queue_, &event, 0) == pdTRUE);
}

/**
 * @brief 从 GPIO ISR 向工作任务投递事件
 * @param event 需要处理的 DIO 事件
 */
void Sx127xContext::QueueEventFromIsr(const Event& event) {
  if (!accept_events_.load() || (event_queue_ == nullptr)) {
    return;
  }

  BaseType_t higher_priority_task_woken = pdFALSE;
  const BaseType_t queue_result =
      xQueueSendFromISR(event_queue_, &event, &higher_priority_task_woken);
  if (queue_result != pdTRUE) {
    switch (event.type) {
      case EventType::kDio0:
        overflow_mask_.fetch_or(kOverflowDio0);
        break;
      case EventType::kDio1:
        overflow_dio1_value_.store(event.value + 1U);
        break;
      case EventType::kDio2:
        overflow_mask_.fetch_or(kOverflowDio2);
        break;
      case EventType::kTimer:
        break;
    }
  }
  if (higher_priority_task_woken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}

/**
 * @brief 在普通任务上下文中调用对应的 Semtech 驱动回调
 * @param event 已从工作队列取出的异步事件
 */
void Sx127xContext::HandleEvent(const Event& event) {
  if ((radio == nullptr) || !initialized) {
    return;
  }

  switch (event.type) {
    case EventType::kDio0:
      if (radio->dio_0_irq_handler != nullptr) {
        radio->dio_0_irq_handler(radio);
      }
      break;
    case EventType::kDio1:
      dio1_latched_level_.store(event.value);
      if (radio->dio_1_irq_handler != nullptr) {
        radio->dio_1_irq_handler(radio);
      }
      break;
    case EventType::kDio2:
      if (radio->dio_2_irq_handler != nullptr) {
        radio->dio_2_irq_handler(radio);
      }
      break;
    case EventType::kTimer: {
      TimerCallback callback = nullptr;
      if (LockTimer()) {
        if ((event.value == timer_generation_) &&
            (timer_state_.load() == TimerState::kPending)) {
          timer_state_.store(TimerState::kIdle);
          callback = timer_callback_;
          timer_callback_ = nullptr;
        }
        UnlockTimer();
      }
      if (callback != nullptr) {
        callback(radio);
      }
      break;
    }
  }
}

/**
 * @brief 处理队列满时锁存的 DIO 或软件超时事件
 */
void Sx127xContext::HandleOverflowEvents() {
  const uint32_t mask = overflow_mask_.exchange(kOverflowNone);
  if ((mask & kOverflowDio0) != 0) {
    HandleEvent({EventType::kDio0, 0});
  }
  const uint32_t dio1_value = overflow_dio1_value_.exchange(0);
  if (dio1_value != 0) {
    HandleEvent({EventType::kDio1, dio1_value - 1U});
  }
  if ((mask & kOverflowDio2) != 0) {
    HandleEvent({EventType::kDio2, 0});
  }
  const uint32_t timer_generation = overflow_timer_generation_.exchange(0);
  if (timer_generation != 0) {
    HandleEvent({EventType::kTimer, timer_generation});
  }
}

/**
 * @brief 推进软件计时器代次，并跳过表示“无待处理事件”的零值
 * @return 新的软件计时器代次
 */
uint32_t Sx127xContext::AdvanceTimerGeneration() {
  ++timer_generation_;
  if (timer_generation_ == 0) {
    ++timer_generation_;
  }
  return timer_generation_;
}

/**
 * @brief 用同一个 ESP 定时器执行屏障回调，等待此前已派发的回调全部结束
 * @return 屏障回调在限定时间内完成时返回 true
 */
bool Sx127xContext::DrainTimerCallbacks() {
  if (timer_ == nullptr) {
    return true;
  }
  if (!LockTimer()) {
    return false;
  }

  timer_drain_completed_.store(false, std::memory_order_relaxed);
  AdvanceTimerGeneration();
  timer_callback_ = nullptr;
  timer_state_.store(TimerState::kDraining);
  const esp_err_t start_result =
      esp_timer_start_once(timer_, kTimerDrainDelayUs);
  if (start_result != ESP_OK) {
    timer_state_.store(TimerState::kIdle);
    UnlockTimer();
    if (bus != nullptr) {
      bus->LogMessage(cpp_bus_driver::Tool::LogLevel::kError, __FILE__,
          __LINE__, "SX127x timer drain start failed: %#X\n",
          static_cast<unsigned>(start_result));
    }
    return false;
  }
  UnlockTimer();

  const TickType_t wait_start = xTaskGetTickCount();
  while (!timer_drain_completed_.load(std::memory_order_acquire)) {
    if ((xTaskGetTickCount() - wait_start) >= kTimerDrainWaitTicks) {
      return false;
    }
    vTaskDelay(1);
  }
  return true;
}

/**
 * @brief 停止异步事件工作任务并等待其退出
 * @return 工作任务已经退出时返回 true
 */
bool Sx127xContext::StopWorkerTask() {
  if (event_task_ == nullptr) {
    return true;
  }
  if (xTaskGetCurrentTaskHandle() == event_task_) {
    return false;
  }

  stop_worker_.store(true);
  if ((event_task_stopped_ != nullptr) &&
      (xSemaphoreTake(event_task_stopped_, kWorkerStopWaitTicks) == pdTRUE)) {
    event_task_ = nullptr;
    return true;
  }
  return false;
}

/**
 * @brief 获取 SPI 事务递归互斥锁
 * @return 成功持有互斥锁时返回 true
 */
bool Sx127xContext::LockSpi() {
  return (spi_mutex_ != nullptr) &&
         (xSemaphoreTakeRecursive(spi_mutex_, portMAX_DELAY) == pdTRUE);
}

/**
 * @brief 释放 SPI 事务递归互斥锁
 */
void Sx127xContext::UnlockSpi() {
  if (spi_mutex_ != nullptr) {
    xSemaphoreGiveRecursive(spi_mutex_);
  }
}

/**
 * @brief 获取软件计时器复合状态互斥锁
 * @return 成功持有互斥锁时返回 true
 */
bool Sx127xContext::LockTimer() {
  return (timer_mutex_ != nullptr) &&
         (xSemaphoreTake(timer_mutex_, portMAX_DELAY) == pdTRUE);
}

/**
 * @brief 释放软件计时器复合状态互斥锁
 */
void Sx127xContext::UnlockTimer() {
  if (timer_mutex_ != nullptr) {
    xSemaphoreGive(timer_mutex_);
  }
}

/**
 * @brief DIO0 GPIO ISR，仅负责投递上升沿事件
 * @param context 当前 SX127x 传输上下文
 */
void IRAM_ATTR Sx127xContext::Dio0Isr(void* context) {
  auto* radio_context = static_cast<Sx127xContext*>(context);
  if (radio_context != nullptr) {
    radio_context->QueueEventFromIsr({EventType::kDio0, 0});
  }
}

/**
 * @brief DIO1 GPIO ISR，锁存边沿电平后投递事件
 * @param context 当前 SX127x 传输上下文
 */
void IRAM_ATTR Sx127xContext::Dio1Isr(void* context) {
  auto* radio_context = static_cast<Sx127xContext*>(context);
  if (radio_context != nullptr) {
    const uint32_t level = static_cast<uint32_t>(
        gpio_get_level(static_cast<gpio_num_t>(radio_context->dio1_pin)));
    radio_context->QueueEventFromIsr({EventType::kDio1, level});
  }
}

/**
 * @brief DIO2 GPIO ISR，仅负责投递上升沿事件
 * @param context 当前 SX127x 传输上下文
 */
void IRAM_ATTR Sx127xContext::Dio2Isr(void* context) {
  auto* radio_context = static_cast<Sx127xContext*>(context);
  if (radio_context != nullptr) {
    radio_context->QueueEventFromIsr({EventType::kDio2, 0});
  }
}

/**
 * @brief 过滤旧回调，并投递驱动超时事件或完成释放屏障
 * @param context 当前 SX127x 传输上下文
 */
void Sx127xContext::TimerExpired(void* context) {
  auto* radio_context = static_cast<Sx127xContext*>(context);
  if ((radio_context == nullptr) || !radio_context->LockTimer()) {
    return;
  }

  // 同一计时器句柄仍处于活动状态时，当前调用属于上一轮延迟回调
  if (esp_timer_is_active(radio_context->timer_)) {
    radio_context->UnlockTimer();
    return;
  }

  const TimerState state = radio_context->timer_state_.load();
  if (state == TimerState::kDraining) {
    radio_context->timer_state_.store(TimerState::kIdle);
    radio_context->UnlockTimer();
    // release-store 是屏障回调对上下文的最后一次访问
    radio_context->timer_drain_completed_.store(
        true, std::memory_order_release);
    return;
  }
  if (state != TimerState::kArmed) {
    radio_context->UnlockTimer();
    return;
  }

  radio_context->timer_state_.store(TimerState::kPending);
  const uint32_t generation = radio_context->timer_generation_;
  radio_context->UnlockTimer();

  const Event event = {EventType::kTimer, generation};
  if (!radio_context->QueueEvent(event) &&
      radio_context->accept_events_.load()) {
    radio_context->overflow_timer_generation_.store(generation);
  }
}

/**
 * @brief 顺序处理 DIO 和软件超时事件的 FreeRTOS 工作任务
 * @param context 当前 SX127x 传输上下文
 */
void Sx127xContext::EventTask(void* context) {
  auto* radio_context = static_cast<Sx127xContext*>(context);
  if (radio_context == nullptr) {
    vTaskDelete(nullptr);
    return;
  }

  while (!radio_context->stop_worker_.load()) {
    Event event = {};
    if ((radio_context->event_queue_ != nullptr) &&
        (xQueueReceive(radio_context->event_queue_, &event,
             kEventTaskWaitTicks) == pdTRUE)) {
      radio_context->HandleEvent(event);
    }
    radio_context->HandleOverflowEvents();
  }

  if (radio_context->event_task_stopped_ != nullptr) {
    xSemaphoreGive(radio_context->event_task_stopped_);
  }
  vTaskDelete(nullptr);
}
}  // namespace semtech_cpp_bus_driver
