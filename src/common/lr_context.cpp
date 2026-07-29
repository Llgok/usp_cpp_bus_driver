/*
 * @Description: 实现 LR11xx 与 LR20xx 共用的传输操作
 * @Author: LILYGO_L
 * @Date: 2026-07-10 00:00:00
 * @LastEditTime: 2026-07-15 01:12:34
 * @License: GPL 3.0
 */
#include "lr_context.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <new>

namespace usp_cpp_bus_driver {
bool LrContext::Init(int32_t new_frequency_hz) {
  if ((bus == nullptr) || !reset_callback || (busy_pin < 0) || (cs_pin < 0)) {
    return false;
  }
  if (!bus->SetGpioMode(busy_pin, cpp_bus_driver::Tool::GpioMode::kInput,
          cpp_bus_driver::Tool::GpioStatus::kDisable)) {
    return false;
  }
  if (!bus->Init(new_frequency_hz, cs_pin)) {
    return false;
  }

  frequency_hz = new_frequency_hz;
  initialized = true;
  sleeping = false;
  return true;
}

bool LrContext::Deinit(bool delete_bus) {
  if (bus == nullptr || !bus->Deinit(delete_bus)) {
    return false;
  }
  sleeping = false;
  initialized = false;
  return true;
}

bool LrContext::WaitWhileBusy(uint32_t timeout_us) const {
  if (!initialized || (bus == nullptr) || (busy_pin < 0)) {
    return false;
  }
  for (uint32_t elapsed_us = 0; elapsed_us < timeout_us; ++elapsed_us) {
    if (bus->GpioRead(busy_pin) == 0) {
      return true;
    }
    bus->DelayUs(1);
  }
  bus->LogMessage(cpp_bus_driver::Tool::LogLevel::kError, __FILE__, __LINE__,
      "USP radio busy timeout\n");
  return false;
}

bool LrContext::CheckReady() {
  return sleeping ? Wakeup() : WaitWhileBusy();
}

bool LrContext::Reset(uint32_t reset_time_ms, uint32_t startup_time_ms) {
  if (!initialized || (bus == nullptr) || !reset_callback) {
    return false;
  }

  bool result = reset_callback(false);
  bus->DelayMs(reset_time_ms);
  result &= reset_callback(true);
  bus->DelayMs(startup_time_ms);
  sleeping = false;
  return result && WaitWhileBusy();
}

bool LrContext::Wakeup() {
  if (!initialized || (bus == nullptr)) {
    return false;
  }
  if (!sleeping) {
    return WaitWhileBusy();
  }

  if (wakeup_callback) {
    if (!wakeup_callback()) {
      return false;
    }
  } else {
    if ((cs_pin < 0) || (frequency_hz <= 0) || !bus->Deinit(false)) {
      return false;
    }

    bool result =
        bus->SetGpioMode(cs_pin, cpp_bus_driver::Tool::GpioMode::kOutput) &&
        bus->GpioWrite(cs_pin, 1) && bus->GpioWrite(cs_pin, 0);
    if (!result) {
      bus->GpioWrite(cs_pin, 1);
      return false;
    }

    if (wakeup_pulse_us != 0) {
      bus->DelayUs(wakeup_pulse_us);
    }
    result = bus->GpioWrite(cs_pin, 1);
    if (!result || !bus->Init(frequency_hz, cs_pin)) {
      return false;
    }
  }

  if (!WaitWhileBusy()) {
    return false;
  }
  sleeping = false;
  return true;
}

bool LrContext::Write(const uint8_t* command, size_t command_length,
    const uint8_t* data, size_t data_length) {
  if (!initialized || (command == nullptr) || (command_length == 0) ||
      ((data == nullptr) && (data_length != 0)) ||
      (command_length > std::numeric_limits<size_t>::max() - data_length) ||
      !CheckReady()) {
    return false;
  }

  const size_t transaction_size = command_length + data_length;
  auto buffer =
      std::unique_ptr<uint8_t[]>(new (std::nothrow) uint8_t[transaction_size]);
  if (buffer == nullptr) {
    return false;
  }
  std::copy_n(command, command_length, buffer.get());
  if (data_length != 0) {
    std::copy_n(data, data_length, buffer.get() + command_length);
  }
  return bus->Write(buffer.get(), transaction_size);
}

bool LrContext::Read(const uint8_t* command, size_t command_length,
    size_t dummy_length, uint8_t* data, size_t data_length) {
  if ((data == nullptr) && (data_length != 0)) {
    return false;
  }
  if (dummy_length > std::numeric_limits<size_t>::max() - data_length) {
    return false;
  }
  if (data_length == 0) {
    return ReadRaw(command, command_length, nullptr, 0);
  }

  const size_t transaction_size = dummy_length + data_length;
  auto response = std::unique_ptr<uint8_t[]>(
      new (std::nothrow) uint8_t[transaction_size]());
  if ((response == nullptr) ||
      !ReadRaw(command, command_length, response.get(), transaction_size)) {
    return false;
  }
  std::copy_n(response.get() + dummy_length, data_length, data);
  return true;
}

bool LrContext::ReadRaw(const uint8_t* command, size_t command_length,
    uint8_t* response, size_t response_length) {
  if (!initialized || (command == nullptr) || (command_length == 0) ||
      ((response == nullptr) && (response_length != 0)) || !CheckReady() ||
      !bus->Write(command, command_length)) {
    return false;
  }
  if (response_length == 0) {
    return true;
  }
  if (!WaitWhileBusy()) {
    return false;
  }

  auto write_buffer =
      std::unique_ptr<uint8_t[]>(new (std::nothrow) uint8_t[response_length]());
  return (write_buffer != nullptr) &&
         bus->WriteRead(write_buffer.get(), response, response_length);
}

bool LrContext::DirectRead(uint8_t* data, size_t data_length) {
  if (!initialized || ((data == nullptr) && (data_length != 0)) ||
      !CheckReady()) {
    return false;
  }
  if (data_length == 0) {
    return true;
  }

  auto write_buffer =
      std::unique_ptr<uint8_t[]>(new (std::nothrow) uint8_t[data_length]());
  return (write_buffer != nullptr) &&
         bus->WriteRead(write_buffer.get(), data, data_length);
}

bool LrContext::DirectRead(const uint8_t* command, size_t command_length,
    uint8_t* data, size_t data_length) {
  if (!initialized || (command == nullptr) || (command_length == 0) ||
      ((data == nullptr) && (data_length != 0)) ||
      (command_length > std::numeric_limits<size_t>::max() - data_length) ||
      !CheckReady()) {
    return false;
  }

  const size_t transaction_size = command_length + data_length;
  auto write_buffer = std::unique_ptr<uint8_t[]>(
      new (std::nothrow) uint8_t[transaction_size]());
  auto read_buffer = std::unique_ptr<uint8_t[]>(
      new (std::nothrow) uint8_t[transaction_size]());
  if ((write_buffer == nullptr) || (read_buffer == nullptr)) {
    return false;
  }
  std::copy_n(command, command_length, write_buffer.get());
  if (!bus->WriteRead(
          write_buffer.get(), read_buffer.get(), transaction_size)) {
    return false;
  }
  if (data_length != 0) {
    std::copy_n(read_buffer.get() + command_length, data_length, data);
  }
  return true;
}
}  // namespace usp_cpp_bus_driver
