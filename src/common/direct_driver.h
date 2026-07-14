/*
 * @Description: None
 * @Author: LILYGO_L
 * @Date: 2026-07-12
 * @LastEditTime: 2026-07-12 12:32:29
 * @License: GPL 3.0
 */
#pragma once

#include <cstdlib>
#include <functional>
#include <type_traits>
#include <utility>

namespace semtech_cpp_bus_driver {

/**
 * @brief 为无线芯片对象提供零额外开销的 Semtech 官方函数转发入口
 * @tparam Derived 持有官方驱动上下文的具体无线芯片类型
 *
 * 派生类型必须公开 initialized() 和 context()。Invoke() 只负责注入第一个
 * 上下文参数，其余参数和返回类型完全保留官方函数定义。
 */
template <typename Derived>
class DirectDriver {
 public:
  /**
   * @brief 调用需要可写上下文的 Semtech 官方直接驱动函数
   * @tparam Function 官方函数指针类型
   * @tparam Args 除上下文以外的参数类型
   * @param function 第一个参数为无线芯片上下文的官方函数
   * @param args 传递给官方函数的其余参数
   * @return 官方函数原始返回值
   * @pre 当前无线芯片对象已经成功初始化
   */
  template <typename Function, typename... Args>
  decltype(auto) Invoke(Function function, Args&&... args) {
    static_assert(std::is_pointer_v<Function>,
        "Invoke requires a Semtech direct-driver function pointer");

    Derived& driver = static_cast<Derived&>(*this);
    auto context = driver.context();
    if (!driver.initialized() || context == nullptr || function == nullptr) {
      std::abort();
    }
    static_assert(std::is_invocable_v<Function, decltype(context), Args...>,
        "Function arguments do not match the Semtech direct-driver API");
    return std::invoke(function, context, std::forward<Args>(args)...);
  }

 protected:
  /**
   * @brief 构造和销毁不持有资源的 CRTP 转发基类
   */
  DirectDriver() = default;
  ~DirectDriver() = default;

  DirectDriver(const DirectDriver&) = default;
  DirectDriver& operator=(const DirectDriver&) = default;
};

}  // namespace semtech_cpp_bus_driver
