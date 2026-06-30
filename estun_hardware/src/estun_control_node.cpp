#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <exception>
#include <memory>
#include <pthread.h>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "controller_manager/controller_manager.hpp"
#include "estun_hardware/estun_hardware_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_helpers.hpp"

namespace
{
constexpr int kDefaultThreadPriority = 50;
constexpr auto kMissWarnInterval = std::chrono::seconds(2);
constexpr auto kDiagLogInterval = std::chrono::seconds(5);
constexpr int64_t kDeadlineMissThresholdUs = 100;
constexpr auto kSignalStopWaitTimeout = std::chrono::seconds(3);

// 初始化阶段也可能失败，作用域清理可以避免 std::thread 析构时触发 terminate。
template<typename CallbackT>
class ScopeExit
{
public:
  explicit ScopeExit(CallbackT callback)
  : callback_(std::move(callback))
  {
  }

  ScopeExit(const ScopeExit &) = delete;
  ScopeExit & operator=(const ScopeExit &) = delete;

  ~ScopeExit()
  {
    if (active_) {
      callback_();
    }
  }

  void dismiss()
  {
    active_ = false;
  }

private:
  bool active_{true};
  CallbackT callback_;
};

template<typename CallbackT>
ScopeExit<CallbackT> make_scope_exit(CallbackT callback)
{
  return ScopeExit<CallbackT>(std::move(callback));
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    sigset_t handled_signals;
    sigemptyset(&handled_signals);
    sigaddset(&handled_signals, SIGINT);
    sigaddset(&handled_signals, SIGTERM);
    if (pthread_sigmask(SIG_BLOCK, &handled_signals, nullptr) != 0) {
      throw std::runtime_error("pthread_sigmask(SIG_BLOCK) 失败");
    }

    rclcpp::InitOptions init_options;
    init_options.shutdown_on_signal = false;
    rclcpp::init(argc, argv, init_options, rclcpp::SignalHandlerOptions::None);

    const auto context = rclcpp::contexts::get_global_default_context();
    std::atomic<bool> stop_signal_watcher{false};
    std::atomic<bool> signal_shutdown_requested{false};
    std::thread signal_watcher(
      [context, &handled_signals, &stop_signal_watcher, &signal_shutdown_requested]() {
        while (!stop_signal_watcher.load(std::memory_order_acquire)) {
          timespec timeout{};
          timeout.tv_sec = 0;
          timeout.tv_nsec = 100 * 1000 * 1000;
          const int signal_number = sigtimedwait(&handled_signals, nullptr, &timeout);
          if (signal_number == -1) {
            if (errno == EAGAIN || errno == EINTR) {
              continue;
            }
            RCLCPP_WARN(
              rclcpp::get_logger("estun_control_node"),
              "signal watcher: sigtimedwait 失败: errno=%d(%s)",
              errno,
              std::strerror(errno));
            continue;
          }

          bool expected = false;
          if (!signal_shutdown_requested.compare_exchange_strong(
                expected, true, std::memory_order_acq_rel))
          {
            continue;
          }

          auto logger = rclcpp::get_logger("estun_control_node");
          RCLCPP_INFO(
            logger,
            "signal watcher: 捕获到信号 %d，先请求硬件 stop() 再执行 rclcpp::shutdown()。",
            signal_number);
          estun_hardware::request_estun_shutdown_stop();
          if (estun_hardware::wait_for_estun_shutdown_stop_completion(kSignalStopWaitTimeout)) {
            RCLCPP_INFO(logger, "signal watcher: 硬件 stop() 已完成，开始执行 rclcpp::shutdown()。");
          } else {
            RCLCPP_WARN(
              logger,
              "signal watcher: 等待硬件 stop() 超时，继续执行 rclcpp::shutdown()。");
          }
          if (rclcpp::ok(context)) {
            rclcpp::shutdown(context, "signal watcher requested shutdown");
          }
          break;
        }
      });
    auto signal_watcher_cleanup = make_scope_exit([&stop_signal_watcher, &signal_watcher]() {
        stop_signal_watcher.store(true, std::memory_order_release);
        if (signal_watcher.joinable()) {
          signal_watcher.join();
        }
      });

    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    auto controller_manager = std::make_shared<controller_manager::ControllerManager>(
      executor, "controller_manager");

    unsigned int update_rate_hz = controller_manager->get_update_rate();
    if (update_rate_hz == 0U) {
      RCLCPP_WARN(
        controller_manager->get_logger(),
        "controller_manager.update_rate 为 0，回退到 250Hz");
      update_rate_hz = 250U;
    }

    const auto period_ns =
      std::chrono::nanoseconds(1'000'000'000LL / static_cast<int64_t>(update_rate_hz));
    const auto fixed_period = rclcpp::Duration::from_nanoseconds(period_ns.count());
    const double period_ms = static_cast<double>(period_ns.count()) / 1.0e6;
    const bool use_sim_time = controller_manager->get_parameter_or("use_sim_time", false);
    const bool use_logical_control_time =
      controller_manager->get_parameter_or("use_logical_control_time", true);
    const bool enable_control_loop_diag_log =
      controller_manager->get_parameter_or("enable_control_loop_diag_log", false);
    const bool use_ros_clock_sleep = (use_sim_time && !use_logical_control_time);
    const bool has_realtime_kernel = realtime_tools::has_realtime_kernel();
    const bool lock_memory = controller_manager->get_parameter_or<bool>(
      "lock_memory", has_realtime_kernel);
    const int thread_priority =
      controller_manager->get_parameter_or<int>("thread_priority", kDefaultThreadPriority);

    if (lock_memory) {
      const auto lock_result = realtime_tools::lock_memory();
      if (!lock_result.first) {
        RCLCPP_WARN(
          controller_manager->get_logger(),
          "内存锁定失败: '%s'",
          lock_result.second.c_str());
      } else {
        RCLCPP_DEBUG(controller_manager->get_logger(), "内存锁定成功");
      }
    }

    RCLCPP_DEBUG(
      controller_manager->get_logger(),
      "estun_control_node 启动: update_rate=%uHz period=%.3fms control_time=%s overrun_policy=phase_jump sleep_clock=%s diag_log=%s",
      update_rate_hz,
      period_ms,
      use_logical_control_time ? "logical" : "wall",
      use_ros_clock_sleep ? "ros" : "steady",
      enable_control_loop_diag_log ? "on" : "off");

    std::atomic<bool> control_loop_failed{false};
    std::thread control_loop(
      [controller_manager, fixed_period, period_ns, period_ms, use_logical_control_time,
      use_ros_clock_sleep, enable_control_loop_diag_log, thread_priority, &control_loop_failed]() {
        try {
          rclcpp::Parameter cpu_affinity_param;
          if (controller_manager->get_parameter("cpu_affinity", cpu_affinity_param)) {
            std::vector<int> cpus{};
            if (cpu_affinity_param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
              cpus = {static_cast<int>(cpu_affinity_param.as_int())};
            } else if (
              cpu_affinity_param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY)
            {
              const auto cpu_affinity_array = cpu_affinity_param.as_integer_array();
              std::for_each(
                cpu_affinity_array.begin(), cpu_affinity_array.end(),
                [&cpus](int cpu) { cpus.push_back(static_cast<int>(cpu)); });
            }
            const auto affinity_result = realtime_tools::set_current_thread_affinity(cpus);
            if (!affinity_result.first) {
              RCLCPP_WARN(
                controller_manager->get_logger(),
                "设置 CPU 亲和性失败: '%s'",
                affinity_result.second.c_str());
            } else {
              RCLCPP_INFO(controller_manager->get_logger(), "已设置 CPU 亲和性");
            }
          }

          if (!realtime_tools::configure_sched_fifo(thread_priority)) {
            RCLCPP_WARN(
              controller_manager->get_logger(),
              "未能启用 FIFO 调度策略: errno=<%d>(%s)",
              errno,
              std::strerror(errno));
          } else {
            RCLCPP_DEBUG(
              controller_manager->get_logger(),
              "已启用 FIFO 调度策略: priority=%d",
              thread_priority);
          }

          auto next_iteration_time = std::chrono::steady_clock::now();
          auto last_miss_warn_time = next_iteration_time;
          auto last_diag_log_time = next_iteration_time;
          uint64_t tick = 0;
          uint64_t deadline_miss_count = 0;
          uint64_t wake_late_count = 0;
          int64_t max_late_us = 0;
          rclcpp::Time logical_now = controller_manager->now();
          next_iteration_time += period_ns;

          while (rclcpp::ok()) {
            if (use_ros_clock_sleep) {
              // 与官方 ros2_control_node 保持一致：仿真时用 ROS 时钟睡眠。
              const rclcpp::Time now = controller_manager->now();
              controller_manager->read(now, fixed_period);
              controller_manager->update(now, fixed_period);
              controller_manager->write(now, fixed_period);
              ++tick;
              controller_manager->get_clock()->sleep_until(now + fixed_period);

              const auto loop_end_time = std::chrono::steady_clock::now();
              if (enable_control_loop_diag_log && (loop_end_time - last_diag_log_time) >= kDiagLogInterval) {
                RCLCPP_INFO(
                  controller_manager->get_logger(),
                  "控制循环诊断: period_ms=%.3f control_time=wall tick=%llu sleep_clock=ros",
                  period_ms,
                  static_cast<unsigned long long>(tick));
                last_diag_log_time = loop_end_time;
              }
              continue;
            }

            std::this_thread::sleep_until(next_iteration_time);
            const auto wake_time = std::chrono::steady_clock::now();
            const auto late_us =
              std::chrono::duration_cast<std::chrono::microseconds>(wake_time - next_iteration_time)
              .count();
            if (late_us > 0) {
              ++wake_late_count;
              if (late_us > max_late_us) {
                max_late_us = late_us;
              }
              if (late_us > kDeadlineMissThresholdUs) {
                ++deadline_miss_count;
                if (enable_control_loop_diag_log && (wake_time - last_miss_warn_time) >= kMissWarnInterval) {
                  RCLCPP_WARN(
                    controller_manager->get_logger(),
                    "控制循环超时: late_us=%lld misses=%llu max_late_us=%lld threshold_us=%lld (phase jump 已执行)",
                    static_cast<long long>(late_us),
                    static_cast<unsigned long long>(deadline_miss_count),
                    static_cast<long long>(max_late_us),
                    static_cast<long long>(kDeadlineMissThresholdUs));
                  last_miss_warn_time = wake_time;
                }
                // 跳相位：下一次调度从当前唤醒点重新对齐，避免补跑追赶。
                next_iteration_time = wake_time + period_ns;
              } else {
                next_iteration_time += period_ns;
              }
            } else {
              next_iteration_time += period_ns;
            }

            const rclcpp::Time now = use_logical_control_time ? logical_now : controller_manager->now();
            controller_manager->read(now, fixed_period);
            controller_manager->update(now, fixed_period);
            controller_manager->write(now, fixed_period);

            if (use_logical_control_time) {
              logical_now = logical_now + fixed_period;
            }
            ++tick;

            const auto loop_end_time = std::chrono::steady_clock::now();
            if (enable_control_loop_diag_log && (loop_end_time - last_diag_log_time) >= kDiagLogInterval) {
              RCLCPP_INFO(
                controller_manager->get_logger(),
                "控制循环诊断: period_ms=%.3f control_time=%s tick=%llu wake_late=%llu misses=%llu max_late_us=%lld threshold_us=%lld",
                period_ms,
                use_logical_control_time ? "logical" : "wall",
                static_cast<unsigned long long>(tick),
                static_cast<unsigned long long>(wake_late_count),
                static_cast<unsigned long long>(deadline_miss_count),
                static_cast<long long>(max_late_us),
                static_cast<long long>(kDeadlineMissThresholdUs));
              last_diag_log_time = loop_end_time;
            }
          }

          if (rclcpp::ok()) {
            RCLCPP_INFO(
              controller_manager->get_logger(),
              "控制循环退出: tick=%llu wake_late=%llu misses=%llu max_late_us=%lld threshold_us=%lld",
              static_cast<unsigned long long>(tick),
              static_cast<unsigned long long>(wake_late_count),
              static_cast<unsigned long long>(deadline_miss_count),
              static_cast<long long>(max_late_us),
              static_cast<long long>(kDeadlineMissThresholdUs));
          }
        } catch (const std::exception & e) {
          control_loop_failed.store(true);
          RCLCPP_FATAL(
            controller_manager->get_logger(),
            "控制循环异常退出: %s",
            e.what());
          if (rclcpp::ok()) {
            rclcpp::shutdown();
          }
        } catch (...) {
          control_loop_failed.store(true);
          RCLCPP_FATAL(controller_manager->get_logger(), "控制循环异常退出: 未知异常");
          if (rclcpp::ok()) {
            rclcpp::shutdown();
          }
        }
      });
    auto control_loop_cleanup = make_scope_exit([&control_loop]() {
        if (rclcpp::ok()) {
          rclcpp::shutdown();
        }
        if (control_loop.joinable()) {
          control_loop.join();
        }
      });

    executor->add_node(controller_manager);
    try {
      executor->spin();
    } catch (const std::exception & e) {
      control_loop_failed.store(true);
      RCLCPP_FATAL(
        controller_manager->get_logger(),
        "executor::spin 异常退出: %s",
        e.what());
      if (rclcpp::ok()) {
        rclcpp::shutdown();
      }
    } catch (...) {
      control_loop_failed.store(true);
      RCLCPP_FATAL(controller_manager->get_logger(), "executor::spin 异常退出: 未知异常");
      if (rclcpp::ok()) {
        rclcpp::shutdown();
      }
    }

    if (control_loop.joinable()) {
      control_loop.join();
    }
    control_loop_cleanup.dismiss();

    stop_signal_watcher.store(true, std::memory_order_release);
    if (signal_watcher.joinable()) {
      signal_watcher.join();
    }
    signal_watcher_cleanup.dismiss();

    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return control_loop_failed.load() ? 1 : 0;
  } catch (const std::exception & e) {
    RCLCPP_FATAL(
      rclcpp::get_logger("estun_control_node"),
      "estun_control_node 初始化失败: %s",
      e.what());
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 1;
  } catch (...) {
    RCLCPP_FATAL(
      rclcpp::get_logger("estun_control_node"),
      "estun_control_node 初始化失败: 未知异常");
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 1;
  }
}
