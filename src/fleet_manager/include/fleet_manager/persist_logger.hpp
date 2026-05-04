#pragma once

#include <fstream>
#include <mutex>
#include <string>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <filesystem>
#include <ctime>
#include <unistd.h>

namespace fleet_manager
{

// ============================================================================
// 持久化日志器 — PersistLogger
// ============================================================================
/// @brief 轻量级持久化日志工具，用于多机器人调度逻辑的现场调试。
///
/// 将单行结构化日志记录写入指定目录下的时间戳文件，支持
/// 多级日志（INFO / WARN / ERROR）以及条件输出。线程安全。
class PersistLogger
{
public:
  // ==========================================================================
  // 初始化
  // ==========================================================================

  /// @brief 初始化日志器，创建日志目录并打开输出文件。
  /// @param enabled         是否启用日志
  /// @param log_dir         日志目录（为空则使用默认值 "test_logs"）
  /// @param file_prefix     日志文件名前缀，默认 "fleet_manager"
  /// @param also_verbose_info 是否启用条件 INFO 输出（配合 log_info_if）
  static void init(
    bool enabled,
    const std::string & log_dir,
    const std::string & file_prefix = "fleet_manager",
    bool also_verbose_info = false)
  {
    std::lock_guard<std::mutex> lk(mu_);
    enabled_ = enabled;
    verbose_info_ = also_verbose_info;
    if (!enabled_) {
      return;
    }
    log_dir_ = log_dir.empty() ? std::string("test_logs") : log_dir;
    // 若目录不存在则自动创建
    std::error_code ec;
    std::filesystem::create_directories(log_dir_, ec);

    // 使用时间戳命名文件，避免无限制增长
    const auto now = std::chrono::system_clock::now();
    const std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
    // 使用本地时间以匹配运维人员预期
    localtime_r(&t, &tm);

    std::ostringstream name;
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    name << file_prefix << "_"
         << std::put_time(&tm, "%Y%m%d_%H%M%S")
         << "_" << std::setw(3) << std::setfill('0') << ms.count()
         << "_pid" << static_cast<long>(::getpid())
         << ".log";
    file_path_ = std::filesystem::path(log_dir_) / name.str();

    if (ofs_.is_open()) {
      ofs_.close();
    }
    ofs_.open(file_path_, std::ios::out | std::ios::app);
  }

  // ==========================================================================
  // 日志写入接口
  // ==========================================================================

  /// @brief 写入 WARN 级别日志。
  static void log_warn(
    const std::string & tag,
    const std::string & robot_id,
    const std::string & task_id,
    const std::string & message,
    const char * src_file,
    int src_line,
    const char * src_func)
  {
    log_impl("WARN", tag, robot_id, task_id, message, src_file, src_line, src_func);
  }

  /// @brief 写入 ERROR 级别日志。
  static void log_error(
    const std::string & tag,
    const std::string & robot_id,
    const std::string & task_id,
    const std::string & message,
    const char * src_file,
    int src_line,
    const char * src_func)
  {
    log_impl("ERROR", tag, robot_id, task_id, message, src_file, src_line, src_func);
  }

  /// @brief 写入 INFO 级别日志。
  static void log_info(
    const std::string & tag,
    const std::string & robot_id,
    const std::string & task_id,
    const std::string & message,
    const char * src_file,
    int src_line,
    const char * src_func)
  {
    log_impl("INFO", tag, robot_id, task_id, message, src_file, src_line, src_func);
  }

  /// @brief 根据开关条件写入 INFO 级别日志（需同时满足 enabled 和 verbose_info_ 为 true）。
  static void log_info_if(
    bool enabled,
    const std::string & tag,
    const std::string & robot_id,
    const std::string & task_id,
    const std::string & message,
    const char * src_file,
    int src_line,
    const char * src_func)
  {
    if (!enabled || !verbose_info_) {
      return;
    }
    log_impl("INFO", tag, robot_id, task_id, message, src_file, src_line, src_func);
  }

private:
  // ==========================================================================
  // 内部实现
  // ==========================================================================

  /// @brief 日志写入内部实现，统一格式化并输出到文件。
  static void log_impl(
    const std::string & level,
    const std::string & tag,
    const std::string & robot_id,
    const std::string & task_id,
    const std::string & message,
    const char * src_file,
    int src_line,
    const char * src_func)
  {
    std::lock_guard<std::mutex> lk(mu_);
    if (!enabled_) {
      return;
    }
    if (!ofs_.is_open()) {
      // 兜底：若 init() 未被调用，仍然创建默认文件
      std::error_code ec;
      std::filesystem::create_directories("test_logs", ec);
      if (file_path_.empty()) {
        file_path_ = std::filesystem::path("test_logs") / "fleet_manager_fallback.log";
      }
      ofs_.open(file_path_, std::ios::out | std::ios::app);
      if (!ofs_.is_open()) {
        return;
      }
    }

    const auto now = std::chrono::system_clock::now();
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    const std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
    localtime_r(&t, &tm);

    ofs_ << std::put_time(&tm, "%Y-%m-%dT%H:%M:%S")
         << '.' << std::setw(3) << std::setfill('0') << ms.count()
         << "Z"
         << " [" << level << ']'
         << " [" << tag << ']'
         << " robot=" << (robot_id.empty() ? "-" : robot_id)
         << " task=" << (task_id.empty() ? "-" : task_id)
         << " src=" << (src_file ? src_file : "-")
         << ':' << src_line
         << ':' << (src_func ? src_func : "-")
         << ' '
         << message
         << '\n';
    ofs_.flush();
  }

  inline static std::mutex mu_;
  inline static bool enabled_{false};
  inline static bool verbose_info_{false};
  inline static std::string log_dir_;
  inline static std::string file_path_;
  inline static std::ofstream ofs_;
};

}  // namespace fleet_manager
