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
// Very small persistent logger for debugging multi-robot logic issues.
// Writes a single-line structured record into `log_dir`.
class PersistLogger
{
public:
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
    // Create folder if missing.
    std::error_code ec;
    std::filesystem::create_directories(log_dir_, ec);

    // Use a timestamped file to avoid unbounded growth.
    const auto now = std::chrono::system_clock::now();
    const std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
    // Prefer local time to match operator expectation.
    // (thread-safe variant if available; gmtime_r/localtime_r is common on Linux)
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
      // Best-effort: if init() wasn't called, still create a default file.
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

