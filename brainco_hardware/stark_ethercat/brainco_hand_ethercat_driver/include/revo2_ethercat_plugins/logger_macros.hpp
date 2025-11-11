#pragma once

#include <chrono>
#include <cstdarg>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <string>

/**
 * @brief Enhanced logging macros with file location and function information for Revo2 EtherCAT
 */

namespace revo2_ethercat_plugins
{

/**
 * @brief 获取文件名（去掉路径）
 */
inline std::string getBasename(const std::string & path)
{
  return std::filesystem::path(path).filename().string();
}

/**
 * @brief 格式化位置信息
 */
inline std::string formatLocation(const char * file, int line, const char * function)
{
  return getBasename(file) + ":" + std::to_string(line) + " " + std::string(function);
}

// 内部辅助函数：支持printf风格格式化
inline std::string formatString(const char * format, ...)
{
  char buffer[1024];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  return std::string(buffer);
}

}  // namespace revo2_ethercat_plugins

// 日志宏，输出中国时区（东八区）时间，支持可变参数，时间精确到毫秒（3位）
#define REVO2_LOG_DEBUG(format, ...)                                                              \
  do                                                                                              \
  {                                                                                               \
    auto now = std::chrono::system_clock::now();                                                  \
    auto now_e8 = now + std::chrono::hours(8);                                                    \
    auto ms =                                                                                     \
      std::chrono::duration_cast<std::chrono::milliseconds>(now_e8.time_since_epoch()) % 1000;    \
    auto time_t = std::chrono::system_clock::to_time_t(now_e8);                                   \
    auto tm = *std::gmtime(&time_t);                                                              \
    std::ostringstream oss_time;                                                                  \
    oss_time << std::put_time(&tm, "%Y-%m-%d %H:%M:%S") << "." << std::setfill('0')               \
             << std::setw(3) << ms.count();                                                       \
    std::cout << "[" << oss_time.str() << " "                                                     \
              << "DEBUG] ["                                                                       \
              << revo2_ethercat_plugins::formatLocation(__FILE__, __LINE__, __FUNCTION__) << "] " \
              << revo2_ethercat_plugins::formatString(format, ##__VA_ARGS__) << std::endl;        \
  } while (0)

#define REVO2_LOG_INFO(format, ...)                                                               \
  do                                                                                              \
  {                                                                                               \
    auto now = std::chrono::system_clock::now();                                                  \
    auto now_e8 = now + std::chrono::hours(8);                                                    \
    auto ms =                                                                                     \
      std::chrono::duration_cast<std::chrono::milliseconds>(now_e8.time_since_epoch()) % 1000;    \
    auto time_t = std::chrono::system_clock::to_time_t(now_e8);                                   \
    auto tm = *std::gmtime(&time_t);                                                              \
    std::ostringstream oss_time;                                                                  \
    oss_time << std::put_time(&tm, "%Y-%m-%d %H:%M:%S") << "." << std::setfill('0')               \
             << std::setw(3) << ms.count();                                                       \
    std::cout << "[" << oss_time.str() << " "                                                     \
              << "INFO] ["                                                                        \
              << revo2_ethercat_plugins::formatLocation(__FILE__, __LINE__, __FUNCTION__) << "] " \
              << revo2_ethercat_plugins::formatString(format, ##__VA_ARGS__) << std::endl;        \
  } while (0)

#define REVO2_LOG_WARN(format, ...)                                                               \
  do                                                                                              \
  {                                                                                               \
    auto now = std::chrono::system_clock::now();                                                  \
    auto now_e8 = now + std::chrono::hours(8);                                                    \
    auto ms =                                                                                     \
      std::chrono::duration_cast<std::chrono::milliseconds>(now_e8.time_since_epoch()) % 1000;    \
    auto time_t = std::chrono::system_clock::to_time_t(now_e8);                                   \
    auto tm = *std::gmtime(&time_t);                                                              \
    std::ostringstream oss_time;                                                                  \
    oss_time << std::put_time(&tm, "%Y-%m-%d %H:%M:%S") << "." << std::setfill('0')               \
             << std::setw(3) << ms.count();                                                       \
    std::cout << "[" << oss_time.str() << " "                                                     \
              << "WARN] ["                                                                        \
              << revo2_ethercat_plugins::formatLocation(__FILE__, __LINE__, __FUNCTION__) << "] " \
              << revo2_ethercat_plugins::formatString(format, ##__VA_ARGS__) << std::endl;        \
  } while (0)

#define REVO2_LOG_ERROR(format, ...)                                                              \
  do                                                                                              \
  {                                                                                               \
    auto now = std::chrono::system_clock::now();                                                  \
    auto now_e8 = now + std::chrono::hours(8);                                                    \
    auto ms =                                                                                     \
      std::chrono::duration_cast<std::chrono::milliseconds>(now_e8.time_since_epoch()) % 1000;    \
    auto time_t = std::chrono::system_clock::to_time_t(now_e8);                                   \
    auto tm = *std::gmtime(&time_t);                                                              \
    std::ostringstream oss_time;                                                                  \
    oss_time << std::put_time(&tm, "%Y-%m-%d %H:%M:%S") << "." << std::setfill('0')               \
             << std::setw(3) << ms.count();                                                       \
    std::cout << "[" << oss_time.str() << " "                                                     \
              << "ERROR] ["                                                                       \
              << revo2_ethercat_plugins::formatLocation(__FILE__, __LINE__, __FUNCTION__) << "] " \
              << revo2_ethercat_plugins::formatString(format, ##__VA_ARGS__) << std::endl;        \
  } while (0)
