#pragma once

#include <chrono>
#include <cstdarg>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

namespace brainco_hand_driver
{
namespace logging
{

inline std::string get_basename(const std::string & path)
{
  return std::filesystem::path(path).filename().string();
}

inline std::string format_location(const char * file, int line, const char * function)
{
  return get_basename(file) + ":" + std::to_string(line) + " " + std::string(function);
}

inline std::string format_string(const char * format, ...)
{
  char buffer[1024];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  return std::string(buffer);
}

inline std::string now_string()
{
  auto now = std::chrono::system_clock::now();
  auto now_e8 = now + std::chrono::hours(8);
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now_e8.time_since_epoch()) % 1000;
  auto time_t = std::chrono::system_clock::to_time_t(now_e8);
  auto tm = *std::gmtime(&time_t);
  std::ostringstream oss_time;
  oss_time << std::put_time(&tm, "%Y-%m-%d %H:%M:%S") << "." << std::setfill('0') << std::setw(3)
           << ms.count();
  return oss_time.str();
}

}  // namespace logging
}  // namespace brainco_hand_driver

#define BRAINCO_HAND_LOG_DEBUG(format, ...)                                                        \
  do                                                                                               \
  {                                                                                                \
    std::cout << "[" << ::brainco_hand_driver::logging::now_string() << " DEBUG] ["                \
              << ::brainco_hand_driver::logging::format_location(__FILE__, __LINE__, __FUNCTION__) \
              << "] " << ::brainco_hand_driver::logging::format_string(format, ##__VA_ARGS__)      \
              << std::endl;                                                                        \
  } while (0)

#define BRAINCO_HAND_LOG_INFO(format, ...)                                                         \
  do                                                                                               \
  {                                                                                                \
    std::cout << "[" << ::brainco_hand_driver::logging::now_string() << " INFO] ["                 \
              << ::brainco_hand_driver::logging::format_location(__FILE__, __LINE__, __FUNCTION__) \
              << "] " << ::brainco_hand_driver::logging::format_string(format, ##__VA_ARGS__)      \
              << std::endl;                                                                        \
  } while (0)

#define BRAINCO_HAND_LOG_WARN(format, ...)                                                         \
  do                                                                                               \
  {                                                                                                \
    std::cout << "[" << ::brainco_hand_driver::logging::now_string() << " WARN] ["                 \
              << ::brainco_hand_driver::logging::format_location(__FILE__, __LINE__, __FUNCTION__) \
              << "] " << ::brainco_hand_driver::logging::format_string(format, ##__VA_ARGS__)      \
              << std::endl;                                                                        \
  } while (0)

#define BRAINCO_HAND_LOG_ERROR(format, ...)                                                        \
  do                                                                                               \
  {                                                                                                \
    std::cout << "[" << ::brainco_hand_driver::logging::now_string() << " ERROR] ["                \
              << ::brainco_hand_driver::logging::format_location(__FILE__, __LINE__, __FUNCTION__) \
              << "] " << ::brainco_hand_driver::logging::format_string(format, ##__VA_ARGS__)      \
              << std::endl;                                                                        \
  } while (0)
