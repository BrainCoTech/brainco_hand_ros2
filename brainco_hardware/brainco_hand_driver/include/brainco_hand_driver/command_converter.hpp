#pragma once

#include <cstdint>
#include <vector>
#include <algorithm>
#include <cmath>

namespace revo2_driver
{

/**
 * @brief Command Converter - 将 double 类型的控制命令转换为硬件所需的整数类型
 * 
 * 提供安全的类型转换，包括范围检查和舍入处理。
 * 支持的转换类型：
 *   - double → uint16_t
 *   - double → int16_t
 *   - double → uint8_t
 *   - double → int8_t
 */
class CommandConverter
{
public:
  /**
   * @brief 将 double 转换为 uint16_t，带范围限制和舍入
   * 
   * @param value 输入的 double 值
   * @param min_val 最小值限制（默认 0）
   * @param max_val 最大值限制（默认 65535）
   * @return 转换后的 uint16_t 值
   */
  static uint16_t to_uint16(double value, double min_val = 0.0, double max_val = 65535.0)
  {
    double clamped = std::clamp(value, min_val, max_val);
    return static_cast<uint16_t>(std::lround(clamped));
  }

  /**
   * @brief 将 double 转换为 int16_t，带范围限制和舍入
   * 
   * @param value 输入的 double 值
   * @param min_val 最小值限制（默认 -32768）
   * @param max_val 最大值限制（默认 32767）
   * @return 转换后的 int16_t 值
   */
  static int16_t to_int16(double value, double min_val = -32768.0, double max_val = 32767.0)
  {
    double clamped = std::clamp(value, min_val, max_val);
    return static_cast<int16_t>(std::lround(clamped));
  }

  /**
   * @brief 将 double 转换为 uint8_t，带范围限制和舍入
   * 
   * @param value 输入的 double 值
   * @param min_val 最小值限制（默认 0）
   * @param max_val 最大值限制（默认 255）
   * @return 转换后的 uint8_t 值
   */
  static uint8_t to_uint8(double value, double min_val = 0.0, double max_val = 255.0)
  {
    double clamped = std::clamp(value, min_val, max_val);
    return static_cast<uint8_t>(std::lround(clamped));
  }

  /**
   * @brief 将 double 转换为 int8_t，带范围限制和舍入
   * 
   * @param value 输入的 double 值
   * @param min_val 最小值限制（默认 -128）
   * @param max_val 最大值限制（默认 127）
   * @return 转换后的 int8_t 值
   */
  static int8_t to_int8(double value, double min_val = -128.0, double max_val = 127.0)
  {
    double clamped = std::clamp(value, min_val, max_val);
    return static_cast<int8_t>(std::lround(clamped));
  }

  /**
   * @brief 批量转换 double vector 到 uint16_t vector
   * 
   * @param src 源 double 向量
   * @param dst 目标 uint16_t 向量（会被调整大小）
   * @param min_val 最小值限制
   * @param max_val 最大值限制
   */
  static void convert_to_uint16_vector(
    const std::vector<double> & src,
    std::vector<uint16_t> & dst,
    double min_val = 0.0,
    double max_val = 65535.0)
  {
    dst.resize(src.size());
    for (size_t i = 0; i < src.size(); ++i)
    {
      dst[i] = to_uint16(src[i], min_val, max_val);
    }
  }

  /**
   * @brief 批量转换 double vector 到 int16_t vector
   * 
   * @param src 源 double 向量
   * @param dst 目标 int16_t 向量（会被调整大小）
   * @param min_val 最小值限制
   * @param max_val 最大值限制
   */
  static void convert_to_int16_vector(
    const std::vector<double> & src,
    std::vector<int16_t> & dst,
    double min_val = -32768.0,
    double max_val = 32767.0)
  {
    dst.resize(src.size());
    for (size_t i = 0; i < src.size(); ++i)
    {
      dst[i] = to_int16(src[i], min_val, max_val);
    }
  }

  /**
   * @brief 批量转换 double vector 到 uint8_t vector
   * 
   * @param src 源 double 向量
   * @param dst 目标 uint8_t 向量（会被调整大小）
   * @param min_val 最小值限制
   * @param max_val 最大值限制
   */
  static void convert_to_uint8_vector(
    const std::vector<double> & src,
    std::vector<uint8_t> & dst,
    double min_val = 0.0,
    double max_val = 255.0)
  {
    dst.resize(src.size());
    for (size_t i = 0; i < src.size(); ++i)
    {
      dst[i] = to_uint8(src[i], min_val, max_val);
    }
  }

  /**
   * @brief 批量转换 double vector 到 int8_t vector
   * 
   * @param src 源 double 向量
   * @param dst 目标 int8_t 向量（会被调整大小）
   * @param min_val 最小值限制
   * @param max_val 最大值限制
   */
  static void convert_to_int8_vector(
    const std::vector<double> & src,
    std::vector<int8_t> & dst,
    double min_val = -128.0,
    double max_val = 127.0)
  {
    dst.resize(src.size());
    for (size_t i = 0; i < src.size(); ++i)
    {
      dst[i] = to_int8(src[i], min_val, max_val);
    }
  }
};

}  // namespace revo2_driver
