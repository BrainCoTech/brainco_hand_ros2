#ifndef EXECUTE_COMMAND_HPP
#define EXECUTE_COMMAND_HPP

#include <array>
#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

class ExecuteCommand
{
public:
  // 执行命令并返回输出
  static std::string run(const std::string & command)
  {
    static constexpr size_t BUFFER_SIZE = 128;
    std::array<char, BUFFER_SIZE> buffer = {};
    std::string result;

    // 自定义删除器，确保只调用一次pclose
    auto deleter = [](FILE * f)
    {
      if (f) pclose(f);
    };
    std::unique_ptr<FILE, decltype(deleter)> pipe(popen(command.c_str(), "r"), deleter);

    if (!pipe)
    {
      throw std::runtime_error("popen() failed!");
    }

    // 读取命令执行结果
    while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe.get()) != nullptr)
    {
      result += buffer.data();
    }

    return result;
  }
};

#endif  // EXECUTE_COMMAND_HPP
