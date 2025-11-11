#ifndef EXECUTE_COMMAND_HPP
#define EXECUTE_COMMAND_HPP

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
    // 创建管道并打开命令
    std::array<char, 128> buffer;
    std::string result;
    std::shared_ptr<FILE> pipe(popen(command.c_str(), "r"), fclose);

    if (!pipe)
    {
      throw std::runtime_error("popen() failed!");
    }

    // 读取命令执行结果
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr)
    {
      result += buffer.data();
    }

    return result;
  }
};

#endif  // EXECUTE_COMMAND_HPP
